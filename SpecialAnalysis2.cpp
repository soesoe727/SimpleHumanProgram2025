#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"
#include "VoxelData.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace SpecialAnalysis2 {

struct SparseCacheMeta {
    int resolution;
    float world_bounds[3][2];
    float bone_radius;
    float sparse_threshold;

    SparseCacheMeta() : resolution(64), bone_radius(0.08f), sparse_threshold(1e-4f) {
        for (int i = 0; i < 3; ++i) {
            world_bounds[i][0] = -1.0f;
            world_bounds[i][1] = 1.0f;
        }
    }
};

struct BoneData {
    Point3f p1, p2;
    Point3f p1_prev, p2_prev;
    Point3f p1_prev2, p2_prev2;
    Point3f p1_prev3, p2_prev3;
    float speed1, speed2;
    float jerk1, jerk2;
    float inertia1, inertia2;
    int segment_index;
    bool valid;

    BoneData()
        : speed1(0.0f), speed2(0.0f), jerk1(0.0f), jerk2(0.0f),
          inertia1(0.0f), inertia2(0.0f), segment_index(-1), valid(false) {}
};

struct FrameData {
    std::vector<Matrix4f> curr_frames;
    std::vector<Matrix4f> prev_frames;
    std::vector<Matrix4f> prev2_frames;
    std::vector<Matrix4f> prev3_frames;
    Point3f curr_root_pos;
    std::vector<Point3f> curr_joint_pos;
    std::vector<Point3f> prev_joint_pos;
    std::vector<Point3f> prev2_joint_pos;
    std::vector<Point3f> prev3_joint_pos;
    float dt;

    FrameData() : dt(0.0f) {}
};

static int sa_linear_index(int x, int y, int z, int resolution) {
    return z * resolution * resolution + y * resolution + x;
}

static void sa_xyz_from_linear_index(int index, int resolution, int& x, int& y, int& z) {
    int xy = resolution * resolution;
    z = index / xy;
    int rem = index - z * xy;
    y = rem / resolution;
    x = rem - y * resolution;
}

static bool sa_world_to_voxel_index(const Point3f& p, int resolution, const float world_bounds[3][2], int& x, int& y, int& z) {
    float rx = world_bounds[0][1] - world_bounds[0][0];
    float ry = world_bounds[1][1] - world_bounds[1][0];
    float rz = world_bounds[2][1] - world_bounds[2][0];
    if (rx <= 1e-8f || ry <= 1e-8f || rz <= 1e-8f)
        return false;

    x = (int)(((p.x - world_bounds[0][0]) / rx) * resolution);
    y = (int)(((p.y - world_bounds[1][0]) / ry) * resolution);
    z = (int)(((p.z - world_bounds[2][0]) / rz) * resolution);

    return x >= 0 && x < resolution && y >= 0 && y < resolution && z >= 0 && z < resolution;
}

static Color3f sa_get_heatmap_color(float value) {
    Color3f color;
    value = (std::max)(0.0f, (std::min)(1.0f, value));

    float hue = 240.0f * (1.0f - value);
    float h = hue / 60.0f;
    int i = (int)h;
    float f = h - i;
    float q = 1.0f - f;
    float t = f;

    switch (i) {
        case 0:  color.set(1.0f, t, 0.0f); break;
        case 1:  color.set(q, 1.0f, 0.0f); break;
        case 2:  color.set(0.0f, 1.0f, t); break;
        case 3:  color.set(0.0f, q, 1.0f); break;
        case 4:  color.set(t, 0.0f, 1.0f); break;
        default: color.set(0.0f, 0.0f, 1.0f); break;
    }
    return color;
}

static void AccumulateFeatureSparseFrame(
    const MotionFrameSparseVoxelCache& cache,
    int frame_index,
    int segment_index,
    int feature,
    std::unordered_map<int, float>& out_values,
    float& out_max_value) {

    out_values.clear();
    out_max_value = 0.0f;

    if (feature < 0 || feature > 3)
        return;
    if (frame_index < 0 || frame_index >= (int)cache.frames.size())
        return;

    const SegmentFrameSparseVoxelData& frame = cache.frames[frame_index];

    auto accumulate_segment = [&](int s) {
        if (s < 0 || s >= frame.num_segments)
            return;
        const std::vector<SparseVoxel>& sparse = frame.segment_sparse_voxels[s];
        for (size_t i = 0; i < sparse.size(); ++i) {
            const SparseVoxel& sv = sparse[i];
            float v = sv.values[feature];
            float& dst = out_values[sv.index];
            if (feature == 0)
                dst += v;
            else if (v > dst)
                dst = v;
        }
    };

    if (segment_index >= 0) {
        accumulate_segment(segment_index);
    } else {
        for (int s = 0; s < frame.num_segments; ++s)
            accumulate_segment(s);
    }

    for (auto it = out_values.begin(); it != out_values.end(); ++it) {
        if (it->second > out_max_value)
            out_max_value = it->second;
    }
}

static void ComputeFrameData(Motion* m, float time, FrameData& frame_data) {
    if (!m || m->interval <= 0.0f)
        return;

    Posture curr_pose(m->body);
    Posture prev_pose(m->body);
    Posture prev2_pose(m->body);
    Posture prev3_pose(m->body);

    frame_data.dt = m->interval;
    float prev_time = (std::max)(0.0f, time - frame_data.dt);
    float prev2_time = (std::max)(0.0f, time - 2.0f * frame_data.dt);
    float prev3_time = (std::max)(0.0f, time - 3.0f * frame_data.dt);

    m->GetPosture(time, curr_pose);
    m->GetPosture(prev_time, prev_pose);
    m->GetPosture(prev2_time, prev2_pose);
    m->GetPosture(prev3_time, prev3_pose);

    curr_pose.ForwardKinematics(frame_data.curr_frames, frame_data.curr_joint_pos);
    prev_pose.ForwardKinematics(frame_data.prev_frames, frame_data.prev_joint_pos);
    prev2_pose.ForwardKinematics(frame_data.prev2_frames, frame_data.prev2_joint_pos);
    prev3_pose.ForwardKinematics(frame_data.prev3_frames, frame_data.prev3_joint_pos);

    frame_data.curr_root_pos.set(curr_pose.root_pos);
}

static void ExtractBoneData(Motion* m, const FrameData& frame_data, std::vector<BoneData>& bones) {
    bones.clear();
    if (!m || !m->body)
        return;

    bones.reserve(m->body->num_segments);

    for (int s = 0; s < m->body->num_segments; ++s) {
        const Segment* seg = m->body->segments[s];
        BoneData bone;
        bone.segment_index = s;

        if (IsFingerSegment(seg)) {
            bones.push_back(bone);
            continue;
        }

        if (seg->num_joints == 1) {
            bone.p1 = Point3f(frame_data.curr_frames[s].m03, frame_data.curr_frames[s].m13, frame_data.curr_frames[s].m23);
            bone.p1_prev = Point3f(frame_data.prev_frames[s].m03, frame_data.prev_frames[s].m13, frame_data.prev_frames[s].m23);
            bone.p1_prev2 = Point3f(frame_data.prev2_frames[s].m03, frame_data.prev2_frames[s].m13, frame_data.prev2_frames[s].m23);
            bone.p1_prev3 = Point3f(frame_data.prev3_frames[s].m03, frame_data.prev3_frames[s].m13, frame_data.prev3_frames[s].m23);

            if (seg->has_site) {
                Matrix3f r0(frame_data.curr_frames[s].m00, frame_data.curr_frames[s].m01, frame_data.curr_frames[s].m02,
                            frame_data.curr_frames[s].m10, frame_data.curr_frames[s].m11, frame_data.curr_frames[s].m12,
                            frame_data.curr_frames[s].m20, frame_data.curr_frames[s].m21, frame_data.curr_frames[s].m22);
                Matrix3f r1(frame_data.prev_frames[s].m00, frame_data.prev_frames[s].m01, frame_data.prev_frames[s].m02,
                            frame_data.prev_frames[s].m10, frame_data.prev_frames[s].m11, frame_data.prev_frames[s].m12,
                            frame_data.prev_frames[s].m20, frame_data.prev_frames[s].m21, frame_data.prev_frames[s].m22);
                Matrix3f r2(frame_data.prev2_frames[s].m00, frame_data.prev2_frames[s].m01, frame_data.prev2_frames[s].m02,
                            frame_data.prev2_frames[s].m10, frame_data.prev2_frames[s].m11, frame_data.prev2_frames[s].m12,
                            frame_data.prev2_frames[s].m20, frame_data.prev2_frames[s].m21, frame_data.prev2_frames[s].m22);
                Matrix3f r3(frame_data.prev3_frames[s].m00, frame_data.prev3_frames[s].m01, frame_data.prev3_frames[s].m02,
                            frame_data.prev3_frames[s].m10, frame_data.prev3_frames[s].m11, frame_data.prev3_frames[s].m12,
                            frame_data.prev3_frames[s].m20, frame_data.prev3_frames[s].m21, frame_data.prev3_frames[s].m22);

                Point3f off0 = seg->site_position;
                Point3f off1 = seg->site_position;
                Point3f off2 = seg->site_position;
                Point3f off3 = seg->site_position;
                r0.transform(&off0);
                r1.transform(&off1);
                r2.transform(&off2);
                r3.transform(&off3);

                bone.p2 = bone.p1 + off0;
                bone.p2_prev = bone.p1_prev + off1;
                bone.p2_prev2 = bone.p1_prev2 + off2;
                bone.p2_prev3 = bone.p1_prev3 + off3;
                bone.valid = true;
            }
        } else if (seg->num_joints >= 2) {
            Joint* root_joint = seg->joints[0];
            Joint* end_joint = seg->joints[1];

            bone.p1 = frame_data.curr_joint_pos[root_joint->index];
            bone.p2 = frame_data.curr_joint_pos[end_joint->index];
            bone.p1_prev = frame_data.prev_joint_pos[root_joint->index];
            bone.p2_prev = frame_data.prev_joint_pos[end_joint->index];
            bone.p1_prev2 = frame_data.prev2_joint_pos[root_joint->index];
            bone.p2_prev2 = frame_data.prev2_joint_pos[end_joint->index];
            bone.p1_prev3 = frame_data.prev3_joint_pos[root_joint->index];
            bone.p2_prev3 = frame_data.prev3_joint_pos[end_joint->index];
            bone.valid = true;
        }

        if (bone.valid && frame_data.dt > 1e-8f) {
            Vector3f spd1_curr = bone.p1 - bone.p1_prev;
            Vector3f spd2_curr = bone.p2 - bone.p2_prev;
            bone.speed1 = spd1_curr.length() / frame_data.dt;
            bone.speed2 = spd2_curr.length() / frame_data.dt;

            Vector3f spd1_prev = bone.p1_prev - bone.p1_prev2;
            Vector3f spd2_prev = bone.p2_prev - bone.p2_prev2;
            Vector3f spd1_prev2 = bone.p1_prev2 - bone.p1_prev3;
            Vector3f spd2_prev2 = bone.p2_prev2 - bone.p2_prev3;

            Vector3f acc1_curr = spd1_curr - spd1_prev;
            Vector3f acc2_curr = spd2_curr - spd2_prev;
            Vector3f acc1_prev = spd1_prev - spd1_prev2;
            Vector3f acc2_prev = spd2_prev - spd2_prev2;

            Vector3f j1 = acc1_curr - acc1_prev;
            Vector3f j2 = acc2_curr - acc2_prev;
            bone.jerk1 = j1.length() / frame_data.dt;
            bone.jerk2 = j2.length() / frame_data.dt;

            Vector3f d1 = bone.p1 - frame_data.curr_root_pos;
            Vector3f d2 = bone.p2 - frame_data.curr_root_pos;
            bone.inertia1 = d1.x * d1.x + d1.y * d1.y + d1.z * d1.z;
            bone.inertia2 = d2.x * d2.x + d2.y * d2.y + d2.z * d2.z;
        }

        bones.push_back(bone);
    }
}

static void AccumulateBoneToSparse(
    const BoneData& bone,
    int resolution,
    const float world_bounds[3][2],
    float bone_radius,
    std::unordered_map<int, std::array<float, 4>>& sparse_map) {

    if (!bone.valid)
        return;

    float world_range[3] = {
        world_bounds[0][1] - world_bounds[0][0],
        world_bounds[1][1] - world_bounds[1][0],
        world_bounds[2][1] - world_bounds[2][0]
    };

    if (world_range[0] <= 1e-8f || world_range[1] <= 1e-8f || world_range[2] <= 1e-8f)
        return;

    float b_min[3] = {
        (std::min)(bone.p1.x, bone.p2.x) - bone_radius,
        (std::min)(bone.p1.y, bone.p2.y) - bone_radius,
        (std::min)(bone.p1.z, bone.p2.z) - bone_radius
    };
    float b_max[3] = {
        (std::max)(bone.p1.x, bone.p2.x) + bone_radius,
        (std::max)(bone.p1.y, bone.p2.y) + bone_radius,
        (std::max)(bone.p1.z, bone.p2.z) + bone_radius
    };

    int idx_min[3];
    int idx_max[3];
    for (int i = 0; i < 3; ++i) {
        idx_min[i] = (std::max)(0, (int)(((b_min[i] - world_bounds[i][0]) / world_range[i]) * resolution));
        idx_max[i] = (std::min)(resolution - 1, (int)(((b_max[i] - world_bounds[i][0]) / world_range[i]) * resolution));
    }

    Point3f bone_vec = bone.p2 - bone.p1;
    float bone_len_sq = bone_vec.x * bone_vec.x + bone_vec.y * bone_vec.y + bone_vec.z * bone_vec.z;
    if (bone_len_sq < 1e-6f)
        return;

    float sigma_sq = 2.0f * (bone_radius / 2.0f) * (bone_radius / 2.0f);
    float radius_sq = bone_radius * bone_radius;

    for (int z = idx_min[2]; z <= idx_max[2]; ++z) {
        for (int y = idx_min[1]; y <= idx_max[1]; ++y) {
            for (int x = idx_min[0]; x <= idx_max[0]; ++x) {
                Point3f vc(
                    world_bounds[0][0] + (x + 0.5f) * (world_range[0] / resolution),
                    world_bounds[1][0] + (y + 0.5f) * (world_range[1] / resolution),
                    world_bounds[2][0] + (z + 0.5f) * (world_range[2] / resolution));

                Point3f to_p1 = vc - bone.p1;
                float t = (to_p1.x * bone_vec.x + to_p1.y * bone_vec.y + to_p1.z * bone_vec.z) / bone_len_sq;
                float k = (std::max)(0.0f, (std::min)(1.0f, t));
                Point3f cp = bone.p1 + bone_vec * k;

                float dx = vc.x - cp.x;
                float dy = vc.y - cp.y;
                float dz = vc.z - cp.z;
                float dist_sq = dx * dx + dy * dy + dz * dz;

                if (dist_sq < radius_sq) {
                    int linear = sa_linear_index(x, y, z, resolution);
                    std::array<float, 4>& values = sparse_map[linear];

                    float occ = std::exp(-dist_sq / sigma_sq);
                    float spd = (1.0f - k) * bone.speed1 + k * bone.speed2;
                    float jrk = (1.0f - k) * bone.jerk1 + k * bone.jerk2;
                    float ine = (1.0f - k) * bone.inertia1 + k * bone.inertia2;

                    values[0] += occ;
                    if (spd > values[1]) values[1] = spd;
                    if (jrk > values[2]) values[2] = jrk;
                    if (ine > values[3]) values[3] = ine;
                }
            }
        }
    }
}

bool BuildSingleMotionSparseCache(Motion* motion, const SparseCacheMeta& meta, MotionFrameSparseVoxelCache& out_cache) {
    out_cache.Clear();
    if (!motion || !motion->body || motion->num_frames <= 0 || meta.resolution <= 0)
        return false;

    int num_segments = motion->body->num_segments;
    out_cache.Resize(motion->num_frames, num_segments, meta.resolution);

    FrameData frame_data;
    std::vector<BoneData> bones;

    for (int f = 0; f < motion->num_frames; ++f) {
        float time = f * motion->interval;
        ComputeFrameData(motion, time, frame_data);
        ExtractBoneData(motion, frame_data, bones);

        SegmentFrameSparseVoxelData& frame_sparse = out_cache.frames[f];
        frame_sparse.Clear();
        frame_sparse.SetReference(motion->frames[f].root_pos, motion->frames[f].root_ori);

        std::vector<std::unordered_map<int, std::array<float, 4>>> seg_maps(num_segments);
        for (size_t i = 0; i < bones.size(); ++i) {
            const BoneData& bone = bones[i];
            if (!bone.valid || bone.segment_index < 0 || bone.segment_index >= num_segments)
                continue;
            AccumulateBoneToSparse(bone, meta.resolution, meta.world_bounds, meta.bone_radius, seg_maps[bone.segment_index]);
        }

        for (int s = 0; s < num_segments; ++s) {
            std::vector<SparseVoxel>& sparse = frame_sparse.segment_sparse_voxels[s];
            sparse.clear();
            sparse.reserve(seg_maps[s].size());

            for (auto it = seg_maps[s].begin(); it != seg_maps[s].end(); ++it) {
                const int index = it->first;
                const std::array<float, 4>& v = it->second;
                if (v[0] > meta.sparse_threshold || v[1] > meta.sparse_threshold || v[2] > meta.sparse_threshold || v[3] > meta.sparse_threshold)
                    sparse.push_back(SparseVoxel(index, v[0], v[1], v[2], v[3]));
            }
        }
    }

    return true;
}

bool SaveSparseCachePair(
    const char* base_name,
    const SparseCacheMeta& meta,
    const MotionFrameSparseVoxelCache& cache1,
    const MotionFrameSparseVoxelCache& cache2) {

    if (!base_name)
        return false;

    std::string base(base_name);
    if (!cache1.SaveToFile((base + "_m1.svxl").c_str()))
        return false;
    if (!cache2.SaveToFile((base + "_m2.svxl").c_str()))
        return false;

    std::ofstream ofs((base + "_meta.txt").c_str());
    if (!ofs)
        return false;

    ofs << meta.resolution << '\n';
    ofs << meta.bone_radius << '\n';
    ofs << meta.sparse_threshold << '\n';
    for (int i = 0; i < 3; ++i)
        ofs << meta.world_bounds[i][0] << ' ' << meta.world_bounds[i][1] << '\n';

    return ofs.good();
}

bool LoadSparseCachePair(
    const char* base_name,
    SparseCacheMeta& meta,
    MotionFrameSparseVoxelCache& cache1,
    MotionFrameSparseVoxelCache& cache2) {

    if (!base_name)
        return false;

    std::string base(base_name);
    std::ifstream ifs((base + "_meta.txt").c_str());
    if (!ifs)
        return false;

    ifs >> meta.resolution;
    ifs >> meta.bone_radius;
    ifs >> meta.sparse_threshold;
    for (int i = 0; i < 3; ++i)
        ifs >> meta.world_bounds[i][0] >> meta.world_bounds[i][1];

    if (!ifs.good())
        return false;

    if (!cache1.LoadFromFile((base + "_m1.svxl").c_str()))
        return false;
    if (!cache2.LoadFromFile((base + "_m2.svxl").c_str()))
        return false;

    return true;
}

// 表示など既存API互換用: 必要時のみ疎→密変換
bool BuildDenseFeatureViewFromSparseFrame(
    const MotionFrameSparseVoxelCache& cache,
    int frame_index,
    int segment_index,
    int feature,
    VoxelGrid& out_dense) {

    if (feature < 0 || feature > 3)
        return false;
    if (cache.resolution <= 0 || frame_index < 0 || frame_index >= (int)cache.frames.size())
        return false;

    out_dense.Resize(cache.resolution);
    out_dense.Clear();

    const SegmentFrameSparseVoxelData& frame = cache.frames[frame_index];
    if (segment_index >= 0) {
        if (segment_index >= frame.num_segments)
            return false;
        const std::vector<SparseVoxel>& sparse = frame.segment_sparse_voxels[segment_index];
        for (size_t i = 0; i < sparse.size(); ++i) {
            const SparseVoxel& sv = sparse[i];
            int x, y, z;
            sa_xyz_from_linear_index(sv.index, cache.resolution, x, y, z);
            out_dense.At(x, y, z) = sv.values[feature];
        }
    } else {
        for (int s = 0; s < frame.num_segments; ++s) {
            const std::vector<SparseVoxel>& sparse = frame.segment_sparse_voxels[s];
            for (size_t i = 0; i < sparse.size(); ++i) {
                const SparseVoxel& sv = sparse[i];
                int x, y, z;
                sa_xyz_from_linear_index(sv.index, cache.resolution, x, y, z);
                float& cell = out_dense.At(x, y, z);
                if (feature == 0)
                    cell += sv.values[feature];
                else if (sv.values[feature] > cell)
                    cell = sv.values[feature];
            }
        }
    }

    return true;
}

bool DrawSparseVoxels3D(
    const MotionFrameSparseVoxelCache& cache,
    int frame_index,
    int segment_index,
    int feature,
    const float world_bounds[3][2],
    float min_value,
    float alpha_scale) {

    if (cache.resolution <= 0)
        return false;

    std::unordered_map<int, float> values;
    float max_value = 0.0f;
    AccumulateFeatureSparseFrame(cache, frame_index, segment_index, feature, values, max_value);
    if (values.empty())
        return false;
    if (max_value < 1e-8f)
        max_value = 1.0f;

    float range_x = world_bounds[0][1] - world_bounds[0][0];
    float range_y = world_bounds[1][1] - world_bounds[1][0];
    float range_z = world_bounds[2][1] - world_bounds[2][0];

    float cell_size_x = range_x / cache.resolution;
    float cell_size_y = range_y / cache.resolution;
    float cell_size_z = range_z / cache.resolution;
    float cube_size = (std::min)(cell_size_x, (std::min)(cell_size_y, cell_size_z)) * 0.8f;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);

    for (auto it = values.begin(); it != values.end(); ++it) {
        float v = it->second;
        if (v < min_value)
            continue;

        int x, y, z;
        sa_xyz_from_linear_index(it->first, cache.resolution, x, y, z);

        float wx = world_bounds[0][0] + (x + 0.5f) * cell_size_x;
        float wy = world_bounds[1][0] + (y + 0.5f) * cell_size_y;
        float wz = world_bounds[2][0] + (z + 0.5f) * cell_size_z;

        float norm = (std::min)(1.0f, v / max_value);
        Color3f c = sa_get_heatmap_color(norm);
        float a = (std::max)(0.05f, (std::min)(1.0f, alpha_scale * norm));

        glColor4f(c.x, c.y, c.z, a);
        glPushMatrix();
        glTranslatef(wx, wy, wz);
        glutSolidCube(cube_size);
        glPopMatrix();
    }

    glDisable(GL_BLEND);
    return true;
}

bool DrawSparseSliceMapXY(
    const MotionFrameSparseVoxelCache& cache,
    int frame_index,
    int segment_index,
    int feature,
    const float world_bounds[3][2],
    int x_pos,
    int y_pos,
    int w,
    int h,
    float slice_z01,
    const char* title) {

    if (cache.resolution <= 0)
        return false;

    std::unordered_map<int, float> values;
    float max_value = 0.0f;
    AccumulateFeatureSparseFrame(cache, frame_index, segment_index, feature, values, max_value);
    if (values.empty())
        return false;
    if (max_value < 1e-8f)
        max_value = 1.0f;

    int slice_z = (int)((std::max)(0.0f, (std::min)(1.0f, slice_z01)) * (cache.resolution - 1));

    glColor4f(0.9f, 0.9f, 0.9f, 1.0f);
    glBegin(GL_QUADS);
    glVertex2i(x_pos, y_pos);
    glVertex2i(x_pos, y_pos + h);
    glVertex2i(x_pos + w, y_pos + h);
    glVertex2i(x_pos + w, y_pos);
    glEnd();

    glColor4f(0, 0, 0, 1);
    glBegin(GL_LINE_LOOP);
    glVertex2i(x_pos, y_pos);
    glVertex2i(x_pos, y_pos + h);
    glVertex2i(x_pos + w, y_pos + h);
    glVertex2i(x_pos + w, y_pos);
    glEnd();

    float cell_w = (float)w / cache.resolution;
    float cell_h = (float)h / cache.resolution;

    glBegin(GL_QUADS);
    for (auto it = values.begin(); it != values.end(); ++it) {
        int x, y, z;
        sa_xyz_from_linear_index(it->first, cache.resolution, x, y, z);
        if (z != slice_z)
            continue;

        float norm = (std::min)(1.0f, it->second / max_value);
        Color3f c = sa_get_heatmap_color(norm);
        glColor3f(c.x, c.y, c.z);

        float sx = x_pos + x * cell_w;
        float sy = y_pos + (cache.resolution - 1 - y) * cell_h;
        glVertex2f(sx, sy);
        glVertex2f(sx, sy + cell_h);
        glVertex2f(sx + cell_w, sy + cell_h);
        glVertex2f(sx + cell_w, sy);
    }
    glEnd();

    if (title) {
        glColor3f(0, 0, 0);
        glRasterPos2i(x_pos, y_pos - 5);
        for (const char* c = title; *c != '\0'; ++c)
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);

        char info[96];
        std::snprintf(info, sizeof(info), "Z slice: %d/%d", slice_z, cache.resolution - 1);
        glRasterPos2i(x_pos, y_pos + h + 14);
        for (const char* c = info; *c != '\0'; ++c)
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
    }

    return true;
}

} // namespace SpecialAnalysis2

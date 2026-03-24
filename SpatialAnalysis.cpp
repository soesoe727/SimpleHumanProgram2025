#include "SpatialAnalysis.h"
#include "SimpleHumanGLUT.h" // OpenGL用
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <fstream>  // ファイルI/O用

using namespace std;

// --- 内部ヘルパー関数 ---

// 指定された軸の値を取得（0:x, 1:y, 2:z）
static float sa_get_axis_value(const Point3f& p, int axis_index) {
    if (axis_index == 0) return p.x; 
    if (axis_index == 1) return p.y;
    return p.z; 
}

// 値（0～1）をHSV色空間でヒートマップカラー（青→赤）に変換
static Color3f sa_get_heatmap_color(float value) {
    Color3f color;
    value = max(0.0f, min(1.0f, value));

    // value=0 → H=240(青), value=1 → H=0(赤)
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

static Point3f sa_voxel_center_from_linear_index(int linear_index, int resolution, const float world_bounds[3][2]) {
    int xy = resolution * resolution;
    int z = linear_index / xy;
    int rem = linear_index - z * xy;
    int y = rem / resolution;
    int x = rem - y * resolution;

    float range_x = world_bounds[0][1] - world_bounds[0][0];
    float range_y = world_bounds[1][1] - world_bounds[1][0];
    float range_z = world_bounds[2][1] - world_bounds[2][0];

    float wx = world_bounds[0][0] + (x + 0.5f) * (range_x / resolution);
    float wy = world_bounds[1][0] + (y + 0.5f) * (range_y / resolution);
    float wz = world_bounds[2][0] + (z + 0.5f) * (range_z / resolution);
    return Point3f(wx, wy, wz);
}

static int sa_get_frame_index_from_time(const Motion* m, float time) {
    if (!m || m->num_frames <= 0 || m->interval <= 1e-8f)
        return 0;
    int idx = (int)(time / m->interval);
    if (idx < 0)
        idx = 0;
    if (idx >= m->num_frames)
        idx = m->num_frames - 1;
    return idx;
}

static bool sa_compute_principal_axis_from_presence_grid(const VoxelGrid& presence, const float world_bounds[3][2],
    const Point3f& root_pos, Vector3f& axis_out) {
    if (presence.resolution <= 0 || presence.data.empty())
        return false;

    double sum_w = 0.0;
    double cx = 0.0, cy = 0.0, cz = 0.0;
    int size = (int)presence.data.size();

    // 加重重心 Ct を計算
    for (int i = 0; i < size; ++i) {
        float w = presence.data[i];
        if (w <= 1e-6f)
            continue;
        Point3f c = sa_voxel_center_from_linear_index(i, presence.resolution, world_bounds);
        cx += (double)w * c.x;
        cy += (double)w * c.y;
        cz += (double)w * c.z;
        sum_w += w;
    }

    if (sum_w <= 1e-8)
        return false;

    cx /= sum_w;
    cy /= sum_w;
    cz /= sum_w;

    (void)root_pos;

    double c00 = 0.0, c01 = 0.0, c02 = 0.0;
    double c11 = 0.0, c12 = 0.0, c22 = 0.0;
    for (int i = 0; i < size; ++i) {
        float w = presence.data[i];
        if (w <= 1e-6f)
            continue;
        Point3f c = sa_voxel_center_from_linear_index(i, presence.resolution, world_bounds);
        double dx = c.x - cx;
        double dy = c.y - cy;
        double dz = c.z - cz;
        c00 += (double)w * dx * dx;
        c01 += (double)w * dx * dy;
        c02 += (double)w * dx * dz;
        c11 += (double)w * dy * dy;
        c12 += (double)w * dy * dz;
        c22 += (double)w * dz * dz;
    }

    c00 /= sum_w; c01 /= sum_w; c02 /= sum_w;
    c11 /= sum_w; c12 /= sum_w; c22 /= sum_w;

    Vector3f v(1.0f, 0.0f, 0.0f);
    for (int it = 0; it < 16; ++it) {
        Vector3f nv;
        nv.x = (float)(c00 * v.x + c01 * v.y + c02 * v.z);
        nv.y = (float)(c01 * v.x + c11 * v.y + c12 * v.z);
        nv.z = (float)(c02 * v.x + c12 * v.y + c22 * v.z);
        float len = nv.length();
        if (len <= 1e-8f)
            return false;
        nv.x /= len; nv.y /= len; nv.z /= len;
        v = nv;
    }

    if (v.length() <= 1e-8f)
        return false;

    v.normalize();
    axis_out = v;
    return true;
}

static bool sa_compute_principal_axis_from_presence_grid_sparse(const VoxelGrid& presence, const float world_bounds[3][2],
    const std::vector<int>& active_indices, const Point3f& root_pos, Vector3f& axis_out) {
    if (presence.resolution <= 0 || presence.data.empty() || active_indices.empty())
        return false;

    double sum_w = 0.0;
    double cx = 0.0, cy = 0.0, cz = 0.0;

    // 加重重心 Ct を計算
    for (size_t k = 0; k < active_indices.size(); ++k) {
        int i = active_indices[k];
        float w = presence.data[i];
        if (w <= 1e-6f)
            continue;
        Point3f c = sa_voxel_center_from_linear_index(i, presence.resolution, world_bounds);
        cx += (double)w * c.x;
        cy += (double)w * c.y;
        cz += (double)w * c.z;
        sum_w += w;
    }

    if (sum_w <= 1e-8)
        return false;

    cx /= sum_w;
    cy /= sum_w;
    cz /= sum_w;

    (void)root_pos;

    double c00 = 0.0, c01 = 0.0, c02 = 0.0;
    double c11 = 0.0, c12 = 0.0, c22 = 0.0;
    for (size_t k = 0; k < active_indices.size(); ++k) {
        int i = active_indices[k];
        float w = presence.data[i];
        if (w <= 1e-6f)
            continue;
        Point3f c = sa_voxel_center_from_linear_index(i, presence.resolution, world_bounds);
        double dx = c.x - cx;
        double dy = c.y - cy;
        double dz = c.z - cz;
        c00 += (double)w * dx * dx;
        c01 += (double)w * dx * dy;
        c02 += (double)w * dx * dz;
        c11 += (double)w * dy * dy;
        c12 += (double)w * dy * dz;
        c22 += (double)w * dz * dz;
    }

    c00 /= sum_w; c01 /= sum_w; c02 /= sum_w;
    c11 /= sum_w; c12 /= sum_w; c22 /= sum_w;

    Vector3f v(1.0f, 0.0f, 0.0f);
    for (int it = 0; it < 16; ++it) {
        Vector3f nv;
        nv.x = (float)(c00 * v.x + c01 * v.y + c02 * v.z);
        nv.y = (float)(c01 * v.x + c11 * v.y + c12 * v.z);
        nv.z = (float)(c02 * v.x + c12 * v.y + c22 * v.z);
        float len = nv.length();
        if (len <= 1e-8f)
            return false;
        nv.x /= len;
        nv.y /= len;
        nv.z /= len;
        v = nv;
    }

    if (v.length() <= 1e-8f)
        return false;

    v.normalize();
    axis_out = v;
    return true;
}

static float sa_compute_principal_axis_angular_speed(const VoxelGrid& curr_presence, const VoxelGrid& prev_presence,
    float dt, const float world_bounds[3][2], const Point3f& curr_root_pos, const Point3f& prev_root_pos) {
    if (dt <= 1e-8f)
        return 0.0f;

    Vector3f e_curr, e_prev;
    if (!sa_compute_principal_axis_from_presence_grid(curr_presence, world_bounds, curr_root_pos, e_curr))
        return 0.0f;
    if (!sa_compute_principal_axis_from_presence_grid(prev_presence, world_bounds, prev_root_pos, e_prev))
        return 0.0f;

    e_curr.normalize();
    e_prev.normalize();

    float dot = fabsf(e_curr.x * e_prev.x + e_curr.y * e_prev.y + e_curr.z * e_prev.z);
    dot = max(-1.0f, min(1.0f, dot));
    float dtheta = acosf(dot);
    return dtheta / dt;
}

static float sa_compute_principal_axis_angular_speed_sparse(const VoxelGrid& curr_presence, const VoxelGrid& prev_presence,
    float dt, const float world_bounds[3][2], const std::vector<int>& curr_active_indices, const std::vector<int>& prev_active_indices,
    const Point3f& curr_root_pos, const Point3f& prev_root_pos) {
    if (dt <= 1e-8f)
        return 0.0f;

    Vector3f e_curr, e_prev;
    if (!sa_compute_principal_axis_from_presence_grid_sparse(curr_presence, world_bounds, curr_active_indices, curr_root_pos, e_curr))
        return 0.0f;
    if (!sa_compute_principal_axis_from_presence_grid_sparse(prev_presence, world_bounds, prev_active_indices, prev_root_pos, e_prev))
        return 0.0f;

    e_curr.normalize();
    e_prev.normalize();

    float dot = fabsf(e_curr.x * e_prev.x + e_curr.y * e_prev.y + e_curr.z * e_prev.z);
    dot = max(-1.0f, min(1.0f, dot));
    float dtheta = acosf(dot);
    return dtheta / dt;
}

static bool sa_nearly_equal_float(float a, float b, float eps = 1e-5f) {
    return fabsf(a - b) <= eps;
}

static bool sa_nearly_equal_point3(const Point3f& a, const Point3f& b, float eps = 1e-5f) {
    return sa_nearly_equal_float(a.x, b.x, eps) &&
           sa_nearly_equal_float(a.y, b.y, eps) &&
           sa_nearly_equal_float(a.z, b.z, eps);
}

static bool sa_nearly_equal_matrix3(const Matrix3f& a, const Matrix3f& b, float eps = 1e-5f) {
    return sa_nearly_equal_float(a.m00, b.m00, eps) && sa_nearly_equal_float(a.m01, b.m01, eps) && sa_nearly_equal_float(a.m02, b.m02, eps) &&
           sa_nearly_equal_float(a.m10, b.m10, eps) && sa_nearly_equal_float(a.m11, b.m11, eps) && sa_nearly_equal_float(a.m12, b.m12, eps) &&
           sa_nearly_equal_float(a.m20, b.m20, eps) && sa_nearly_equal_float(a.m21, b.m21, eps) && sa_nearly_equal_float(a.m22, b.m22, eps);
}

static bool sa_world_to_voxel_index(const Point3f& p, int resolution, const float world_bounds[3][2], int& x, int& y, int& z) {
    float range_x = world_bounds[0][1] - world_bounds[0][0];
    float range_y = world_bounds[1][1] - world_bounds[1][0];
    float range_z = world_bounds[2][1] - world_bounds[2][0];
    if (range_x <= 1e-8f || range_y <= 1e-8f || range_z <= 1e-8f)
        return false;

    x = (int)(((p.x - world_bounds[0][0]) / range_x) * resolution);
    y = (int)(((p.y - world_bounds[1][0]) / range_y) * resolution);
    z = (int)(((p.z - world_bounds[2][0]) / range_z) * resolution);

    return x >= 0 && x < resolution && y >= 0 && y < resolution && z >= 0 && z < resolution;
}

static Point3f sa_transform_world_by_root_delta(const Point3f& world_pos,
                                                const Point3f& from_root_pos,
                                                const Matrix3f& from_root_ori,
                                                const Point3f& to_root_pos,
                                                const Matrix3f& to_root_ori) {
    float dx = world_pos.x - from_root_pos.x;
    float dy = world_pos.y - from_root_pos.y;
    float dz = world_pos.z - from_root_pos.z;

    // local = from_root_ori^T * (world - from_root_pos)
    float lx = from_root_ori.m00 * dx + from_root_ori.m10 * dy + from_root_ori.m20 * dz;
    float ly = from_root_ori.m01 * dx + from_root_ori.m11 * dy + from_root_ori.m21 * dz;
    float lz = from_root_ori.m02 * dx + from_root_ori.m12 * dy + from_root_ori.m22 * dz;

    // world' = to_root_ori * local + to_root_pos
    float wx = to_root_ori.m00 * lx + to_root_ori.m01 * ly + to_root_ori.m02 * lz + to_root_pos.x;
    float wy = to_root_ori.m10 * lx + to_root_ori.m11 * ly + to_root_ori.m12 * lz + to_root_pos.y;
    float wz = to_root_ori.m20 * lx + to_root_ori.m21 * ly + to_root_ori.m22 * lz + to_root_pos.z;

    return Point3f(wx, wy, wz);
}

// --- SpatialAnalyzer Implementation ---

// コンストラクタ：初期値を設定し、グリッドを初期化
SpatialAnalyzer::SpatialAnalyzer() {
    grid_resolution = 64; 
    ResizeGrids(grid_resolution);
    
    zoom = 1.0f;
    pan_center.set(0.0f, 0.0f);
    is_manual_view = false;
    show_planes = true;
    show_maps = true;
    show_voxels = false;
    feature_mode = 0;
    norm_mode = 0;
    for (int i = 0; i < SA_FEATURE_COUNT; ++i) {
        max_val[i] = 1.0f;
        max_accumulated_val[i] = 1.0f;
        accumulated_pose_cache[i].valid = false;
    }
    
    // 部位別表示モード
    selected_segment_index = -1;
    show_segment_mode = false;
    
    // 部位選択キャッシュ
    segment_cache_dirty = true;
    cached_segment_max_val = 1.0f;
    cached_feature_mode = -1;
    cached_norm_mode = -1;
    cached_selected_segment_index = -2;
    
    // スライス平面の変換行列を単位行列で初期化
    slice_plane_transform.setIdentity();
    
    // 表示用オイラー角
    for(int i = 0; i < 3; i++)
		slice_rotation[i] = 0.0f;
    
    use_rotated_slice = true;
    slice_positions.push_back(0.5f);
    has_world_bounds_initialized = false;
    slice_display_base_range = 1.0f;

    has_frame_cache = false;
    sparse_threshold = 1e-4f;
    temp_voxelize_num_segments = -1;
    temp_voxelize_resolution = -1;
    prev_presence_cache_entries[0] = PrevPresenceCacheEntry();
    prev_presence_cache_entries[1] = PrevPresenceCacheEntry();
}

// デストラクタ
SpatialAnalyzer::~SpatialAnalyzer() {}

// 全ボクセルグリッドを指定解像度でリサイズ
void SpatialAnalyzer::ResizeGrids(int res) {
    grid_resolution = res;

    for (int i = 0; i < SA_FEATURE_COUNT; ++i) {
        voxels1[i].Resize(res); voxels2[i].Resize(res); voxels_diff[i].Resize(res);
        voxels1_accumulated[i].Resize(res); voxels2_accumulated[i].Resize(res); voxels_accumulated_diff[i].Resize(res);
        accumulated_pose_cache[i].valid = false;
    }

    temp_voxelize_num_segments = -1;
    temp_voxelize_resolution = -1;
    prev_presence_cache_entries[0].valid = false;
    prev_presence_cache_entries[1].valid = false;
}

// ワールド座標の境界を設定し、スライス平面を中心に配置
void SpatialAnalyzer::SetWorldBounds(float bounds[3][2]) {
    bool first_initialize = !has_world_bounds_initialized;

    for (int i = 0; i < 3; ++i) {
        world_bounds[i][0] = bounds[i][0];
        world_bounds[i][1] = bounds[i][1];
    }

    float world_range[3];
    for (int i = 0; i < 3; ++i)
        world_range[i] = world_bounds[i][1] - world_bounds[i][0];
    float max_range = max(world_range[0], max(world_range[1], world_range[2]));
    
    // 初回のみスライス平面をワールド中心へ配置（再計算時は位置・向きを維持）
    if (first_initialize) {
        float cx = (world_bounds[0][0] + world_bounds[0][1]) / 2.0f;
        float cy = (world_bounds[1][0] + world_bounds[1][1]) / 2.0f;
        float cz = (world_bounds[2][0] + world_bounds[2][1]) / 2.0f;

        slice_plane_transform.setIdentity();
        slice_plane_transform.m03 = cx;
        slice_plane_transform.m13 = cy;
        slice_plane_transform.m23 = cz;
        UpdateEulerAnglesFromTransform();

        if (max_range > 1e-6f)
            slice_display_base_range = max_range;
    } else if (slice_display_base_range <= 1e-6f && max_range > 1e-6f) {
        slice_display_base_range = max_range;
    }

    has_world_bounds_initialized = true;

    for (int i = 0; i < SA_FEATURE_COUNT; ++i)
        accumulated_pose_cache[i].valid = false;
}

// 指定時刻のモーションを占有率・速度・ジャーク・慣性モーメントのボクセルグリッドに変換
void SpatialAnalyzer::VoxelizeMotion(Motion* m, float time, VoxelGrid& occ, VoxelGrid& spd, VoxelGrid& jrk, VoxelGrid& ine, VoxelGrid& pax) {
    if (!m)
        return;

    int num_segments = m->body->num_segments;
    bool need_resize = (temp_voxelize_num_segments != num_segments || temp_voxelize_resolution != grid_resolution);
    if (need_resize) {
		for (int i = 0; i < SA_FEATURE_COUNT; ++i)
			temp_voxelize_seg[i].Resize(num_segments, grid_resolution);
        temp_voxelize_num_segments = num_segments;
        temp_voxelize_resolution = grid_resolution;
    } else {
        for (int i = 0; i < SA_FEATURE_COUNT; ++i)
			temp_voxelize_seg[i].Clear();
    }

	SegmentVoxelData seg_voxels[SA_FEATURE_COUNT];
	for (int i = 0; i < SA_FEATURE_COUNT; ++i)
        seg_voxels[i] = temp_voxelize_seg[i];
        
    VoxelizeMotionBySegment(m, time, seg_voxels[0], seg_voxels[1], seg_voxels[2], seg_voxels[3], seg_voxels[4]);

    int size = grid_resolution * grid_resolution * grid_resolution;
    if ((int)occ.data.size() != size) occ.Resize(grid_resolution);
    if ((int)spd.data.size() != size) spd.Resize(grid_resolution);
    if ((int)jrk.data.size() != size) jrk.Resize(grid_resolution);
    if ((int)ine.data.size() != size) ine.Resize(grid_resolution);
    if ((int)pax.data.size() != size) pax.Resize(grid_resolution);

    std::fill(occ.data.begin(), occ.data.end(), 0.0f);
    std::fill(spd.data.begin(), spd.data.end(), 0.0f);
    std::fill(jrk.data.begin(), jrk.data.end(), 0.0f);
    std::fill(ine.data.begin(), ine.data.end(), 0.0f);
    std::fill(pax.data.begin(), pax.data.end(), 0.0f);

    for (int s = 0; s < num_segments; ++s) {
        const std::vector<float>& psc_data = seg_voxels[0].segment_grids[s].data;
        const std::vector<float>& spd_data = seg_voxels[1].segment_grids[s].data;
        const std::vector<float>& jrk_data = seg_voxels[2].segment_grids[s].data;
        const std::vector<float>& ine_data = seg_voxels[3].segment_grids[s].data;
        const std::vector<float>& pax_data = seg_voxels[4].segment_grids[s].data;

        for (int i = 0; i < size; ++i) {
            occ.data[i] += psc_data[i];
            if (spd_data[i] > spd.data[i]) spd.data[i] = spd_data[i];
            if (jrk_data[i] > jrk.data[i]) jrk.data[i] = jrk_data[i];
            if (ine_data[i] > ine.data[i]) ine.data[i] = ine_data[i];
            if (pax_data[i] > pax.data[i]) pax.data[i] = pax_data[i];
        }
    }
}

// 現在時刻の両モーションのボクセルを更新し、差分を計算
void SpatialAnalyzer::UpdateVoxels(Motion* m1, Motion* m2, float current_time) {
    int selected_count = GetSelectedSegmentCount();
    bool use_segment_mode = show_segment_mode && (selected_count > 0 || selected_segment_index >= 0);
    int feature = feature_mode;
    if (feature < 0 || feature >= SA_FEATURE_COUNT)
        feature = 0;

    voxels1[feature].Clear();
    voxels2[feature].Clear();
    voxels_diff[feature].Clear();
    max_val[feature] = 0.0f;

    bool compose_segment_data = use_segment_mode && (norm_mode == 0);
    if (!ComposeInstantFeatureFromFrameCache(m1, m2, feature, current_time, compose_segment_data)) {
        VoxelizeMotion(m1, current_time, voxels1[0], voxels1[1], voxels1[2], voxels1[3], voxels1[4]);
        VoxelizeMotion(m2, current_time, voxels2[0], voxels2[1], voxels2[2], voxels2[3], voxels2[4]);
    }

    // 差分計算と最大値更新（現在のfeature_modeのみ）
    int size = grid_resolution * grid_resolution * grid_resolution;

    for (int i = 0; i < size; ++i) {
        float d = fabsf(voxels1[feature].data[i] - voxels2[feature].data[i]);
        voxels_diff[feature].data[i] = d;
        if (d > max_val[feature]) max_val[feature] = d;
    }
    if (max_val[feature] < 1e-5f) max_val[feature] = 1.0f;
}

bool SpatialAnalyzer::ComposeInstantFeatureFromFrameCache(Motion* m1, Motion* m2, int feature, float current_time, bool compose_segment_data) {
    if (!m1 || !m2 || !has_frame_cache)
        return false;
    if (frame_cache1.frames.empty() || frame_cache2.frames.empty())
        return false;

    int num_segments = m1->body ? m1->body->num_segments : 0;
    if (num_segments <= 0)
        return false;

    int f1 = sa_get_frame_index_from_time(m1, current_time);
    int f2 = sa_get_frame_index_from_time(m2, current_time);
    if (f1 >= (int)frame_cache1.frames.size() || f2 >= (int)frame_cache2.frames.size())
        return false;

    SegmentVoxelData* seg1 = nullptr;
    SegmentVoxelData* seg2 = nullptr;
    VoxelGrid* out1 = nullptr;
    VoxelGrid* out2 = nullptr;

    if (feature < 0 || feature >= SA_FEATURE_COUNT)
        return false;

    seg1 = &segment_voxels1[feature];
    seg2 = &segment_voxels2[feature];
    out1 = &voxels1[feature];
    out2 = &voxels2[feature];

    int size = grid_resolution * grid_resolution * grid_resolution;
    if ((int)out1->data.size() != size) out1->Resize(grid_resolution);
    if ((int)out2->data.size() != size) out2->Resize(grid_resolution);
    if (compose_segment_data) {
        seg1->Resize(num_segments, grid_resolution);
        seg2->Resize(num_segments, grid_resolution);
        seg1->Clear();
        seg2->Clear();
    }

    auto compose_one = [&](Motion* m, const MotionFrameSegmentVoxelGridCache& cache, int frame_idx, SegmentVoxelData& out_seg, VoxelGrid& out_acc) {
        const FrameSegmentVoxelGrid& frame_sparse = cache.frames[frame_idx];
        const Point3f& curr_root_pos = m->frames[frame_idx].root_pos;
        const Matrix3f& curr_root_ori = m->frames[frame_idx].root_ori;

        int seg_count = (std::min)(cache.num_segments, out_seg.num_segments);
        for (int s = 0; s < seg_count; ++s) {
            const SegmentVoxelGrid& segment_sparse = frame_sparse.segment_grids[s];
            VoxelGrid* seg_grid_ptr = compose_segment_data ? &out_seg.segment_grids[s] : nullptr;
            const std::vector<SparseVoxel>& sparse_list = segment_sparse.voxels;

            for (size_t k = 0; k < sparse_list.size(); ++k) {
                const SparseVoxel& sv = sparse_list[k];
                float v = sv.values[feature];
                if (v <= sparse_threshold)
                    continue;

                Point3f cached_world = sa_voxel_center_from_linear_index(sv.index, grid_resolution, world_bounds);
                Point3f transformed_world = cached_world;
                if (segment_sparse.has_reference) {
                    transformed_world = sa_transform_world_by_root_delta(
                        cached_world,
                        segment_sparse.reference_root_pos,
                        segment_sparse.reference_root_ori,
                        curr_root_pos,
                        curr_root_ori);
                }

                int x, y, z;
                if (!sa_world_to_voxel_index(transformed_world, grid_resolution, world_bounds, x, y, z))
                    continue;

                if (feature == 0) {
                    if (seg_grid_ptr)
                        seg_grid_ptr->At(x, y, z) += v;
                    out_acc.At(x, y, z) += v;
                } else {
                    if (seg_grid_ptr) {
                        float& seg_v = seg_grid_ptr->At(x, y, z);
                        if (v > seg_v) seg_v = v;
                    }
                    float& acc_v = out_acc.At(x, y, z);
                    if (v > acc_v) acc_v = v;
                }
            }
        }
    };

    compose_one(m1, frame_cache1, f1, *seg1, *out1);
    compose_one(m2, frame_cache2, f2, *seg2, *out2);
    return true;
}

// スライス平面を描画
void SpatialAnalyzer::DrawSlicePlanes() {
    DrawRotatedSlicePlane();
}

// ズームとパンをリセット
void SpatialAnalyzer::ResetView() { 
    zoom = 1.0f;
    pan_center.set(0.0f, 0.0f); 
    is_manual_view = false; 
}

// スライス平面のパン操作
void SpatialAnalyzer::Pan(float dx, float dy) { 
    pan_center.x += dx;
    pan_center.y += dy; 
    is_manual_view = true; 
}

// スライス平面のズーム操作
void SpatialAnalyzer::Zoom(float factor) { 
    zoom *= factor;
    is_manual_view = true; 
}

// 2Dヒートマップ（CT風断面図）を画面に描画
void SpatialAnalyzer::DrawCTMaps(int win_width, int win_height) {
    if (!show_maps)
        return;
    
    // OpenGL 2D設定
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
    gluOrtho2D(0.0, win_width, win_height, 0.0);
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
    glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT);
    glDisable(GL_DEPTH_TEST); glDisable(GL_LIGHTING);

    int margin = 50, gap = 15;
    int num_rows = slice_positions.size();
    int map_w = (win_width - 2*margin - 2*gap) / 3;
    if (map_w > 250) map_w = 250; 
    int map_h = map_w; 
    
    int start_x = margin;
    int start_y = win_height - margin - (num_rows * map_h + (num_rows - 1) * gap);
    int y_pos = start_y;
    
    // 描画するグリッドと最大値を決定
    VoxelGrid* grid1 = nullptr;
    VoxelGrid* grid2 = nullptr;
    VoxelGrid* grid_diff = nullptr;
    float max_value = 1.0f;
    
    int selected_count = GetSelectedSegmentCount();
    bool use_segment_mode = show_segment_mode && (selected_count > 0 || selected_segment_index >= 0);
    
    int feature = feature_mode;
    if (feature < 0 || feature >= SA_FEATURE_COUNT)
        feature = 0;

    if (use_segment_mode) {
        UpdateSegmentCache();
        grid1 = &cached_segment_grid1;
        grid2 = &cached_segment_grid2;
        grid_diff = &cached_segment_diff;
        max_value = cached_segment_max_val;
    } else if (norm_mode == 0) {
        grid1 = &voxels1[feature];
        grid2 = &voxels2[feature];
        grid_diff = &voxels_diff[feature];
        max_value = max_val[feature];
    } else {
        grid1 = &voxels1_accumulated[feature];
        grid2 = &voxels2_accumulated[feature];
        grid_diff = &voxels_accumulated_diff[feature];
        max_value = max_accumulated_val[feature];
    }
    
    DrawRotatedSliceMap(start_x, y_pos, map_w, map_h, *grid1, max_value, "Rotated M1");
    DrawRotatedSliceMap(start_x + map_w + gap, y_pos, map_w, map_h, *grid2, max_value, "Rotated M2");
    DrawRotatedSliceMap(start_x + 2*(map_w + gap), y_pos, map_w, map_h, *grid_diff, max_value, "Rotated Diff");
    
    glPopAttrib();
    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW); glPopMatrix();
}

// 3D空間にボクセルを半透明キューブとして描画
void SpatialAnalyzer::DrawVoxels3D() {
    if (!show_voxels)
        return;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);

    float world_range[3];
    for (int i = 0; i < 3; ++i) 
        world_range[i] = world_bounds[i][1] - world_bounds[i][0];
    
    float cell_size_x = world_range[0] / grid_resolution;
    float cell_size_y = world_range[1] / grid_resolution;
    float cell_size_z = world_range[2] / grid_resolution;
    
    int step = grid_resolution / 32;
    if (step < 1) step = 1;
    
    // 描画するグリッドと最大値を決定
    VoxelGrid* grid_to_draw = nullptr;
    float draw_max_val = 1.0f;
    
    int selected_count = GetSelectedSegmentCount();
    bool use_segment_mode = show_segment_mode && (selected_count > 0 || selected_segment_index >= 0);
    
    int feature = feature_mode;
    if (feature < 0 || feature >= SA_FEATURE_COUNT)
        feature = 0;

    if (use_segment_mode) {
        UpdateSegmentCache();
        grid_to_draw = &cached_segment_diff;
        draw_max_val = cached_segment_max_val;
    } else if (norm_mode == 0) {
        grid_to_draw = &voxels_diff[feature];
        draw_max_val = max_val[feature];
    } else {
        grid_to_draw = &voxels_accumulated_diff[feature];
        draw_max_val = max_accumulated_val[feature];
    }
    
    // ボクセルを描画
    for (int z = 0; z < grid_resolution; z += step) {
        for (int y = 0; y < grid_resolution; y += step) {
            for (int x = 0; x < grid_resolution; x += step) {
                float val = grid_to_draw->Get(x, y, z);
                if (val < 0.01f) 
                    continue;
                
                float wx = world_bounds[0][0] + (x + 0.5f) * cell_size_x;
                float wy = world_bounds[1][0] + (y + 0.5f) * cell_size_y;
                float wz = world_bounds[2][0] + (z + 0.5f) * cell_size_z;
                
                float norm_val = min(val / draw_max_val, 1.0f);
                Color3f color = sa_get_heatmap_color(norm_val);
                
                glColor4f(color.x, color.y, color.z, 0.3f + 0.5f * norm_val);
                
                glPushMatrix();
                glTranslatef(wx, wy, wz);
                glutSolidCube(cell_size_x * 0.8f * step);
                glPopMatrix();
            }
        }
    }
    
    glDisable(GL_BLEND);
}

// モーション全体を通して累積ボクセルを計算（占有率・速度・ジャーク + 部位ごと）
void SpatialAnalyzer::AccumulateAllFrames(Motion* m1, Motion* m2)
{
    if (!m1 || !m2) 
        return;
    
    int num_segments = m1->body->num_segments;
    int size = grid_resolution * grid_resolution * grid_resolution;
    
    ClearAccumulatedData();
    
    for (int f = 0; f < SA_FEATURE_COUNT; ++f) {
        voxels1_accumulated[f].Resize(grid_resolution);
        voxels2_accumulated[f].Resize(grid_resolution);
        voxels_accumulated_diff[f].Resize(grid_resolution);
        segment_voxels1[f].Resize(num_segments, grid_resolution);
        segment_voxels2[f].Resize(num_segments, grid_resolution);
    }
    
    // 基準姿勢を保存
    if (m1->num_frames > 0)
        for (int f = 0; f < SA_FEATURE_COUNT; ++f)
            voxels1_accumulated[f].SetReference(m1->frames[0].root_pos, m1->frames[0].root_ori);
    if (m2->num_frames > 0)
        for (int f = 0; f < SA_FEATURE_COUNT; ++f)
            voxels2_accumulated[f].SetReference(m2->frames[0].root_pos, m2->frames[0].root_ori);
    
    std::cout << "Accumulating voxels (integrated) for " << num_segments << " segments..." << std::endl;
    
    // モーション処理用のラムダ関数
    auto processMotion = [&](Motion* m, SegmentVoxelData* seg_data, VoxelGrid* acc_data) {
        for (int f = 0; f < m->num_frames; ++f) {
            float time = f * m->interval;
            SegmentVoxelData temp_seg[SA_FEATURE_COUNT];
            for (int i = 0; i < SA_FEATURE_COUNT; ++i)
                temp_seg[i].Resize(num_segments, grid_resolution);

            // モーションに基づくボクセル化
            VoxelizeMotionBySegment(m, time, temp_seg[0], temp_seg[1], temp_seg[2], temp_seg[3], temp_seg[4]);

            // セグメントごとのボクセルデータを累積
            for (int s = 0; s < num_segments; ++s) {
                for (int i = 0; i < size; ++i) {
                    for (int feat = 0; feat < SA_FEATURE_COUNT; ++feat) {
                        float v = temp_seg[feat].segment_grids[s].data[i];
                        if (feat == 0) {
                            seg_data[feat].segment_grids[s].data[i] += v;
                            acc_data[feat].data[i] += v;
                        } else {
                            if (v > seg_data[feat].segment_grids[s].data[i])
                                seg_data[feat].segment_grids[s].data[i] = v;
                            if (v > acc_data[feat].data[i])
                                acc_data[feat].data[i] = v;
                        }
                    }
                }
            }
        }
    };

    processMotion(m1, segment_voxels1, voxels1_accumulated);
    processMotion(m2, segment_voxels2, voxels2_accumulated);
    
    // 差分計算と最大値更新
    for (int feat = 0; feat < SA_FEATURE_COUNT; ++feat)
        max_accumulated_val[feat] = 0.0f;

    for (int i = 0; i < size; ++i) {
        for (int feat = 0; feat < SA_FEATURE_COUNT; ++feat) {
            float d = abs(voxels1_accumulated[feat].data[i] - voxels2_accumulated[feat].data[i]);
            voxels_accumulated_diff[feat].data[i] = d;
            float candidate = (feat == 0) ? d : max(voxels1_accumulated[feat].data[i], voxels2_accumulated[feat].data[i]);
            if (candidate > max_accumulated_val[feat])
                max_accumulated_val[feat] = candidate;
        }
    }

    for (int feat = 0; feat < SA_FEATURE_COUNT; ++feat)
        if (max_accumulated_val[feat] < 1e-5f)
            max_accumulated_val[feat] = 1.0f;
    
    if (voxels1_accumulated[0].has_reference)
        voxels_accumulated_diff[0].SetReference(
            voxels1_accumulated[0].reference_root_pos,
            voxels1_accumulated[0].reference_root_ori);
    
    // 部位ごとの最大値を計算
    InitializeSegmentSelection(num_segments);
    for (int s = 0; s < num_segments; ++s) {
        float seg_max_vals[SA_FEATURE_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        for (int i = 0; i < size; ++i) {
            for (int feat = 0; feat < SA_FEATURE_COUNT; ++feat) {
                float d = abs(segment_voxels1[feat].segment_grids[s].data[i] -
                              segment_voxels2[feat].segment_grids[s].data[i]);
                if (d > seg_max_vals[feat])
                    seg_max_vals[feat] = d;
            }
        }
        for (int feat = 0; feat < SA_FEATURE_COUNT; ++feat)
            segment_max[feat][s] = (seg_max_vals[feat] < 1e-5f) ? 1.0f : seg_max_vals[feat];
    }

    std::cout << "Integrated accumulation complete." << std::endl;
    std::cout << "  Max presence diff: " << max_accumulated_val[0] << std::endl;
    std::cout << "  Max speed: " << max_accumulated_val[1] << std::endl;
    std::cout << "  Max jerk: " << max_accumulated_val[2] << std::endl;
    std::cout << "  Max inertia: " << max_accumulated_val[3] << std::endl;
    std::cout << "  Max principal-axis speed: " << max_accumulated_val[4] << std::endl;
}

// 累積ボクセルグリッドをクリアし、最大値をリセット
void SpatialAnalyzer::ClearAccumulatedData()
{
    for (int i = 0; i < SA_FEATURE_COUNT; ++i) {
        voxels1_accumulated[i].Clear();
        voxels2_accumulated[i].Clear();
        voxels_accumulated_diff[i].Clear();
        max_accumulated_val[i] = 0.0f;
    }
}

void SpatialAnalyzer::BuildSingleMotionFeatureFrameCache(Motion* m, MotionFrameSegmentVoxelGridCache& cache) {
    cache.Clear();
    if (!m || !m->body || m->num_frames <= 0)
        return;

    int num_segments = m->body->num_segments;
    int size = grid_resolution * grid_resolution * grid_resolution;
    cache.Resize(m->num_frames, num_segments, grid_resolution);

    SegmentVoxelData curr_seg_psc, prev_seg_psc, curr_seg_spd, curr_seg_jrk, curr_seg_ine, curr_seg_pax;
    curr_seg_psc.Resize(num_segments, grid_resolution);
    prev_seg_psc.Resize(num_segments, grid_resolution);
    curr_seg_spd.Resize(num_segments, grid_resolution);
    curr_seg_jrk.Resize(num_segments, grid_resolution);
    curr_seg_ine.Resize(num_segments, grid_resolution);
    curr_seg_pax.Resize(num_segments, grid_resolution);
    prev_seg_psc.Clear();

    FrameData frame_data;
    vector<BoneData> bones;
    float world_range[3];
    for (int i = 0; i < 3; ++i)
        world_range[i] = world_bounds[i][1] - world_bounds[i][0];
    float bone_radius = 0.08f;
    bool has_prev_presence = false;
    std::vector<std::vector<int>> curr_active_indices(num_segments), prev_active_indices(num_segments);
    std::vector<std::vector<unsigned char>> curr_active_marks(num_segments, std::vector<unsigned char>(size, 0));

    for (int f = 0; f < m->num_frames; ++f) {
        float time = f * m->interval;
        curr_seg_psc.Clear();
        curr_seg_spd.Clear();
        curr_seg_jrk.Clear();
        curr_seg_ine.Clear();
        curr_seg_pax.Clear();
        for (int s = 0; s < num_segments; ++s)
            curr_active_indices[s].clear();

        ComputeFrameData(m, time, frame_data);
        ExtractBoneData(m, frame_data, bones);
        for (const BoneData& bone : bones) {
            if (!bone.valid)
                continue;
            int s = bone.segment_index;
            VoxelGrid& seg_pres_grid = curr_seg_psc.GetSegmentGrid(bone.segment_index);
            VoxelGrid& seg_spd_grid = curr_seg_spd.GetSegmentGrid(bone.segment_index);
            VoxelGrid& seg_jrk_grid = curr_seg_jrk.GetSegmentGrid(bone.segment_index);
            VoxelGrid& seg_ine_grid = curr_seg_ine.GetSegmentGrid(bone.segment_index);
            WriteToVoxelGrid(bone, bone_radius, world_range, &seg_pres_grid, &seg_spd_grid, &seg_jrk_grid, &seg_ine_grid,
                             &curr_active_indices[s], &curr_active_marks[s]);
        }

        if (has_prev_presence) {
            const Point3f& curr_root_pos = m->frames[f].root_pos;
            const Point3f& prev_root_pos = m->frames[f - 1].root_pos;
            for (int s = 0; s < num_segments; ++s) {
                float omega_axis = sa_compute_principal_axis_angular_speed_sparse(
                    curr_seg_psc.segment_grids[s],
                    prev_seg_psc.segment_grids[s],
                    m->interval,
                    world_bounds,
                    curr_active_indices[s],
                    prev_active_indices[s],
                    curr_root_pos,
                    prev_root_pos);

                if (omega_axis <= 0.0f)
                    continue;

                VoxelGrid& curr_presence = curr_seg_psc.segment_grids[s];
                VoxelGrid& seg_pax_grid = curr_seg_pax.segment_grids[s];
                const std::vector<int>& active = curr_active_indices[s];
                for (size_t k = 0; k < active.size(); ++k) {
                    int i = active[k];
                    if (curr_presence.data[i] <= sparse_threshold)
                        continue;
                    if (omega_axis > seg_pax_grid.data[i])
                        seg_pax_grid.data[i] = omega_axis;
                }
            }
        }

        FrameSegmentVoxelGrid& frame_sparse = cache.frames[f];
        frame_sparse.Clear();

        for (int s = 0; s < num_segments; ++s) {
            const VoxelGrid& g0 = curr_seg_psc.segment_grids[s];
            const VoxelGrid& g1 = curr_seg_spd.segment_grids[s];
            const VoxelGrid& g2 = curr_seg_jrk.segment_grids[s];
            const VoxelGrid& g3 = curr_seg_ine.segment_grids[s];
            const VoxelGrid& g4 = curr_seg_pax.segment_grids[s];
            SegmentVoxelGrid& segment_sparse = frame_sparse.segment_grids[s];
            segment_sparse.SetReference(m->frames[f].root_pos, m->frames[f].root_ori);
            std::vector<SparseVoxel>& sparse_list = segment_sparse.voxels;
            sparse_list.clear();
            const std::vector<int>& active = curr_active_indices[s];
            sparse_list.reserve(active.size());
            for (size_t k = 0; k < active.size(); ++k) {
                int i = active[k];
                float v0 = g0.data[i];
                float v1 = g1.data[i];
                float v2 = g2.data[i];
                float v3 = g3.data[i];
                float v4 = g4.data[i];
                if (v0 > sparse_threshold || v1 > sparse_threshold || v2 > sparse_threshold || v3 > sparse_threshold || v4 > sparse_threshold)
                    sparse_list.push_back(SparseVoxel(i, v0, v1, v2, v3, v4));
            }
        }

        for (int s = 0; s < num_segments; ++s) {
            std::vector<int>& active = curr_active_indices[s];
            std::vector<unsigned char>& marks = curr_active_marks[s];
            for (size_t k = 0; k < active.size(); ++k)
                marks[active[k]] = 0;
        }

        std::swap(curr_seg_psc, prev_seg_psc);
        std::swap(curr_active_indices, prev_active_indices);
        has_prev_presence = true;
    }
}

void SpatialAnalyzer::BuildAllFeatureFrameCaches(Motion* m1, Motion* m2) {
    has_frame_cache = false;
    frame_cache1.Clear();
    frame_cache2.Clear();
    for (int f = 0; f < SA_FEATURE_COUNT; ++f)
        accumulated_pose_cache[f].valid = false;

    if (!m1 || !m2)
        return;

    BuildSingleMotionFeatureFrameCache(m1, frame_cache1);
    BuildSingleMotionFeatureFrameCache(m2, frame_cache2);
    has_frame_cache = !frame_cache1.frames.empty() && !frame_cache2.frames.empty();
}

void SpatialAnalyzer::ComposeAccumulatedFeatureFromFrameCache(Motion* m1, Motion* m2, int feature) {
    MotionFrameSegmentVoxelGridCache* c1 = nullptr;
    MotionFrameSegmentVoxelGridCache* c2 = nullptr;
    bool has_cache = false;
    AccumulatedPoseCache* pose_cache = nullptr;
    SegmentVoxelData* seg1 = nullptr;
    SegmentVoxelData* seg2 = nullptr;
    VoxelGrid* acc1 = nullptr;
    VoxelGrid* acc2 = nullptr;
    VoxelGrid* diff = nullptr;
    float* max_val = nullptr;
    std::vector<float>* seg_max = nullptr;

    c1 = &frame_cache1;
    c2 = &frame_cache2;
    has_cache = has_frame_cache;

    if (feature < 0 || feature >= SA_FEATURE_COUNT)
        return;

    pose_cache = &accumulated_pose_cache[feature];
    seg1 = &segment_voxels1[feature];
    seg2 = &segment_voxels2[feature];
    acc1 = &voxels1_accumulated[feature];
    acc2 = &voxels2_accumulated[feature];
    diff = &voxels_accumulated_diff[feature];
    max_val = &max_accumulated_val[feature];
    seg_max = &segment_max[feature];

    if (!m1 || !m2 || !has_cache || m1->num_frames <= 0 || m2->num_frames <= 0)
        return;

    const Point3f& curr_m1_root_pos = m1->frames[0].root_pos;
    const Matrix3f& curr_m1_root_ori = m1->frames[0].root_ori;
    const Point3f& curr_m2_root_pos = m2->frames[0].root_pos;
    const Matrix3f& curr_m2_root_ori = m2->frames[0].root_ori;

    if (pose_cache->valid &&
        sa_nearly_equal_point3(pose_cache->motion1_root_pos, curr_m1_root_pos) &&
        sa_nearly_equal_matrix3(pose_cache->motion1_root_ori, curr_m1_root_ori) &&
        sa_nearly_equal_point3(pose_cache->motion2_root_pos, curr_m2_root_pos) &&
        sa_nearly_equal_matrix3(pose_cache->motion2_root_ori, curr_m2_root_ori)) {
        return;
    }

    int num_segments = m1->body->num_segments;
    int size = grid_resolution * grid_resolution * grid_resolution;

    acc1->Resize(grid_resolution); acc2->Resize(grid_resolution); diff->Resize(grid_resolution);
    acc1->Clear(); acc2->Clear(); diff->Clear();
    seg1->Resize(num_segments, grid_resolution); seg2->Resize(num_segments, grid_resolution);
    seg1->Clear(); seg2->Clear();

    auto compose_single_motion = [&](Motion* m, const MotionFrameSegmentVoxelGridCache& cache, SegmentVoxelData& out_segment, VoxelGrid& out_acc) {
        int frame_count = (std::min)((int)cache.frames.size(), m->num_frames);
        for (int f = 0; f < frame_count; ++f) {
            const FrameSegmentVoxelGrid& frame_sparse = cache.frames[f];
            const Point3f& curr_root_pos = m->frames[f].root_pos;
            const Matrix3f& curr_root_ori = m->frames[f].root_ori;
            for (int s = 0; s < cache.num_segments; ++s) {
                if (s >= out_segment.num_segments) continue;
                const SegmentVoxelGrid& segment_sparse = frame_sparse.segment_grids[s];
                const std::vector<SparseVoxel>& sparse_list = segment_sparse.voxels;
                VoxelGrid& seg_grid = out_segment.segment_grids[s];
                for (size_t k = 0; k < sparse_list.size(); ++k) {
                    const SparseVoxel& sv = sparse_list[k];
                    Point3f cached_world = sa_voxel_center_from_linear_index(sv.index, grid_resolution, world_bounds);
                    Point3f transformed_world = cached_world;
                    if (segment_sparse.has_reference) {
                        transformed_world = sa_transform_world_by_root_delta(
                            cached_world,
                            segment_sparse.reference_root_pos,
                            segment_sparse.reference_root_ori,
                            curr_root_pos,
                            curr_root_ori);
                    }
                    int x, y, z;
                    if (!sa_world_to_voxel_index(transformed_world, grid_resolution, world_bounds, x, y, z))
                        continue;

                    float v = sv.values[feature];
                    if (feature == 0) {
                        seg_grid.At(x, y, z) += v;
                        out_acc.At(x, y, z) += v;
                    } else {
                        float& sv_seg = seg_grid.At(x, y, z);
                        if (v > sv_seg) sv_seg = v;
                        float& sv_acc = out_acc.At(x, y, z);
                        if (v > sv_acc) sv_acc = v;
                    }
                }
            }
        }
    };

    compose_single_motion(m1, *c1, *seg1, *acc1);
    compose_single_motion(m2, *c2, *seg2, *acc2);

    *max_val = 0.0f;
    for (int i = 0; i < size; ++i) {
        float d = abs(acc1->data[i] - acc2->data[i]);
        diff->data[i] = d;
        if (d > *max_val) *max_val = d;
    }
    if (*max_val < 1e-5f) *max_val = 1.0f;

    if (seg_max->size() != (size_t)num_segments)
        InitializeSegmentSelection(num_segments);
    for (int s = 0; s < num_segments; ++s) {
        float smax = 0.0f;
        const std::vector<float>& g1 = seg1->segment_grids[s].data;
        const std::vector<float>& g2 = seg2->segment_grids[s].data;
        for (int i = 0; i < size; ++i) {
            float d = abs(g1[i] - g2[i]);
            if (d > smax) smax = d;
        }
        (*seg_max)[s] = (smax < 1e-5f) ? 1.0f : smax;
    }

    segment_cache_dirty = true;
    pose_cache->motion1_root_pos = curr_m1_root_pos;
    pose_cache->motion1_root_ori = curr_m1_root_ori;
    pose_cache->motion2_root_pos = curr_m2_root_pos;
    pose_cache->motion2_root_ori = curr_m2_root_ori;
    pose_cache->valid = true;
}

void SpatialAnalyzer::VoxelizeMotionSegmentPresenceOnly(Motion* m, float time, SegmentVoxelData& seg_presence_data) {
    if (!m)
        return;

    FrameData frame_data;
    ComputeFrameData(m, time, frame_data);

    vector<BoneData> bones;
    ExtractBoneData(m, frame_data, bones);

    float bone_radius = 0.08f;
    float world_range[3];
    for (int i = 0; i < 3; ++i)
        world_range[i] = world_bounds[i][1] - world_bounds[i][0];

    for (const BoneData& bone : bones) {
        if (!bone.valid)
            continue;
        VoxelGrid& seg_pres_grid = seg_presence_data.GetSegmentGrid(bone.segment_index);
        WriteToVoxelGrid(bone, bone_radius, world_range, &seg_pres_grid, nullptr, nullptr, nullptr);
    }
}

// 指定時刻のモーションを部位ごとにボクセル化
void SpatialAnalyzer::VoxelizeMotionBySegment(Motion* m, float time, SegmentVoxelData& seg_presence_data, SegmentVoxelData& seg_speed_data, SegmentVoxelData& seg_jerk_data, SegmentVoxelData& seg_inertia_data, SegmentVoxelData& seg_principal_axis_data){
    if (!m) 
        return;

    FrameData frame_data;
    ComputeFrameData(m, time, frame_data);

    vector<BoneData> bones;
    ExtractBoneData(m, frame_data, bones);

    float bone_radius = 0.08f;
    float world_range[3];

    for (int i = 0; i < 3; ++i) 
        world_range[i] = world_bounds[i][1] - world_bounds[i][0];

    for (const BoneData& bone : bones) {
        if (!bone.valid) 
            continue;

        VoxelGrid& seg_pres_grid = seg_presence_data.GetSegmentGrid(bone.segment_index);
        VoxelGrid& seg_spd_grid = seg_speed_data.GetSegmentGrid(bone.segment_index);
        VoxelGrid& seg_jrk_grid = seg_jerk_data.GetSegmentGrid(bone.segment_index);
        VoxelGrid& seg_ine_grid = seg_inertia_data.GetSegmentGrid(bone.segment_index);
        WriteToVoxelGrid(bone, bone_radius, world_range, &seg_pres_grid, &seg_spd_grid, &seg_jrk_grid, &seg_ine_grid);
    }

    float prev_time = time - m->interval;
    if (prev_time < 0.0f)
        prev_time = 0.0f;

    const SegmentVoxelData* prev_seg_presence_ptr = nullptr;
    SegmentVoxelData prev_seg_presence;

    PrevPresenceCacheEntry* entry_for_motion = nullptr;
    for (int ci = 0; ci < 2; ++ci) {
        PrevPresenceCacheEntry& e = prev_presence_cache_entries[ci];
        if (e.valid && e.motion == m) {
            entry_for_motion = &e;
            break;
        }
    }

    bool can_reuse_prev = false;
    if (entry_for_motion && entry_for_motion->valid) {
        float dt_err = fabsf((time - entry_for_motion->time) - m->interval);
        can_reuse_prev = dt_err <= 1e-4f &&
                         entry_for_motion->seg_presence.num_segments == seg_presence_data.num_segments &&
                         !entry_for_motion->seg_presence.segment_grids.empty() &&
                         entry_for_motion->seg_presence.segment_grids[0].resolution == grid_resolution;
    }

    if (can_reuse_prev) {
        prev_seg_presence_ptr = &entry_for_motion->seg_presence;
    } else {
        prev_seg_presence.Resize(seg_presence_data.num_segments, grid_resolution);
        prev_seg_presence.Clear();
        VoxelizeMotionSegmentPresenceOnly(m, prev_time, prev_seg_presence);
        prev_seg_presence_ptr = &prev_seg_presence;
    }

    for (int s = 0; s < seg_presence_data.num_segments; ++s) {
        float omega_axis = sa_compute_principal_axis_angular_speed(
            seg_presence_data.segment_grids[s],
            prev_seg_presence_ptr->segment_grids[s],
            m->interval,
            world_bounds,
            frame_data.curr_root_pos,
            frame_data.prev_root_pos);

        if (omega_axis <= 0.0f)
            continue;

        VoxelGrid& curr_presence = seg_presence_data.segment_grids[s];
        VoxelGrid& seg_pax_grid = seg_principal_axis_data.segment_grids[s];
        int voxel_count = (int)curr_presence.data.size();
        for (int i = 0; i < voxel_count; ++i) {
            if (curr_presence.data[i] <= sparse_threshold)
                continue;
            if (omega_axis > seg_pax_grid.data[i])
                seg_pax_grid.data[i] = omega_axis;
        }
    }

    if (!entry_for_motion) {
        for (int ci = 0; ci < 2; ++ci) {
            if (!prev_presence_cache_entries[ci].valid) {
                entry_for_motion = &prev_presence_cache_entries[ci];
                break;
            }
        }
    }
    if (!entry_for_motion)
        entry_for_motion = &prev_presence_cache_entries[0];

    entry_for_motion->seg_presence = seg_presence_data;
    entry_for_motion->motion = m;
    entry_for_motion->time = time;
    entry_for_motion->valid = true;
}

// モーション名からキャッシュファイルのベース名を生成
std::string SpatialAnalyzer::GenerateCacheFilename(const char* motion1_name, const char* motion2_name) const {
    std::string filename = "voxel_cache_";
    filename += motion1_name;
    filename += "_";
    filename += motion2_name;
    filename += ".vxl";
    return filename;
}

// 累積ボクセルデータをファイルに保存してキャッシュ
bool SpatialAnalyzer::SaveVoxelCache(const char* motion1_name, const char* motion2_name) {
    std::string base = GenerateCacheFilename(motion1_name, motion2_name);
    
    std::cout << "Saving voxel cache to " << base << "..." << std::endl;

    const char* feature_file_prefixes[SA_FEATURE_COUNT] = {"acc", "spd_acc", "jrk_acc", "ine_acc", "pax_acc"};
    const char* segment_file_suffixes[SA_FEATURE_COUNT] = {"seg", "seg_spd", "seg_jrk", "seg_ine", "seg_pax"};
    
    // 累積ボクセルデータを保存
    for (int f = 0; f < SA_FEATURE_COUNT; ++f) {
        std::string prefix = feature_file_prefixes[f];
        if (!voxels1_accumulated[f].SaveToFile((base + "_" + prefix + "1.bin").c_str())) return false;
        if (!voxels2_accumulated[f].SaveToFile((base + "_" + prefix + "2.bin").c_str())) return false;
        if (!voxels_accumulated_diff[f].SaveToFile((base + "_" + prefix + "_diff.bin").c_str())) return false;
    }

    // 部位ごとのボクセルデータを保存
    for (int f = 0; f < SA_FEATURE_COUNT; ++f) {
        std::string suffix = segment_file_suffixes[f];
        if (!segment_voxels1[f].SaveToFile((base + "_" + suffix + "1.bin").c_str())) return false;
        if (!segment_voxels2[f].SaveToFile((base + "_" + suffix + "2.bin").c_str())) return false;
    }
    
    // メタデータを保存
    ofstream meta_ofs(base + "_meta.txt");
    if (!meta_ofs) 
        return false;
    
    meta_ofs << grid_resolution << std::endl;
    meta_ofs << max_accumulated_val[0] << std::endl;
    meta_ofs << max_accumulated_val[1] << std::endl;
    meta_ofs << max_accumulated_val[2] << std::endl;
    meta_ofs << max_accumulated_val[3] << std::endl;
    for (int i = 0; i < 3; ++i)
        meta_ofs << world_bounds[i][0] << " " << world_bounds[i][1] << std::endl;
    meta_ofs.close();
    
    std::cout << "Voxel cache saved successfully!" << std::endl;
    return true;
}

// ファイルからボクセルキャッシュを読み込み
bool SpatialAnalyzer::LoadVoxelCache(const char* motion1_name, const char* motion2_name) {
    std::string base = GenerateCacheFilename(motion1_name, motion2_name);
    const char* feature_file_prefixes[SA_FEATURE_COUNT] = {"acc", "spd_acc", "jrk_acc", "ine_acc", "pax_acc"};
    const char* segment_file_suffixes[SA_FEATURE_COUNT] = {"seg", "seg_spd", "seg_jrk", "seg_ine", "seg_pax"};
    
    std::cout << "Loading voxel cache from " << base << "..." << std::endl;
    
    // メタデータを読み込み
    ifstream meta_ifs(base + "_meta.txt");
    if (!meta_ifs) {
        std::cout << "Cache file not found: " << base + "_meta.txt" << std::endl;
        return false;
    }
    
    int res;
    meta_ifs >> res;
    meta_ifs >> max_accumulated_val[0];
    meta_ifs >> max_accumulated_val[1];
    meta_ifs >> max_accumulated_val[2];
    meta_ifs >> max_accumulated_val[3];
    max_accumulated_val[4] = 1.0f;
    for (int i = 0; i < 3; ++i)
        meta_ifs >> world_bounds[i][0] >> world_bounds[i][1];
    meta_ifs.close();
    
    ResizeGrids(res);
    
    // ファイル読み込み用ラムダ
    auto loadFile = [](VoxelGrid& grid, const std::string& file) -> bool {
        if (!grid.LoadFromFile(file.c_str())) {
            std::cout << "Failed to load: " << file << std::endl;
            return false;
        }
        return true;
    };
    
    auto loadSegmentFile = [](SegmentVoxelData& data, const std::string& file) -> bool {
        if (!data.LoadFromFile(file.c_str())) {
            std::cout << "Failed to load: " << file << std::endl;
            return false;
        }
        return true;
    };
    
    // 累積ボクセルデータを読み込み
    for (int f = 0; f < SA_FEATURE_COUNT; ++f) {
        std::string prefix = feature_file_prefixes[f];
        if (!loadFile(voxels1_accumulated[f], base + "_" + prefix + "1.bin")) return false;
        if (!loadFile(voxels2_accumulated[f], base + "_" + prefix + "2.bin")) return false;
        if (!loadFile(voxels_accumulated_diff[f], base + "_" + prefix + "_diff.bin")) return false;
    }

    // 部位ごとのボクセルデータを読み込み
    for (int f = 0; f < SA_FEATURE_COUNT; ++f) {
        std::string suffix = segment_file_suffixes[f];
        if (!loadSegmentFile(segment_voxels1[f], base + "_" + suffix + "1.bin")) return false;
        if (!loadSegmentFile(segment_voxels2[f], base + "_" + suffix + "2.bin")) return false;
    }

    max_accumulated_val[4] = 0.0f;
    for (size_t i = 0; i < voxels_accumulated_diff[4].data.size(); ++i)
        if (voxels_accumulated_diff[4].data[i] > max_accumulated_val[4])
            max_accumulated_val[4] = voxels_accumulated_diff[4].data[i];
    if (max_accumulated_val[4] < 1e-5f)
        max_accumulated_val[4] = 1.0f;
    
    std::cout << "Voxel cache loaded successfully!" << std::endl;
    return true;
}

// スライス平面を角度指定で回転（キーボード入力用）
void SpatialAnalyzer::RotateSlicePlane(float dx, float dy, float dz) {
    float rx = dx * 3.14159265f / 180.0f;
    float ry = dy * 3.14159265f / 180.0f;
    float rz = dz * 3.14159265f / 180.0f;
    
    if (fabsf(dx) > 0.001f) {
        Matrix4f rot_x;
        rot_x.setIdentity();
        float c = cosf(rx), s = sinf(rx);
        rot_x.m11 = c;  rot_x.m12 = -s;
        rot_x.m21 = s;  rot_x.m22 = c;
        ApplySlicePlaneRotation(rot_x);
    }
    if (fabsf(dy) > 0.001f) {
        Matrix4f rot_y;
        rot_y.setIdentity();
        float c = cosf(ry), s = sinf(ry);
        rot_y.m00 = c;  rot_y.m02 = s;
        rot_y.m20 = -s; rot_y.m22 = c;
        ApplySlicePlaneRotation(rot_y);
    }
    if (fabsf(dz) > 0.001f) {
        Matrix4f rot_z;
        rot_z.setIdentity();
        float c = cosf(rz), s = sinf(rz);
        rot_z.m00 = c;  rot_z.m01 = -s;
        rot_z.m10 = s;  rot_z.m11 = c;
        ApplySlicePlaneRotation(rot_z);
    }
}

// スライス平面の回転をリセットし、ワールド中心に配置
void SpatialAnalyzer::ResetSliceRotation() {
    float cx = (world_bounds[0][0] + world_bounds[0][1]) / 2.0f;
    float cy = (world_bounds[1][0] + world_bounds[1][1]) / 2.0f;
    float cz = (world_bounds[2][0] + world_bounds[2][1]) / 2.0f;
    
    slice_plane_transform.setIdentity();
    slice_plane_transform.m03 = cx;
    slice_plane_transform.m13 = cy;
    slice_plane_transform.m23 = cz;
    
    for(int i = 0; i < 3; i++)
		slice_rotation[i] = 0.0f;
    pan_center.set(0.0f, 0.0f);
}

// 回転スライスモードのON/OFFを切り替え
void SpatialAnalyzer::ToggleRotatedSliceMode() {
    use_rotated_slice = !use_rotated_slice;
    if (use_rotated_slice)
        ResetSliceRotation();
    std::cout << "Rotated slice mode: " << (use_rotated_slice ? "ON" : "OFF") << std::endl;
}

// 回転可能なスライス平面を3D空間に描画
void SpatialAnalyzer::DrawRotatedSlicePlane() {
    if (!use_rotated_slice)
        return;
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);  // 両面描画を有効化
    
    float world_range[3];
    for (int i = 0; i < 3; ++i)
        world_range[i] = world_bounds[i][1] - world_bounds[i][0];
    float max_range = slice_display_base_range;
    if (max_range <= 1e-6f)
        max_range = max(world_range[0], max(world_range[1], world_range[2]));
    float half_size = max_range * 0.6f * zoom;

    Point3f center = GetSlicePlaneCenter();
    Vector3f slice_u = GetSlicePlaneU();
    Vector3f slice_v = GetSlicePlaneV();

    // パン操作を適用
    center.x += slice_u.x * pan_center.x + slice_v.x * pan_center.y;
    center.y += slice_u.y * pan_center.x + slice_v.y * pan_center.y;
    center.z += slice_u.z * pan_center.x + slice_v.z * pan_center.y;
    
    for (size_t i = 0; i < slice_positions.size(); ++i) {
        Point3f p[4];
        p[0].set(center.x - slice_u.x * half_size - slice_v.x * half_size,
                 center.y - slice_u.y * half_size - slice_v.y * half_size,
                 center.z - slice_u.z * half_size - slice_v.z * half_size);
        p[1].set(center.x + slice_u.x * half_size - slice_v.x * half_size,
                 center.y + slice_u.y * half_size - slice_v.y * half_size,
                 center.z + slice_u.z * half_size - slice_v.z * half_size);
        p[2].set(center.x + slice_u.x * half_size + slice_v.x * half_size,
                 center.y + slice_u.y * half_size + slice_v.y * half_size,
                 center.z + slice_u.z * half_size + slice_v.z * half_size);
        p[3].set(center.x - slice_u.x * half_size + slice_v.x * half_size,
                 center.y - slice_u.y * half_size + slice_v.y * half_size,
                 center.z - slice_u.z * half_size + slice_v.z * half_size);
        
        // 半透明の面を描画（奥が透けて見えるように深度書き込みを無効化）
        glDepthMask(GL_FALSE);
        glColor4f(1.0f, 1.0f, 0.0f, 0.15f);
        glBegin(GL_QUADS);
        for (int k = 0; k < 4; ++k)
            glVertex3f(p[k].x, p[k].y, p[k].z);
        glEnd();
        glDepthMask(GL_TRUE);

        // 枠線を描画
        glColor4f(1.0f, 1.0f, 0.0f, 0.9f);
        glLineWidth(5.0f);

        glBegin(GL_LINE_LOOP);
        for (int k = 0; k < 4; ++k) 
            glVertex3f(p[k].x, p[k].y, p[k].z);
        glEnd();   
    }
    
    glEnable(GL_CULL_FACE);  // カリングを復元
    glDisable(GL_BLEND);
}

// 回転スライス平面上の2Dヒートマップを描画
void SpatialAnalyzer::DrawRotatedSliceMap(int x_pos, int y_pos, int w, int h, VoxelGrid& grid, float max_val, const char* title) {
    // 背景
    glColor4f(0.9f, 0.9f, 0.9f, 1.0f);
    glLineWidth(2.0f);
    glBegin(GL_QUADS);
    glVertex2i(x_pos, y_pos);
    glVertex2i(x_pos, y_pos + h);
    glVertex2i(x_pos + w, y_pos + h);
    glVertex2i(x_pos + w, y_pos);
    glEnd();
    
    // 枠
    glColor4f(0, 0, 0, 1);
    glBegin(GL_LINE_LOOP);
    glVertex2i(x_pos, y_pos);
    glVertex2i(x_pos, y_pos + h);
    glVertex2i(x_pos + w, y_pos + h);
    glVertex2i(x_pos + w, y_pos);
    glEnd();
    
    float world_range[3];
    for (int i = 0; i < 3; ++i)
        world_range[i] = world_bounds[i][1] - world_bounds[i][0];
    float max_range = slice_display_base_range;
    if (max_range <= 1e-6f)
        max_range = max(world_range[0], max(world_range[1], world_range[2]));
    float half_size = max_range * 0.6f * zoom;

    Point3f center = GetSlicePlaneCenter();
    Vector3f slice_u = GetSlicePlaneU();
    Vector3f slice_v = GetSlicePlaneV();

    center.x += slice_u.x * pan_center.x + slice_v.x * pan_center.y;
    center.y += slice_u.y * pan_center.x + slice_v.y * pan_center.y;
    center.z += slice_u.z * pan_center.x + slice_v.z * pan_center.y;
    
    int draw_res = 64;
    float cell_w = (float)w / draw_res;
    float cell_h = (float)h / draw_res;
    
    // ヒートマップ描画
    glBegin(GL_QUADS);
    for (int iy = 0; iy < draw_res; ++iy) {
        for (int ix = 0; ix < draw_res; ++ix) {
            float u = ((float)ix / draw_res - 0.5f) * 2.0f * half_size;
            float v = ((float)iy / draw_res - 0.5f) * 2.0f * half_size;
            
            Point3f world_pos;
            world_pos.x = center.x + slice_u.x * u + slice_v.x * v;
            world_pos.y = center.y + slice_u.y * u + slice_v.y * v;
            world_pos.z = center.z + slice_u.z * u + slice_v.z * v;
            
            float val = SampleVoxelAtWorldPos(grid, world_pos);
            
            if (val > 0.01f) {
                Color3f c = sa_get_heatmap_color(val / max_val);
                glColor3f(c.x, c.y, c.z);
                
                float sx = x_pos + ix * cell_w;
                float sy = y_pos + (draw_res - 1 - iy) * cell_h;
                
                glVertex2f(sx, sy);
                glVertex2f(sx, sy + cell_h);
                glVertex2f(sx + cell_w, sy + cell_h);
                glVertex2f(sx + cell_w, sy);
            }
        }
    }
    glEnd();
    
    // タイトルと回転情報を描画
    if(strstr(title, "M1") != nullptr)
        glColor3f(1,0,0);
    else if(strstr(title, "M2") != nullptr)
        glColor3f(0,0,1);
    else
        glColor3f(0,0,0);

    glRasterPos2i(x_pos, y_pos - 5);
    for (const char* c = title; *c != '\0'; c++) 
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
    
    char rot_info[128];
    sprintf(rot_info, "Rot: X=%.0f Y=%.0f Z=%.0f", slice_rotation[0], slice_rotation[1], slice_rotation[2]);
    glRasterPos2i(x_pos, y_pos + h + 22);
    for (const char* c = rot_info; *c != '\0'; c++) 
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
}

// ワールド座標からボクセルグリッド値をサンプリング
float SpatialAnalyzer::SampleVoxelAtWorldPos(VoxelGrid& grid, const Point3f& world_pos) {
    float world_range[3];
    for (int i = 0; i < 3; ++i)
        world_range[i] = world_bounds[i][1] - world_bounds[i][0];
    
    int gx = (int)(((world_pos.x - world_bounds[0][0]) / world_range[0]) * grid_resolution);
    int gy = (int)(((world_pos.y - world_bounds[1][0]) / world_range[1]) * grid_resolution);
    int gz = (int)(((world_pos.z - world_bounds[2][0]) / world_range[2]) * grid_resolution);
    
    return grid.Get(gx, gy, gz);
}

// === スライス平面の変換行列ベース操作 ===

// スライス平面の中心座標を取得
Point3f SpatialAnalyzer::GetSlicePlaneCenter() const {
    return Point3f(slice_plane_transform.m03, slice_plane_transform.m13, slice_plane_transform.m23);
}

// スライス平面のU軸ベクトルを取得
Vector3f SpatialAnalyzer::GetSlicePlaneU() const {
    return Vector3f(slice_plane_transform.m00, slice_plane_transform.m10, slice_plane_transform.m20);
}

// スライス平面のV軸ベクトルを取得
Vector3f SpatialAnalyzer::GetSlicePlaneV() const {
    return Vector3f(slice_plane_transform.m01, slice_plane_transform.m11, slice_plane_transform.m21);
}

// スライス平面の法線ベクトルを取得
Vector3f SpatialAnalyzer::GetSlicePlaneNormal() const {
    return Vector3f(slice_plane_transform.m02, slice_plane_transform.m12, slice_plane_transform.m22);
}

// スライス平面にローカル座標系での回転を適用
void SpatialAnalyzer::ApplySlicePlaneRotation(const Matrix4f& local_rotation) {
    // 現在の中心位置を保存
    float cx = slice_plane_transform.m03;
    float cy = slice_plane_transform.m13;
    float cz = slice_plane_transform.m23;
    
    // 平行移動を除去した回転のみの行列
    Matrix4f current_rot = slice_plane_transform;
    current_rot.m03 = 0;
    current_rot.m13 = 0;
    current_rot.m23 = 0;
    
    // ローカル座標系での回転を適用
    Matrix4f result;
    result.mul(current_rot, local_rotation);
    
    // 中心位置を復元
    result.m03 = cx;
    result.m13 = cy;
    result.m23 = cz;
    
    slice_plane_transform = result;
    UpdateEulerAnglesFromTransform();
}

// スライス平面にワールド座標系での回転を適用
void SpatialAnalyzer::ApplySlicePlaneWorldRotation(const Matrix4f& world_rotation) {
    // 現在の中心位置を保存
    float cx = slice_plane_transform.m03;
    float cy = slice_plane_transform.m13;
    float cz = slice_plane_transform.m23;

    // 平行移動を除去した回転のみの行列
    Matrix4f current_rot = slice_plane_transform;
    current_rot.m03 = 0;
    current_rot.m13 = 0;
    current_rot.m23 = 0;

    // ワールド座標系での回転を適用: new = world_rotation * current_rot
    Matrix4f result;
    result.mul(world_rotation, current_rot);

    // 中心位置を復元
    result.m03 = cx;
    result.m13 = cy;
    result.m23 = cz;

    slice_plane_transform = result;
    UpdateEulerAnglesFromTransform();
}

// スライス平面にワールド座標での平行移動を適用
void SpatialAnalyzer::ApplySlicePlaneTranslation(const Point3f& world_translation) {
    slice_plane_transform.m03 += world_translation.x;
    slice_plane_transform.m13 += world_translation.y;
    slice_plane_transform.m23 += world_translation.z;
}

// 変換行列から表示用のオイラー角を計算
void SpatialAnalyzer::UpdateEulerAnglesFromTransform() {
    Vector3f n = GetSlicePlaneNormal();
    Vector3f u = GetSlicePlaneU();
    Vector3f v = GetSlicePlaneV();
    
    float ny = n.y;
    if (ny > 1.0f) ny = 1.0f;
    if (ny < -1.0f) ny = -1.0f;
    float nxz = sqrtf(n.x * n.x + n.z * n.z);
    
    slice_rotation[0] = atan2f(-ny, nxz) * 180.0f / 3.14159265f;
    slice_rotation[1] = atan2f(n.x, n.z) * 180.0f / 3.14159265f;
    slice_rotation[2] = atan2f(u.y, v.y) * 180.0f / 3.14159265f;
}

// === 共通ボクセル化ヘルパー関数 ===

// 指定時刻と前3フレーム分の姿勢データを計算（速度・ジャーク計算用）
void SpatialAnalyzer::ComputeFrameData(Motion* m, float time, FrameData& frame_data) {
    if (!m) 
        return;
    
    Posture curr_pose(m->body);
    Posture prev_pose(m->body);
    Posture prev2_pose(m->body);
    Posture prev3_pose(m->body);
    
    frame_data.dt = m->interval;
    float prev_time = time - frame_data.dt;
    float prev2_time = time - 2.0f * frame_data.dt;
    float prev3_time = time - 3.0f * frame_data.dt;
    if (prev_time < 0) prev_time = 0;
    if (prev2_time < 0) prev2_time = 0;
    if (prev3_time < 0) prev3_time = 0;

    m->GetPosture(time, curr_pose);
    m->GetPosture(prev_time, prev_pose);
    m->GetPosture(prev2_time, prev2_pose);
    m->GetPosture(prev3_time, prev3_pose);

    curr_pose.ForwardKinematics(frame_data.curr_frames, frame_data.curr_joint_pos);
    prev_pose.ForwardKinematics(frame_data.prev_frames, frame_data.prev_joint_pos);
    prev2_pose.ForwardKinematics(frame_data.prev2_frames, frame_data.prev2_joint_pos);
    prev3_pose.ForwardKinematics(frame_data.prev3_frames, frame_data.prev3_joint_pos);

    frame_data.curr_root_pos.set(curr_pose.root_pos);
    frame_data.prev_root_pos.set(prev_pose.root_pos);
}

// フレームデータから全ボーンの位置・速度・ジャークを抽出
void SpatialAnalyzer::ExtractBoneData(Motion* m, const FrameData& frame_data, vector<BoneData>& bones) {
    bones.clear();
    bones.reserve(m->body->num_segments);
    
    for (int s = 0; s < m->body->num_segments; ++s) {
        const Segment* seg = m->body->segments[s];
        BoneData bone;
        bone.segment_index = s;
        bone.valid = false;
        
        // 指をスキップ
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
                // site_positionをワールド座標に変換
                Matrix3f R_curr(frame_data.curr_frames[s].m00, frame_data.curr_frames[s].m01, frame_data.curr_frames[s].m02, 
                                frame_data.curr_frames[s].m10, frame_data.curr_frames[s].m11, frame_data.curr_frames[s].m12, 
                                frame_data.curr_frames[s].m20, frame_data.curr_frames[s].m21, frame_data.curr_frames[s].m22);
                Point3f offset = seg->site_position;
                R_curr.transform(&offset);
                bone.p2 = bone.p1 + offset;

                Matrix3f R_prev(frame_data.prev_frames[s].m00, frame_data.prev_frames[s].m01, frame_data.prev_frames[s].m02, 
                                frame_data.prev_frames[s].m10, frame_data.prev_frames[s].m11, frame_data.prev_frames[s].m12, 
                                frame_data.prev_frames[s].m20, frame_data.prev_frames[s].m21, frame_data.prev_frames[s].m22);
                Point3f offset_prev = seg->site_position;
                R_prev.transform(&offset_prev);
                bone.p2_prev = bone.p1_prev + offset_prev;

                Matrix3f R_prev2(frame_data.prev2_frames[s].m00, frame_data.prev2_frames[s].m01, frame_data.prev2_frames[s].m02, 
                                 frame_data.prev2_frames[s].m10, frame_data.prev2_frames[s].m11, frame_data.prev2_frames[s].m12, 
                                 frame_data.prev2_frames[s].m20, frame_data.prev2_frames[s].m21, frame_data.prev2_frames[s].m22);
                Point3f offset_prev2 = seg->site_position;
                R_prev2.transform(&offset_prev2);
                bone.p2_prev2 = bone.p1_prev2 + offset_prev2;

                Matrix3f R_prev3(frame_data.prev3_frames[s].m00, frame_data.prev3_frames[s].m01, frame_data.prev3_frames[s].m02, 
                                 frame_data.prev3_frames[s].m10, frame_data.prev3_frames[s].m11, frame_data.prev3_frames[s].m12,
                                 frame_data.prev3_frames[s].m20, frame_data.prev3_frames[s].m21, frame_data.prev3_frames[s].m22);
                Point3f offset_prev3 = seg->site_position;
                R_prev3.transform(&offset_prev3);
                bone.p2_prev3 = bone.p1_prev3 + offset_prev3;

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
        
        // 速度とジャークと慣性モーメントを計算
        if (bone.valid) {
            Vector3f spd1_curr = bone.p1 - bone.p1_prev;
            Vector3f spd2_curr = bone.p2 - bone.p2_prev;
            bone.speed1 = spd1_curr.length() / frame_data.dt;
            bone.speed2 = spd2_curr.length() / frame_data.dt;

            Vector3f spd1_prev = bone.p1_prev - bone.p1_prev2;
            Vector3f spd2_prev = bone.p2_prev - bone.p2_prev2;
            Vector3f spd1_prev2 = bone.p1_prev2 - bone.p1_prev3;
            Vector3f spd2_prev2 = bone.p2_prev2 - bone.p2_prev3;

            Vector3f accel1_curr = spd1_curr - spd1_prev;
            Vector3f accel2_curr = spd2_curr - spd2_prev;
            Vector3f accel1_prev = spd1_prev - spd1_prev2;
            Vector3f accel2_prev = spd2_prev - spd2_prev2;

            Vector3f jerk1_curr = accel1_curr - accel1_prev;
            Vector3f jerk2_curr = accel2_curr - accel2_prev;
            bone.jerk1 = jerk1_curr.length() / frame_data.dt;
            bone.jerk2 = jerk2_curr.length() / frame_data.dt;

            // 慣性モーメント: ルートからの距離の2乗
            Vector3f d1 = bone.p1 - frame_data.curr_root_pos;
            Vector3f d2 = bone.p2 - frame_data.curr_root_pos;
            bone.inertia1 = d1.x*d1.x + d1.y*d1.y + d1.z*d1.z;
            bone.inertia2 = d2.x*d2.x + d2.y*d2.y + d2.z*d2.z;
        }
        
        bones.push_back(bone);
    }
}

// ボーンの軸平行境界ボックス（AABB）をボクセル座標で計算
void SpatialAnalyzer::ComputeAABB(const Point3f& p1, const Point3f& p2, float radius,
                                   int idx_min[3], int idx_max[3], const float world_range[3]) {
    float b_min[3], b_max[3];
    for (int i = 0; i < 3; ++i) {
        b_min[i] = min(sa_get_axis_value(p1, i), sa_get_axis_value(p2, i)) - radius;
        b_max[i] = max(sa_get_axis_value(p1, i), sa_get_axis_value(p2, i)) + radius;
        
        idx_min[i] = max(0, (int)(((b_min[i] - world_bounds[i][0]) / world_range[i]) * grid_resolution));
        idx_max[i] = min(grid_resolution - 1, (int)(((b_max[i] - world_bounds[i][0]) / world_range[i]) * grid_resolution));
    }
}

// ボーンの影響をガウス分布で重み付けしてボクセルグリッドに書き込み
void SpatialAnalyzer::WriteToVoxelGrid(const BoneData& bone, float bone_radius, const float world_range[3],
                                        VoxelGrid* occ_grid, VoxelGrid* spd_grid, VoxelGrid* jrk_grid, VoxelGrid* ine_grid,
                                        std::vector<int>* touched_indices, std::vector<unsigned char>* touched_marks) {
    if (!bone.valid) 
        return;
    
    int idx_min[3], idx_max[3];
    ComputeAABB(bone.p1, bone.p2, bone_radius, idx_min, idx_max, world_range);
    
    Point3f bone_vec = bone.p2 - bone.p1;
    float bone_len_sq = bone_vec.x*bone_vec.x + bone_vec.y*bone_vec.y + bone_vec.z*bone_vec.z;
    if (bone_len_sq < 1e-6f) 
        return;
    
    float sigma_sq = 2.0f * (bone_radius/2.0f) * (bone_radius/2.0f);
    float radius_sq = bone_radius * bone_radius;

    for (int z = idx_min[2]; z <= idx_max[2]; ++z) {
        for (int y = idx_min[1]; y <= idx_max[1]; ++y) {
            for (int x = idx_min[0]; x <= idx_max[0]; ++x) {
                float wc[3];
                wc[0] = world_bounds[0][0] + (x + 0.5f) * (world_range[0] / grid_resolution);
                wc[1] = world_bounds[1][0] + (y + 0.5f) * (world_range[1] / grid_resolution);
                wc[2] = world_bounds[2][0] + (z + 0.5f) * (world_range[2] / grid_resolution);

                Point3f voxel_center(wc[0], wc[1], wc[2]);
                Point3f v_to_p1 = voxel_center - bone.p1;
                float t = (v_to_p1.x*bone_vec.x + v_to_p1.y*bone_vec.y + v_to_p1.z*bone_vec.z) / bone_len_sq;
                float k_clamped = max(0.0f, min(1.0f, t));
                Point3f closest = bone.p1 + bone_vec * k_clamped;
                
                float dist_sq = pow(voxel_center.x - closest.x, 2.0) + pow(voxel_center.y - closest.y, 2.0) + pow(voxel_center.z - closest.z, 2.0);

                if (dist_sq < radius_sq) {                
                    if (occ_grid) {
                        float presence = exp(-dist_sq / sigma_sq);
                        occ_grid->At(x, y, z) += presence;
                    }
                    
                    if (spd_grid) {
                        float s_interp = (1.0f - k_clamped) * bone.speed1 + k_clamped * bone.speed2;
                        float& current_spd = spd_grid->At(x, y, z);
                        if (s_interp > current_spd) 
                            current_spd = s_interp;
                    }
                    
                    if (jrk_grid) {
                        float j_interp = (1.0f - k_clamped) * bone.jerk1 + k_clamped * bone.jerk2;
                        float& current_jrk = jrk_grid->At(x, y, z);
                        if (j_interp > current_jrk) 
                            current_jrk = j_interp;
                    }

                    if (ine_grid) {
                        float i_interp = (1.0f - k_clamped) * bone.inertia1 + k_clamped * bone.inertia2;
                        float& current_ine = ine_grid->At(x, y, z);
                        if (i_interp > current_ine) 
                            current_ine = i_interp;
                    }

                    if (touched_indices && touched_marks) {
                        int idx = x + y * grid_resolution + z * grid_resolution * grid_resolution;
                        if (idx >= 0 && idx < (int)touched_marks->size() && !(*touched_marks)[idx]) {
                            (*touched_marks)[idx] = 1;
                            touched_indices->push_back(idx);
                        }
                    }
                }
            }
        }
    }
}

// === 部位選択操作（複数選択対応） ===

// 部位選択状態を初期化し、最大値配列を確保
void SpatialAnalyzer::InitializeSegmentSelection(int num_segments) {
    selected_segments.assign(num_segments, false);
    for (int f = 0; f < SA_FEATURE_COUNT; ++f)
        segment_max[f].assign(num_segments, 1.0f);
}

// 指定部位の選択状態をトグル
void SpatialAnalyzer::ToggleSegmentSelection(int segment_index) {
    if (segment_index >= 0 && segment_index < (int)selected_segments.size()) {
        selected_segments[segment_index] = !selected_segments[segment_index];
        InvalidateSegmentCache();
        std::cout << "Segment " << segment_index << " selection: " 
                  << (selected_segments[segment_index] ? "ON" : "OFF") << std::endl;
    }
}

// 全部位を選択状態にする
void SpatialAnalyzer::SelectAllSegments() {
    for (size_t i = 0; i < selected_segments.size(); ++i)
        selected_segments[i] = true;
    InvalidateSegmentCache();
    std::cout << "All segments selected" << std::endl;
}

// 全部位の選択を解除
void SpatialAnalyzer::ClearSegmentSelection() {
    for (size_t i = 0; i < selected_segments.size(); ++i)
        selected_segments[i] = false;
    InvalidateSegmentCache();
    std::cout << "Segment selection cleared" << std::endl;
}

// 指定部位が選択されているか確認
bool SpatialAnalyzer::IsSegmentSelected(int segment_index) const {
    if (segment_index >= 0 && segment_index < (int)selected_segments.size())
        return selected_segments[segment_index];
    return false;
}

// 現在選択されている部位の数を取得
int SpatialAnalyzer::GetSelectedSegmentCount() const {
    int count = 0;
    for (size_t i = 0; i < selected_segments.size(); ++i)
        if (selected_segments[i]) 
            count++;
    return count;
}

// 指定部位の最大値を現在の特徴量モードに応じて取得
float SpatialAnalyzer::GetSegmentMaxValue(int segment_index) const {
    if (feature_mode < 0 || feature_mode >= SA_FEATURE_COUNT)
        return 1.0f;
    if (segment_index < 0 || segment_index >= (int)segment_max[feature_mode].size())
        return 1.0f;

    return segment_max[feature_mode][segment_index];
}

// 部位選択キャッシュを無効化し、次回更新を強制
void SpatialAnalyzer::InvalidateSegmentCache() {
    segment_cache_dirty = true;
}

// 選択部位のボクセルデータをキャッシュに集約（差分描画用）
void SpatialAnalyzer::UpdateSegmentCache() {
    // キャッシュ有効性チェック
    bool selection_changed = false;
    if (cached_selected_segments.size() != selected_segments.size()) {
        selection_changed = true;
    } else {
        for (size_t i = 0; i < selected_segments.size(); ++i) {
            if (cached_selected_segments[i] != selected_segments[i]) {
                selection_changed = true;
                break;
            }
        }
    }
    
    if (!segment_cache_dirty && !selection_changed &&
        cached_feature_mode == feature_mode &&
        cached_norm_mode == norm_mode &&
        cached_selected_segment_index == selected_segment_index)
        return;
    
    // キャッシュグリッドをリサイズ・クリア
    cached_segment_grid1.Resize(grid_resolution);
    cached_segment_grid2.Resize(grid_resolution);
    cached_segment_diff.Resize(grid_resolution);
    cached_segment_grid1.Clear();
    cached_segment_grid2.Clear();
    cached_segment_diff.Clear();
    
    int feature = feature_mode;
    if (feature < 0 || feature >= SA_FEATURE_COUNT)
        feature = 0;

    // 特徴量に応じたデータソース選択
    SegmentVoxelData* seg_data1 = &segment_voxels1[feature];
    SegmentVoxelData* seg_data2 = &segment_voxels2[feature];
    
    if (seg_data1->num_segments == 0) {
        segment_cache_dirty = false;
        cached_feature_mode = feature_mode;
        cached_norm_mode = norm_mode;
        cached_selected_segment_index = selected_segment_index;
        cached_selected_segments = selected_segments;
        return;
    }
    
    int size = grid_resolution * grid_resolution * grid_resolution;
    cached_segment_max_val = 0.0f;
    
    // 表示対象部位のリストを作成
    std::vector<int> active_segments;
    for (size_t s = 0; s < selected_segments.size(); ++s)
        if (selected_segments[s])
            active_segments.push_back((int)s);
    
    // ナビゲート中の部位も追加
    if (selected_segment_index >= 0 && 
        selected_segment_index < (int)selected_segments.size() &&
        !selected_segments[selected_segment_index])
        active_segments.push_back(selected_segment_index);
    
    if (active_segments.empty()) {
        segment_cache_dirty = false;
        cached_feature_mode = feature_mode;
        cached_norm_mode = norm_mode;
        cached_selected_segment_index = selected_segment_index;
        cached_selected_segments = selected_segments;
        return;
    }
    
    // 選択部位のボクセルを集約
    for (int i = 0; i < size; ++i) {
        float val1 = 0.0f, val2 = 0.0f;
        
        for (int s : active_segments) {
            if (s < seg_data1->num_segments) {
                float v = seg_data1->segment_grids[s].data[i];
                if (v > val1) val1 = v;
            }
            if (s < seg_data2->num_segments) {
                float v = seg_data2->segment_grids[s].data[i];
                if (v > val2) val2 = v;
            }
        }
        
        cached_segment_grid1.data[i] = val1;
        cached_segment_grid2.data[i] = val2;
        float diff = abs(val1 - val2);
        cached_segment_diff.data[i] = diff;
        if (diff > cached_segment_max_val) 
            cached_segment_max_val = diff;
    }
    
    if (cached_segment_max_val < 1e-5f) 
        cached_segment_max_val = 1.0f;
    
    segment_cache_dirty = false;
    cached_feature_mode = feature_mode;
    cached_norm_mode = norm_mode;
    cached_selected_segment_index = selected_segment_index;
    cached_selected_segments = selected_segments;
}
#include "SpatialAnalysis.h"
#include "SimpleHumanGLUT.h" // OpenGL用
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <fstream>  // NEW: ファイルI/O用

using namespace std;

// --- 内部ヘルパー関数 (MotionApp.cppから移植) ---

static float sa_get_axis_value(const Point3f& p, int axis_index) {
	if (axis_index == 0) return p.x; 
	if (axis_index == 1) return p.y;
	return p.z; 
}

static Color3f sa_get_heatmap_color(float value) { 
	Color3f color; value = max(0.0f, min(1.0f, value));
	if (value < 0.5f) color.set(0.0f, value * 2.0f, 1.0f - (value * 2.0f)); 
	else color.set((value - 0.5f) * 2.0f, 1.0f - ((value - 0.5f) * 2.0f), 0.0f); 
	return color; 
}

static const char* sa_get_axis_name(int axis) {
    return (axis == 0) ? "X" : ((axis == 1) ? "Y" : "Z");
}

// --- VoxelGrid Implementation ---

void VoxelGrid::Resize(int res) {
    resolution = res;
    data.assign(res * res * res, 0.0f);
}

void VoxelGrid::Clear() {
    std::fill(data.begin(), data.end(), 0.0f);
}

float& VoxelGrid::At(int x, int y, int z) {
    if(x<0||x>=resolution||y<0||y>=resolution||z<0||z>=resolution) {
        static float dummy = 0.0f; return dummy;
    }
    return data[z * resolution * resolution + y * resolution + x];
}

float VoxelGrid::Get(int x, int y, int z) const {
    if (x < 0 || x >= resolution || y < 0 || y >= resolution || z < 0 || z >= resolution) return 0.0f;
    return data[static_cast<std::vector<float, std::allocator<float>>::size_type>(z) * resolution * resolution + y * resolution + x];
}

// NEW: ファイルへ保存（基準姿勢情報も含む）
bool VoxelGrid::SaveToFile(const char* filename) const {
    ofstream ofs(filename, ios::binary);
    if (!ofs) return false;
    
    // バージョン情報を書き込み
    int version = 2;  // バージョン2：基準姿勢情報を含む
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(int));
    
    // 解像度を書き込み
    ofs.write(reinterpret_cast<const char*>(&resolution), sizeof(int));
    
    // データサイズを書き込み
    int data_size = data.size();
    ofs.write(reinterpret_cast<const char*>(&data_size), sizeof(int));
    
    // データを書き込み
    ofs.write(reinterpret_cast<const char*>(data.data()), data_size * sizeof(float));
    
    // NEW: 基準姿勢情報を書き込み
    ofs.write(reinterpret_cast<const char*>(&has_reference), sizeof(bool));
    if (has_reference) {
        ofs.write(reinterpret_cast<const char*>(&reference_root_pos.x), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&reference_root_pos.y), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&reference_root_pos.z), sizeof(float));
        
        // 回転行列を保存
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                float val = 0.0f;
                if (i == 0 && j == 0) val = reference_root_ori.m00;
                else if (i == 0 && j == 1) val = reference_root_ori.m01;
                else if (i == 0 && j == 2) val = reference_root_ori.m02;
                else if (i == 1 && j == 0) val = reference_root_ori.m10;
                else if (i == 1 && j == 1) val = reference_root_ori.m11;
                else if (i == 1 && j == 2) val = reference_root_ori.m12;
                else if (i == 2 && j == 0) val = reference_root_ori.m20;
                else if (i == 2 && j == 1) val = reference_root_ori.m21;
                else if (i == 2 && j == 2) val = reference_root_ori.m22;
                ofs.write(reinterpret_cast<const char*>(&val), sizeof(float));
            }
        }
    }
    
    return ofs.good();
}

// NEW: ファイルから読み込み（バージョン対応）
bool VoxelGrid::LoadFromFile(const char* filename) {
    ifstream ifs(filename, ios::binary);
    if (!ifs) return false;
    
    // バージョン情報を読み込み
    int version;
    ifs.read(reinterpret_cast<char*>(&version), sizeof(int));
    
    // 解像度を読み込み
    int res;
    ifs.read(reinterpret_cast<char*>(&res), sizeof(int));
    
    // データサイズを読み込み
    int data_size;
    ifs.read(reinterpret_cast<char*>(&data_size), sizeof(int));
    
    // リサイズして読み込み
    Resize(res);
    ifs.read(reinterpret_cast<char*>(data.data()), data_size * sizeof(float));
    
    // バージョン2以降は基準姿勢情報を読み込み
    if (version >= 2) {
        ifs.read(reinterpret_cast<char*>(&has_reference), sizeof(bool));
        if (has_reference) {
            ifs.read(reinterpret_cast<char*>(&reference_root_pos.x), sizeof(float));
            ifs.read(reinterpret_cast<char*>(&reference_root_pos.y), sizeof(float));
            ifs.read(reinterpret_cast<char*>(&reference_root_pos.z), sizeof(float));
            
            // 回転行列を読み込み
            float m[9];
            for (int i = 0; i < 9; ++i) {
                ifs.read(reinterpret_cast<char*>(&m[i]), sizeof(float));
            }
            reference_root_ori.set(m[0], m[1], m[2], 
                                  m[3], m[4], m[5], 
                                  m[6], m[7], m[8]);
        }
    } else {
        // 旧バージョンは基準姿勢なし
        has_reference = false;
    }
    
    return ifs.good();
}

// NEW: SegmentVoxelDataのファイル保存
bool SegmentVoxelData::SaveToFile(const char* filename) const {
    ofstream ofs(filename, ios::binary);
    if (!ofs) return false;
    
    // セグメント数を書き込み
    ofs.write(reinterpret_cast<const char*>(&num_segments), sizeof(int));
    
    // 各セグメントのグリッドを書き込み
    for (int i = 0; i < num_segments; ++i) {
        const VoxelGrid& grid = segment_grids[i];
        
        // 解像度
        ofs.write(reinterpret_cast<const char*>(&grid.resolution), sizeof(int));
        
        // データサイズ
        int data_size = grid.data.size();
        ofs.write(reinterpret_cast<const char*>(&data_size), sizeof(int));
        
        // データ
        ofs.write(reinterpret_cast<const char*>(grid.data.data()), data_size * sizeof(float));
    }
    
    return ofs.good();
}

// NEW: SegmentVoxelDataのファイル読み込み
bool SegmentVoxelData::LoadFromFile(const char* filename) {
    ifstream ifs(filename, ios::binary);
    if (!ifs) return false;
    
    // セグメント数を読み込み
    int num_seg;
    ifs.read(reinterpret_cast<char*>(&num_seg), sizeof(int));
    
    num_segments = num_seg;
    segment_grids.resize(num_seg);
    
    // 各セグメントのグリッドを読み込み
    for (int i = 0; i < num_segments; ++i) {
        VoxelGrid& grid = segment_grids[i];
        
        // 解像度
        int res;
        ifs.read(reinterpret_cast<char*>(&res), sizeof(int));
        
        // データサイズ
        int data_size;
        ifs.read(reinterpret_cast<char*>(&data_size), sizeof(int));
        
        // リサイズして読み込み
        grid.Resize(res);
        ifs.read(reinterpret_cast<char*>(grid.data.data()), data_size * sizeof(float));
    }
    
    return ifs.good();
}

// --- SpatialAnalyzer Implementation ---

SpatialAnalyzer::SpatialAnalyzer() {
    grid_resolution = 64; 
    h_axis = 0; v_axis = 1;  // 初期はXY平面
    ResizeGrids(grid_resolution);
    
    zoom = 1.0f;
    pan_center.set(0.0f, 0.0f);
    is_manual_view = false;
    show_planes = true;
    show_maps = true;
    show_voxels = false;
    feature_mode = 0;
    norm_mode = 0;
    global_max_spd = 1.0f;
    max_accumulated_val = 1.0f;
    
    // NEW: 部位別表示モード
    selected_segment_index = -1;  // -1は全体表示
    show_segment_mode = false;
    
    slice_positions.push_back(1.0f);
    slice_positions.push_back(0.5f);
    active_slice_index = 0;
}

SpatialAnalyzer::~SpatialAnalyzer() {}

void SpatialAnalyzer::ResizeGrids(int res) {
    grid_resolution = res;
    voxels1_occ.Resize(res); voxels2_occ.Resize(res); voxels_diff.Resize(res);
    voxels1_spd.Resize(res); voxels2_spd.Resize(res); voxels_spd_diff.Resize(res);
    
    // NEW: 累積グリッドもリサイズ
    voxels1_accumulated.Resize(res);
    voxels2_accumulated.Resize(res);
    voxels_accumulated_diff.Resize(res);
}

void SpatialAnalyzer::SetWorldBounds(float bounds[3][2]) {
    for(int i=0; i<3; ++i) {
        world_bounds[i][0] = bounds[i][0];
        world_bounds[i][1] = bounds[i][1];
    }
}

// 内部用FK (Motionクラスのメソッドを利用)
static void ComputeFK(Motion* m, float time, vector<Point3f>& joints) {
    if(!m) return;
    Posture posture(m->body);
    m->GetPosture(time, posture); // 指定時間の姿勢を取得
    
    // 全関節のワールド座標を計算
    vector<Matrix4f> frames;
    // SimpleHumanの仕様に合わせてFK計算 (ここでは簡易的に実装するか、m->body->FKがあればそれを使う)
    // 一般的なSimpleHumanの実装を想定:
    posture.ForwardKinematics(frames);
    
    joints.clear();
    for(int i=0; i<m->body->num_joints; ++i) {
        // Jointの位置を取得
        // ※SimpleHumanの構造によるが、ここではsegmentsの変換行列から取得すると仮定
        // SegmentとJointの対応が必要だが、簡易的にsegmentsの座標を使う
        if(i < m->body->num_segments) {
             const Matrix4f& M = frames[i];
             joints.push_back(Point3f(M.m03, M.m13, M.m23));
        } else {
             joints.push_back(Point3f(0,0,0));
        }
    }
}

void SpatialAnalyzer::VoxelizeMotion(Motion* m, float time, VoxelGrid& occ, VoxelGrid& spd) {
    if (!m) return;
    
    // 現在フレームと前フレームの姿勢を取得
    Posture curr_pose(m->body);
    Posture prev_pose(m->body);
    
    float dt = m->interval;
    float prev_time = time - dt;
    if(prev_time < 0) prev_time = 0;

    m->GetPosture(time, curr_pose);
    m->GetPosture(prev_time, prev_pose);

    // 順運動学計算（関節位置も取得）
    vector<Matrix4f> curr_frames, prev_frames;
    vector<Point3f> curr_joint_pos, prev_joint_pos;
    curr_pose.ForwardKinematics(curr_frames, curr_joint_pos);
    prev_pose.ForwardKinematics(prev_frames, prev_joint_pos);

    float bone_radius = 0.08f;
    float world_range[3];
    for(int i=0; i<3; ++i) world_range[i] = world_bounds[i][1] - world_bounds[i][0];

    // 各セグメント（ボーン）について計算
    for (int s = 0; s < m->body->num_segments; ++s) {
        const Segment* seg = m->body->segments[s];
        
        // MODIFIED: 体節名ベースで指をスキップ（BVHファイルによる体節数の違いに対応）
        if (IsFingerSegment(seg)) {
            continue;
        }

        Point3f p1, p2, p1_prev, p2_prev;
        
        // セグメントの接続関節数に応じて位置を取得
        if (seg->num_joints == 1) {
            // 末端セグメント：セグメントの原点から末端位置（site）まで
            p1 = Point3f(curr_frames[s].m03, curr_frames[s].m13, curr_frames[s].m23);
            p1_prev = Point3f(prev_frames[s].m03, prev_frames[s].m13, prev_frames[s].m23);
            
            if (seg->has_site) {
                // site_positionをワールド座標に変換
                Matrix3f R_curr(curr_frames[s].m00, curr_frames[s].m01, curr_frames[s].m02, 
                                curr_frames[s].m10, curr_frames[s].m11, curr_frames[s].m12, 
                                curr_frames[s].m20, curr_frames[s].m21, curr_frames[s].m22);
                Point3f offset = seg->site_position;
                R_curr.transform(&offset);
                p2 = p1 + offset;

                Matrix3f R_prev(prev_frames[s].m00, prev_frames[s].m01, prev_frames[s].m02, 
                                prev_frames[s].m10, prev_frames[s].m11, prev_frames[s].m12, 
                                prev_frames[s].m20, prev_frames[s].m21, prev_frames[s].m22);
                Point3f offset_prev = seg->site_position;
                R_prev.transform(&offset_prev);
                p2_prev = p1_prev + offset_prev;
            } else {
                // siteがない場合はスキップ
                continue;
            }

        } else if (seg->num_joints >= 2) {
            // 通常のボーン：親関節から子関節へ
            // seg->joints[0] がルート側、seg->joints[1] が末端側
            Joint* root_joint = seg->joints[0];
            Joint* end_joint = seg->joints[1];
            
            // 関節位置を取得
            p1 = curr_joint_pos[root_joint->index];
            p2 = curr_joint_pos[end_joint->index];
            
            p1_prev = prev_joint_pos[root_joint->index];
            p2_prev = prev_joint_pos[end_joint->index];
        } else {
            continue;
        }

        // 速度計算
        Vector3f vel1 = p1 - p1_prev;
        Vector3f vel2 = p2 - p2_prev;
        float speed = ((vel1.length() + vel2.length()) / 2.0f) / dt;

        // ボクセルへの書き込み (AABB最適化付き)
        float b_min[3], b_max[3];
        for(int i=0; i<3; ++i) {
            b_min[i] = min(sa_get_axis_value(p1, i), sa_get_axis_value(p2, i)) - bone_radius;
            b_max[i] = max(sa_get_axis_value(p1, i), sa_get_axis_value(p2, i)) + bone_radius;
        }

        int idx_min[3], idx_max[3];
        for(int i=0; i<3; ++i) {
            idx_min[i] = max(0, (int)(((b_min[i] - world_bounds[i][0]) / world_range[i]) * grid_resolution));
            idx_max[i] = min(grid_resolution - 1, (int)(((b_max[i] - world_bounds[i][0]) / world_range[i]) * grid_resolution));
        }

        for (int z = idx_min[2]; z <= idx_max[2]; ++z) {
            for (int y = idx_min[1]; y <= idx_max[1]; ++y) {
                for (int x = idx_min[0]; x <= idx_max[0]; ++x) {
                    float wc[3];
                    wc[0] = world_bounds[0][0] + (x + 0.5f) * (world_range[0] / grid_resolution);
                    wc[1] = world_bounds[1][0] + (y + 0.5f) * (world_range[1] / grid_resolution);
                    wc[2] = world_bounds[2][0] + (z + 0.5f) * (world_range[2] / grid_resolution);

                    Point3f voxel_center(wc[0], wc[1], wc[2]);
                    Point3f bone_vec = p2 - p1;
                    float bone_len_sq = bone_vec.x*bone_vec.x + bone_vec.y*bone_vec.y + bone_vec.z*bone_vec.z;
                    if(bone_len_sq < 1e-6) continue;
                    
                    Point3f v_to_p1 = voxel_center - p1;
                    float t = (v_to_p1.x*bone_vec.x + v_to_p1.y*bone_vec.y + v_to_p1.z*bone_vec.z) / bone_len_sq;
                    t = max(0.0f, min(1.0f, t));
                    Point3f closest = p1 + bone_vec * t;
                    
                    float dist_sq = (voxel_center.x-closest.x)*(voxel_center.x-closest.x) + 
                                    (voxel_center.y-closest.y)*(voxel_center.y-closest.y) + 
                                    (voxel_center.z-closest.z)*(voxel_center.z-closest.z);

                    if (dist_sq < bone_radius * bone_radius) {
                        float presence = exp(-dist_sq / (2.0f * (bone_radius/2.0f)*(bone_radius/2.0f)));
                        occ.At(x, y, z) += presence;
                        float& current_spd = spd.At(x, y, z);
                        if(speed > current_spd) current_spd = speed;
                    }
                }
            }
        }
    }
}

void SpatialAnalyzer::UpdateVoxels(Motion* m1, Motion* m2, float current_time) {
    voxels1_occ.Clear(); voxels2_occ.Clear();
    voxels1_spd.Clear(); voxels2_spd.Clear();

    VoxelizeMotion(m1, current_time, voxels1_occ, voxels1_spd);
    VoxelizeMotion(m2, current_time, voxels2_occ, voxels2_spd);

    max_occ_val = 0.0f;
    int size = grid_resolution * grid_resolution * grid_resolution;
    for(int i=0; i<size; ++i) {
        float v1 = voxels1_occ.data[i];
        float v2 = voxels2_occ.data[i];
        float diff = abs(v1 - v2);
        voxels_diff.data[i] = diff;
        if(diff > max_occ_val) max_occ_val = diff;
        
        float s1 = voxels1_spd.data[i];
        float s2 = voxels2_spd.data[i];
        voxels_spd_diff.data[i] = abs(s1 - s2);
    }
    if (max_occ_val < 1e-5f) max_occ_val = 1.0f;
}


void SpatialAnalyzer::CalculateViewBounds(float& h_min, float& h_max, float& v_min, float& v_max) {
    float raw_h_min = world_bounds[h_axis][0];
    float raw_h_max = world_bounds[h_axis][1];
    float raw_v_min = world_bounds[v_axis][0];
    float raw_v_max = world_bounds[v_axis][1];
    
    float h_range = raw_h_max - raw_h_min;
    float v_range = raw_v_max - raw_v_min;
    float max_range = max(h_range, v_range);
    float h_center = (raw_h_min + raw_h_max) / 2.0f;
    float v_center = (raw_v_min + raw_v_max) / 2.0f;

    if (is_manual_view) {
        h_center += pan_center.x;
        v_center += pan_center.y;
        max_range *= zoom;
    }

    h_min = h_center - max_range / 2.0f;
    h_max = h_center + max_range / 2.0f;
    v_min = v_center - max_range / 2.0f;
    v_max = v_center + max_range / 2.0f;
}

void SpatialAnalyzer::DrawSlicePlanes() {
    int d_axis = 3 - h_axis - v_axis;
    float h_min, h_max, v_min, v_max;
    CalculateViewBounds(h_min, h_max, v_min, v_max);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    for (size_t i = 0; i < slice_positions.size(); ++i) {
        float slice_val = slice_positions[i];
        Point3f p[4];
        
        if (d_axis == 0) { // Xスライス
            p[0].set(slice_val, v_min, h_min); p[1].set(slice_val, v_max, h_min);
            p[2].set(slice_val, v_max, h_max); p[3].set(slice_val, v_min, h_max);
        } else if (d_axis == 1) { // Yスライス
            p[0].set(h_min, slice_val, v_min); p[1].set(h_max, slice_val, v_min);
            p[2].set(h_max, slice_val, v_max); p[3].set(h_min, slice_val, v_max);
        } else { // Zスライス
            p[0].set(h_min, v_min, slice_val); p[1].set(h_max, v_min, slice_val);
            p[2].set(h_max, v_max, slice_val); p[3].set(h_min, v_max, slice_val);
        }

        if ((int)i == active_slice_index) {
            glColor4f(1.0f, 1.0f, 0.8f, 0.25f); glLineWidth(2.0f);
        } else {
            glColor4f(0.7f, 0.7f, 1.0f, 0.15f); glLineWidth(1.0f);
        }

        glBegin(GL_QUADS);
        for(int k=0; k<4; ++k) glVertex3f(p[k].x, p[k].y, p[k].z);
        glEnd();
        
        glBegin(GL_LINE_LOOP);
        if ((int)i == active_slice_index) glColor4f(1.0f, 1.0f, 0.0f, 0.8f);
        else glColor4f(0.5f, 0.5f, 1.0f, 0.5f);
        for(int k=0; k<4; ++k) glVertex3f(p[k].x, p[k].y, p[k].z);
        glEnd();
    }
    glDisable(GL_BLEND);
}

void SpatialAnalyzer::DrawCTMaps(int win_width, int win_height) {
    if (!show_maps) return;
    
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

    float h_min, h_max, v_min, v_max;
    CalculateViewBounds(h_min, h_max, v_min, v_max);

    for (int i = 0; i < num_rows; ++i) {
        int y_pos = start_y + i * (map_h + gap);
        float slice_val = slice_positions[i];
        
        char t1[64], t2[64], t3[64];
        
        // 部位別表示モードの場合はタイトルを変更
        if (show_segment_mode && selected_segment_index >= 0) {
            sprintf(t1, "S%d M1 (Seg:%d)", i+1, selected_segment_index);
            sprintf(t2, "S%d M2 (Seg:%d)", i+1, selected_segment_index);
            sprintf(t3, "S%d Diff (Seg:%d)", i+1, selected_segment_index);
        } else {
            sprintf(t1, "S%d M1", i+1);
            sprintf(t2, "S%d M2", i+1);
            sprintf(t3, "S%d Diff", i+1);
        }

        VoxelGrid* grid1 = nullptr;
        VoxelGrid* grid2 = nullptr;
        VoxelGrid* grid_diff = nullptr;
        float max_value = 1.0f;

        // NEW: ボクセルデータを直接使用するように修正
        if (show_segment_mode && selected_segment_index >= 0 && 
            selected_segment_index < segment_voxels1.num_segments) {
            // 部位別表示モード: 部位ごとの累積ボクセルを使用
            grid1 = &segment_voxels1.GetSegmentGrid(selected_segment_index);
            grid2 = &segment_voxels2.GetSegmentGrid(selected_segment_index);
            
            // 差分を計算
            static VoxelGrid temp_diff;
            temp_diff.Resize(grid_resolution);
            temp_diff.Clear();
            int size = grid_resolution * grid_resolution * grid_resolution;
            max_value = 0.0f;
            for (int j = 0; j < size; ++j) {
                float diff = abs(grid1->data[j] - grid2->data[j]);
                temp_diff.data[j] = diff;
                if (diff > max_value) max_value = diff;
            }
            if (max_value < 1e-5f) max_value = 1.0f;
            grid_diff = &temp_diff;
            
        } else {
            // 全体表示モード: norm_modeで累積/現在フレームを切り替え
            if (norm_mode == 0) {
                // 現在フレームのボクセルデータ
                if (feature_mode == 0) {
                    // 占有率
                    grid1 = &voxels1_occ;
                    grid2 = &voxels2_occ;
                    grid_diff = &voxels_diff;
                    max_value = max_occ_val;
                } else {
                    // 速度
                    grid1 = &voxels1_spd;
                    grid2 = &voxels2_spd;
                    grid_diff = &voxels_spd_diff;
                    max_value = global_max_spd;
                }
            } else {
                // 累積ボクセルデータ
                grid1 = &voxels1_accumulated;
                grid2 = &voxels2_accumulated;
                grid_diff = &voxels_accumulated_diff;
                max_value = max_accumulated_val;
            }
        }

        // ボクセルデータをそのまま2Dマップに投影して描画
        DrawSingleMap(start_x, y_pos, map_w, map_h, *grid1, max_value, t1, slice_val, h_min, h_max, v_min, v_max);
        DrawSingleMap(start_x + map_w + gap, y_pos, map_w, map_h, *grid2, max_value, t2, slice_val, h_min, h_max, v_min, v_max);
        DrawSingleMap(start_x + 2*(map_w + gap), y_pos, map_w, map_h, *grid_diff, max_value, t3, slice_val, h_min, h_max, v_min, v_max);
    }

    glPopAttrib();
    glMatrixMode(GL_PROJECTION); glPopMatrix();
	glMatrixMode(GL_MODELVIEW); glPopMatrix();
}

void SpatialAnalyzer::DrawAxes(int x, int y, int w, int h, float h_min, float h_max, float v_min, float v_max, const char* h_lbl, const char* v_lbl) {
    glColor4f(0.9f, 0.9f, 0.9f, 1.0f);
    glBegin(GL_QUADS); glVertex2i(x, y); glVertex2i(x, y+h); glVertex2i(x+w, y+h); glVertex2i(x+w, y); glEnd();
    glColor4f(0,0,0,1);
    glBegin(GL_LINE_LOOP); glVertex2i(x, y); glVertex2i(x, y+h); glVertex2i(x+w, y+h); glVertex2i(x+w, y); glEnd();
    
    glColor3f(0,0,0);
    char buf[64];
    glRasterPos2i(x, y+h+12);
    sprintf(buf, "%.2f", h_min);
    for(const char* c=buf; *c; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
    glRasterPos2i(x+w-30, y+h+12);
    sprintf(buf, "%.2f", h_max);
    for(const char* c=buf; *c; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
}

void SpatialAnalyzer::DrawSingleMap(int x_pos, int y_pos, int w, int h, VoxelGrid& grid, float max_val, const char* title, float slice_val, float h_min, float h_max, float v_min, float v_max) {
    DrawAxes(x_pos, y_pos, w, h, h_min, h_max, v_min, v_max, sa_get_axis_name(h_axis), sa_get_axis_name(v_axis));

    int d_axis = 3 - h_axis - v_axis;
    float world_range[3];
    for(int i=0; i<3; ++i) world_range[i] = world_bounds[i][1] - world_bounds[i][0];
    
    // スライス位置に対応する深さ軸のインデックスを計算
    int z_idx = (int)(((slice_val - world_bounds[d_axis][0]) / world_range[d_axis]) * grid_resolution);
    if (z_idx < 0 || z_idx >= grid_resolution) return;

    glBegin(GL_QUADS);
    
    // 2D平面での描画解像度（高速化のため粗く設定）
    int draw_res = 64; 
    float cell_w = (float)w / draw_res;
    float cell_h = (float)h / draw_res;
    
    float h_range_view = h_max - h_min;
    float v_range_view = v_max - v_min;
    
    // NEW: Y軸が垂直軸の場合は上下を反転（実際のワールド座標の下が画面の下になるように）
    bool flip_v = (v_axis == 1);

    // 2D平面の各ピクセルに対してボクセル値を取得して描画
    for (int iy = 0; iy < draw_res; ++iy) {
        for (int ix = 0; ix < draw_res; ++ix) {
            // スクリーン座標 -> ワールド座標への変換
            float wx = h_min + (ix + 0.5f) * (h_range_view / draw_res);
            float wy = v_min + (iy + 0.5f) * (v_range_view / draw_res);
            
            // ワールド座標 -> ボクセルグリッドインデックスへの変換
            int gx = (int)(((wx - world_bounds[h_axis][0]) / world_range[h_axis]) * grid_resolution);
            int gy = (int)(((wy - world_bounds[v_axis][0]) / world_range[v_axis]) * grid_resolution);
            
            // 3D軸マッピング (h_axis, v_axis, d_axis -> x, y, z)
            int idx[3];
            idx[h_axis] = gx;
            idx[v_axis] = gy;
            idx[d_axis] = z_idx;
            
            // ボクセルグリッドから値を直接取得（これがメインの密度データ）
            float val = grid.Get(idx[0], idx[1], idx[2]);

            // 閾値以上の値を持つボクセルのみ描画
            if (val > 0.01f) {
                // 値を正規化してヒートマップカラーに変換
                Color3f c = sa_get_heatmap_color(val / max_val);
                glColor3f(c.x, c.y, c.z);
                
                // スクリーン座標での四角形を描画
                float sx = x_pos + ix * cell_w;
                // NEW: Y軸が垂直の場合は上下反転
                float sy = flip_v ? (y_pos + (draw_res - 1 - iy) * cell_h) : (y_pos + iy * cell_h);
                
                glVertex2f(sx, sy);
                glVertex2f(sx, sy+cell_h);
                glVertex2f(sx+cell_w, sy+cell_h);
                glVertex2f(sx+cell_w, sy);
            }
        }
    }
    glEnd();
    
    // タイトルを描画
    glColor3f(0,0,0);
    glRasterPos2i(x_pos, y_pos - 5);
    for (const char* c = title; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
}


// --- 操作 ---
void SpatialAnalyzer::ResetView() { zoom = 1.0f; pan_center.set(0.0f, 0.0f); is_manual_view = false; }
void SpatialAnalyzer::Pan(float dx, float dy) { pan_center.x += dx; pan_center.y += dy; is_manual_view = true; }
void SpatialAnalyzer::Zoom(float factor) { zoom *= factor; is_manual_view = true; }

// NEW: 3Dボクセルの描画
void SpatialAnalyzer::DrawVoxels3D() {
    if (!show_voxels) return;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);

    float world_range[3];
    for(int i=0; i<3; ++i) world_range[i] = world_bounds[i][1] - world_bounds[i][0];
    
    float cell_size_x = world_range[0] / grid_resolution;
    float cell_size_y = world_range[1] / grid_resolution;
    float cell_size_z = world_range[2] / grid_resolution;
    
    // 描画解像度を下げて高速化
    int step = grid_resolution / 16;
    if (step < 1) step = 1;
    
    VoxelGrid* grid_to_draw = nullptr;
    float max_val = 1.0f;
    
    // NEW: 部位別表示モード
    if (show_segment_mode && selected_segment_index >= 0 && 
        selected_segment_index < segment_voxels1.num_segments) {
        // 特定の部位のみ表示（差分を計算）
        static VoxelGrid temp_seg_diff;
        temp_seg_diff.Resize(grid_resolution);
        temp_seg_diff.Clear();
        
        VoxelGrid& seg1 = segment_voxels1.GetSegmentGrid(selected_segment_index);
        VoxelGrid& seg2 = segment_voxels2.GetSegmentGrid(selected_segment_index);
        
        int size = grid_resolution * grid_resolution * grid_resolution;
        max_val = 0.0f;
        for (int i = 0; i < size; ++i) {
            float diff = abs(seg1.data[i] - seg2.data[i]);
            temp_seg_diff.data[i] = diff;
            if (diff > max_val) max_val = diff;
        }
        if (max_val < 1e-5f) max_val = 1.0f;
        
        grid_to_draw = &temp_seg_diff;
        
    } else if (norm_mode == 0) {
        // 現在フレームのみ
        if (feature_mode == 0) {
            grid_to_draw = &voxels_diff;
            max_val = max_occ_val;
        } else {
            grid_to_draw = &voxels_spd_diff;
            max_val = global_max_spd;
        }
    } else {
        // 動作全体の累積データ
        grid_to_draw = &voxels_accumulated_diff;
        max_val = max_accumulated_val;
    }
    
    // ボクセルを描画
    for (int z = 0; z < grid_resolution; z += step) {
        for (int y = 0; y < grid_resolution; y += step) {
            for (int x = 0; x < grid_resolution; x += step) {
                float val = grid_to_draw->Get(x, y, z);
                
                if (val < 0.01f) continue;
                
                float wx = world_bounds[0][0] + (x + 0.5f) * cell_size_x;
                float wy = world_bounds[1][0] + (y + 0.5f) * cell_size_y;
                float wz = world_bounds[2][0] + (z + 0.5f) * cell_size_z;
                
                float norm_val = val / max_val;
                if (norm_val > 1.0f) norm_val = 1.0f;
                Color3f color = sa_get_heatmap_color(norm_val);
                
                float alpha = 0.3f + 0.5f * norm_val;
                glColor4f(color.x, color.y, color.z, alpha);
                
                glPushMatrix();
                glTranslatef(wx, wy, wz);
                float cube_size = cell_size_x * 0.8f * step;
                glutSolidCube(cube_size);
                glPopMatrix();
            }
        }
    }
    
    glDisable(GL_BLEND);
}

// NEW: 累積ボクセルのクリア
void SpatialAnalyzer::ClearAccumulatedVoxels() {
    voxels1_accumulated.Clear();
    voxels2_accumulated.Clear();
    voxels_accumulated_diff.Clear();
    max_accumulated_val = 0.0f;
}

// NEW: 動作全体を通した累積ボクセル計算
void SpatialAnalyzer::AccumulateVoxelsAllFrames(Motion* m1, Motion* m2) {
    if (!m1 || !m2) return;
    
    // 累積グリッドをクリア
    ClearAccumulatedVoxels();
    
    // NEW: 基準姿勢を保存（初期フレームの腰の位置・回転）
    if (m1->num_frames > 0) {
        voxels1_accumulated.SetReference(m1->frames[0].root_pos, m1->frames[0].root_ori);
    }
    if (m2->num_frames > 0) {
        voxels2_accumulated.SetReference(m2->frames[0].root_pos, m2->frames[0].root_ori);
    }
    
    // 両方のモーションの全フレームを処理
    int max_frames = max(m1->num_frames, m2->num_frames);
    
    std::cout << "Accumulating voxels for " << max_frames << " frames..." << std::endl;
    
    // モーション1の全フレームを累積
    for (int f = 0; f < m1->num_frames; ++f) {
        float time = f * m1->interval;
        VoxelGrid temp_occ, temp_spd;
        temp_occ.Resize(grid_resolution);
        temp_spd.Resize(grid_resolution);
        
        VoxelizeMotion(m1, time, temp_occ, temp_spd);
        
        // 累積グリッドに加算
        int size = grid_resolution * grid_resolution * grid_resolution;
        for (int i = 0; i < size; ++i) {
            voxels1_accumulated.data[i] += temp_occ.data[i];
        }
    }
    
    // モーション2の全フレームを累積
    for (int f = 0; f < m2->num_frames; ++f) {
        float time = f * m2->interval;
        VoxelGrid temp_occ, temp_spd;
        temp_occ.Resize(grid_resolution);
        temp_spd.Resize(grid_resolution);
        
        VoxelizeMotion(m2, time, temp_occ, temp_spd);
        
        // 累積グリッドに加算
        int size = grid_resolution * grid_resolution * grid_resolution;
        for (int i = 0; i < size; ++i) {
            voxels2_accumulated.data[i] += temp_occ.data[i];
        }
    }
    
    // 差分を計算し、最大値を更新
    int size = grid_resolution * grid_resolution * grid_resolution;
    max_accumulated_val = 0.0f;
    
    for (int i = 0; i < size; ++i) {
        float diff = abs(voxels1_accumulated.data[i] - voxels2_accumulated.data[i]);
        voxels_accumulated_diff.data[i] = diff;
        if (diff > max_accumulated_val) {
            max_accumulated_val = diff;
        }
    }
    
    if (max_accumulated_val < 1e-5f) max_accumulated_val = 1.0f;
    
    // NEW: 差分グリッドにも基準姿勢を設定（モーション1の基準を使用）
    if (voxels1_accumulated.has_reference) {
        voxels_accumulated_diff.SetReference(voxels1_accumulated.reference_root_pos, 
                                             voxels1_accumulated.reference_root_ori);
    }
    
    std::cout << "Accumulation complete. Max accumulated value: " << max_accumulated_val << std::endl;
}

// NEW: 部位ごとのボクセル化
void SpatialAnalyzer::VoxelizeMotionBySegment(Motion* m, float time, SegmentVoxelData& seg_data) {
    if (!m) return;
    
    // 現在フレームと前フレームの姿勢を取得
    Posture curr_pose(m->body);
    Posture prev_pose(m->body);
    
    float dt = m->interval;
    float prev_time = time - dt;
    if(prev_time < 0) prev_time = 0;

    m->GetPosture(time, curr_pose);
    m->GetPosture(prev_time, prev_pose);

    // 順運動学計算（関節位置も取得）
    vector<Matrix4f> curr_frames, prev_frames;
    vector<Point3f> curr_joint_pos, prev_joint_pos;
    curr_pose.ForwardKinematics(curr_frames, curr_joint_pos);
    prev_pose.ForwardKinematics(prev_frames, prev_joint_pos);

    float bone_radius = 0.08f;
    float world_range[3];
    for(int i=0; i<3; ++i) world_range[i] = world_bounds[i][1] - world_bounds[i][0];

    // 各セグメント（ボーン）について個別に計算
    for (int s = 0; s < m->body->num_segments; ++s) {
        const Segment* seg = m->body->segments[s];
        
        // MODIFIED: 体節名ベースで指をスキップ（BVHファイルによる体節数の違いに対応）
        if (IsFingerSegment(seg)) {
            continue;
        }

        Point3f p1, p2, p1_prev, p2_prev;
        
        // セグメントの接続関節数に応じて位置を取得
        if (seg->num_joints == 1) {
            // 末端セグメント：セグメントの原点から末端位置（site）まで
            p1 = Point3f(curr_frames[s].m03, curr_frames[s].m13, curr_frames[s].m23);
            p1_prev = Point3f(prev_frames[s].m03, prev_frames[s].m13, prev_frames[s].m23);
            
            if (seg->has_site) {
                // site_positionをワールド座標に変換
                Matrix3f R_curr(curr_frames[s].m00, curr_frames[s].m01, curr_frames[s].m02, 
                                curr_frames[s].m10, curr_frames[s].m11, curr_frames[s].m12, 
                                curr_frames[s].m20, curr_frames[s].m21, curr_frames[s].m22);
                Point3f offset = seg->site_position;
                R_curr.transform(&offset);
                p2 = p1 + offset;

                Matrix3f R_prev(prev_frames[s].m00, prev_frames[s].m01, prev_frames[s].m02, 
                                prev_frames[s].m10, prev_frames[s].m11, prev_frames[s].m12, 
                                prev_frames[s].m20, prev_frames[s].m21, prev_frames[s].m22);
                Point3f offset_prev = seg->site_position;
                R_prev.transform(&offset_prev);
                p2_prev = p1_prev + offset_prev;
            } else {
                // siteがない場合はスキップ
                continue;
            }

        } else if (seg->num_joints >= 2) {
            // 通常のボーン：親関節から子関節へ
            // seg->joints[0] がルート側、seg->joints[1] が末端側
            Joint* root_joint = seg->joints[0];
            Joint* end_joint = seg->joints[1];
            
            // 関節位置を取得
            p1 = curr_joint_pos[root_joint->index];
            p2 = curr_joint_pos[end_joint->index];
            
            p1_prev = prev_joint_pos[root_joint->index];
            p2_prev = prev_joint_pos[end_joint->index];
        } else {
            continue;
        }

        // AABB最適化
        float b_min[3], b_max[3];
        for(int i=0; i<3; ++i) {
            b_min[i] = min(sa_get_axis_value(p1, i), sa_get_axis_value(p2, i)) - bone_radius;
            b_max[i] = max(sa_get_axis_value(p1, i), sa_get_axis_value(p2, i)) + bone_radius;
        }

        int idx_min[3], idx_max[3];
        for(int i=0; i<3; ++i) {
            idx_min[i] = max(0, (int)(((b_min[i] - world_bounds[i][0]) / world_range[i]) * grid_resolution));
            idx_max[i] = min(grid_resolution - 1, (int)(((b_max[i] - world_bounds[i][0]) / world_range[i]) * grid_resolution));
        }

        // このセグメント専用のグリッドに書き込み
        VoxelGrid& seg_grid = seg_data.GetSegmentGrid(s);

        for (int z = idx_min[2]; z <= idx_max[2]; ++z) {
            for (int y = idx_min[1]; y <= idx_max[1]; ++y) {
                for (int x = idx_min[0]; x <= idx_max[0]; ++x) {
                    float wc[3];
                    wc[0] = world_bounds[0][0] + (x + 0.5f) * (world_range[0] / grid_resolution);
                    wc[1] = world_bounds[1][0] + (y + 0.5f) * (world_range[1] / grid_resolution);
                    wc[2] = world_bounds[2][0] + (z + 0.5f) * (world_range[2] / grid_resolution);

                    Point3f voxel_center(wc[0], wc[1], wc[2]);
                    Point3f bone_vec = p2 - p1;
                    float bone_len_sq = bone_vec.x*bone_vec.x + bone_vec.y*bone_vec.y + bone_vec.z*bone_vec.z;
                    if(bone_len_sq < 1e-6) continue;
                    
                    Point3f v_to_p1 = voxel_center - p1;
                    float t = (v_to_p1.x*bone_vec.x + v_to_p1.y*bone_vec.y + v_to_p1.z*bone_vec.z) / bone_len_sq;
                    t = max(0.0f, min(1.0f, t));
                    Point3f closest = p1 + bone_vec * t;
                    
                    float dist_sq = (voxel_center.x-closest.x)*(voxel_center.x-closest.x) + 
                                    (voxel_center.y-closest.y)*(voxel_center.y-closest.y) + 
                                    (voxel_center.z-closest.z)*(voxel_center.z-closest.z);

                    if (dist_sq < bone_radius * bone_radius) {
                        float presence = exp(-dist_sq / (2.0f * (bone_radius/2.0f)*(bone_radius/2.0f)));
                        seg_grid.At(x, y, z) += presence;
                    }
                }
            }
        }
    }
}

// NEW: 部位ごとの累積ボクセル計算
void SpatialAnalyzer::AccumulateVoxelsBySegmentAllFrames(Motion* m1, Motion* m2) {
    if (!m1 || !m2) return;
    
    // 部位数を取得
    int num_segments = m1->body->num_segments;
    
    // 部位ごとのグリッドをリサイズ
    segment_voxels1.Resize(num_segments, grid_resolution);
    segment_voxels2.Resize(num_segments, grid_resolution);
    
    std::cout << "Accumulating voxels by segment for " << num_segments << " segments..." << std::endl;
    
    // モーション1の全フレームを部位ごとに累積
    for (int f = 0; f < m1->num_frames; ++f) {
        float time = f * m1->interval;
        SegmentVoxelData temp_seg_data;
        temp_seg_data.Resize(num_segments, grid_resolution);
        
        VoxelizeMotionBySegment(m1, time, temp_seg_data);
        
        // 累積
        for (int s = 0; s < num_segments; ++s) {
            int size = grid_resolution * grid_resolution * grid_resolution;
            for (int i = 0; i < size; ++i) {
                segment_voxels1.segment_grids[s].data[i] += temp_seg_data.segment_grids[s].data[i];
            }
        }
    }
    
    // モーション2の全フレームを部位ごとに累積
    for (int f = 0; f < m2->num_frames; ++f) {
        float time = f * m2->interval;
        SegmentVoxelData temp_seg_data;
        temp_seg_data.Resize(num_segments, grid_resolution);
        
        VoxelizeMotionBySegment(m2, time, temp_seg_data);
        
        // 累積
        for (int s = 0; s < num_segments; ++s) {
            int size = grid_resolution * grid_resolution * grid_resolution;
            for (int i = 0; i < size; ++i) {
                segment_voxels2.segment_grids[s].data[i] += temp_seg_data.segment_grids[s].data[i];
            }
        }
    }
    
    std::cout << "Segment-wise accumulation complete." << std::endl;
}

// NEW: キャッシュファイル名を生成
std::string SpatialAnalyzer::GenerateCacheFilename(const char* motion1_name, const char* motion2_name) const {
    std::string filename = "voxel_cache_";
    filename += motion1_name;
    filename += "_";
    filename += motion2_name;
    filename += ".vxl";
    return filename;
}

// NEW: ボクセルキャッシュを保存
bool SpatialAnalyzer::SaveVoxelCache(const char* motion1_name, const char* motion2_name) {
    std::string base_filename = GenerateCacheFilename(motion1_name, motion2_name);
    
    std::cout << "Saving voxel cache to " << base_filename << "..." << std::endl;
    
    // 累積ボクセルデータを保存
    std::string accumulated1_file = base_filename + "_acc1.bin";
    std::string accumulated2_file = base_filename + "_acc2.bin";
    std::string accumulated_diff_file = base_filename + "_acc_diff.bin";
    
    if (!voxels1_accumulated.SaveToFile(accumulated1_file.c_str())) return false;
    if (!voxels2_accumulated.SaveToFile(accumulated2_file.c_str())) return false;
    if (!voxels_accumulated_diff.SaveToFile(accumulated_diff_file.c_str())) return false;
    
    // 部位ごとのボクセルデータを保存
    std::string segment1_file = base_filename + "_seg1.bin";
    std::string segment2_file = base_filename + "_seg2.bin";
    
    if (!segment_voxels1.SaveToFile(segment1_file.c_str())) return false;
    if (!segment_voxels2.SaveToFile(segment2_file.c_str())) return false;
    
    // メタデータを保存
    std::string meta_file = base_filename + "_meta.txt";
    ofstream meta_ofs(meta_file);
    if (!meta_ofs) return false;
    
    meta_ofs << grid_resolution << std::endl;
    meta_ofs << max_accumulated_val << std::endl;
    for (int i = 0; i < 3; ++i) {
        meta_ofs << world_bounds[i][0] << " " << world_bounds[i][1] << std::endl;
    }
    meta_ofs.close();
    
    std::cout << "Voxel cache saved successfully!" << std::endl;
    return true;
}

// NEW: ボクセルキャッシュを読み込み
bool SpatialAnalyzer::LoadVoxelCache(const char* motion1_name, const char* motion2_name) {
    std::string base_filename = GenerateCacheFilename(motion1_name, motion2_name);
    
    std::cout << "Loading voxel cache from " << base_filename << "..." << std::endl;
    
    // メタデータを読み込み
    std::string meta_file = base_filename + "_meta.txt";
    ifstream meta_ifs(meta_file);
    if (!meta_ifs) {
        std::cout << "Cache file not found: " << meta_file << std::endl;
        return false;
    }
    
    int res;
    meta_ifs >> res;
    meta_ifs >> max_accumulated_val;
    for (int i = 0; i < 3; ++i) {
        meta_ifs >> world_bounds[i][0] >> world_bounds[i][1];
    }
    meta_ifs.close();
    
    // グリッドをリサイズ
    ResizeGrids(res);
    
    // 累積ボクセルデータを読み込み
    std::string accumulated1_file = base_filename + "_acc1.bin";
    std::string accumulated2_file = base_filename + "_acc2.bin";
    std::string accumulated_diff_file = base_filename + "_acc_diff.bin";
    
    if (!voxels1_accumulated.LoadFromFile(accumulated1_file.c_str())) {
        std::cout << "Failed to load: " << accumulated1_file << std::endl;
        return false;
    }
    if (!voxels2_accumulated.LoadFromFile(accumulated2_file.c_str())) {
        std::cout << "Failed to load: " << accumulated2_file << std::endl;
        return false;
    }
    if (!voxels_accumulated_diff.LoadFromFile(accumulated_diff_file.c_str())) {
        std::cout << "Failed to load: " << accumulated_diff_file << std::endl;
        return false;
    }
    
    // 部位ごとのボクセルデータを読み込み
    std::string segment1_file = base_filename + "_seg1.bin";
    std::string segment2_file = base_filename + "_seg2.bin";
    
    if (!segment_voxels1.LoadFromFile(segment1_file.c_str())) {
        std::cout << "Failed to load: " << segment1_file << std::endl;
        return false;
    }
    if (!segment_voxels2.LoadFromFile(segment2_file.c_str())) {
        std::cout << "Failed to load: " << segment2_file << std::endl;
        return false;
    }
    
    std::cout << "Voxel cache loaded successfully!" << std::endl;
    return true;
}
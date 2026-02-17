#include "SpatialAnalysis.h"
#include "SimpleHumanGLUT.h" // OpenGL用
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <fstream>  // ファイルI/O用

using namespace std;

// --- 内部ヘルパー関数 ---

static float sa_get_axis_value(const Point3f& p, int axis_index) {
    if (axis_index == 0) return p.x; 
    if (axis_index == 1) return p.y;
    return p.z; 
}

// HSV to RGBヒートマップ変換
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

// --- VoxelGrid Implementation ---

void VoxelGrid::Resize(int res) {
    resolution = res;
    data.assign(res * res * res, 0.0f);
}

void VoxelGrid::Clear() {
    std::fill(data.begin(), data.end(), 0.0f);
}

float& VoxelGrid::At(int x, int y, int z) {
    if (x < 0 || x >= resolution || y < 0 || y >= resolution || z < 0 || z >= resolution) {
        static float dummy = 0.0f; 
        return dummy;
    }
    return data[z * resolution * resolution + y * resolution + x];
}

float VoxelGrid::Get(int x, int y, int z) const {
    if (x < 0 || x >= resolution || y < 0 || y >= resolution || z < 0 || z >= resolution) 
        return 0.0f;
    return data[static_cast<size_t>(z) * resolution * resolution + y * resolution + x];
}

// ファイルへ保存（基準姿勢情報も含む）
bool VoxelGrid::SaveToFile(const char* filename) const {
    ofstream ofs(filename, ios::binary);
    if (!ofs) 
        return false;
    
    int version = 2;
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(int));
    ofs.write(reinterpret_cast<const char*>(&resolution), sizeof(int));
    
    int data_size = data.size();
    ofs.write(reinterpret_cast<const char*>(&data_size), sizeof(int));
    ofs.write(reinterpret_cast<const char*>(data.data()), data_size * sizeof(float));
    
    // 基準姿勢情報を書き込み
    ofs.write(reinterpret_cast<const char*>(&has_reference), sizeof(bool));
    if (has_reference) {
        ofs.write(reinterpret_cast<const char*>(&reference_root_pos.x), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&reference_root_pos.y), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&reference_root_pos.z), sizeof(float));
        
        float m[9] = {
            reference_root_ori.m00, reference_root_ori.m01, reference_root_ori.m02,
            reference_root_ori.m10, reference_root_ori.m11, reference_root_ori.m12,
            reference_root_ori.m20, reference_root_ori.m21, reference_root_ori.m22
        };
        ofs.write(reinterpret_cast<const char*>(m), sizeof(m));
    }
    
    return ofs.good();
}

// ファイルから読み込み（バージョン対応）
bool VoxelGrid::LoadFromFile(const char* filename) {
    ifstream ifs(filename, ios::binary);
    if (!ifs) 
        return false;
    
    int version;
    ifs.read(reinterpret_cast<char*>(&version), sizeof(int));
    
    int res;
    ifs.read(reinterpret_cast<char*>(&res), sizeof(int));
    
    int data_size;
    ifs.read(reinterpret_cast<char*>(&data_size), sizeof(int));
    
    Resize(res);
    ifs.read(reinterpret_cast<char*>(data.data()), data_size * sizeof(float));
    
    // バージョン2以降は基準姿勢情報を読み込み
    if (version >= 2) {
        ifs.read(reinterpret_cast<char*>(&has_reference), sizeof(bool));
        if (has_reference) {
            ifs.read(reinterpret_cast<char*>(&reference_root_pos.x), sizeof(float));
            ifs.read(reinterpret_cast<char*>(&reference_root_pos.y), sizeof(float));
            ifs.read(reinterpret_cast<char*>(&reference_root_pos.z), sizeof(float));
            
            float m[9];
            ifs.read(reinterpret_cast<char*>(m), sizeof(m));
            reference_root_ori.set(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
        }
    } else {
        has_reference = false;
    }
    
    return ifs.good();
}

// SegmentVoxelDataのファイル保存
bool SegmentVoxelData::SaveToFile(const char* filename) const {
    ofstream ofs(filename, ios::binary);
    if (!ofs) 
        return false;
    
    ofs.write(reinterpret_cast<const char*>(&num_segments), sizeof(int));
    
    for (int i = 0; i < num_segments; ++i) {
        const VoxelGrid& grid = segment_grids[i];
        ofs.write(reinterpret_cast<const char*>(&grid.resolution), sizeof(int));
        
        int data_size = grid.data.size();
        ofs.write(reinterpret_cast<const char*>(&data_size), sizeof(int));
        ofs.write(reinterpret_cast<const char*>(grid.data.data()), data_size * sizeof(float));
    }
    
    return ofs.good();
}

// SegmentVoxelDataのファイル読み込み
bool SegmentVoxelData::LoadFromFile(const char* filename) {
    ifstream ifs(filename, ios::binary);
    if (!ifs) 
        return false;
    
    int num_seg;
    ifs.read(reinterpret_cast<char*>(&num_seg), sizeof(int));
    
    num_segments = num_seg;
    segment_grids.resize(num_seg);
    
    for (int i = 0; i < num_segments; ++i) {
        VoxelGrid& grid = segment_grids[i];
        
        int res;
        ifs.read(reinterpret_cast<char*>(&res), sizeof(int));
        
        int data_size;
        ifs.read(reinterpret_cast<char*>(&data_size), sizeof(int));
        
        grid.Resize(res);
        ifs.read(reinterpret_cast<char*>(grid.data.data()), data_size * sizeof(float));
    }
    
    return ifs.good();
}

// --- SpatialAnalyzer Implementation ---

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
    max_spd_val = 1.0f;
    max_jrk_val = 1.0f;
    max_psc_accumulated_val = 1.0f;
    max_spd_accumulated_val = 1.0f;
    max_jrk_accumulated_val = 1.0f;
    
    // 部位別表示モード
    selected_segment_index = -1;
    show_segment_mode = false;
    
    // 部位選択キャッシュ
    segment_cache_dirty = true;
    cached_segment_max_val = 1.0f;
    cached_feature_mode = -1;
    cached_selected_segment_index = -2;
    
    // スライス平面の変換行列を単位行列で初期化
    slice_plane_transform.setIdentity();
    
    // 表示用オイラー角
    slice_rotation_x = 0.0f;
    slice_rotation_y = 0.0f;
    slice_rotation_z = 0.0f;
    
    use_rotated_slice = true;
    slice_positions.push_back(0.5f);
}

SpatialAnalyzer::~SpatialAnalyzer() {}

void SpatialAnalyzer::ResizeGrids(int res) {
    grid_resolution = res;
    
    // 瞬間グリッド
    voxels1_psc.Resize(res); voxels2_psc.Resize(res); voxels_psc_diff.Resize(res);
    voxels1_spd.Resize(res); voxels2_spd.Resize(res); voxels_spd_diff.Resize(res);
    voxels1_jrk.Resize(res); voxels2_jrk.Resize(res); voxels_jrk_diff.Resize(res);
    
    // 累積グリッド
    voxels1_psc_accumulated.Resize(res);
    voxels2_psc_accumulated.Resize(res);
    voxels_psc_accumulated_diff.Resize(res);
    voxels1_spd_accumulated.Resize(res);
    voxels2_spd_accumulated.Resize(res);
    voxels_spd_accumulated_diff.Resize(res);
    voxels1_jrk_accumulated.Resize(res);
    voxels2_jrk_accumulated.Resize(res);
    voxels_jrk_accumulated_diff.Resize(res);
}

void SpatialAnalyzer::SetWorldBounds(float bounds[3][2]) {
    for (int i = 0; i < 3; ++i) {
        world_bounds[i][0] = bounds[i][0];
        world_bounds[i][1] = bounds[i][1];
    }
    
    // スライス平面の中心をワールド中心に設定
    float cx = (world_bounds[0][0] + world_bounds[0][1]) / 2.0f;
    float cy = (world_bounds[1][0] + world_bounds[1][1]) / 2.0f;
    float cz = (world_bounds[2][0] + world_bounds[2][1]) / 2.0f;
    
    slice_plane_transform.setIdentity();
    slice_plane_transform.m03 = cx;
    slice_plane_transform.m13 = cy;
    slice_plane_transform.m23 = cz;
    
    UpdateEulerAnglesFromTransform();
}

void SpatialAnalyzer::VoxelizeMotion(Motion* m, float time, VoxelGrid& occ, VoxelGrid& spd, VoxelGrid& jrk) {
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

    for (const BoneData& bone : bones)
        WriteToVoxelGrid(bone, bone_radius, world_range, &occ, &spd, &jrk);
}

void SpatialAnalyzer::UpdateVoxels(Motion* m1, Motion* m2, float current_time) {
    // グリッドをクリア
    voxels1_psc.Clear(); voxels2_psc.Clear();
    voxels1_spd.Clear(); voxels2_spd.Clear();
    voxels1_jrk.Clear(); voxels2_jrk.Clear();

    VoxelizeMotion(m1, current_time, voxels1_psc, voxels1_spd, voxels1_jrk);
    VoxelizeMotion(m2, current_time, voxels2_psc, voxels2_spd, voxels2_jrk);

    // 差分計算と最大値更新
    max_psc_val = 0.0f;
    max_spd_val = 0.0f;
    max_jrk_val = 0.0f;
    int size = grid_resolution * grid_resolution * grid_resolution;

    for (int i = 0; i < size; ++i) {
        float v1 = voxels1_psc.data[i];
        float v2 = voxels2_psc.data[i];
        float diff = abs(v1 - v2);
        voxels_psc_diff.data[i] = diff;
        if (diff > max_psc_val) 
            max_psc_val = diff;
        
        float s1 = voxels1_spd.data[i];
        float s2 = voxels2_spd.data[i];
        voxels_spd_diff.data[i] = abs(s1 - s2);
        if (abs(s1 - s2) > max_spd_val) 
            max_spd_val = abs(s1 - s2);

        float j1 = voxels1_jrk.data[i];
        float j2 = voxels2_jrk.data[i];
        voxels_jrk_diff.data[i] = abs(j1 - j2);
        if (abs(j1 - j2) > max_jrk_val) 
            max_jrk_val = abs(j1 - j2);
    }
    
    if (max_psc_val < 1e-5f) max_psc_val = 1.0f;
    if (max_spd_val < 1e-5f) max_spd_val = 1.0f;
    if (max_jrk_val < 1e-5f) max_jrk_val = 1.0f;
}

void SpatialAnalyzer::DrawSlicePlanes() {
    DrawRotatedSlicePlane();
}

void SpatialAnalyzer::ResetView() { 
    zoom = 1.0f; 
    pan_center.set(0.0f, 0.0f); 
    is_manual_view = false; 
}

void SpatialAnalyzer::Pan(float dx, float dy) { 
    pan_center.x += dx; 
    pan_center.y += dy; 
    is_manual_view = true; 
}

void SpatialAnalyzer::Zoom(float factor) { 
    zoom *= factor; 
    is_manual_view = true; 
}

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
    
    if (use_segment_mode) {
        UpdateSegmentCache();
        grid1 = &cached_segment_grid1;
        grid2 = &cached_segment_grid2;
        grid_diff = &cached_segment_diff;
        max_value = cached_segment_max_val;
    } else if (norm_mode == 0) {
        // 瞬間表示
        VoxelGrid* grids[][3] = {
            {&voxels1_psc, &voxels2_psc, &voxels_psc_diff},
            {&voxels1_spd, &voxels2_spd, &voxels_spd_diff},
            {&voxels1_jrk, &voxels2_jrk, &voxels_jrk_diff}
        };
        float max_vals[] = {max_psc_val, max_spd_val, max_jrk_val};
        grid1 = grids[feature_mode][0];
        grid2 = grids[feature_mode][1];
        grid_diff = grids[feature_mode][2];
        max_value = max_vals[feature_mode];
    } else {
        // 累積表示
        VoxelGrid* grids[][3] = {
            {&voxels1_psc_accumulated, &voxels2_psc_accumulated, &voxels_psc_accumulated_diff},
            {&voxels1_spd_accumulated, &voxels2_spd_accumulated, &voxels_spd_accumulated_diff},
            {&voxels1_jrk_accumulated, &voxels2_jrk_accumulated, &voxels_jrk_accumulated_diff}
        };
        float max_vals[] = {max_psc_accumulated_val, max_spd_accumulated_val, max_jrk_accumulated_val};
        grid1 = grids[feature_mode][0];
        grid2 = grids[feature_mode][1];
        grid_diff = grids[feature_mode][2];
        max_value = max_vals[feature_mode];
    }
    
    DrawRotatedSliceMap(start_x, y_pos, map_w, map_h, *grid1, max_value, "Rotated M1");
    DrawRotatedSliceMap(start_x + map_w + gap, y_pos, map_w, map_h, *grid2, max_value, "Rotated M2");
    DrawRotatedSliceMap(start_x + 2*(map_w + gap), y_pos, map_w, map_h, *grid_diff, max_value, "Rotated Diff");
    
    glPopAttrib();
    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW); glPopMatrix();
}

// 3Dボクセルの描画
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
    float max_val = 1.0f;
    
    int selected_count = GetSelectedSegmentCount();
    bool use_segment_mode = show_segment_mode && (selected_count > 0 || selected_segment_index >= 0);
    
    if (use_segment_mode) {
        UpdateSegmentCache();
        grid_to_draw = &cached_segment_diff;
        max_val = cached_segment_max_val;
    } else if (norm_mode == 0) {
        VoxelGrid* grids[] = {&voxels_psc_diff, &voxels_spd_diff, &voxels_jrk_diff};
        float max_vals[] = {max_psc_val, max_spd_val, max_jrk_val};
        grid_to_draw = grids[feature_mode];
        max_val = max_vals[feature_mode];
    } else {
        VoxelGrid* grids[] = {&voxels_psc_accumulated_diff, &voxels_spd_accumulated_diff, &voxels_jrk_accumulated_diff};
        float max_vals[] = {max_psc_accumulated_val, max_spd_accumulated_val, max_jrk_accumulated_val};
        grid_to_draw = grids[feature_mode];
        max_val = max_vals[feature_mode];
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
                
                float norm_val = min(val / max_val, 1.0f);
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

// 動作全体を通った累積ボクセル計算（占有率と速度とジャーク + 部位ごと）
void SpatialAnalyzer::AccumulateAllFrames(Motion* m1, Motion* m2)
{
    if (!m1 || !m2) 
        return;
    
    int num_segments = m1->body->num_segments;
    int size = grid_resolution * grid_resolution * grid_resolution;
    
    ClearAccumulatedData();
    
    // グリッドをリサイズ
    voxels1_psc_accumulated.Resize(grid_resolution);
    voxels2_psc_accumulated.Resize(grid_resolution);
    voxels_psc_accumulated_diff.Resize(grid_resolution);
    voxels1_spd_accumulated.Resize(grid_resolution);
    voxels2_spd_accumulated.Resize(grid_resolution);
    voxels_spd_accumulated_diff.Resize(grid_resolution);
    voxels1_jrk_accumulated.Resize(grid_resolution);
    voxels2_jrk_accumulated.Resize(grid_resolution);
    voxels_jrk_accumulated_diff.Resize(grid_resolution);
    
    // 部位ごとのグリッドをリサイズ
    segment_presence_voxels1.Resize(num_segments, grid_resolution);
    segment_presence_voxels2.Resize(num_segments, grid_resolution);
    segment_speed_voxels1.Resize(num_segments, grid_resolution);
    segment_speed_voxels2.Resize(num_segments, grid_resolution);
    segment_jerk_voxels1.Resize(num_segments, grid_resolution);
    segment_jerk_voxels2.Resize(num_segments, grid_resolution);
    
    // 基準姿勢を保存
    if (m1->num_frames > 0)
        voxels1_psc_accumulated.SetReference(m1->frames[0].root_pos, m1->frames[0].root_ori);
    if (m2->num_frames > 0)
        voxels2_psc_accumulated.SetReference(m2->frames[0].root_pos, m2->frames[0].root_ori);
    
    std::cout << "Accumulating voxels (integrated) for " << num_segments << " segments..." << std::endl;
    
    // モーション処理用のラムダ関数
    auto processMotion = [&](Motion* m, 
                             SegmentVoxelData& seg_psc, SegmentVoxelData& seg_spd, SegmentVoxelData& seg_jrk,
                             VoxelGrid& acc_psc, VoxelGrid& acc_spd, VoxelGrid& acc_jrk) {
        for (int f = 0; f < m->num_frames; ++f) {
            float time = f * m->interval;
            SegmentVoxelData temp_seg_psc, temp_seg_spd, temp_seg_jrk;
            temp_seg_psc.Resize(num_segments, grid_resolution);
            temp_seg_spd.Resize(num_segments, grid_resolution);
            temp_seg_jrk.Resize(num_segments, grid_resolution);
            
            // モーションに基づくボクセル化
            VoxelizeMotionBySegment(m, time, temp_seg_psc, temp_seg_spd, temp_seg_jrk);
            
            // セグメントごとのボクセルデータを累積
            for (int s = 0; s < num_segments; ++s) {
                for (int i = 0; i < size; ++i) {
                    float psc_val = temp_seg_psc.segment_grids[s].data[i];
                    float spd_val = temp_seg_spd.segment_grids[s].data[i];
                    float jrk_val = temp_seg_jrk.segment_grids[s].data[i];
                    
                    seg_psc.segment_grids[s].data[i] += psc_val;
                    if (spd_val > seg_spd.segment_grids[s].data[i])
                        seg_spd.segment_grids[s].data[i] = spd_val;
                    if (jrk_val > seg_jrk.segment_grids[s].data[i])
                        seg_jrk.segment_grids[s].data[i] = jrk_val;
                    
                    acc_psc.data[i] += psc_val;
                    if (spd_val > acc_spd.data[i]) acc_spd.data[i] = spd_val;
                    if (jrk_val > acc_jrk.data[i]) acc_jrk.data[i] = jrk_val;
                }
            }
        }
    };
    
    processMotion(m1, segment_presence_voxels1, segment_speed_voxels1, segment_jerk_voxels1,
                  voxels1_psc_accumulated, voxels1_spd_accumulated, voxels1_jrk_accumulated);
    processMotion(m2, segment_presence_voxels2, segment_speed_voxels2, segment_jerk_voxels2,
                  voxels2_psc_accumulated, voxels2_spd_accumulated, voxels2_jrk_accumulated);
    
    // 差分計算と最大値更新
    for (int i = 0; i < size; ++i) {
        float diff_psc = abs(voxels1_psc_accumulated.data[i] - voxels2_psc_accumulated.data[i]);
        voxels_psc_accumulated_diff.data[i] = diff_psc;
        if (diff_psc > max_psc_accumulated_val) 
            max_psc_accumulated_val = diff_psc;
        
        float diff_spd = abs(voxels1_spd_accumulated.data[i] - voxels2_spd_accumulated.data[i]);
        voxels_spd_accumulated_diff.data[i] = diff_spd;
        float max_spd = max(voxels1_spd_accumulated.data[i], voxels2_spd_accumulated.data[i]);
        if (max_spd > max_spd_accumulated_val) 
            max_spd_accumulated_val = max_spd;

        float diff_jrk = abs(voxels1_jrk_accumulated.data[i] - voxels2_jrk_accumulated.data[i]);
        voxels_jrk_accumulated_diff.data[i] = diff_jrk;
        float max_jrk = max(voxels1_jrk_accumulated.data[i], voxels2_jrk_accumulated.data[i]);
        if (max_jrk > max_jrk_accumulated_val) 
            max_jrk_accumulated_val = max_jrk;
    }
    
    if (max_psc_accumulated_val < 1e-5f) max_psc_accumulated_val = 1.0f;
    if (max_spd_accumulated_val < 1e-5f) max_spd_accumulated_val = 1.0f;
    if (max_jrk_accumulated_val < 1e-5f) max_jrk_accumulated_val = 1.0f;
    
    if (voxels1_psc_accumulated.has_reference)
        voxels_psc_accumulated_diff.SetReference(
            voxels1_psc_accumulated.reference_root_pos, 
            voxels1_psc_accumulated.reference_root_ori);
    
    // 部位ごとの最大値を計算
    InitializeSegmentSelection(num_segments);
    for (int s = 0; s < num_segments; ++s) {
        float max_psc = 0.0f, max_spd = 0.0f, max_jrk = 0.0f;
        for (int i = 0; i < size; ++i) {
            float diff_psc = abs(segment_presence_voxels1.segment_grids[s].data[i] - 
                                  segment_presence_voxels2.segment_grids[s].data[i]);
            float diff_spd = abs(segment_speed_voxels1.segment_grids[s].data[i] - 
                                 segment_speed_voxels2.segment_grids[s].data[i]);
            float diff_jrk = abs(segment_jerk_voxels1.segment_grids[s].data[i] - 
                                 segment_jerk_voxels2.segment_grids[s].data[i]);
            if (diff_psc > max_psc) max_psc = diff_psc;
            if (diff_spd > max_spd) max_spd = diff_spd;
            if (diff_jrk > max_jrk) max_jrk = diff_jrk;
        }
        segment_max_presence[s] = (max_psc < 1e-5f) ? 1.0f : max_psc;
        segment_max_speed[s] = (max_spd < 1e-5f) ? 1.0f : max_spd;
        segment_max_jerk[s] = (max_jrk < 1e-5f) ? 1.0f : max_jrk;
    }
    
    std::cout << "Integrated accumulation complete." << std::endl;
    std::cout << "  Max presence diff: " << max_psc_accumulated_val << std::endl;
    std::cout << "  Max speed: " << max_spd_accumulated_val << std::endl;
    std::cout << "  Max jerk: " << max_jrk_accumulated_val << std::endl;
}

// 累積グリッドのクリア
void SpatialAnalyzer::ClearAccumulatedData()
{
    voxels1_psc_accumulated.Clear();
    voxels2_psc_accumulated.Clear();
    voxels_psc_accumulated_diff.Clear();
    max_psc_accumulated_val = 0.0f;

    voxels1_spd_accumulated.Clear();
    voxels2_spd_accumulated.Clear();
    voxels_spd_accumulated_diff.Clear();
    max_spd_accumulated_val = 0.0f;

    voxels1_jrk_accumulated.Clear();
    voxels2_jrk_accumulated.Clear();
    voxels_jrk_accumulated_diff.Clear();
    max_jrk_accumulated_val = 0.0f;
}

// 部位ごとのボクセル化
void SpatialAnalyzer::VoxelizeMotionBySegment(Motion* m, float time, SegmentVoxelData& seg_presence_data, SegmentVoxelData& seg_speed_data, SegmentVoxelData& seg_jerk_data)
{
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
        WriteToVoxelGrid(bone, bone_radius, world_range, &seg_pres_grid, &seg_spd_grid, &seg_jrk_grid);
    }
}

// キャッシュファイル名を生成
std::string SpatialAnalyzer::GenerateCacheFilename(const char* motion1_name, const char* motion2_name) const {
    std::string filename = "voxel_cache_";
    filename += motion1_name;
    filename += "_";
    filename += motion2_name;
    filename += ".vxl";
    return filename;
}

// ボクセルキャッシュを保存
bool SpatialAnalyzer::SaveVoxelCache(const char* motion1_name, const char* motion2_name) {
    std::string base = GenerateCacheFilename(motion1_name, motion2_name);
    
    std::cout << "Saving voxel cache to " << base << "..." << std::endl;
    
    // 累積ボクセルデータを保存
    if (!voxels1_psc_accumulated.SaveToFile((base + "_acc1.bin").c_str())) return false;
    if (!voxels2_psc_accumulated.SaveToFile((base + "_acc2.bin").c_str())) return false;
    if (!voxels_psc_accumulated_diff.SaveToFile((base + "_acc_diff.bin").c_str())) return false;
    
    if (!voxels1_spd_accumulated.SaveToFile((base + "_spd_acc1.bin").c_str())) return false;
    if (!voxels2_spd_accumulated.SaveToFile((base + "_spd_acc2.bin").c_str())) return false;
    if (!voxels_spd_accumulated_diff.SaveToFile((base + "_spd_acc_diff.bin").c_str())) return false;
    
    if (!voxels1_jrk_accumulated.SaveToFile((base + "_jrk_acc1.bin").c_str())) return false;
    if (!voxels2_jrk_accumulated.SaveToFile((base + "_jrk_acc2.bin").c_str())) return false;
    if (!voxels_jrk_accumulated_diff.SaveToFile((base + "_jrk_acc_diff.bin").c_str())) return false;
    
    // 部位ごとのボクセルデータを保存
    if (!segment_presence_voxels1.SaveToFile((base + "_seg1.bin").c_str())) return false;
    if (!segment_presence_voxels2.SaveToFile((base + "_seg2.bin").c_str())) return false;
    if (!segment_speed_voxels1.SaveToFile((base + "_seg_spd1.bin").c_str())) return false;
    if (!segment_speed_voxels2.SaveToFile((base + "_seg_spd2.bin").c_str())) return false;
    if (!segment_jerk_voxels1.SaveToFile((base + "_seg_jrk1.bin").c_str())) return false;
    if (!segment_jerk_voxels2.SaveToFile((base + "_seg_jrk2.bin").c_str())) return false;
    
    // メタデータを保存
    ofstream meta_ofs(base + "_meta.txt");
    if (!meta_ofs) 
        return false;
    
    meta_ofs << grid_resolution << std::endl;
    meta_ofs << max_psc_accumulated_val << std::endl;
    meta_ofs << max_spd_accumulated_val << std::endl;
    meta_ofs << max_jrk_accumulated_val << std::endl;
    for (int i = 0; i < 3; ++i)
        meta_ofs << world_bounds[i][0] << " " << world_bounds[i][1] << std::endl;
    meta_ofs.close();
    
    std::cout << "Voxel cache saved successfully!" << std::endl;
    return true;
}

// ボクセルキャッシュを読み込み
bool SpatialAnalyzer::LoadVoxelCache(const char* motion1_name, const char* motion2_name) {
    std::string base = GenerateCacheFilename(motion1_name, motion2_name);
    
    std::cout << "Loading voxel cache from " << base << "..." << std::endl;
    
    // メタデータを読み込み
    ifstream meta_ifs(base + "_meta.txt");
    if (!meta_ifs) {
        std::cout << "Cache file not found: " << base + "_meta.txt" << std::endl;
        return false;
    }
    
    int res;
    meta_ifs >> res;
    meta_ifs >> max_psc_accumulated_val;
    meta_ifs >> max_spd_accumulated_val;
    meta_ifs >> max_jrk_accumulated_val;
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
    if (!loadFile(voxels1_psc_accumulated, base + "_acc1.bin")) return false;
    if (!loadFile(voxels2_psc_accumulated, base + "_acc2.bin")) return false;
    if (!loadFile(voxels_psc_accumulated_diff, base + "_acc_diff.bin")) return false;
    
    if (!loadFile(voxels1_spd_accumulated, base + "_spd_acc1.bin")) return false;
    if (!loadFile(voxels2_spd_accumulated, base + "_spd_acc2.bin")) return false;
    if (!loadFile(voxels_spd_accumulated_diff, base + "_spd_acc_diff.bin")) return false;
    
    if (!loadFile(voxels1_jrk_accumulated, base + "_jrk_acc1.bin")) return false;
    if (!loadFile(voxels2_jrk_accumulated, base + "_jrk_acc2.bin")) return false;
    if (!loadFile(voxels_jrk_accumulated_diff, base + "_jrk_acc_diff.bin")) return false;
    
    // 部位ごとのボクセルデータを読み込み
    if (!loadSegmentFile(segment_presence_voxels1, base + "_seg1.bin")) return false;
    if (!loadSegmentFile(segment_presence_voxels2, base + "_seg2.bin")) return false;
    if (!loadSegmentFile(segment_speed_voxels1, base + "_seg_spd1.bin")) return false;
    if (!loadSegmentFile(segment_speed_voxels2, base + "_seg_spd2.bin")) return false;
    if (!loadSegmentFile(segment_jerk_voxels1, base + "_seg_jrk1.bin")) return false;
    if (!loadSegmentFile(segment_jerk_voxels2, base + "_seg_jrk2.bin")) return false;
    
    std::cout << "Voxel cache loaded successfully!" << std::endl;
    return true;
}

// スライス平面の回転操作（キーボード用）
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

void SpatialAnalyzer::ResetSliceRotation() {
    float cx = (world_bounds[0][0] + world_bounds[0][1]) / 2.0f;
    float cy = (world_bounds[1][0] + world_bounds[1][1]) / 2.0f;
    float cz = (world_bounds[2][0] + world_bounds[2][1]) / 2.0f;
    
    slice_plane_transform.setIdentity();
    slice_plane_transform.m03 = cx;
    slice_plane_transform.m13 = cy;
    slice_plane_transform.m23 = cz;
    
    slice_rotation_x = 0.0f;
    slice_rotation_y = 0.0f;
    slice_rotation_z = 0.0f;
    pan_center.set(0.0f, 0.0f);
}

void SpatialAnalyzer::ToggleRotatedSliceMode() {
    use_rotated_slice = !use_rotated_slice;
    if (use_rotated_slice)
        ResetSliceRotation();
    std::cout << "Rotated slice mode: " << (use_rotated_slice ? "ON" : "OFF") << std::endl;
}

// 回転スライス平面の描画
void SpatialAnalyzer::DrawRotatedSlicePlane() {
    if (!use_rotated_slice) 
        return;
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    float world_range[3];
    for (int i = 0; i < 3; ++i)
        world_range[i] = world_bounds[i][1] - world_bounds[i][0];
    float max_range = max(world_range[0], max(world_range[1], world_range[2]));
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
        
        glColor4f(1.0f, 1.0f, 0.0f, 0.9f);
        glLineWidth(5.0f);

        glBegin(GL_LINE_LOOP);
        for (int k = 0; k < 4; ++k) 
            glVertex3f(p[k].x, p[k].y, p[k].z);
        glEnd();   
    }
    glDisable(GL_BLEND);
}

// 回転スライス用の2Dマップ描画
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
    float max_range = max(world_range[0], max(world_range[1], world_range[2]));
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
    glColor3f(0,0,0);
    glRasterPos2i(x_pos, y_pos - 5);
    for (const char* c = title; *c != '\0'; c++) 
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
    
    char rot_info[128];
    sprintf(rot_info, "Rot: X=%.0f Y=%.0f Z=%.0f", slice_rotation_x, slice_rotation_y, slice_rotation_z);
    glRasterPos2i(x_pos, y_pos + h + 22);
    for (const char* c = rot_info; *c != '\0'; c++) 
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
}

// ワールド座標でボクセル値をサンプリング
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

Point3f SpatialAnalyzer::GetSlicePlaneCenter() const {
    return Point3f(slice_plane_transform.m03, slice_plane_transform.m13, slice_plane_transform.m23);
}

Vector3f SpatialAnalyzer::GetSlicePlaneU() const {
    return Vector3f(slice_plane_transform.m00, slice_plane_transform.m10, slice_plane_transform.m20);
}

Vector3f SpatialAnalyzer::GetSlicePlaneV() const {
    return Vector3f(slice_plane_transform.m01, slice_plane_transform.m11, slice_plane_transform.m21);
}

Vector3f SpatialAnalyzer::GetSlicePlaneNormal() const {
    return Vector3f(slice_plane_transform.m02, slice_plane_transform.m12, slice_plane_transform.m22);
}

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

void SpatialAnalyzer::ApplySlicePlaneTranslation(const Point3f& world_translation) {
    slice_plane_transform.m03 += world_translation.x;
    slice_plane_transform.m13 += world_translation.y;
    slice_plane_transform.m23 += world_translation.z;
}

void SpatialAnalyzer::UpdateEulerAnglesFromTransform() {
    Vector3f n = GetSlicePlaneNormal();
    Vector3f u = GetSlicePlaneU();
    Vector3f v = GetSlicePlaneV();
    
    float ny = n.y;
    if (ny > 1.0f) ny = 1.0f;
    if (ny < -1.0f) ny = -1.0f;
    float nxz = sqrtf(n.x * n.x + n.z * n.z);
    
    slice_rotation_x = atan2f(-ny, nxz) * 180.0f / 3.14159265f;
    slice_rotation_y = atan2f(n.x, n.z) * 180.0f / 3.14159265f;
    slice_rotation_z = atan2f(u.y, v.y) * 180.0f / 3.14159265f;
}

// === 共通ボクセル化ヘルパー関数 ===

// フレームデータを計算（4フレーム分を取得）
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
}

// 全ボーンのデータを抽出
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
        
        // 速度とジャークを計算
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
        }
        
        bones.push_back(bone);
    }
}

// AABB計算
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

// ボクセルグリッドへの書き込み
void SpatialAnalyzer::WriteToVoxelGrid(const BoneData& bone, float bone_radius, const float world_range[3],
                                        VoxelGrid* occ_grid, VoxelGrid* spd_grid, VoxelGrid* jrk_grid) {
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
                    float presence = exp(-dist_sq / sigma_sq);
                    
                    if (occ_grid)
                        occ_grid->At(x, y, z) += presence;
                    
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
                }
            }
        }
    }
}

// === 部位選択操作（複数選択対応） ===

void SpatialAnalyzer::InitializeSegmentSelection(int num_segments) {
    selected_segments.assign(num_segments, false);
    segment_max_presence.assign(num_segments, 1.0f);
    segment_max_speed.assign(num_segments, 1.0f);
    segment_max_jerk.assign(num_segments, 1.0f);
}

void SpatialAnalyzer::ToggleSegmentSelection(int segment_index) {
    if (segment_index >= 0 && segment_index < (int)selected_segments.size()) {
        selected_segments[segment_index] = !selected_segments[segment_index];
        InvalidateSegmentCache();
        std::cout << "Segment " << segment_index << " selection: " 
                  << (selected_segments[segment_index] ? "ON" : "OFF") << std::endl;
    }
}

void SpatialAnalyzer::SelectAllSegments() {
    for (size_t i = 0; i < selected_segments.size(); ++i)
        selected_segments[i] = true;
    InvalidateSegmentCache();
    std::cout << "All segments selected" << std::endl;
}

void SpatialAnalyzer::ClearSegmentSelection() {
    for (size_t i = 0; i < selected_segments.size(); ++i)
        selected_segments[i] = false;
    InvalidateSegmentCache();
    std::cout << "Segment selection cleared" << std::endl;
}

bool SpatialAnalyzer::IsSegmentSelected(int segment_index) const {
    if (segment_index >= 0 && segment_index < (int)selected_segments.size())
        return selected_segments[segment_index];
    return false;
}

int SpatialAnalyzer::GetSelectedSegmentCount() const {
    int count = 0;
    for (size_t i = 0; i < selected_segments.size(); ++i)
        if (selected_segments[i]) 
            count++;
    return count;
}

float SpatialAnalyzer::GetSegmentMaxValue(int segment_index) const {
    if (segment_index < 0 || segment_index >= (int)segment_max_presence.size())
        return 1.0f;
    
    if (feature_mode == 0)
        return segment_max_presence[segment_index];
    else if (feature_mode == 1)
        return segment_max_speed[segment_index];
    else
        return segment_max_jerk[segment_index];
}

void SpatialAnalyzer::InvalidateSegmentCache() {
    segment_cache_dirty = true;
}

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
        cached_selected_segment_index == selected_segment_index)
        return;
    
    // キャッシュグリッドをリサイズ・クリア
    cached_segment_grid1.Resize(grid_resolution);
    cached_segment_grid2.Resize(grid_resolution);
    cached_segment_diff.Resize(grid_resolution);
    cached_segment_grid1.Clear();
    cached_segment_grid2.Clear();
    cached_segment_diff.Clear();
    
    // 特徴量に応じたデータソース選択
    SegmentVoxelData* seg_data1 = nullptr;
    SegmentVoxelData* seg_data2 = nullptr;
    
    if (feature_mode == 0) {
        seg_data1 = &segment_presence_voxels1;
        seg_data2 = &segment_presence_voxels2;
    } else if (feature_mode == 1) {
        seg_data1 = &segment_speed_voxels1;
        seg_data2 = &segment_speed_voxels2;
    } else {
        seg_data1 = &segment_jerk_voxels1;
        seg_data2 = &segment_jerk_voxels2;
    }
    
    if (seg_data1->num_segments == 0) {
        segment_cache_dirty = false;
        cached_feature_mode = feature_mode;
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
    cached_selected_segment_index = selected_segment_index;
    cached_selected_segments = selected_segments;
}
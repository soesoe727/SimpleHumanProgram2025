#include "SpatialAnalysis.h"
#include "SimpleHumanGLUT.h" // OpenGL用
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <fstream>  // ファイルI/O用

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

// --- VoxelGrid Implementation ---

void VoxelGrid::Resize(int res) {
    resolution = res;
    data.assign(res * res * res, 0.0f);
}

void VoxelGrid::Clear() {
    std::fill(data.begin(), data.end(), 0.0f);
}

float& VoxelGrid::At(int x, int y, int z) {
    if(x<0 || x>=resolution || y<0 || y>=resolution || z<0 || z>=resolution) {
        static float dummy = 0.0f; return dummy;
    }
    return data[z * resolution * resolution + y * resolution + x];
}

float VoxelGrid::Get(int x, int y, int z) const {
    if (x < 0 || x >= resolution || y < 0 || y >= resolution || z < 0 || z >= resolution) return 0.0f;
    return data[static_cast<std::vector<float, std::allocator<float>>::size_type>(z) * resolution * resolution + y * resolution + x];
}

// ファイルへ保存（基準姿勢情報も含む）
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
    
    // 基準姿勢情報を書き込み
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

// ファイルから読み込み（バージョン対応）
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

// SegmentVoxelDataのファイル保存
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

// SegmentVoxelDataのファイル読み込み
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
    selected_segment_index = -1;  // -1は全体表示
    show_segment_mode = false;
    
    // スライス平面の変換行列を単位行列で初期化
    slice_plane_transform.setIdentity();
    
    // 表示用オイラー角
    slice_rotation_x = 0.0f;
    slice_rotation_y = 0.0f;
    slice_rotation_z = 0.0f;
    
    use_rotated_slice = true;
    
    // スライス位置は1つだけ
    slice_positions.push_back(0.5f);
}

SpatialAnalyzer::~SpatialAnalyzer() {}

void SpatialAnalyzer::ResizeGrids(int res) {
    grid_resolution = res;
    voxels1_psc.Resize(res); voxels2_psc.Resize(res); voxels_psc_diff.Resize(res);
    voxels1_spd.Resize(res); voxels2_spd.Resize(res); voxels_spd_diff.Resize(res);
    voxels1_jrk.Resize(res); voxels2_jrk.Resize(res); voxels_jrk_diff.Resize(res);
    
    // 累積グリッドもリサイズ
    voxels1_psc_accumulated.Resize(res);
    voxels2_psc_accumulated.Resize(res);
    voxels_psc_accumulated_diff.Resize(res);
    
    // 速度累積グリッドもリサイズ
    voxels1_spd_accumulated.Resize(res);
    voxels2_spd_accumulated.Resize(res);
    voxels_spd_accumulated_diff.Resize(res);
    
    // ジャーク累積グリッドもリサイズ
    voxels1_jrk_accumulated.Resize(res);
    voxels2_jrk_accumulated.Resize(res);
    voxels_jrk_accumulated_diff.Resize(res);
}

void SpatialAnalyzer::SetWorldBounds(float bounds[3][2]) {
    for(int i=0; i<3; ++i) {
        world_bounds[i][0] = bounds[i][0];
        world_bounds[i][1] = bounds[i][1];
    }
    
    // スライス平面の中心をワールド中心に設定
    float cx = (world_bounds[0][0] + world_bounds[0][1]) / 2.0f;
    float cy = (world_bounds[1][0] + world_bounds[1][1]) / 2.0f;
    float cz = (world_bounds[2][0] + world_bounds[2][1]) / 2.0f;
    
    // 変換行列の平行移動成分を設定
    slice_plane_transform.setIdentity();
    slice_plane_transform.m03 = cx;
    slice_plane_transform.m13 = cy;
    slice_plane_transform.m23 = cz;
    
    UpdateEulerAnglesFromTransform();
}

void SpatialAnalyzer::VoxelizeMotion(Motion* m, float time, VoxelGrid& occ, VoxelGrid& spd, VoxelGrid& jrk) {
    if (!m) return;
    
    // ヘルパー関数でフレームデータを計算
    FrameData frame_data;
    ComputeFrameData(m, time, frame_data);
    
    // ヘルパー関数で全ボーンのデータを抽出
    vector<BoneData> bones;
    ExtractBoneData(m, frame_data, bones);

    float bone_radius = 0.08f;
    float world_range[3];
    for(int i=0; i<3; ++i) world_range[i] = world_bounds[i][1] - world_bounds[i][0];

    // 各ボーンについてボクセル化
    for (const BoneData& bone : bones) {
        WriteToVoxelGrid(bone, bone_radius, world_range, &occ, &spd, &jrk);
    }
}

void SpatialAnalyzer::UpdateVoxels(Motion* m1, Motion* m2, float current_time) {
    voxels1_psc.Clear(); voxels2_psc.Clear();
    voxels1_spd.Clear(); voxels2_spd.Clear();
	voxels1_jrk.Clear(); voxels2_jrk.Clear();

    VoxelizeMotion(m1, current_time, voxels1_psc, voxels1_spd, voxels1_jrk);
    VoxelizeMotion(m2, current_time, voxels2_psc, voxels2_spd, voxels2_jrk);

    max_psc_val = 0.0f;
	max_spd_val = 0.0f;
	max_jrk_val = 0.0f;
    int size = grid_resolution * grid_resolution * grid_resolution;

    for(int i=0; i<size; ++i) {
        float v1 = voxels1_psc.data[i];
        float v2 = voxels2_psc.data[i];
        float diff = abs(v1 - v2);
        voxels_psc_diff.data[i] = diff;
        if(diff > max_psc_val) max_psc_val = diff;
        
        float s1 = voxels1_spd.data[i];
        float s2 = voxels2_spd.data[i];
        voxels_spd_diff.data[i] = abs(s1 - s2);
		if (abs(s1 - s2) > max_spd_val) max_spd_val = abs(s1 - s2);

		float j1 = voxels1_jrk.data[i];
		float j2 = voxels2_jrk.data[i];
		voxels_jrk_diff.data[i] = abs(j1 - j2);
		if (abs(j1 - j2) > max_jrk_val) max_jrk_val = abs(j1 - j2);
    }
    if (max_psc_val < 1e-5f) max_psc_val = 1.0f;
	if (max_spd_val < 1e-5f) max_spd_val = 1.0f;
	if (max_jrk_val < 1e-5f) max_jrk_val = 1.0f;
}

void SpatialAnalyzer::DrawSlicePlanes() {
    // 回転スライスモードのみ使用
    DrawRotatedSlicePlane();
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

    // 回転スライスモードでは1つのスライス（アクティブなもの）のみ表示
    int y_pos = start_y;
    
    char t1[64], t2[64], t3[64];
    sprintf(t1, "Rotated M1");
    sprintf(t2, "Rotated M2");
    sprintf(t3, "Rotated Diff");
    
    VoxelGrid* grid1 = nullptr;
    VoxelGrid* grid2 = nullptr;
    VoxelGrid* grid_diff = nullptr;
    float max_value = 1.0f;
    
    // 表示するボクセルグリッドを選択
    if (show_segment_mode && selected_segment_index >= 0 && 
        selected_segment_index < segment_presence_voxels1.num_segments) {
        // 部位別表示モード: feature_modeに応じて占有率、速度、ジャークを使用
        if (feature_mode == 0) {
            // 占有率
            grid1 = &segment_presence_voxels1.GetSegmentGrid(selected_segment_index);
            grid2 = &segment_presence_voxels2.GetSegmentGrid(selected_segment_index);
        } else if (feature_mode == 1) {
            // 速度
            grid1 = &segment_speed_voxels1.GetSegmentGrid(selected_segment_index);
            grid2 = &segment_speed_voxels2.GetSegmentGrid(selected_segment_index);
        } else {
            // ジャーク
            grid1 = &segment_jerk_voxels1.GetSegmentGrid(selected_segment_index);
            grid2 = &segment_jerk_voxels2.GetSegmentGrid(selected_segment_index);
        }
        
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
        if (norm_mode == 0) {
            // 瞬間表示
            if (feature_mode == 0) {
                grid1 = &voxels1_psc;
                grid2 = &voxels2_psc;
                grid_diff = &voxels_psc_diff;
                max_value = max_psc_val;
            } else if (feature_mode == 1) {
                grid1 = &voxels1_spd;
                grid2 = &voxels2_spd;
                grid_diff = &voxels_spd_diff;
                max_value = max_spd_val;
            } else {
                grid1 = &voxels1_jrk;
                grid2 = &voxels2_jrk;
                grid_diff = &voxels_jrk_diff;
                max_value = max_jrk_val;
            }
        } else {
            // 累積ボクセルデータ
            if (feature_mode == 0) {
                // 累積占有率
                grid1 = &voxels1_psc_accumulated;
                grid2 = &voxels2_psc_accumulated;
                grid_diff = &voxels_psc_accumulated_diff;
                max_value = max_psc_accumulated_val;
            } else if (feature_mode == 1) {
                // 累積最大速度
                grid1 = &voxels1_spd_accumulated;
                grid2 = &voxels2_spd_accumulated;
                grid_diff = &voxels_spd_accumulated_diff;
                max_value = max_spd_accumulated_val;
            } else {
                // 累積最大ジャーク
                grid1 = &voxels1_jrk_accumulated;
                grid2 = &voxels2_jrk_accumulated;
                grid_diff = &voxels_jrk_accumulated_diff;
                max_value = max_jrk_accumulated_val;
            }
        }
    }
    
    // 回転スライスマップを描画
    DrawRotatedSliceMap(start_x, y_pos, map_w, map_h, *grid1, max_value, t1);
    DrawRotatedSliceMap(start_x + map_w + gap, y_pos, map_w, map_h, *grid2, max_value, t2);
    DrawRotatedSliceMap(start_x + 2*(map_w + gap), y_pos, map_w, map_h, *grid_diff, max_value, t3);
    
    glPopAttrib();
    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW); glPopMatrix();
}

void SpatialAnalyzer::ResetView() { zoom = 1.0f; pan_center.set(0.0f, 0.0f); is_manual_view = false; }
void SpatialAnalyzer::Pan(float dx, float dy) { pan_center.x += dx; pan_center.y += dy; is_manual_view = true; }
void SpatialAnalyzer::Zoom(float factor) { zoom *= factor; is_manual_view = true; }

// 3Dボクセルの描画
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
    int step = grid_resolution / 32;
    if (step < 1) step = 1;
    
    VoxelGrid* grid_to_draw = nullptr;
    float max_val = 1.0f;
    
    // 部位別表示モード
    if (show_segment_mode && selected_segment_index >= 0 && 
        selected_segment_index < segment_presence_voxels1.num_segments) {
        // 特定の部位のみ表示（差分を計算）
        static VoxelGrid temp_seg_diff;
        temp_seg_diff.Resize(grid_resolution);
        temp_seg_diff.Clear();
        
        VoxelGrid* seg1;
        VoxelGrid* seg2;
        
        // feature_modeに応じて占有率、速度、ジャークを使用
        if (feature_mode == 0) {
            seg1 = &segment_presence_voxels1.GetSegmentGrid(selected_segment_index);
            seg2 = &segment_presence_voxels2.GetSegmentGrid(selected_segment_index);
        } else if (feature_mode == 1) {
            seg1 = &segment_speed_voxels1.GetSegmentGrid(selected_segment_index);
            seg2 = &segment_speed_voxels2.GetSegmentGrid(selected_segment_index);
        } else {
            seg1 = &segment_jerk_voxels1.GetSegmentGrid(selected_segment_index);
            seg2 = &segment_jerk_voxels2.GetSegmentGrid(selected_segment_index);
        }
        
        int size = grid_resolution * grid_resolution * grid_resolution;
        max_val = 0.0f;
        for (int i = 0; i < size; ++i) {
            float diff = abs(seg1->data[i] - seg2->data[i]);
            temp_seg_diff.data[i] = diff;
            if (diff > max_val) max_val = diff;
        }
        if (max_val < 1e-5f) max_val = 1.0f;
        
        grid_to_draw = &temp_seg_diff;
        
    } else if (norm_mode == 0) {
        // 現在フレームのみ（瞬間表示）
        if (feature_mode == 0) {
            grid_to_draw = &voxels_psc_diff;
            max_val = max_psc_val;
        } else if (feature_mode == 1) {
            grid_to_draw = &voxels_spd_diff;
            max_val = max_spd_val;
        } else {
            grid_to_draw = &voxels_jrk_diff;
            max_val = max_jrk_val;
        }
    } else {
        // 動作全体の累積データ
        if (feature_mode == 0) {
            // 累積占有率
            grid_to_draw = &voxels_psc_accumulated_diff;
            max_val = max_psc_accumulated_val;
        } else if (feature_mode == 1) {
            // 累積最大速度
            grid_to_draw = &voxels_spd_accumulated_diff;
            max_val = max_spd_accumulated_val;
        } else {
            // 累積最大ジャーク
            grid_to_draw = &voxels_jrk_accumulated_diff;
            max_val = max_jrk_accumulated_val;
        }
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

// 動作全体を通った累積ボクセル計算（占有率と速度とジャークの両方 + 部位ごと）
void SpatialAnalyzer::AccumulateAllFrames(Motion* m1, Motion* m2)
{
    if (!m1 || !m2) return;
    
    // 部位数を取得
    int num_segments = m1->body->num_segments;
    int size = grid_resolution * grid_resolution * grid_resolution;
    
    // === グリッドの初期化 ===
    // 累積グリッドをクリア
    ClearAccumulatedData();
    
    // 全体の累積グリッドをリサイズ
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
    
    // 基準姿勢を保存（初期フレームの腰の位置・回転）
    if (m1->num_frames > 0) {
        voxels1_psc_accumulated.SetReference(m1->frames[0].root_pos, m1->frames[0].root_ori);
    }
    if (m2->num_frames > 0) {
        voxels2_psc_accumulated.SetReference(m2->frames[0].root_pos, m2->frames[0].root_ori);
    }
    
    std::cout << "Accumulating voxels (integrated) for " << num_segments << " segments..." << std::endl;
    
    // === モーション1の全フレームを処理 ===
    for (int f = 0; f < m1->num_frames; ++f) {
        float time = f * m1->interval;
        SegmentVoxelData temp_seg_psc, temp_seg_spd, temp_seg_jrk;
        temp_seg_psc.Resize(num_segments, grid_resolution);
        temp_seg_spd.Resize(num_segments, grid_resolution);
        temp_seg_jrk.Resize(num_segments, grid_resolution);
        
        // 部位ごとにボクセル化
        VoxelizeMotionBySegment(m1, time, temp_seg_psc, temp_seg_spd, temp_seg_jrk);
        
        // 部位ごとに累積 ＋ 全体に加算
        for (int s = 0; s < num_segments; ++s) {
            for (int i = 0; i < size; ++i) {
                float psc_val = temp_seg_psc.segment_grids[s].data[i];
                float spd_val = temp_seg_spd.segment_grids[s].data[i];
                float jrk_val = temp_seg_jrk.segment_grids[s].data[i];
                
                // 部位ごとの占有率累積
                segment_presence_voxels1.segment_grids[s].data[i] += psc_val;
                // 部位ごとの速度（最大値保持）
                if (spd_val > segment_speed_voxels1.segment_grids[s].data[i]) {
                    segment_speed_voxels1.segment_grids[s].data[i] = spd_val;
                }
                // 部位ごとのジャーク（最大値保持）
                if (jrk_val > segment_jerk_voxels1.segment_grids[s].data[i]) {
                    segment_jerk_voxels1.segment_grids[s].data[i] = jrk_val;
                }
                
                // 全体への加算
                voxels1_psc_accumulated.data[i] += psc_val;
                if (spd_val > voxels1_spd_accumulated.data[i]) {
                    voxels1_spd_accumulated.data[i] = spd_val;
                }
                if (jrk_val > voxels1_jrk_accumulated.data[i]) {
                    voxels1_jrk_accumulated.data[i] = jrk_val;
                }
            }
        }
    }
    
    // === モーション2の全フレームを処理 ===
    for (int f = 0; f < m2->num_frames; ++f) {
        float time = f * m2->interval;
        SegmentVoxelData temp_seg_psc, temp_seg_spd, temp_seg_jrk;
        temp_seg_psc.Resize(num_segments, grid_resolution);
        temp_seg_spd.Resize(num_segments, grid_resolution);
        temp_seg_jrk.Resize(num_segments, grid_resolution);
        
        // 部位ごとにボクセル化
        VoxelizeMotionBySegment(m2, time, temp_seg_psc, temp_seg_spd, temp_seg_jrk);
        
        // 部位ごとに累積 ＋ 全体に加算
        for (int s = 0; s < num_segments; ++s) {
            for (int i = 0; i < size; ++i) {
                float psc_val = temp_seg_psc.segment_grids[s].data[i];
                float spd_val = temp_seg_spd.segment_grids[s].data[i];
                float jrk_val = temp_seg_jrk.segment_grids[s].data[i];
                
                // 部位ごとの占有率累積
                segment_presence_voxels2.segment_grids[s].data[i] += psc_val;
                // 部位ごとの速度（最大値保持）
                if (spd_val > segment_speed_voxels2.segment_grids[s].data[i]) {
                    segment_speed_voxels2.segment_grids[s].data[i] = spd_val;
                }
                // 部位ごとのジャーク（最大値保持）
                if (jrk_val > segment_jerk_voxels2.segment_grids[s].data[i]) {
                    segment_jerk_voxels2.segment_grids[s].data[i] = jrk_val;
                }
                
                // 全体への加算
                voxels2_psc_accumulated.data[i] += psc_val;
                if (spd_val > voxels2_spd_accumulated.data[i]) {
                    voxels2_spd_accumulated.data[i] = spd_val;
                }
                if (jrk_val > voxels2_jrk_accumulated.data[i]) {
                    voxels2_jrk_accumulated.data[i] = jrk_val;
                }
            }
        }
    }
    
    // === 差分計算と最大値更新 ===
    for (int i = 0; i < size; ++i) {
        // 占有率差分
        float diff_psc = abs(voxels1_psc_accumulated.data[i] - voxels2_psc_accumulated.data[i]);
        voxels_psc_accumulated_diff.data[i] = diff_psc;
        if (diff_psc > max_psc_accumulated_val) {
            max_psc_accumulated_val = diff_psc;
        }
        
        // 速度差分
        float diff_spd = abs(voxels1_spd_accumulated.data[i] - voxels2_spd_accumulated.data[i]);
        voxels_spd_accumulated_diff.data[i] = diff_spd;
        float max_spd = max(voxels1_spd_accumulated.data[i], voxels2_spd_accumulated.data[i]);
        if (max_spd > max_spd_accumulated_val) {
            max_spd_accumulated_val = max_spd;
        }

        // ジャーク差分
        float diff_jrk = abs(voxels1_jrk_accumulated.data[i] - voxels2_jrk_accumulated.data[i]);
        voxels_jrk_accumulated_diff.data[i] = diff_jrk;
        float max_jrk = max(voxels1_jrk_accumulated.data[i], voxels2_jrk_accumulated.data[i]);
        if (max_jrk > max_jrk_accumulated_val) {
            max_jrk_accumulated_val = max_jrk;
        }
    }
    
    if (max_psc_accumulated_val < 1e-5f) max_psc_accumulated_val = 1.0f;
    if (max_spd_accumulated_val < 1e-5f) max_spd_accumulated_val = 1.0f;
    if (max_jrk_accumulated_val < 1e-5f) max_jrk_accumulated_val = 1.0f;
    
    // 差分グリッドにも基準姿勢を設定（モーション1の基準を使用）
    if (voxels1_psc_accumulated.has_reference) {
        voxels_psc_accumulated_diff.SetReference(
            voxels1_psc_accumulated.reference_root_pos, 
            voxels1_psc_accumulated.reference_root_ori);
    }
    
    std::cout << "Integrated accumulation complete." << std::endl;
    std::cout << "  Max presence diff: " << max_psc_accumulated_val << std::endl;
    std::cout << "  Max speed: " << max_spd_accumulated_val << std::endl;
    std::cout << "  Max jerk: " << max_jrk_accumulated_val << std::endl;
}

// 占有率・速度・ジャーク累積グリッドのクリア
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
    if (!m) return;
    
    // ヘルパー関数でフレームデータを計算
    FrameData frame_data;
    ComputeFrameData(m, time, frame_data);
    
    // ヘルパー関数で全ボーンのデータを抽出
    vector<BoneData> bones;
    ExtractBoneData(m, frame_data, bones);

    float bone_radius = 0.08f;
    float world_range[3];

    for(int i=0; i<3; ++i) 
        world_range[i] = world_bounds[i][1] - world_bounds[i][0];

    // 各ボーンについて、対応するセグメントのグリッドに書き込み
    for (const BoneData& bone : bones) {
        if (!bone.valid) continue;
        
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
    std::string base_filename = GenerateCacheFilename(motion1_name, motion2_name);
    
    std::cout << "Saving voxel cache to " << base_filename << "..." << std::endl;
    
    // 累積ボクセルデータを保存
    std::string accumulated1_file = base_filename + "_acc1.bin";
    std::string accumulated2_file = base_filename + "_acc2.bin";
    std::string accumulated_diff_file = base_filename + "_acc_diff.bin";
    
    if (!voxels1_psc_accumulated.SaveToFile(accumulated1_file.c_str())) return false;
    if (!voxels2_psc_accumulated.SaveToFile(accumulated2_file.c_str())) return false;
    if (!voxels_psc_accumulated_diff.SaveToFile(accumulated_diff_file.c_str())) return false;
    
    // 速度累積ボクセルデータを保存
    std::string spd_acc1_file = base_filename + "_spd_acc1.bin";
    std::string spd_acc2_file = base_filename + "_spd_acc2.bin";
    std::string spd_acc_diff_file = base_filename + "_spd_acc_diff.bin";
    
    if (!voxels1_spd_accumulated.SaveToFile(spd_acc1_file.c_str())) return false;
    if (!voxels2_spd_accumulated.SaveToFile(spd_acc2_file.c_str())) return false;
    if (!voxels_spd_accumulated_diff.SaveToFile(spd_acc_diff_file.c_str())) return false;
    
    // ジャーク累積ボクセルデータを保存
    std::string jrk_acc1_file = base_filename + "_jrk_acc1.bin";
    std::string jrk_acc2_file = base_filename + "_jrk_acc2.bin";
    std::string jrk_acc_diff_file = base_filename + "_jrk_acc_diff.bin";
    
    if (!voxels1_jrk_accumulated.SaveToFile(jrk_acc1_file.c_str())) return false;
    if (!voxels2_jrk_accumulated.SaveToFile(jrk_acc2_file.c_str())) return false;
    if (!voxels_jrk_accumulated_diff.SaveToFile(jrk_acc_diff_file.c_str())) return false;
    
    // 部位ごとのボクセルデータを保存
    std::string segment1_file = base_filename + "_seg1.bin";
    std::string segment2_file = base_filename + "_seg2.bin";
    
    if (!segment_presence_voxels1.SaveToFile(segment1_file.c_str())) return false;
    if (!segment_presence_voxels2.SaveToFile(segment2_file.c_str())) return false;
    
    // 部位ごとの速度ボクセルデータを保存
    std::string seg_spd1_file = base_filename + "_seg_spd1.bin";
    std::string seg_spd2_file = base_filename + "_seg_spd2.bin";
    
    if (!segment_speed_voxels1.SaveToFile(seg_spd1_file.c_str())) return false;
    if (!segment_speed_voxels2.SaveToFile(seg_spd2_file.c_str())) return false;
    
    // 部位ごとのジャークボクセルデータを保存
    std::string seg_jrk1_file = base_filename + "_seg_jrk1.bin";
    std::string seg_jrk2_file = base_filename + "_seg_jrk2.bin";
    
    if (!segment_jerk_voxels1.SaveToFile(seg_jrk1_file.c_str())) return false;
    if (!segment_jerk_voxels2.SaveToFile(seg_jrk2_file.c_str())) return false;
    
    // メタデータを保存
    std::string meta_file = base_filename + "_meta.txt";
    ofstream meta_ofs(meta_file);
    if (!meta_ofs) return false;
    
    meta_ofs << grid_resolution << std::endl;
    meta_ofs << max_psc_accumulated_val << std::endl;
    meta_ofs << max_spd_accumulated_val << std::endl;
    meta_ofs << max_jrk_accumulated_val << std::endl;
    for (int i = 0; i < 3; ++i) {
        meta_ofs << world_bounds[i][0] << " " << world_bounds[i][1] << std::endl;
    }
    meta_ofs.close();
    
    std::cout << "Voxel cache saved successfully!" << std::endl;
    return true;
}

// ボクセルキャッシュを読み込み
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
    meta_ifs >> max_psc_accumulated_val;
    meta_ifs >> max_spd_accumulated_val;
    meta_ifs >> max_jrk_accumulated_val;
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
    
    if (!voxels1_psc_accumulated.LoadFromFile(accumulated1_file.c_str())) {
        std::cout << "Failed to load: " << accumulated1_file << std::endl;
        return false;
    }
    if (!voxels2_psc_accumulated.LoadFromFile(accumulated2_file.c_str())) {
        std::cout << "Failed to load: " << accumulated2_file << std::endl;
        return false;
    }
    if (!voxels_psc_accumulated_diff.LoadFromFile(accumulated_diff_file.c_str())) {
        std::cout << "Failed to load: " << accumulated_diff_file << std::endl;
        return false;
    }
    
    // 速度累積ボクセルデータを読み込み
    std::string spd_acc1_file = base_filename + "_spd_acc1.bin";
    std::string spd_acc2_file = base_filename + "_spd_acc2.bin";
    std::string spd_acc_diff_file = base_filename + "_spd_acc_diff.bin";
    
    if (!voxels1_spd_accumulated.LoadFromFile(spd_acc1_file.c_str())) {
        std::cout << "Failed to load: " << spd_acc1_file << std::endl;
        return false;
    }
    if (!voxels2_spd_accumulated.LoadFromFile(spd_acc2_file.c_str())) {
        std::cout << "Failed to load: " << spd_acc2_file << std::endl;
        return false;
    }
    if (!voxels_spd_accumulated_diff.LoadFromFile(spd_acc_diff_file.c_str())) {
        std::cout << "Failed to load: " << spd_acc_diff_file << std::endl;
        return false;
    }
    
    // ジャーク累積ボクセルデータを読み込み
    std::string jrk_acc1_file = base_filename + "_jrk_acc1.bin";
    std::string jrk_acc2_file = base_filename + "_jrk_acc2.bin";
    std::string jrk_acc_diff_file = base_filename + "_jrk_acc_diff.bin";
    
    if (!voxels1_jrk_accumulated.LoadFromFile(jrk_acc1_file.c_str())) {
        std::cout << "Failed to load: " << jrk_acc1_file << std::endl;
        return false;
    }
    if (!voxels2_jrk_accumulated.LoadFromFile(jrk_acc2_file.c_str())) {
        std::cout << "Failed to load: " << jrk_acc2_file << std::endl;
        return false;
    }
    if (!voxels_jrk_accumulated_diff.LoadFromFile(jrk_acc_diff_file.c_str())) {
        std::cout << "Failed to load: " << jrk_acc_diff_file << std::endl;
        return false;
    }
    
    // 部位ごとのボクセルデータを読み込み
    std::string segment1_file = base_filename + "_seg1.bin";
    std::string segment2_file = base_filename + "_seg2.bin";
    
    if (!segment_presence_voxels1.LoadFromFile(segment1_file.c_str())) {
        std::cout << "Failed to load: " << segment1_file << std::endl;
        return false;
    }
    if (!segment_presence_voxels2.LoadFromFile(segment2_file.c_str())) {
        std::cout << "Failed to load: " << segment2_file << std::endl;
        return false;
    }
    
    // 部位ごとの速度ボクセルデータを読み込み
    std::string seg_spd1_file = base_filename + "_seg_spd1.bin";
    std::string seg_spd2_file = base_filename + "_seg_spd2.bin";
    
    if (!segment_speed_voxels1.LoadFromFile(seg_spd1_file.c_str())) {
        std::cout << "Failed to load: " << seg_spd1_file << std::endl;
        return false;
    }
    if (!segment_speed_voxels2.LoadFromFile(seg_spd2_file.c_str())) {
        std::cout << "Failed to load: " << seg_spd2_file << std::endl;
        return false;
    }
    
    // 部位ごとのジャークボクセルデータを読み込み
    std::string seg_jrk1_file = base_filename + "_seg_jrk1.bin";
    std::string seg_jrk2_file = base_filename + "_seg_jrk2.bin";
    
    if (!segment_jerk_voxels1.LoadFromFile(seg_jrk1_file.c_str())) {
        std::cout << "Failed to load: " << seg_jrk1_file << std::endl;
        return false;
    }
    if (!segment_jerk_voxels2.LoadFromFile(seg_jrk2_file.c_str())) {
        std::cout << "Failed to load: " << seg_jrk2_file << std::endl;
        return false;
    }
    
    std::cout << "Voxel cache loaded successfully!" << std::endl;
    return true;
}

// スライス平面の回転操作（キーボード用）
void SpatialAnalyzer::RotateSlicePlane(float dx, float dy, float dz) {
    // 角度をラジアンに変換
    float rx = dx * 3.14159265f / 180.0f;
    float ry = dy * 3.14159265f / 180.0f;
    float rz = dz * 3.14159265f / 180.0f;
    
    // 各軸周りの回転行列を作成して適用
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
    // ワールド中心を計算
    float cx = (world_bounds[0][0] + world_bounds[0][1]) / 2.0f;
    float cy = (world_bounds[1][0] + world_bounds[1][1]) / 2.0f;
    float cz = (world_bounds[2][0] + world_bounds[2][1]) / 2.0f;
    
    // 変換行列を単位行列にリセット（中心位置のみ保持）
    slice_plane_transform.setIdentity();
    slice_plane_transform.m03 = cx;
    slice_plane_transform.m13 = cy;
    slice_plane_transform.m23 = cz;
    
    slice_rotation_x = 0.0f;
    slice_rotation_y = 0.0f;
    slice_rotation_z = 0.0f;
    
    // パンもリセット
    pan_center.set(0.0f, 0.0f);
}

void SpatialAnalyzer::ToggleRotatedSliceMode() {
    use_rotated_slice = !use_rotated_slice;
    if (use_rotated_slice) {
        // 回転スライスモードをオンにした時、リセット
        ResetSliceRotation();
    }
    std::cout << "Rotated slice mode: " << (use_rotated_slice ? "ON" : "OFF") << std::endl;
}

// 回転スライス平面の描画
void SpatialAnalyzer::DrawRotatedSlicePlane() {
    if (!use_rotated_slice) return;
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    float world_range[3];
    for (int i = 0; i < 3; ++i) {
        world_range[i] = world_bounds[i][1] - world_bounds[i][0];
    }
    float max_range = max(world_range[0], max(world_range[1], world_range[2]));
    float half_size = max_range * 0.6f * zoom;

    // アクセサを使用してベクトルを取得
    Point3f center = GetSlicePlaneCenter();
    Vector3f slice_u = GetSlicePlaneU();
    Vector3f slice_v = GetSlicePlaneV();
    Vector3f slice_n = GetSlicePlaneNormal();

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
        
        // 枠線
        glColor4f(1.0f, 1.0f, 0.0f, 0.9f);
        glLineWidth(5.0f);

        glBegin(GL_LINE_LOOP);
        for (int k = 0; k < 4; ++k) glVertex3f(p[k].x, p[k].y, p[k].z);
        glEnd();   
    }
    glDisable(GL_BLEND);
}

// 回転スライス用の2Dマップ描画
void SpatialAnalyzer::DrawRotatedSliceMap(int x_pos, int y_pos, int w, int h, VoxelGrid& grid, float max_val, const char* title) {
    glColor4f(0.9f, 0.9f, 0.9f, 1.0f);
    glLineWidth(2.0f);
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
    
    float world_range[3];
    for (int i = 0; i < 3; ++i) {
        world_range[i] = world_bounds[i][1] - world_bounds[i][0];
    }
    float max_range = max(world_range[0], max(world_range[1], world_range[2]));
    float half_size = max_range * 0.6f * zoom;

    // アクセサを使用してベクトルを取得
    Point3f center = GetSlicePlaneCenter();
    Vector3f slice_u = GetSlicePlaneU();
    Vector3f slice_v = GetSlicePlaneV();

    // パン操作を適用
    center.x += slice_u.x * pan_center.x + slice_v.x * pan_center.y;
    center.y += slice_u.y * pan_center.x + slice_v.y * pan_center.y;
    center.z += slice_u.z * pan_center.x + slice_v.z * pan_center.y;
    
    int draw_res = 64;
    float cell_w = (float)w / draw_res;
    float cell_h = (float)h / draw_res;
    
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
    
    // タイトルを描画
    glColor3f(0,0,0);
    glRasterPos2i(x_pos, y_pos - 5);
    for (const char* c = title; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
    
    char rot_info[128];
    sprintf(rot_info, "Rot: X=%.0f Y=%.0f Z=%.0f", slice_rotation_x, slice_rotation_y, slice_rotation_z);
    glRasterPos2i(x_pos, y_pos + h + 22);
    for (const char* c = rot_info; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
}

// ワールド座標でボクセル値をサンプリング
float SpatialAnalyzer::SampleVoxelAtWorldPos(VoxelGrid& grid, const Point3f& world_pos) {
    float world_range[3];
    for (int i = 0; i < 3; ++i) {
        world_range[i] = world_bounds[i][1] - world_bounds[i][0];
    }
    
    // ワールド座標をボクセルインデックスに変換
    int gx = (int)(((world_pos.x - world_bounds[0][0]) / world_range[0]) * grid_resolution);
    int gy = (int)(((world_pos.y - world_bounds[1][0]) / world_range[1]) * grid_resolution);
    int gz = (int)(((world_pos.z - world_bounds[2][0]) / world_range[2]) * grid_resolution);
    
    return grid.Get(gx, gy, gz);
}

// === スライス平面の変換行列ベース操作 ===

void SpatialAnalyzer::SetSlicePlaneTransform(const Matrix4f& transform) {
    slice_plane_transform = transform;
    UpdateEulerAnglesFromTransform();
}

Matrix4f SpatialAnalyzer::GetSlicePlaneTransform() const {
    return slice_plane_transform;
}

Point3f SpatialAnalyzer::GetSlicePlaneCenter() const {
    return Point3f(slice_plane_transform.m03, 
                   slice_plane_transform.m13, 
                   slice_plane_transform.m23);
}

Vector3f SpatialAnalyzer::GetSlicePlaneU() const {
    return Vector3f(slice_plane_transform.m00, 
                    slice_plane_transform.m10, 
                    slice_plane_transform.m20);
}

Vector3f SpatialAnalyzer::GetSlicePlaneV() const {
    return Vector3f(slice_plane_transform.m01, 
                    slice_plane_transform.m11, 
                    slice_plane_transform.m21);
}

Vector3f SpatialAnalyzer::GetSlicePlaneNormal() const {
    return Vector3f(slice_plane_transform.m02, 
                    slice_plane_transform.m12, 
                    slice_plane_transform.m22);
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
    
    // ローカル座標系での回転を適用: new_rot = current_rot * local_rotation
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
    // 変換行列からオイラー角を逆算（表示用）
    Vector3f n = GetSlicePlaneNormal();
    Vector3f u = GetSlicePlaneU();
    Vector3f v = GetSlicePlaneV();
    
    // 法線ベクトルからX軸回転とY軸回転を計算
    float ny = n.y;
    if (ny > 1.0f) ny = 1.0f;
    if (ny < -1.0f) ny = -1.0f;
    float nxz = sqrtf(n.x * n.x + n.z * n.z);
    
    slice_rotation_x = atan2f(-ny, nxz) * 180.0f / 3.14159265f;
    slice_rotation_y = atan2f(n.x, n.z) * 180.0f / 3.14159265f;
    slice_rotation_z = atan2f(u.y, v.y) * 180.0f / 3.14159265f;
}

// === 共通ボクセル化ヘルパー関数 ===
// フレームデータを計算（3フレーム分を取得）
void SpatialAnalyzer::ComputeFrameData(Motion* m, float time, FrameData& frame_data) {
    if (!m) return;
    
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
}

// 全ボーンのデータを抽出（加速度計算を追加）
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
            bone.p1 = Point3f(frame_data.curr_frames[s].m03, 
                              frame_data.curr_frames[s].m13, 
                              frame_data.curr_frames[s].m23);
            bone.p1_prev = Point3f(frame_data.prev_frames[s].m03, 
                                   frame_data.prev_frames[s].m13, 
                                   frame_data.prev_frames[s].m23);
            bone.p1_prev2 = Point3f(frame_data.prev2_frames[s].m03, 
                                    frame_data.prev2_frames[s].m13, 
			                    	frame_data.prev2_frames[s].m23);
            bone.p1_prev3 = Point3f(frame_data.prev3_frames[s].m03, 
			                    	frame_data.prev3_frames[s].m13,
			                    	frame_data.prev3_frames[s].m23);
            
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
        
        if (bone.valid) {
            // 速度を計算
            Vector3f spd1_curr = bone.p1 - bone.p1_prev;
            Vector3f spd2_curr = bone.p2 - bone.p2_prev;
            bone.speed1 = spd1_curr.length() / frame_data.dt;
            bone.speed2 = spd2_curr.length() / frame_data.dt;
            
            // 加速度を計算（前フレームの速度との差）
            Vector3f spd1_prev = bone.p1_prev - bone.p1_prev2;
            Vector3f spd2_prev = bone.p2_prev - bone.p2_prev2;
			Vector3f spd1_prev2 = bone.p1_prev2 - bone.p1_prev3;
			Vector3f spd2_prev2 = bone.p2_prev2 - bone.p2_prev3;
            
            Vector3f accel1_curr = spd1_curr - spd1_prev;
            Vector3f accel2_curr = spd2_curr - spd2_prev;
            
            bone.accel1 = accel1_curr.length() / frame_data.dt;  // magnitude of acceleration
            bone.accel2 = accel2_curr.length() / frame_data.dt;

			// ジャークを計算（加速度の時間微分）
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

// AABB計算（共通化）
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

// ボクセルグリッドへの書き込み（占有率、速度、ジャークを同時に処理）
void SpatialAnalyzer::WriteToVoxelGrid(const BoneData& bone, float bone_radius, const float world_range[3],
                                        VoxelGrid* occ_grid, VoxelGrid* spd_grid, VoxelGrid* jrk_grid) {
    if (!bone.valid) return;
    
    int idx_min[3], idx_max[3];
    ComputeAABB(bone.p1, bone.p2, bone_radius, idx_min, idx_max, world_range);
    
    Point3f bone_vec = bone.p2 - bone.p1;
    float bone_len_sq = bone_vec.x*bone_vec.x + bone_vec.y*bone_vec.y + bone_vec.z*bone_vec.z;
    if (bone_len_sq < 1e-6f) return;
    
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
                
                float dist_sq = (voxel_center.x-closest.x)*(voxel_center.x-closest.x) + 
                                (voxel_center.y-closest.y)*(voxel_center.y-closest.y) + 
                                (voxel_center.z-closest.z)*(voxel_center.z-closest.z);

                if (dist_sq < radius_sq) {
                    float presence = exp(-dist_sq / sigma_sq);
                    
                    if (occ_grid) {
                        occ_grid->At(x, y, z) += presence;
                    }
                    
                    if (spd_grid) {
                        float s_interp = (1.0f - k_clamped) * bone.speed1 + k_clamped * bone.speed2;
                        float& current_spd = spd_grid->At(x, y, z);
                        if (s_interp > current_spd) current_spd = s_interp;
                    }
                    
                    if (jrk_grid) {
                        // ジャークは加速度の時間微分（速度の2次微分）
                        // 現在フレームの加速度を補間して格納（最大値保持）
                        float j_interp = (1.0f - k_clamped) * bone.jerk1 + k_clamped * bone.jerk2;
                        float& current_jrk = jrk_grid->At(x, y, z);
                        if (j_interp > current_jrk) current_jrk = j_interp;
                    }
                }
            }
        }
    }
}
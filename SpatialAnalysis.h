#pragma once
#include <vector>
#include <cmath>
#include <string>
#include <Point3.h>
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"

// 2D point structure for spatial analysis
struct SpatialPoint2f {
    float x, y;
    SpatialPoint2f() : x(0.0f), y(0.0f) {}
    SpatialPoint2f(float _x, float _y) : x(_x), y(_y) {}
    void set(float _x, float _y) { x = _x; y = _y; }
};


struct VoxelGrid {
    int resolution;
    std::vector<float> data; 
    
    // NEW: 基準姿勢情報（腰の位置・回転）
    Point3f reference_root_pos;
    Matrix3f reference_root_ori;
    bool has_reference;
    
    VoxelGrid() : resolution(0), has_reference(false) {
        reference_root_pos.set(0, 0, 0);
        reference_root_ori.setIdentity();
    }
    
    void Resize(int res);
    void Clear();
    float& At(int x, int y, int z);
    float Get(int x, int y, int z) const;
    
    // NEW: 基準姿勢の設定・取得
    void SetReference(const Point3f& root_pos, const Matrix3f& root_ori) {
        reference_root_pos = root_pos;
        reference_root_ori = root_ori;
        has_reference = true;
    }
    
    void GetReference(Point3f& root_pos, Matrix3f& root_ori) const {
        root_pos = reference_root_pos;
        root_ori = reference_root_ori;
    }
    
    // ファイル保存・読み込み
    bool SaveToFile(const char* filename) const;
    bool LoadFromFile(const char* filename);
};

// NEW: 部位ごとのボクセルグリッドコレクション
struct SegmentVoxelData {
    std::vector<VoxelGrid> segment_grids;
    int num_segments;
    
    SegmentVoxelData() : num_segments(0) {}
    
    void Resize(int num_seg, int resolution) {
        num_segments = num_seg;
        segment_grids.resize(num_seg);
        for (int i = 0; i < num_seg; ++i) {
            segment_grids[i].Resize(resolution);
        }
    }
    
    void Clear() {
        for (auto& grid : segment_grids) {
            grid.Clear();
        }
    }
    
    VoxelGrid& GetSegmentGrid(int segment_index) {
        if (segment_index >= 0 && segment_index < num_segments) {
            return segment_grids[segment_index];
        }
        static VoxelGrid dummy;
        return dummy;
    }
    
    // NEW: ファイル保存・読み込み
    bool SaveToFile(const char* filename) const;
    bool LoadFromFile(const char* filename);
};

class SpatialAnalyzer {
public:

    int grid_resolution;
    float world_bounds[3][2];
    
    VoxelGrid voxels1_occ, voxels2_occ, voxels_diff;
    VoxelGrid voxels1_spd, voxels2_spd, voxels_spd_diff;
    
    // NEW: 累積ボクセル（動作全体を通した占有率）
    VoxelGrid voxels1_accumulated;
    VoxelGrid voxels2_accumulated;
    VoxelGrid voxels_accumulated_diff;
    
    float max_occ_val;
    float max_spd_val;
    float global_max_spd;
    
    // NEW: 累積用の最大値
    float max_accumulated_val;

    // --- ビュー/スライス設定 ---
    int h_axis, v_axis;
    bool projection_mode;
    std::vector<float> slice_positions;
    int active_slice_index;

    // --- インタラクティブ操作用 ---
    float zoom;
    SpatialPoint2f pan_center;
    bool is_manual_view;

    // --- 表示フラグ ---
    bool show_planes;
    bool show_maps;
    bool show_voxels;  // NEW: 3Dボクセル表示フラグ
    int feature_mode; 
    int norm_mode;    

    // NEW: 部位ごとのボクセルデータ
    SegmentVoxelData segment_voxels1;
    SegmentVoxelData segment_voxels2;
    
    // NEW: 表示設定
    int selected_segment_index;  // 表示する部位のインデックス (-1で全体)
    bool show_segment_mode;      // 部位別表示モードON/OFF

public:
    SpatialAnalyzer();
    ~SpatialAnalyzer();

    void ResizeGrids(int res);
    void SetWorldBounds(float bounds[3][2]);
    
    // ボクセル計算
    void UpdateVoxels(Motion* m1, Motion* m2, float current_time);
    
    // 累積ボクセル計算
    void AccumulateVoxelsAllFrames(Motion* m1, Motion* m2);
    void ClearAccumulatedVoxels();

    // 部位ごとのボクセル計算
    void VoxelizeMotionBySegment(Motion* m, float time, SegmentVoxelData& seg_data);
    void AccumulateVoxelsBySegmentAllFrames(Motion* m1, Motion* m2);
    
    // NEW: ボクセルキャッシュ（ファイル保存・読み込み）
    bool SaveVoxelCache(const char* motion1_name, const char* motion2_name);
    bool LoadVoxelCache(const char* motion1_name, const char* motion2_name);
    std::string GenerateCacheFilename(const char* motion1_name, const char* motion2_name) const;

    // 描画関連
    void DrawSlicePlanes();
    void DrawCTMaps(int win_width, int win_height);
    void DrawVoxels3D();

    // 操作関連
    void ResetView();
    void Pan(float dx, float dy);
    void Zoom(float factor);

    // ヘルパー
    void CalculateViewBounds(float& h_min, float& h_max, float& v_min, float& v_max);
    
private:
    void VoxelizeMotion(Motion* m, float time, VoxelGrid& occ, VoxelGrid& spd);
    void DrawSingleMap(int x, int y, int w, int h, VoxelGrid& grid, float max_val, const char* title, float slice_val, float h_min, float h_max, float v_min, float v_max);
    void DrawAxes(int x, int y, int w, int h, float h_min, float h_max, float v_min, float v_max, const char* h_lbl, const char* v_lbl);
};
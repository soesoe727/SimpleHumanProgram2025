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
	std::vector<float> data; //data数=resolution^3
    
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
	std::vector<VoxelGrid> segment_grids; // 各部位のボクセルグリッド
	int num_segments; // 部位数
    
    SegmentVoxelData() : num_segments(0) {}
    
	// 部位数と各グリッドの解像度を設定
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
    
	// 指定部位のボクセルグリッドを取得
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

	int grid_resolution; // ボクセルグリッド解像度

    float world_bounds[3][2];//ワールド座標系の表示・ボクセル化対象領域を表す3軸分の最小値/最大値
    
	// 瞬間表示用占有率ボクセルデータ
    VoxelGrid voxels1_psc, voxels2_psc, voxels_psc_diff;

    // 瞬間表示用速度ボクセルデータ
    VoxelGrid voxels1_spd, voxels2_spd, voxels_spd_diff;
    
    // 占有率累積ボクセル（動作全体を通した占有率累積値）
    VoxelGrid voxels1_psc_accumulated, voxels2_psc_accumulated, voxels_psc_accumulated_diff;

    // 速度累計ボクセル（動作全体を通した最大速度）
    VoxelGrid voxels1_spd_accumulated, voxels2_spd_accumulated, voxels_spd_accumulated_diff;
    
	float max_psc_val; // 全体の最大占有率
	float max_spd_val; // 全体の最大速度
    
    float max_psc_accumulated_val; // 占有率累積用の最大値
    float max_spd_accumulated_val; // 速度累計用の最大値

    // --- ビュー/スライス設定 ---
	int h_axis, v_axis; // 水平・垂直軸 (0:X, 1:Y, 2:Z)
	bool projection_mode; // 投影モードフラグ
	std::vector<float> slice_positions; // スライス位置 (0.0 - 1.0)
	int active_slice_index; // アクティブなスライスインデックス
    
    // NEW: スライス平面の回転パラメータ
    float slice_rotation_x;  // X軸周りの回転角度（度）
    float slice_rotation_y;  // Y軸周りの回転角度（度）
    float slice_rotation_z;  // Z軸周りの回転角度（度）
    Point3f slice_plane_center;  // 回転中心
    Point3f slice_plane_normal;  // 平面の法線ベクトル
    Point3f slice_plane_u;       // 平面上のU方向ベクトル
    Point3f slice_plane_v;       // 平面上のV方向ベクトル
    bool use_rotated_slice;      // 回転スライスモードを使用するか

    // --- インタラクティブ操作用 ---
	float zoom; // ズーム倍率
	SpatialPoint2f pan_center; // パン操作中心点
	bool is_manual_view; // 手動ビュー操作フラグ

    // --- 表示フラグ ---
	bool show_planes; // スライス平面表示フラグ
	bool show_maps; // 2Dマップ表示フラグ
    bool show_voxels;  // 3Dボクセル表示フラグ
	int feature_mode; // 0: 占有率, 1: 速度
	int norm_mode; // 0: 瞬間表示, 1: 累積表示

    // 部位ごとのボクセルデータ
	SegmentVoxelData segment_presence_voxels1; // Motion1 部位ごとの占有率ボクセルデータ
	SegmentVoxelData segment_presence_voxels2; // Motion2 部位ごとの占有率ボクセルデータ
	SegmentVoxelData segment_speed_voxels1; // Motion1 部位ごとの速度ボクセルデータ
	SegmentVoxelData segment_speed_voxels2; // Motion2 部位ごとの速度ボクセルデータ
    
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
    
    // 累積占有率ボクセル計算
    void AccumulateVoxelsAllFrames(Motion* m1, Motion* m2);
    void ClearAccumulatedVoxels();

    // 速度累計ボクセル計算
    void AccumulateSpeedAllFrames(Motion* m1, Motion* m2);
    void ClearAccumulatedSpeed();

    // 部位ごとの占有率ボクセル計算
    void VoxelizeMotionBySegment(Motion* m, float time, SegmentVoxelData& seg_data);
    void AccumulateVoxelsBySegmentAllFrames(Motion* m1, Motion* m2);
    
    // 部位ごとの速度ボクセル計算
    void VoxelizeMotionSpeedBySegment(Motion* m, float time, SegmentVoxelData& seg_speed_data);
    void AccumulateSpeedBySegmentAllFrames(Motion* m1, Motion* m2);
    
    // ボクセルキャッシュ（ファイル保存・読み込み）
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
    
    // スライス平面の回転操作
    void RotateSlicePlane(float dx, float dy, float dz);
	void ResetSliceRotation(); // 回転をリセット
	void SetSliceRotation(float rx, float ry, float rz); // 回転を設定
    void UpdateSlicePlaneVectors();  // 回転後の平面ベクトルを更新
    void ToggleRotatedSliceMode();   // 回転スライスモードの切り替え

    // ヘルパー
    void CalculateViewBounds(float& h_min, float& h_max, float& v_min, float& v_max);
    
private:
	// ボクセル化ヘルパー
	void VoxelizeMotion(Motion* m, float time, VoxelGrid& occ, VoxelGrid& spd);
    void DrawSingleMap(int x, int y, int w, int h, VoxelGrid& grid, float max_val, const char* title, float slice_val, float h_min, float h_max, float v_min, float v_max);
    void DrawAxes(int x, int y, int w, int h, float h_min, float h_max, float v_min, float v_max, const char* h_lbl, const char* v_lbl);
    
    // 回転スライス用のヘルパー
    void DrawRotatedSlicePlane();
    void DrawRotatedSliceMap(int x, int y, int w, int h, VoxelGrid& grid, float max_val, const char* title);
    float SampleVoxelAtWorldPos(VoxelGrid& grid, const Point3f& world_pos);
};
#pragma once
#include <vector>
#include <cmath>
#include <string>
#include <Point3.h>
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"
#include "VoxelData.h"

static const int SA_FEATURE_COUNT = 5;

// 2D point structure for spatial analysis
struct SpatialPoint2f {
    float x, y;
    SpatialPoint2f() : x(0.0f), y(0.0f) {}
    SpatialPoint2f(float _x, float _y) : x(_x), y(_y) {}
    void set(float _x, float _y) { x = _x; y = _y; }
};

// ボーン情報を格納する構造体（ボクセル化処理の共通化用）
struct BoneData {
    Point3f p1, p2;           // 現在フレームの両端点
    Point3f p1_prev, p2_prev; // 前フレームの両端点
    Point3f p1_prev2, p2_prev2; // 2フレーム前の両端点（加速度計算用）
	Point3f p1_prev3, p2_prev3; // 3フレーム前の両端点（ジャーク計算用）
	float speed1, speed2;     // 両端の速度
	float jerk1, jerk2;       // 両端のジャーク
	float inertia1, inertia2; // 両端の慣性モーメント（ルートからの距離の2乗）
	int segment_index;        // セグメントインデックス
	bool valid;               // 有効なボーンかどうか

	BoneData() : speed1(0), speed2(0), jerk1(0), jerk2(0), inertia1(0), inertia2(0), segment_index(-1), valid(false) {}
};

// フレームデータを格納する構造体（FK計算結果の共通化用）
struct FrameData {  
    std::vector<Matrix4f> curr_frames;     // 現在フレームの変換行列
    std::vector<Matrix4f> prev_frames;     // 前フレームの変換行列
    std::vector<Matrix4f> prev2_frames;    // 2フレーム前の変換行列（加速度計算用）
	std::vector<Matrix4f> prev3_frames;    // 3フレーム前の変換行列（ジャーク計算用）
    Point3f curr_root_pos;  // 現在フレームのルート位置
    std::vector<Point3f> curr_joint_pos;   // 現在フレームの関節位置
    std::vector<Point3f> prev_joint_pos;   // 前フレームの関節位置
    std::vector<Point3f> prev2_joint_pos;  // 2フレーム前の関節位置（加速度計算用）
	std::vector<Point3f> prev3_joint_pos;  // 3フレーム前の関節位置（ジャーク計算用）
    float dt;                              // フレーム間隔
    Point3f prev_root_pos;                 // 前フレームのルート位置
    
	FrameData() : dt(0) {}
};

class SpatialAnalyzer {
public:

	int grid_resolution; // ボクセルグリッド解像度

    float world_bounds[3][2];//ワールド座標系の表示・ボクセル化対象領域を表す3軸分の最小値/最大値
    
	// 瞬間表示用のボクセルデータ（必要に応じて追加）
	VoxelGrid voxels1[SA_FEATURE_COUNT], voxels2[SA_FEATURE_COUNT], voxels_diff[SA_FEATURE_COUNT]; // 0:占有率, 1:速度, 2:ジャーク, 3:慣性モーメント, 4:慣性主軸角速度

	// 累積ボクセルデータ（必要に応じて追加）
	VoxelGrid voxels1_accumulated[SA_FEATURE_COUNT], voxels2_accumulated[SA_FEATURE_COUNT], voxels_accumulated_diff[SA_FEATURE_COUNT]; // 0:占有率, 1:速度, 2:ジャーク, 3:慣性モーメント, 4:慣性主軸角速度
    
    // Motion1 部位ごとの互換用密グリッド（内部はフレーム疎キャッシュから再合成）
    std::vector<VoxelGrid> segment_voxels1[SA_FEATURE_COUNT]; // 0:占有率, 1:速度, 2:ジャーク, 3:慣性モーメント, 4:慣性主軸角速度

  // Motion2 部位ごとの互換用密グリッド（内部はフレーム疎キャッシュから再合成）
    std::vector<VoxelGrid> segment_voxels2[SA_FEATURE_COUNT]; // 0:占有率, 1:速度, 2:ジャーク, 3:慣性モーメント, 4:慣性主軸角速度

    // フレーム単位の疎ボクセルキャッシュ
    MotionFrameSegmentVoxelGridCache frame_cache1;
    MotionFrameSegmentVoxelGridCache frame_cache2;
    bool has_frame_cache;
    float sparse_threshold;

    // 再合成済みキャッシュの姿勢スナップショット
    struct AccumulatedPoseCache {
        bool valid;
        Point3f motion1_root_pos;
        Matrix3f motion1_root_ori;
        Point3f motion2_root_pos;
        Matrix3f motion2_root_ori;

        AccumulatedPoseCache() : valid(false) {
            motion1_root_pos.set(0, 0, 0);
            motion2_root_pos.set(0, 0, 0);
            motion1_root_ori.setIdentity();
            motion2_root_ori.setIdentity();
        }
    };
    
	AccumulatedPoseCache accumulated_pose_cache[SA_FEATURE_COUNT]; // 0:占有率, 1:速度, 2:ジャーク, 3:慣性モーメント, 4:慣性主軸角速度

	// 瞬間表示用の最大値（必要に応じて追加）
	float max_val[SA_FEATURE_COUNT]; // 0:占有率, 1:速度, 2:ジャーク, 3:慣性モーメント, 4:慣性主軸角速度

	// 累積用の最大値（必要に応じて追加）
	float max_accumulated_val[SA_FEATURE_COUNT]; // 0:占有率, 1:速度, 2:ジャーク, 3:慣性モーメント, 4:慣性主軸角速度

	// 部位ごとの正規化用最大値（必要に応じて追加）
	std::vector<float> segment_max[SA_FEATURE_COUNT]; // 0:占有率, 1:速度, 2:ジャーク, 3:慣性モーメント, 4:慣性主軸角速度

    // --- ビュー/スライス設定 ---
	std::vector<float> slice_positions; // スライス位置 (0.0 - 1.0)
	    
    // スライス平面の変換行列（ローカル座標系→ワールド座標系）
    // 列0: U方向（平面上の横方向）
    // 列1: V方向（平面上の縦方向）
    // 列2: 法線方向
    // 列3: 平面中心位置
    Matrix4f slice_plane_transform;
    
    // 表示用オイラー角（読み取り専用、逆算値）
	float slice_rotation[3]; // 0: X軸回転, 1: Y軸回転, 2: Z軸回転
    
    bool use_rotated_slice;      // 回転スライスモードを使用するか

    // --- インタラクティブ操作用 ---
	float zoom; // ズーム倍率
	SpatialPoint2f pan_center; // パン操作中心点
	bool is_manual_view; // 手動ビュー操作フラグ

    // --- 表示フラグ ---
	bool show_planes; // スライス平面表示フラグ
	bool show_maps; // 2Dマップ表示フラグ
    bool show_voxels;  // 3Dボクセル表示フラグ
    int feature_mode; // 0: 占有率, 1: 速度, 2: ジャーク, 3: 慣性モーメント, 4: 慣性主軸角速度
	int norm_mode; // 0: 瞬間表示, 1: 累積表示
    bool has_world_bounds_initialized;
    float slice_display_base_range;
    
    // 表示設定
    int selected_segment_index;  // 表示する部位のインデックス (-1で全体) ※後方互換用
    std::vector<bool> selected_segments;  // 各部位の選択状態（複数選択対応）
    bool show_segment_mode;      // 部位別表示モードON/OFF
    
    // 部位選択キャッシュ（パフォーマンス最適化用）
    VoxelGrid cached_segment_grid1;   // 選択部位を集約したグリッド（M1）
    VoxelGrid cached_segment_grid2;   // 選択部位を集約したグリッド（M2）
    VoxelGrid cached_segment_diff;    // 選択部位の差分グリッド
    float cached_segment_max_val;     // 選択部位の最大差分値
    bool segment_cache_dirty;         // キャッシュが無効かどうか
    int cached_feature_mode;          // キャッシュ作成時のfeature_mode
    int cached_norm_mode;             // キャッシュ作成時のnorm_mode
    int cached_selected_segment_index;// キャッシュ作成時のselected_segment_index
    std::vector<bool> cached_selected_segments; // キャッシュ作成時の選択状態

public:
    SpatialAnalyzer();
    ~SpatialAnalyzer();

    void ResizeGrids(int res);
    void SetWorldBounds(float bounds[3][2]);
    
    // ボクセル計算
    void UpdateVoxels(Motion* m1, Motion* m2, float current_time);
    void VoxelizeMotion(Motion* m, float time, VoxelGrid& occ, VoxelGrid& spd, VoxelGrid& jrk, VoxelGrid& ine, VoxelGrid& pax);

    // 総合累積ボクセル計算（全体＋部位ごとを同時に計算）
    void AccumulateAllFrames(Motion* m1, Motion* m2);
    void ClearAccumulatedData();
    void BuildAllFeatureFrameCaches(Motion* m1, Motion* m2);
    void ComposeAccumulatedFeatureFromFrameCache(Motion* m1, Motion* m2, int feature);

    void VoxelizeMotionBySegmentGrids(Motion* m, float time,
                                      std::vector<VoxelGrid>& seg_presence_grids,
                                      std::vector<VoxelGrid>& seg_speed_grids,
                                      std::vector<VoxelGrid>& seg_jerk_grids,
                                      std::vector<VoxelGrid>& seg_inertia_grids,
                                      std::vector<VoxelGrid>& seg_principal_axis_grids);
    
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
    
    // スライス平面の変換行列操作
    void ApplySlicePlaneRotation(const Matrix4f& local_rotation);  // ローカル座標系での回転適用
    void ApplySlicePlaneWorldRotation(const Matrix4f& world_rotation);  // ワールド座標系での回転適用
    void ApplySlicePlaneTranslation(const Point3f& world_translation);  // ワールド座標系での平行移動
    
    // スライス平面のアクセサ
    Point3f GetSlicePlaneCenter() const;
    Vector3f GetSlicePlaneU() const;
    Vector3f GetSlicePlaneV() const;
    Vector3f GetSlicePlaneNormal() const;
    
    // スライス平面の回転操作（キーボード用）
    void RotateSlicePlane(float dx, float dy, float dz);
    void ResetSliceRotation();
    void ToggleRotatedSliceMode();
    
    // 部位選択操作（複数選択対応）
    void InitializeSegmentSelection(int num_segments);  // 選択状態の初期化
    void ToggleSegmentSelection(int segment_index);     // 部位の選択をトグル
    void SelectAllSegments();                           // 全部位を選択
    void ClearSegmentSelection();                       // 選択をクリア
    bool IsSegmentSelected(int segment_index) const;    // 選択状態を取得
    int GetSelectedSegmentCount() const;                // 選択部位数を取得
    float GetSegmentMaxValue(int segment_index) const;  // 部位ごとの最大値を取得
    void InvalidateSegmentCache();                      // キャッシュを無効化
    void UpdateSegmentCache();                          // キャッシュを更新

private:
    struct PrevPresenceCacheEntry {
        const Motion* motion;
        float time;
        std::vector<VoxelGrid> seg_presence_grids;
        int num_segments;
        int resolution;
        bool valid;

        PrevPresenceCacheEntry() : motion(nullptr), time(0.0f), num_segments(0), resolution(0), valid(false) {}
    };

    // VoxelizeMotion用の再利用バッファ（毎フレームの再確保を回避）
    std::vector<VoxelGrid> temp_voxelize_seg[SA_FEATURE_COUNT]; // 0:占有率, 1:速度, 2:ジャーク, 3:慣性モーメント, 4:慣性主軸角速度
    int temp_voxelize_num_segments;
    int temp_voxelize_resolution;
    PrevPresenceCacheEntry prev_presence_cache_entries[2];

    // 回転スライス用のヘルパー
    void DrawRotatedSlicePlane();
    void DrawRotatedSliceMap(int x, int y, int w, int h, VoxelGrid& grid, float max_val, const char* title);
    float SampleVoxelAtWorldPos(VoxelGrid& grid, const Point3f& world_pos);
    
    // オイラー角を変換行列から逆算（表示用）
    void UpdateEulerAnglesFromTransform();
    
    // 共通ボクセル化ヘルパー関数
    void ComputeFrameData(Motion* m, float time, FrameData& frame_data);
    void ExtractBoneData(Motion* m, const FrameData& frame_data, std::vector<BoneData>& bones);
    void ComputeAABB(const Point3f& p1, const Point3f& p2, float radius, 
                     int idx_min[3], int idx_max[3], const float world_range[3]);
    void WriteToVoxelGrid(const BoneData& bone, float bone_radius, const float world_range[3],
                          VoxelGrid* occ_grid, VoxelGrid* spd_grid, VoxelGrid* jrk_grid = nullptr, VoxelGrid* ine_grid = nullptr,
                          std::vector<int>* touched_indices = nullptr, std::vector<unsigned char>* touched_marks = nullptr);
    void BuildSingleMotionFeatureFrameCache(Motion* m, MotionFrameSegmentVoxelGridCache& cache);
    bool ComposeInstantFeatureFromFrameCache(Motion* m1, Motion* m2, int feature, float current_time, bool compose_segment_data);
    void VoxelizeMotionSegmentPresenceOnly(Motion* m, float time, std::vector<VoxelGrid>& seg_presence_grids);
};
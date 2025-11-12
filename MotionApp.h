#pragma once/**
*** 動作再生＆詳細プロット/カラーマップアプリケーション
**/

#ifndef  _MOTION_APP_H_
#define  _MOTION_APP_H_

#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"
#include "Timeline.h"


//
//  メインアプリケーションクラス
//
class  MotionApp : public GLUTBaseApp
{
protected:
    Motion *motion, *motion2;
    Posture *curr_posture, *curr_posture2;
    bool on_animation;
    float animation_time;
    float animation_speed;
    int view_mode;

    // --- プロット用 ---
    int plot_segment_index;
    int plot_vertical_axis, plot_horizontal_axis;
    std::vector<Point3f> plot_data1, plot_data2;

    // --- カラーマップ(時間vs部位)用 ---
    std::vector<std::vector<float>> colormap_data;
    float cmap_min_diff, cmap_max_diff;

    // --- カラーマップ(空間XvsZ)用 ---
    int   ct_h_axis, ct_v_axis;
    bool  projection_mode;
    int   grid_resolution;
    float world_bounds[3][2];
    int   ct_feature_mode;
    
    // --- 速度ノーマライズ用 ---
    int   speed_norm_mode;
    float global_max_speed;

    // ==================================================================
    // REVISED: 複数スライスを管理するためのデータ構造
    // ==================================================================
    
    // 複数のスライス平面のY座標
    std::vector<float> slice_values; 

    // 各スライスに対応するグリッドデータ (グリッドのベクトル)
    std::vector<std::vector<std::vector<float>>> occupancy_grids1;
    std::vector<std::vector<std::vector<float>>> occupancy_grids2;
    std::vector<std::vector<std::vector<float>>> difference_grids;
    std::vector<std::vector<std::vector<float>>> speed_grids1;
    std::vector<std::vector<std::vector<float>>> speed_grids2;
    std::vector<std::vector<std::vector<float>>> speed_diff_grids;

    // 各スライスに対応する最大値 (最大値のベクトル)
    std::vector<float> max_occupancy_diffs;
    std::vector<float> max_speeds;
    std::vector<float> max_speed_diffs;

    bool show_slice_planes;
    bool show_ct_maps;
    
    // ==================================================================

public:
    MotionApp();
    virtual ~MotionApp();
    virtual void Initialize() override;
    virtual void Start() override;
    virtual void Display() override;
    virtual void Keyboard(unsigned char key, int mx, int my) override;
    virtual void Animation(float delta) override;
    
protected:
    void LoadBVH(const char *file_name);
    void LoadBVH2(const char *file_name);
    void OpenNewBVH();
    void OpenNewBVH2();
    void AlignInitialPositions();
    void AlignInitialOrientations();
    void PrepareAllData();
    void PreparePlotData();
    void PrepareColormapData();
    void CalculateWorldBounds();
    void CalculateCtScanBounds(float& h_min, float& h_max, float& v_min, float& v_max);
    void PrepareSpeedData();
    void UpdateOccupancyGrids();
    void DrawPositionPlot();
    void DrawColormap();
    void DrawCtScanView();
    void DrawSlicePlane();
    void BeginScreenMode();
    void EndScreenMode();
    void DrawText(int x, int y, const char* text, void* font);
    void DrawPlotAxes(int x, int y, int w, int h, float h_min, float h_max, float v_min, float v_max, const char* h_label, const char* v_label);
};

#endif // _MOTION_APP_H_

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
    float animation_time, animation_speed;
    int view_mode;

    int plot_segment_index, plot_vertical_axis, plot_horizontal_axis;
    std::vector<Point3f> plot_data1, plot_data2;
    
    std::vector<std::vector<float>> colormap_data;
    float cmap_min_diff, cmap_max_diff;
    
    // REVISED: CTスキャンモード用の変数を更新
    int ct_h_axis, ct_v_axis;    // 0:X, 1:Y, 2:Z
    bool projection_mode;       // false:スライス, true:投影
    float slice_value;          // スライスする座標値
    int grid_resolution;
    float world_bounds[3][2];   // X,Y,Zそれぞれのmin/max
    std::vector<std::vector<float>> occupancy_grid1, occupancy_grid2, difference_grid;
    float max_occupancy_diff;

    // 速度表示用の変数
    int   ct_feature_mode; // 0: 占有率, 1: 速度
    vector<vector<float>> speed_grid1;
    vector<vector<float>> speed_grid2;
    vector<vector<float>> speed_diff_grid;
    float max_speed;
    float max_speed_diff;

    // ADDED: 速度のノーマライゼーションに関する変数
    int   speed_norm_mode;      // 0: フレーム基準, 1: 全体基準
    float global_max_speed;     // 全てのフレーム/部位を通した最大速度

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
    void  CalculateCtScanBounds(float& h_min, float& h_max, float& v_min, float& v_max);
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

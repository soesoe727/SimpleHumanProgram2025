#pragma once

#ifndef  _MOTION_APP_H_
#define  _MOTION_APP_H_

#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"
#include "Timeline.h"
#include "SpatialAnalysis.h" // 新規追加

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

    // --- 空間解析機能（ボクセル・CTスキャン） ---
    SpatialAnalyzer analyzer;

public:
    MotionApp();
    virtual ~MotionApp();
    virtual void Initialize() override;
    virtual void Start() override;
    virtual void Display() override;
    virtual void Keyboard(unsigned char key, int mx, int my) override;
    void Special(int key, int mx, int my);
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
    
    // ラッパー
    void UpdateVoxelDataWrapper();
    void CalculateWorldBounds(); 

    // 描画関連
    void DrawPositionPlot();
    void DrawColormap();
    
    // ユーティリティ
    void BeginScreenMode();
    void EndScreenMode();
    void DrawText(int x, int y, const char* text, void* font);
    void DrawPlotAxes(int x, int y, int w, int h, float h_min, float h_max, float v_min, float v_max, const char* h_label, const char* v_label);
};

#endif // _MOTION_APP_H_
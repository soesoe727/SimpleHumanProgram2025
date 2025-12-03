#pragma once

#ifndef  _MOTION_APP_H_
#define  _MOTION_APP_H_

#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"
#include "SpatialAnalysis.h"

class  MotionApp : public GLUTBaseApp
{
protected:
    Motion *motion, *motion2;
    Posture *curr_posture, *curr_posture2;
    bool on_animation;
    float animation_time;
    float animation_speed;

    // --- 空間解析機能（ボクセル・CTスキャン）---
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
    
    // ラッパー
    void UpdateVoxelDataWrapper();
    void CalculateWorldBounds(); 

    // ユーティリティ
    void DrawText(int x, int y, const char* text, void* font);
};

#endif // _MOTION_APP_H_
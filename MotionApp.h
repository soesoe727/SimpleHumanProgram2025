#pragma once

#ifndef  _MOTION_APP_H_
#define  _MOTION_APP_H_

#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"
#include "SpatialAnalysis.h"
#include "TransformGizmo.h"

class  MotionApp : public GLUTBaseApp
{
protected:
    Motion *motion, *motion2;
    Posture *curr_posture, *curr_posture2;
    bool on_animation;
    float animation_time;
    float animation_speed;

    // 空間解析機能（ボクセル・CTスキャン）
    SpatialAnalyzer analyzer;

    // スライス用トランスフォームギズモ
    TransformGizmo slice_gizmo;
    bool use_slice_gizmo;
    bool gizmo_dragging;

    // SpaceMouse用：スライス平面操作の有効/無効
    bool use_spacemouse_slice;

public:
    MotionApp();
    virtual ~MotionApp();
    virtual void Initialize() override;
    virtual void Start() override;
    virtual void Display() override;
    virtual void Keyboard(unsigned char key, int mx, int my) override;
    void Special(int key, int mx, int my);
    virtual void MouseClick(int button, int state, int mx, int my) override;
    virtual void MouseDrag(int mx, int my) override;
    virtual void MouseMotion(int mx, int my) override;
    virtual void Animation(float delta) override;

    // SpaceMouse入力処理（スライス平面に適用）
    virtual void ProcessSpaceMouseInput() override;
    
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

    // ボクセル情報を持つ部位かチェック（指などの細部を除外）
    bool HasVoxelData(int segment_index);
    // 次の有効な部位を取得
    int GetNextValidSegment(int current_segment, int direction);

    // ギズモ補助
    void ToggleSliceGizmo();
    void ToggleSliceGizmoMode();
    float ComputeGizmoScale() const;
    Matrix3f GetSliceGizmoOrientation() const;
    Point3f GetSliceGizmoPosition() const;
    // ギズモのデルタ適用（Matrix4fベース）
    void ApplySliceGizmoDelta(const Point3f& translation, const Matrix4f& local_rotation);
    void SyncGizmoToSliceState();

    // ユーティリティ
    void DrawText(int x, int y, const char* text, void* font);
};

#endif // _MOTION_APP_H_
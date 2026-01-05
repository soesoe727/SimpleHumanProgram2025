#pragma once
#include "SimpleHuman.h"
#include "Point3.h"
#include "Matrix4.h"
#include <vector>

// ギズモのモード
enum GizmoMode {
    GIZMO_TRANSLATE,  // 移動モード
    GIZMO_ROTATE      // 回転モード
};

// 選択された軸/円
enum GizmoAxis {
    GIZMO_NONE = -1,
    GIZMO_X = 0,
    GIZMO_Y = 1,
    GIZMO_Z = 2
};

// 3D変換ギズモクラス
class TransformGizmo {
public:
    TransformGizmo();
    ~TransformGizmo();

    // ギズモの描画
    void Draw(const Point3f& position, const Matrix3f& orientation, float scale = 1.0f);
    
    // マウスピッキング（軸/円の選択）
    GizmoAxis PickAxis(int mouse_x, int mouse_y, const Point3f& position, const Matrix3f& orientation, 
                       float scale, int win_width, int win_height);
    
    // ドラッグによる変換の計算
    void StartDrag(const Point3f& position, int mouse_x, int mouse_y, 
                   int win_width, int win_height);
    void UpdateDrag(int mouse_x, int mouse_y, int win_width, int win_height,
                    Point3f& out_translation, Matrix3f& out_rotation);
    
    // 設定
    void SetMode(GizmoMode mode) { m_mode = mode; }
    GizmoMode GetMode() const { return m_mode; }
    
    void SetSelectedAxis(GizmoAxis axis) { m_selected_axis = axis; }
    GizmoAxis GetSelectedAxis() const { return m_selected_axis; }
    
    void SetActiveOrientation(const Matrix3f& orientation) { m_active_orientation = orientation; }
    
    bool IsActive() const { return m_selected_axis != GIZMO_NONE; }

private:
    GizmoMode m_mode;
    GizmoAxis m_selected_axis;
    Point3f m_drag_start_pos;
    Point3f m_drag_start_world_pos;
    Vector3f m_drag_plane_normal;
    float m_drag_start_angle;
    int m_last_mouse_x, m_last_mouse_y;
    Matrix3f m_active_orientation;
    
    // 描画ヘルパー
    void DrawTranslateGizmo(const Point3f& position, const Matrix3f& orientation, float scale);
    void DrawRotateGizmo(const Point3f& position, const Matrix3f& orientation, float scale);
    void DrawArrow(const Vector3f& direction, float length, float thickness);
    void DrawCircle(const Vector3f& normal, float radius, int segments);
    
    // ピッキングヘルパー
    bool RayIntersectArrow(const Point3f& ray_origin, const Vector3f& ray_dir,
                          const Point3f& arrow_start, const Vector3f& arrow_dir,
                          float length, float thickness, float& out_t);
    bool RayIntersectCircle(const Point3f& ray_origin, const Vector3f& ray_dir,
                           const Point3f& circle_center, const Vector3f& circle_normal,
                           float radius, float thickness, float& out_t);
    
    // レイキャスト（スクリーン座標→ワールド空間レイ）
    void ScreenToWorldRay(int mouse_x, int mouse_y, int win_width, int win_height,
                         Point3f& out_ray_origin, Vector3f& out_ray_dir);
    
    // 平面との交点計算
    bool RayIntersectPlane(const Point3f& ray_origin, const Vector3f& ray_dir,
                          const Point3f& plane_point, const Vector3f& plane_normal,
                          Point3f& out_intersection);
};

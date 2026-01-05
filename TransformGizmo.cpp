#include "TransformGizmo.h"
#include "SimpleHumanGLUT.h"
#include <cmath>
#include <algorithm>

#define M_PI 3.14159265358979323846

TransformGizmo::TransformGizmo() 
    : m_mode(GIZMO_TRANSLATE)
    , m_selected_axis(GIZMO_NONE)
    , m_drag_start_angle(0.0f)
    , m_last_mouse_x(0)
    , m_last_mouse_y(0)
{
    m_drag_start_pos.set(0, 0, 0);
    m_drag_start_world_pos.set(0, 0, 0);
    m_drag_plane_normal.set(0, 1, 0);
}

TransformGizmo::~TransformGizmo() {}

void TransformGizmo::Draw(const Point3f& position, const Matrix3f& orientation, float scale) {
    // OpenGL状態の保存
    glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_DEPTH_BUFFER_BIT);
    
    // ライティングを無効化（ギズモは明示的に描画）
    glDisable(GL_LIGHTING);
    
    // 深度テストは呼び出し側の設定に従う
    glPushMatrix();
    
    // ギズモの位置に移動
    glTranslatef(position.x, position.y, position.z);
    
    // 回転の適用（Matrix3fからGLfloat[16]への行列変換）
    GLfloat m[16];
    m[0] = orientation.m00;  m[4] = orientation.m01;  m[8]  = orientation.m02;  m[12] = 0.0f;
    m[1] = orientation.m10;  m[5] = orientation.m11;  m[9]  = orientation.m12;  m[13] = 0.0f;
    m[2] = orientation.m20;  m[6] = orientation.m21;  m[10] = orientation.m22;  m[14] = 0.0f;
    m[3] = 0.0f;             m[7] = 0.0f;             m[11] = 0.0f;             m[15] = 1.0f;
    glMultMatrixf(m);
    
    // ローカルに描画
    Matrix3f identity_mat;
    identity_mat.setIdentity();
    
    if (m_mode == GIZMO_TRANSLATE) {
        DrawTranslateGizmo(Point3f(0, 0, 0), identity_mat, scale);
    } else {
        DrawRotateGizmo(Point3f(0, 0, 0), identity_mat, scale);
    }
    
    glPopMatrix();
    
    // OpenGL状態の復元
    glPopAttrib();
}

void TransformGizmo::DrawTranslateGizmo(const Point3f& position, const Matrix3f& orientation, float scale) {
    float arrow_length = 0.3f * scale;
    float arrow_thickness = 0.02f * scale;
    
    // X軸（赤）- 右方向
    if (m_selected_axis == GIZMO_X) {
        glColor3f(1.0f, 1.0f, 0.0f);  // 選択時は黄色
    } else {
        glColor3f(1.0f, 0.0f, 0.0f);  // 赤
    }
    glPushMatrix();
    glRotatef(90, 0, 1, 0);  // Z軸をX軸方向に回転
    DrawArrow(Vector3f(1, 0, 0), arrow_length, arrow_thickness);
    glPopMatrix();
    
    // Y軸（緑）- 上方向
    if (m_selected_axis == GIZMO_Y) {
        glColor3f(1.0f, 1.0f, 0.0f);  // 選択時は黄色
    } else {
        glColor3f(0.0f, 1.0f, 0.0f);  // 緑
    }
    glPushMatrix();
    glRotatef(-90, 1, 0, 0);  // Z軸をY軸方向に回転
    DrawArrow(Vector3f(0, 1, 0), arrow_length, arrow_thickness);
    glPopMatrix();
    
    // Z軸（青）- 前方向（デフォルトの向き）
    if (m_selected_axis == GIZMO_Z) {
        glColor3f(1.0f, 1.0f, 0.0f);  // 選択時は黄色
    } else {
        glColor3f(0.0f, 0.0f, 1.0f);  // 青
    }
    glPushMatrix();
    DrawArrow(Vector3f(0, 0, 1), arrow_length, arrow_thickness);
    glPopMatrix();
}

void TransformGizmo::DrawRotateGizmo(const Point3f& position, const Matrix3f& orientation, float scale) {
    float radius = 0.25f * scale;
    int segments = 64;
    
    glLineWidth(2.0f);
    
    // X軸周り（赤い円）
    if (m_selected_axis == GIZMO_X) {
        glColor3f(1.0f, 1.0f, 0.0f);  // 選択時は黄色
    } else {
        glColor3f(1.0f, 0.0f, 0.0f);  // 赤
    }
    glPushMatrix();
    glRotatef(90, 0, 1, 0);
    DrawCircle(Vector3f(1, 0, 0), radius, segments);
    glPopMatrix();
    
    // Y軸周り（緑の円）
    if (m_selected_axis == GIZMO_Y) {
        glColor3f(1.0f, 1.0f, 0.0f);  // 選択時は黄色
    } else {
        glColor3f(0.0f, 1.0f, 0.0f);  // 緑
    }
    glPushMatrix();
    glRotatef(90, 1, 0, 0);
    DrawCircle(Vector3f(0, 1, 0), radius, segments);
    glPopMatrix();
    
    // Z軸周り（青い円）
    if (m_selected_axis == GIZMO_Z) {
        glColor3f(1.0f, 1.0f, 0.0f);  // 選択時は黄色
    } else {
        glColor3f(0.0f, 0.0f, 1.0f);  // 青
    }
    DrawCircle(Vector3f(0, 0, 1), radius, segments);
    
    glLineWidth(1.0f);
}

void TransformGizmo::DrawArrow(const Vector3f& direction, float length, float thickness) {
    GLUquadricObj* quad = gluNewQuadric();
    gluQuadricDrawStyle(quad, GLU_FILL);
    gluQuadricNormals(quad, GLU_SMOOTH);
    
    // 軸の円柱（根元から先端方向へ）
    gluCylinder(quad, thickness, thickness, length * 0.75f, 16, 1);
    
    // 矢印の先端（円錐）
    glPushMatrix();
    glTranslatef(0, 0, length * 0.75f);
    gluCylinder(quad, thickness * 3, 0, length * 0.25f, 16, 1);
    glPopMatrix();
    
    gluDeleteQuadric(quad);
}

void TransformGizmo::DrawCircle(const Vector3f& normal, float radius, int segments) {
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        float x = radius * cos(angle);
        float y = radius * sin(angle);
        glVertex3f(x, y, 0);
    }
    glEnd();
}

GizmoAxis TransformGizmo::PickAxis(int mouse_x, int mouse_y, const Point3f& position, 
                                   const Matrix3f& orientation, float scale, 
                                   int win_width, int win_height) {
    Point3f ray_origin;
    Vector3f ray_dir;
    ScreenToWorldRay(mouse_x, mouse_y, win_width, win_height, ray_origin, ray_dir);
    
    float closest_t = 1e6f;
    GizmoAxis closest_axis = GIZMO_NONE;
    
    if (m_mode == GIZMO_TRANSLATE) {
        // 各軸の矢印との交差判定
        Vector3f axes[3] = {
            Vector3f(1, 0, 0),  // X
            Vector3f(0, 1, 0),  // Y
            Vector3f(0, 0, 1)   // Z
        };
        
        for (int i = 0; i < 3; ++i) {
            Vector3f axis_world = axes[i];
            orientation.transform(&axis_world);
            
            float t;
            if (RayIntersectArrow(ray_origin, ray_dir, position, axis_world, 
                                 0.3f * scale, 0.05f * scale, t)) {
                if (t < closest_t) {
                    closest_t = t;
                    closest_axis = (GizmoAxis)i;
                }
            }
        }
    } else {
        // 各軸周りの円との交差判定
        Vector3f normals[3] = {
            Vector3f(1, 0, 0),  // X軸周り
            Vector3f(0, 1, 0),  // Y軸周り
            Vector3f(0, 0, 1)   // Z軸周り
        };
        
        for (int i = 0; i < 3; ++i) {
            Vector3f normal_world = normals[i];
            orientation.transform(&normal_world);
            
            float t;
            if (RayIntersectCircle(ray_origin, ray_dir, position, normal_world,
                                  0.25f * scale, 0.05f * scale, t)) {
                if (t < closest_t) {
                    closest_t = t;
                    closest_axis = (GizmoAxis)i;
                }
            }
        }
    }
    
    return closest_axis;
}

void TransformGizmo::StartDrag(const Point3f& position, int mouse_x, int mouse_y,
                               int win_width, int win_height) {
    m_drag_start_pos.set(position.x, position.y, position.z);
    
    // ドラッグ開始時のマウス座標を記録
    m_last_mouse_x = mouse_x;
    m_last_mouse_y = mouse_y;
    
    // 回転モードの場合のみ、平面との交差計算が必要
    if (m_mode == GIZMO_ROTATE) {
        Point3f ray_origin;
        Vector3f ray_dir;
        ScreenToWorldRay(mouse_x, mouse_y, win_width, win_height, ray_origin, ray_dir);
        
        // 回転モード：回転軸が法線の平面
        if (m_selected_axis == GIZMO_X) {
            m_drag_plane_normal.set(1, 0, 0);
        } else if (m_selected_axis == GIZMO_Y) {
            m_drag_plane_normal.set(0, 1, 0);
        } else {
            m_drag_plane_normal.set(0, 0, 1);
        }
        
        // ドラッグ開始点のワールド座標
        RayIntersectPlane(ray_origin, ray_dir, position, m_drag_plane_normal, m_drag_start_world_pos);
    }
}

void TransformGizmo::UpdateDrag(int mouse_x, int mouse_y, int win_width, int win_height,
                                Point3f& out_translation, Matrix3f& out_rotation) {
    // マウスの移動量を計算
    int delta_x = mouse_x - m_last_mouse_x;
    int delta_y = mouse_y - m_last_mouse_y;
    
    // マウス座標を更新
    m_last_mouse_x = mouse_x;
    m_last_mouse_y = mouse_y;
    
    if (m_mode == GIZMO_TRANSLATE) {
        // 移動モード：カメラの向きを考慮したスクリーン空間での移動
        
        // カメラのビュー行列を取得
        GLdouble modelview[16];
        glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
        
        // カメラの右方向ベクトル（ビュー行列の第1列）
        Vector3f camera_right(modelview[0], modelview[4], modelview[8]);
        camera_right.normalize();
        
        // カメラの上方向ベクトル（ビュー行列の第2列）
        Vector3f camera_up(modelview[1], modelview[5], modelview[9]);
        camera_up.normalize();
        
        // カメラの前方向ベクトル（ビュー行列の第3列の逆）
        Vector3f camera_forward(-modelview[2], -modelview[6], -modelview[10]);
        camera_forward.normalize();
        
        // 移動感度（ピクセル当たりのワールド単位）
        float sensitivity = 0.005f;
        
        Vector3f translation(0, 0, 0);
        
        if (m_selected_axis == GIZMO_X) {
            // X軸：選択した軸とカメラの右・上ベクトルの内積から、
            // スクリーン空間での移動量をワールド空間に変換
            Vector3f axis_dir(1, 0, 0);
            
            // スクリーンX方向の移動をX軸方向に投影
            float move_x = delta_x * camera_right.dot(axis_dir);
            // スクリーンY方向の移動をX軸方向に投影
            float move_y = -delta_y * camera_up.dot(axis_dir);
            
            float total_move = (move_x + move_y) * sensitivity;
            translation.x = total_move;
            
        } else if (m_selected_axis == GIZMO_Y) {
            // Y軸：同様の処理
            Vector3f axis_dir(0, 1, 0);
            
            float move_x = delta_x * camera_right.dot(axis_dir);
            float move_y = -delta_y * camera_up.dot(axis_dir);
            
            float total_move = (move_x + move_y) * sensitivity;
            translation.y = total_move;
            
        } else if (m_selected_axis == GIZMO_Z) {
            // Z軸：同様の処理
            Vector3f axis_dir(0, 0, 1);
            
            float move_x = delta_x * camera_right.dot(axis_dir);
            float move_y = -delta_y * camera_up.dot(axis_dir);
            
            float total_move = (move_x + move_y) * sensitivity;
            translation.z = total_move;
        }
        
        out_translation.set(translation.x, translation.y, translation.z);
        out_rotation.setIdentity();
        
    } else {
        // 回転モード：元の実装を維持
        Point3f ray_origin;
        Vector3f ray_dir;
        ScreenToWorldRay(mouse_x, mouse_y, win_width, win_height, ray_origin, ray_dir);
        
        Point3f current_pos;
        if (!RayIntersectPlane(ray_origin, ray_dir, m_drag_start_pos, m_drag_plane_normal, current_pos)) {
            out_translation.set(0, 0, 0);
            out_rotation.setIdentity();
            return;
        }
        
        // 回転量を計算
        Vector3f v1 = m_drag_start_world_pos - m_drag_start_pos;
        Vector3f v2 = current_pos - m_drag_start_pos;
        
        v1.normalize();
        v2.normalize();
        
        float dot_product = v1.dot(v2);
        if (dot_product > 1.0f) dot_product = 1.0f;
        if (dot_product < -1.0f) dot_product = -1.0f;
        float angle = acos(dot_product);
        
        // 回転方向を決定
        Vector3f cross;
        cross.cross(v1, v2);
        if (cross.dot(m_drag_plane_normal) < 0) {
            angle = -angle;
        }
        
        // 感度調整
        angle *= 0.5f;
        
        // 回転行列を作成
        if (m_selected_axis == GIZMO_X) {
            out_rotation.rotX(angle);
        } else if (m_selected_axis == GIZMO_Y) {
            out_rotation.rotY(angle);
        } else if (m_selected_axis == GIZMO_Z) {
            out_rotation.rotZ(angle);
        }
        
        out_translation.set(0, 0, 0);
        
        // 次の回転計算のために開始位置を更新
        m_drag_start_world_pos = current_pos;
    }
}

void TransformGizmo::ScreenToWorldRay(int mouse_x, int mouse_y, int win_width, int win_height,
                                      Point3f& out_ray_origin, Vector3f& out_ray_dir) {
    GLdouble modelview[16], projection[16];
    GLint viewport[4];
    
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);
    
    // スクリーン座標をOpenGL座標系に変換（Y軸反転）
    float y = viewport[3] - mouse_y;
    
    // 近平面と遠平面の2点をアンプロジェクト
    GLdouble near_x, near_y, near_z;
    GLdouble far_x, far_y, far_z;
    
    gluUnProject(mouse_x, y, 0.0, modelview, projection, viewport, &near_x, &near_y, &near_z);
    gluUnProject(mouse_x, y, 1.0, modelview, projection, viewport, &far_x, &far_y, &far_z);
    
    out_ray_origin.set(near_x, near_y, near_z);
    out_ray_dir.set(far_x - near_x, far_y - near_y, far_z - near_z);
    out_ray_dir.normalize();
}

bool TransformGizmo::RayIntersectPlane(const Point3f& ray_origin, const Vector3f& ray_dir,
                                       const Point3f& plane_point, const Vector3f& plane_normal,
                                       Point3f& out_intersection) {
    float denom = ray_dir.dot(plane_normal);
    if (fabs(denom) < 1e-6f) return false;
    
    Vector3f p0_to_plane = plane_point - ray_origin;
    float t = p0_to_plane.dot(plane_normal) / denom;
    
    if (t < 0) return false;
    
    out_intersection.set(
        ray_origin.x + ray_dir.x * t,
        ray_origin.y + ray_dir.y * t,
        ray_origin.z + ray_dir.z * t
    );
    
    return true;
}

bool TransformGizmo::RayIntersectArrow(const Point3f& ray_origin, const Vector3f& ray_dir,
                                       const Point3f& arrow_start, const Vector3f& arrow_dir,
                                       float length, float thickness, float& out_t) {
    // 簡易的な実装：軸周りのシリンダーとの交差判定
    Vector3f oc = ray_origin - arrow_start;
    
    float a = ray_dir.dot(ray_dir) - pow(ray_dir.dot(arrow_dir), 2);
    float b = 2.0f * (oc.dot(ray_dir) - oc.dot(arrow_dir) * ray_dir.dot(arrow_dir));
    float c = oc.dot(oc) - pow(oc.dot(arrow_dir), 2) - thickness * thickness;
    
    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return false;
    
    out_t = (-b - sqrt(discriminant)) / (2.0f * a);
    
    // 軸の範囲内かチェック
    Point3f hit_point(
        ray_origin.x + ray_dir.x * out_t,
        ray_origin.y + ray_dir.y * out_t,
        ray_origin.z + ray_dir.z * out_t
    );
    
    Vector3f to_hit = hit_point - arrow_start;
    float proj = to_hit.dot(arrow_dir);
    
    return (proj >= 0 && proj <= length);
}

bool TransformGizmo::RayIntersectCircle(const Point3f& ray_origin, const Vector3f& ray_dir,
                                        const Point3f& circle_center, const Vector3f& circle_normal,
                                        float radius, float thickness, float& out_t) {
    // 円の平面との交点を求める
    Point3f intersection;
    if (!RayIntersectPlane(ray_origin, ray_dir, circle_center, circle_normal, intersection)) {
        return false;
    }
    
    // 中心からの距離を計算
    Vector3f to_intersection = intersection - circle_center;
    float dist = to_intersection.length();
    
    // 円の半径付近かチェック
    if (fabs(dist - radius) < thickness) {
        Vector3f to_hit = intersection - ray_origin;
        out_t = to_hit.length();
        return true;
    }
    
    return false;
}

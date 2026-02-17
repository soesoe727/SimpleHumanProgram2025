#include "MotionApp.h"
#include "BVH.h"
#include <string>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <iostream>

using namespace std;

static const float kPi = 3.14159265358979323846f;

// インスタンスを保持する static ポインタ
static MotionApp* g_app_instance = NULL;

// 'Special'関数を呼び出すための静的ラッパー
static void SpecialKeyWrapper(int key, int x, int y)
{
    if (g_app_instance) {
        g_app_instance->Special(key, x, y);
    }
}

MotionApp::MotionApp() {
    g_app_instance = this;
    app_name = "Motion Analysis App";
    motion = NULL; 
    motion2 = NULL; 
    curr_posture = NULL; 
    curr_posture2 = NULL;
    on_animation = true; 
    animation_time = 0.0f; 
    animation_speed = 1.0f;

    // ギズモ初期化
    use_slice_gizmo = false;
    gizmo_dragging = false;
    slice_gizmo.SetMode(GIZMO_TRANSLATE);

    // SpaceMouseによるスライス操作を有効化
    use_spacemouse_slice = true;
}

MotionApp::~MotionApp() 
{
    if ( motion ) { 
        if (motion->body) delete motion->body;
        delete motion;
    } 
    if ( motion2 ) delete motion2; 
    if ( curr_posture ) delete curr_posture;
    if ( curr_posture2 ) delete curr_posture2;
}

void MotionApp::Initialize() 
{
    GLUTBaseApp::Initialize();
    glutSpecialFunc(SpecialKeyWrapper);
    OpenNewBVH(); 
    OpenNewBVH2();
}

void MotionApp::Start()
{
    GLUTBaseApp::Start();
    on_animation = true;
    animation_time = 0.0f;
    Animation(0.0f);
}

void MotionApp::Keyboard(unsigned char key, int mx, int my) {
    GLUTBaseApp::Keyboard(key, mx, my);
    
    // 基本操作
    switch (key) {
        case ' ': 
            on_animation = !on_animation;
            break;
        case 'l': 
            OpenNewBVH();
            OpenNewBVH2();
            break;
    }

    // CTスキャン操作
    switch(key) {
        case 'w': 
            if (!analyzer.slice_positions.empty()) {
                Vector3f n = analyzer.GetSlicePlaneNormal(); 
                n.normalize();
                analyzer.ApplySlicePlaneTranslation(Point3f(n.x * 0.02f, n.y * 0.02f, n.z * 0.02f));
            }
            break;
        case 's': 
            if (!analyzer.slice_positions.empty()) {
                Vector3f n = analyzer.GetSlicePlaneNormal(); 
                n.normalize();
                analyzer.ApplySlicePlaneTranslation(Point3f(-n.x * 0.02f, -n.y * 0.02f, -n.z * 0.02f));
            }
            break;

        case 'f': 
            analyzer.feature_mode = (analyzer.feature_mode + 1) % 3; 
            break;
        case 'n': 
            analyzer.norm_mode = (analyzer.norm_mode + 1) % 2; 
            break;
        
        case 'b': 
            analyzer.show_planes = !analyzer.show_planes; 
            break;
        case 'm': 
            analyzer.show_maps = !analyzer.show_maps; 
            break;
        case 'k': 
            analyzer.show_voxels = !analyzer.show_voxels; 
            break;
        
        case '1': analyzer.RotateSlicePlane(5.0f, 0.0f, 0.0f); break;
        case '2': analyzer.RotateSlicePlane(-5.0f, 0.0f, 0.0f); break;
        case '3': analyzer.RotateSlicePlane(0.0f, 5.0f, 0.0f); break;
        case '4': analyzer.RotateSlicePlane(0.0f, -5.0f, 0.0f); break;
        case '5': analyzer.RotateSlicePlane(0.0f, 0.0f, 5.0f); break;
        case '6': analyzer.RotateSlicePlane(0.0f, 0.0f, -5.0f); break;
        case '0': analyzer.ResetSliceRotation(); break;

        case 'i': analyzer.Zoom(0.9f); break;
        case 'o': analyzer.Zoom(1.1f); break;
        case 'r': analyzer.ResetView(); break;

        case 'y': ToggleSliceGizmo(); break;
        case 'u': ToggleSliceGizmoMode(); break;

        case 'p':
            use_spacemouse_slice = !use_spacemouse_slice;
            if (use_spacemouse_slice && !analyzer.use_rotated_slice)
                analyzer.ToggleRotatedSliceMode();
            break;
        
        case 'e': 
            analyzer.show_segment_mode = !analyzer.show_segment_mode;
            if (analyzer.show_segment_mode && motion) {
                if (analyzer.selected_segments.size() != (size_t)motion->body->num_segments)
                    analyzer.InitializeSegmentSelection(motion->body->num_segments);
                if (analyzer.selected_segment_index < 0 || !HasVoxelData(analyzer.selected_segment_index))
                    analyzer.selected_segment_index = GetNextValidSegment(-1, 1);
            }
            std::cout << "Segment mode: " << (analyzer.show_segment_mode ? "ON" : "OFF") << std::endl;
            break;
            
        case '[':
        case ']':
            if (motion && analyzer.show_segment_mode) {
                int direction = (key == '[') ? -1 : 1;
                analyzer.selected_segment_index = GetNextValidSegment(analyzer.selected_segment_index, direction);
                analyzer.InvalidateSegmentCache();
                std::cout << "Current segment: " << analyzer.selected_segment_index 
                          << " (" << motion->body->segments[analyzer.selected_segment_index]->name << ")" << std::endl;
            }
            break;
            
        case 't':
            if (motion && analyzer.show_segment_mode && analyzer.selected_segment_index >= 0) {
                analyzer.ToggleSegmentSelection(analyzer.selected_segment_index);
                std::cout << "Segment " << analyzer.selected_segment_index 
                          << " (" << motion->body->segments[analyzer.selected_segment_index]->name << ") "
                          << (analyzer.IsSegmentSelected(analyzer.selected_segment_index) ? "SELECTED" : "DESELECTED")
                          << " | Total selected: " << analyzer.GetSelectedSegmentCount() << std::endl;
            }
            break;
            
        case '9':
            if (motion && analyzer.show_segment_mode) {
                for (int i = 0; i < motion->body->num_segments; ++i)
                    if (HasVoxelData(i))
                        analyzer.selected_segments[i] = true;
                analyzer.InvalidateSegmentCache();
                std::cout << "All valid segments selected: " << analyzer.GetSelectedSegmentCount() << std::endl;
            }
            break;
            
        case '8':
            if (analyzer.show_segment_mode) {
                analyzer.ClearSegmentSelection();
                std::cout << "Segment selection cleared" << std::endl;
            }
            break;
            
        case '\\':
            analyzer.selected_segment_index = -1;
            analyzer.show_segment_mode = false;
            analyzer.ClearSegmentSelection();
            std::cout << "Reset to all segments view" << std::endl;
            break;
    }
}

void MotionApp::Special(int key, int mx, int my)
{
    float world_range[3];
    for (int i = 0; i < 3; ++i)
        world_range[i] = analyzer.world_bounds[i][1] - analyzer.world_bounds[i][0];
    float max_range = max(world_range[0], max(world_range[1], world_range[2]));
    float step = max_range * 0.05f;

    switch(key)
    {
        case GLUT_KEY_LEFT:  analyzer.Pan(-step, 0); break;
        case GLUT_KEY_RIGHT: analyzer.Pan(step, 0); break;
        case GLUT_KEY_DOWN:  analyzer.Pan(0, -step); break;
        case GLUT_KEY_UP:    analyzer.Pan(0, step); break;
    }
}

void MotionApp::MouseClick(int button, int state, int mx, int my)
{
    GLUTBaseApp::MouseClick(button, state, mx, my);

    if (!use_slice_gizmo || !analyzer.use_rotated_slice)
        return;

    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        SyncGizmoToSliceState();
        Point3f gizmo_pos = GetSliceGizmoPosition();
        Matrix3f gizmo_ori = GetSliceGizmoOrientation();
        slice_gizmo.SetActiveOrientation(gizmo_ori);
        GizmoAxis axis = slice_gizmo.PickAxis(mx, my, gizmo_pos, gizmo_ori, ComputeGizmoScale(), win_width, win_height);
        slice_gizmo.SetSelectedAxis(axis);
        if (axis != GIZMO_NONE) {
            slice_gizmo.StartDrag(gizmo_pos, mx, my, win_width, win_height);
            gizmo_dragging = true;
        }
    } else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
        slice_gizmo.SetSelectedAxis(GIZMO_NONE);
        gizmo_dragging = false;
    }
}

void MotionApp::MouseDrag(int mx, int my)
{
    if (use_slice_gizmo && gizmo_dragging && slice_gizmo.GetSelectedAxis() != GIZMO_NONE) {
        Point3f translation;
        Matrix4f local_rotation;
        slice_gizmo.UpdateDrag(mx, my, win_width, win_height, translation, local_rotation);
        ApplySliceGizmoDelta(translation, local_rotation);
    }

    GLUTBaseApp::MouseDrag(mx, my);
}

void MotionApp::MouseMotion(int mx, int my)
{
    GLUTBaseApp::MouseMotion(mx, my);
}

void MotionApp::Animation(float delta)
{
    if (!on_animation || drag_mouse_l || !motion || !motion2) 
        return;
    animation_time += delta * animation_speed;
    float max_duration = max(motion->GetDuration(), motion2->GetDuration());
    if (animation_time >= max_duration) 
        animation_time = 0.0f;
    if (animation_time < 0.0f) 
        animation_time = 0.0f;
    motion->GetPosture(animation_time, *curr_posture);
    motion2->GetPosture(animation_time, *curr_posture2);
}

void MotionApp::Display()
{
    GLUTBaseApp::Display();
    
    // 1. 3Dモデルの描画 - Motion1 (赤色)
    if (curr_posture) {
        DrawPostureWithSegmentMode(*curr_posture, Color3f(1.0f, 0.0f, 0.0f));
        glColor3f(0.0f, 0.0f, 0.0f);
        DrawPostureShadow(*curr_posture, shadow_dir, shadow_color); 
    }
    
    // 2. 3Dモデルの描画 - Motion2 (青色)
    if (curr_posture2) {
        DrawPostureWithSegmentMode(*curr_posture2, Color3f(0.0f, 0.0f, 1.0f));
        glColor3f(0.0f, 0.0f, 0.0f);
        DrawPostureShadow(*curr_posture2, shadow_dir, shadow_color); 
    }

    // 3. Analyzerによる3D描画
    if (analyzer.show_planes)
        analyzer.DrawSlicePlanes();
    UpdateVoxelDataWrapper();
    analyzer.DrawVoxels3D();

    // 4. ギズモの描画
    if (use_slice_gizmo && analyzer.use_rotated_slice) {
        SyncGizmoToSliceState();
        Matrix3f gizmo_ori = GetSliceGizmoOrientation();
        Point3f gizmo_pos = GetSliceGizmoPosition();
        glPushAttrib(GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT);
        glDepthMask(GL_FALSE);
        glDepthFunc(GL_ALWAYS);
        glDisable(GL_DEPTH_TEST);
        slice_gizmo.Draw(gizmo_pos, gizmo_ori, ComputeGizmoScale());
        glPopAttrib();
    }

    // 5. 2D UIの描画（CTマップ）
    if (analyzer.show_maps)
        analyzer.DrawCTMaps(win_width, win_height);

    // 6. 情報テキストの表示
    const char* feature_names[] = {"Occupancy", "Speed", "Jerk"};
    const char* feature_mode_str = feature_names[analyzer.feature_mode];
    const char* norm_mode_str = (analyzer.norm_mode == 0) ? "CurrentFrame" : "Accumulated";

    // 部位情報
    char segment_info[256] = "All";
    if (analyzer.show_segment_mode && motion) {
        int selected_count = analyzer.GetSelectedSegmentCount();
        if (selected_count > 0) {
            sprintf(segment_info, "%d selected", selected_count);
        } else if (analyzer.selected_segment_index >= 0) {
            sprintf(segment_info, "%s[%d]", 
                    motion->body->segments[analyzer.selected_segment_index]->name.c_str(),
                    analyzer.selected_segment_index);
        }
    }

    char title[512];
    sprintf(title, "CT-Scan | Rot:X%.0f Y%.0f Z%.0f | Gizmo:%s(%s) | SpaceMouse:%s | Feature:%s | Data:%s | Seg:%s | Planes:%s | Voxels:%s",
        analyzer.slice_rotation_x, analyzer.slice_rotation_y, analyzer.slice_rotation_z,
        use_slice_gizmo ? "ON" : "OFF",
        slice_gizmo.GetMode() == GIZMO_TRANSLATE ? "Move" : "Rotate",
        use_spacemouse_slice ? "ON" : "OFF",
        feature_mode_str, norm_mode_str, segment_info,
        analyzer.show_planes ? "ON" : "OFF",
        analyzer.show_voxels ? "ON" : "OFF");
    
    DrawTextInformation(0, title);
    
    // 2行目：時間情報と操作説明
    if (motion) { 
        char msg[128]; 
        sprintf(msg, "Time:%.2f(%d) | e:SegMode [/]:NavSeg t:Toggle 9:All 8:Clear", 
                animation_time, (int)(animation_time / motion->interval)); 
        DrawTextInformation(1, msg); 
    }
}

void MotionApp::LoadBVH(const char* file_name) {
    Motion* new_motion = LoadAndCoustructBVHMotion(file_name);
    if (!new_motion) 
        return;
    if (motion) { 
        if (motion->body) 
            delete motion->body; 
        delete motion; 
    }
    if (curr_posture) 
        delete curr_posture;
    motion = new_motion;
    curr_posture = new Posture(motion->body);
    Start();
}

void MotionApp::LoadBVH2(const char* file_name)
{ 
    if (!motion) 
        return;
    Motion* m2 = LoadAndCoustructBVHMotion(file_name);
    if (!m2) 
        return;
    if (motion2) 
        delete motion2; 
    if (curr_posture2) 
        delete curr_posture2;
    motion2 = m2;
    curr_posture2 = new Posture(motion2->body); 
    PrepareAllData();
    Start();
}

void MotionApp::OpenNewBVH()
{
#ifdef WIN32
    char file_name[256] = "";
    OPENFILENAMEA ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "BVH Motion Data (*.bvh)\0*.bvh\0All (*.*)\0*.*\0";
    ofn.lpstrFile = file_name;
    ofn.nMaxFile = 256;
    ofn.lpstrTitle = "Select first BVH file";
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
    if (GetOpenFileNameA(&ofn)) LoadBVH(file_name);
#endif
}

void MotionApp::OpenNewBVH2()
{
#ifdef WIN32
    char file_name[256] = "";
    OPENFILENAMEA ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "BVH Motion Data (*.bvh)\0*.bvh\0All (*.*)\0*.*\0";
    ofn.lpstrFile = file_name;
    ofn.nMaxFile = 256;
    ofn.lpstrTitle = "Select second BVH file";
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
    if (GetOpenFileNameA(&ofn)) LoadBVH2(file_name);
#endif
}

void MotionApp::AlignInitialPositions() {
    if (!motion || !motion2 || motion->num_frames == 0 || motion2->num_frames == 0) 
        return;
    
    float y1 = motion->frames[0].root_pos.y;
    float y2 = motion2->frames[0].root_pos.y;
    float target_y = min(y1, y2);

    Point3f target_pos(0.0f, target_y, 0.0f);
    Point3f offset1 = motion->frames[0].root_pos - target_pos;
    Point3f offset2 = motion2->frames[0].root_pos - target_pos;

    for (int i = 0; i < motion->num_frames; i++)
        motion->frames[i].root_pos -= offset1; 
    for (int i = 0; i < motion2->num_frames; i++)
        motion2->frames[i].root_pos -= offset2; 

    printf("Initial positions aligned.\n");
}

void MotionApp::AlignInitialOrientations() {
    if (!motion || !motion2 || motion->num_frames == 0 || motion2->num_frames == 0) 
        return;
    
    Matrix3f ori1 = motion->frames[0].root_ori;
    float angle1 = -atan2(ori1.m02, ori1.m22);
    Matrix3f rot1; rot1.rotY(angle1);
    for (int i = 0; i < motion->num_frames; i++) {
        rot1.transform(&motion->frames[i].root_pos);
        motion->frames[i].root_ori.mul(rot1, motion->frames[i].root_ori);
    }
    
    Matrix3f ori2 = motion2->frames[0].root_ori;
    float angle2 = -atan2(ori2.m02, ori2.m22);
    Matrix3f rot2; rot2.rotY(angle2);
    for (int i = 0; i < motion2->num_frames; i++) {
        rot2.transform(&motion2->frames[i].root_pos);
        motion2->frames[i].root_ori.mul(rot2, motion2->frames[i].root_ori);
    }
    
    printf("Initial orientations aligned to face +Z axis.\n");
}

void MotionApp::CalculateWorldBounds() {
    if (!motion || !motion2) 
        return;

    float bounds[3][2];
    for (int i = 0; i < 3; ++i) {
        bounds[i][0] = 1e6;  
        bounds[i][1] = -1e6;
    }

    auto update_bounds = [&](const Motion* m) {
        vector<Matrix4f> seg_frames;
        for (int f = 0; f < m->num_frames; ++f) {
            ForwardKinematics(m->frames[f], seg_frames);
            for (int s = 0; s < m->body->num_segments; ++s) {
                Point3f p(seg_frames[s].m03, seg_frames[s].m13, seg_frames[s].m23);
                if (p.x < bounds[0][0]) bounds[0][0] = p.x;
                if (p.x > bounds[0][1]) bounds[0][1] = p.x;
                if (p.y < bounds[1][0]) bounds[1][0] = p.y;
                if (p.y > bounds[1][1]) bounds[1][1] = p.y;
                if (p.z < bounds[2][0]) bounds[2][0] = p.z;
                if (p.z > bounds[2][1]) bounds[2][1] = p.z;
            }
        }
    };
    
    update_bounds(motion);
    update_bounds(motion2);

    for (int i = 0; i < 3; ++i) {
        float range = bounds[i][1] - bounds[i][0];
        bounds[i][0] -= range * 0.05f;
        bounds[i][1] += range * 0.05f;
    }

    analyzer.SetWorldBounds(bounds);
    printf("World bounds calculated and set to Analyzer.\n");
}

void MotionApp::PrepareAllData() {
    if (!motion || !motion2) 
        return;
    AlignInitialPositions();
    AlignInitialOrientations();
    CalculateWorldBounds();
    
    bool cache_loaded = analyzer.LoadVoxelCache(motion->name.c_str(), motion2->name.c_str());
    
    if (!cache_loaded) {
        std::cout << "Cache not found. Calculating accumulated voxels (integrated)..." << std::endl;
        analyzer.AccumulateAllFrames(motion, motion2);
        std::cout << "Saving voxel cache..." << std::endl;
        analyzer.SaveVoxelCache(motion->name.c_str(), motion2->name.c_str());
    } else {
        std::cout << "Voxel cache loaded successfully. Skipping calculation." << std::endl;
    }
    
    UpdateVoxelDataWrapper();
}

void MotionApp::UpdateVoxelDataWrapper() {
    if (!motion || !motion2) 
        return;
    analyzer.UpdateVoxels(motion, motion2, animation_time);
}

void MotionApp::DrawText(int x, int y, const char *text, void *font)
{
    glRasterPos2i(x, y);
    for (const char* c = text; *c != '\0'; c++) 
        glutBitmapCharacter(font, *c);
}

bool MotionApp::HasVoxelData(int segment_index)
{
    if (!motion || !motion->body) 
        return false;
    if (segment_index < 0 || segment_index >= motion->body->num_segments) 
        return false;
    
    const Segment* segment = motion->body->segments[segment_index];
    return !IsFingerSegment(segment);
}

int MotionApp::GetNextValidSegment(int current_segment, int direction)
{
    if (!motion || !motion->body) 
        return -1;
    
    int num_segments = motion->body->num_segments;
    int next_segment = current_segment;
    
    for (int i = 0; i < num_segments; i++) {
        next_segment += direction;
        
        if (next_segment < 0)
            next_segment = num_segments - 1;
        else if (next_segment >= num_segments)
            next_segment = 0;
        
        if (HasVoxelData(next_segment))
            return next_segment;
    }
    
    return current_segment;
}

void MotionApp::ToggleSliceGizmo()
{
    use_slice_gizmo = !use_slice_gizmo;
    if (use_slice_gizmo && !analyzer.use_rotated_slice)
        analyzer.ToggleRotatedSliceMode();
    gizmo_dragging = false;
    slice_gizmo.SetSelectedAxis(GIZMO_NONE);
    SyncGizmoToSliceState();
}

void MotionApp::ToggleSliceGizmoMode()
{
    GizmoMode mode = slice_gizmo.GetMode();
    slice_gizmo.SetMode(mode == GIZMO_TRANSLATE ? GIZMO_ROTATE : GIZMO_TRANSLATE);
}

Matrix3f MotionApp::GetSliceGizmoOrientation() const
{
    Matrix3f ori;
    Vector3f u = analyzer.GetSlicePlaneU();
    Vector3f v = analyzer.GetSlicePlaneV();
    Vector3f n = analyzer.GetSlicePlaneNormal();

    if (u.length() < 1e-4f || v.length() < 1e-4f || n.length() < 1e-4f) {
        ori.setIdentity();
        return ori;
    }

    u.normalize();
    v.normalize();
    n.normalize();

    ori.m00 = u.x; ori.m10 = u.y; ori.m20 = u.z;
    ori.m01 = v.x; ori.m11 = v.y; ori.m21 = v.z;
    ori.m02 = n.x; ori.m12 = n.y; ori.m22 = n.z;
    return ori;
}

Point3f MotionApp::GetSliceGizmoPosition() const
{
    return analyzer.GetSlicePlaneCenter();
}

float MotionApp::ComputeGizmoScale() const
{
    float extent_x = analyzer.world_bounds[0][1] - analyzer.world_bounds[0][0];
    float extent_y = analyzer.world_bounds[1][1] - analyzer.world_bounds[1][0];
    float extent_z = analyzer.world_bounds[2][1] - analyzer.world_bounds[2][0];
    float max_extent = (std::max)(extent_x, (std::max)(extent_y, extent_z));
    if (max_extent <= 0.0f) 
        max_extent = 1.0f;
    return max_extent * 0.6f;
}

void MotionApp::ApplySliceGizmoDelta(const Point3f& translation, const Matrix4f& local_rotation)
{
    if (fabsf(translation.x) > 1e-6f || fabsf(translation.y) > 1e-6f || fabsf(translation.z) > 1e-6f)
        analyzer.ApplySlicePlaneTranslation(translation);

    if (slice_gizmo.GetMode() == GIZMO_ROTATE) {
        float trace = local_rotation.m00 + local_rotation.m11 + local_rotation.m22;
        if (fabsf(trace - 3.0f) > 0.0001f)
            analyzer.ApplySlicePlaneRotation(local_rotation);
    }
}

void MotionApp::SyncGizmoToSliceState()
{
    if (!analyzer.use_rotated_slice)
        slice_gizmo.SetMode(GIZMO_TRANSLATE);
}

void MotionApp::ProcessSpaceMouseInput()
{
    if (!use_spacemouse_slice)
        return;

    if (!analyzer.use_rotated_slice)
        analyzer.ToggleRotatedSliceMode();

    const Matrix4f& transform = GetSpaceMouseTransform();

    const float translation_sensitivity = 0.01f;
    const float deadzone = 0.001f;

    if (fabs(transform.m03) > deadzone || fabs(transform.m13) > deadzone || fabs(transform.m23) > deadzone) {
        Point3f translation(
            transform.m03 * translation_sensitivity,
            transform.m13 * translation_sensitivity,
            transform.m23 * translation_sensitivity
        );
        analyzer.ApplySlicePlaneTranslation(translation);
    }

    float trace = transform.m00 + transform.m11 + transform.m22;
    if (fabs(trace - 3.0f) > deadzone) {
        Matrix4f local_rotation;
        local_rotation.setIdentity();
        local_rotation.m00 = transform.m00; local_rotation.m01 = transform.m01; local_rotation.m02 = transform.m02;
        local_rotation.m10 = transform.m10; local_rotation.m11 = transform.m11; local_rotation.m12 = transform.m12;
        local_rotation.m20 = transform.m20; local_rotation.m21 = transform.m21; local_rotation.m22 = transform.m22;
        analyzer.ApplySlicePlaneRotation(local_rotation);
    }

    ResetSpaceMouseTransform();
}

void MotionApp::DrawPostureWithSegmentMode(Posture& posture, const Color3f& highlight_color)
{
    if (analyzer.show_segment_mode) {
        Color3f base_color(0.7f, 0.7f, 0.7f);
        
        std::vector<bool> combined_selection = analyzer.selected_segments;
        if (analyzer.selected_segment_index >= 0 && 
            analyzer.selected_segment_index < (int)combined_selection.size())
            combined_selection[analyzer.selected_segment_index] = true;
        
        int combined_count = 0;
        for (size_t i = 0; i < combined_selection.size(); ++i)
            if (combined_selection[i]) 
                combined_count++;
        
        if (combined_count > 0)
            DrawPostureMultiSelect(posture, combined_selection, highlight_color, base_color);
        else {
            glColor3f(highlight_color.x, highlight_color.y, highlight_color.z);
            DrawPosture(posture);
        }
    } else {
        glColor3f(highlight_color.x, highlight_color.y, highlight_color.z);
        DrawPosture(posture);
    }
}
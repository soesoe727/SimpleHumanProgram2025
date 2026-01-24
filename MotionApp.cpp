#include "MotionApp.h"
#include "BVH.h"
#include <string>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <iostream>

using namespace std;

static const float kPi = 3.14159265358979323846f;

// ADDED: インスタンスを保持する static ポインタ
static MotionApp* g_app_instance = NULL;

// ADDED: 'Special'関数を呼び出すための静的ラッパー
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
        case ' ': on_animation = !on_animation; break;
        case 'l': {OpenNewBVH(); OpenNewBVH2();} break;
    }

    // CTスキャン操作
    switch(key) {
        // 軸の切り替え（XY, YZ, XZ平面の順に循環）
        case 'x': 
            // XY平面 (h=X, v=Y, depth=Z) → YZ平面 (h=Z, v=Y, depth=X) → XZ平面 (h=X, v=Z, depth=Y) → XY平面...
            if (analyzer.h_axis == 0 && analyzer.v_axis == 1) {
                // XY → YZ (Z軸が横、Y軸が縦)
                analyzer.h_axis = 2;
                analyzer.v_axis = 1;
            } else if (analyzer.h_axis == 2 && analyzer.v_axis == 1) {
                // YZ → XZ
                analyzer.h_axis = 0;
                analyzer.v_axis = 2;
            } else {
                // XZ → XY
                analyzer.h_axis = 0;
                analyzer.v_axis = 1;
            }
            analyzer.ResetView(); 
            break;
            
        case 'z': 
            analyzer.projection_mode = !analyzer.projection_mode; 
            break;
        
        // スライスの移動（回転スライスモードでも動作）
        case 'w': 
            if(!analyzer.slice_positions.empty()) 
            if (analyzer.use_rotated_slice) {
                Vector3f n = analyzer.slice_plane_normal; n.normalize();
                analyzer.slice_plane_center.x += n.x * 0.02f;
                analyzer.slice_plane_center.y += n.y * 0.02f;
                analyzer.slice_plane_center.z += n.z * 0.02f;
            } else if(!analyzer.slice_positions.empty()) {
                analyzer.slice_positions[analyzer.active_slice_index] += 0.02f; 
            }
            break;
        case 's': 
            if(!analyzer.slice_positions.empty()) 
            if (analyzer.use_rotated_slice) {
                Vector3f n = analyzer.slice_plane_normal; n.normalize();
                analyzer.slice_plane_center.x -= n.x * 0.02f;
                analyzer.slice_plane_center.y -= n.y * 0.02f;
                analyzer.slice_plane_center.z -= n.z * 0.02f;
            } else if(!analyzer.slice_positions.empty()) {
                analyzer.slice_positions[analyzer.active_slice_index] -= 0.02f; 
            }
            break;
        case 't': 
            if(analyzer.slice_positions.size() > 1) 
                analyzer.slice_positions[1] += 0.02f; 
            break;
        case 'g': 
            if(analyzer.slice_positions.size() > 1) 
                analyzer.slice_positions[1] -= 0.02f; 
            break;
        
        // NEW: アクティブスライスの切り替え
        case 'q':
            analyzer.active_slice_index = (analyzer.active_slice_index + 1) % analyzer.slice_positions.size();
            break;

        // 特徴量・正規化モード
        case 'f': 
            analyzer.feature_mode = (analyzer.feature_mode + 1) % 2; 
            break;
        case 'n': 
            analyzer.norm_mode = (analyzer.norm_mode + 1) % 2; 
            break;
        
        // 表示切り替え
        case 'b': 
            analyzer.show_planes = !analyzer.show_planes; 
            break;
        case 'm': 
            analyzer.show_maps = !analyzer.show_maps; 
            break;
        case 'k': 
            analyzer.show_voxels = !analyzer.show_voxels; 
            break;
        
        // 回転スライスモードの切り替え
        case 'p':
            analyzer.ToggleRotatedSliceMode();
            break;
        
        // 平面の回転操作（回転スライスモード時のみ有効）
        case '1':  // X軸周りに+5度回転
            if (analyzer.use_rotated_slice)
                analyzer.RotateSlicePlane(5.0f, 0.0f, 0.0f);
            break;
        case '2':  // X軸周りに-5度回転
            if (analyzer.use_rotated_slice)
                analyzer.RotateSlicePlane(-5.0f, 0.0f, 0.0f);
            break;
        case '3':  // Y軸周りに+5度回転
            if (analyzer.use_rotated_slice)
                analyzer.RotateSlicePlane(0.0f, 5.0f, 0.0f);
            break;
        case '4':  // Y軸周りに-5度回転
            if (analyzer.use_rotated_slice)
                analyzer.RotateSlicePlane(0.0f, -5.0f, 0.0f);
            break;
        case '5':  // Z軸周りに+5度回転
            if (analyzer.use_rotated_slice)
                analyzer.RotateSlicePlane(0.0f, 0.0f, 5.0f);
            break;
        case '6':  // Z軸周りに-5度回転
            if (analyzer.use_rotated_slice)
                analyzer.RotateSlicePlane(0.0f, 0.0f, -5.0f);
            break;
        case '0':  // 回転リセット
            if (analyzer.use_rotated_slice)
                analyzer.ResetSliceRotation();
            break;
        
        // 部位別表示モード
        case 'e': 
            analyzer.show_segment_mode = !analyzer.show_segment_mode;
            // 部位別表示モードをオンにする時、最初の有効な部位を選択
            if (analyzer.show_segment_mode && motion) {
                // 現在の選択が無効な場合、最初の有効な部位を探す
                if (!HasVoxelData(analyzer.selected_segment_index)) {
                    analyzer.selected_segment_index = 0;
                    // 0が無効なら次の有効な部位を探す
                    if (!HasVoxelData(analyzer.selected_segment_index)) {
                        analyzer.selected_segment_index = GetNextValidSegment(analyzer.selected_segment_index, 1);
                    }
                }
            }
            break;
        case '[': 
            if (motion && analyzer.show_segment_mode) {
                // 前のボクセル情報を持つ部位へ移動
                analyzer.selected_segment_index = GetNextValidSegment(analyzer.selected_segment_index, -1);
            }
            break;
        case ']': 
            if (motion && analyzer.show_segment_mode) {
                // 次のボクセル情報を持つ部位へ移動
                analyzer.selected_segment_index = GetNextValidSegment(analyzer.selected_segment_index, 1);
            }
            break;
        case '\\':  // リセット（全体表示に戻す）
            analyzer.selected_segment_index = -1;
            analyzer.show_segment_mode = false;
            break;
        
        // ズーム・リセット
        case 'i': 
            analyzer.Zoom(0.9f); 
            break;
        case 'o': 
            analyzer.Zoom(1.1f); 
            break;
        case 'r': 
            analyzer.ResetView(); 
            break;

        // NEW: ギズモON/OFF
        case 'y':
            ToggleSliceGizmo();
            break;
        // NEW: ギズモモード切り替え
        case 'u':
            ToggleSliceGizmoMode();
            break;
    }
}

void MotionApp::Special(int key, int mx, int my)
{
    // パン操作
    float bounds_w = analyzer.world_bounds[analyzer.h_axis][1] - analyzer.world_bounds[analyzer.h_axis][0];
    float bounds_h = analyzer.world_bounds[analyzer.v_axis][1] - analyzer.world_bounds[analyzer.v_axis][0];
    float step_x = bounds_w * 0.05f;
    float step_y = bounds_h * 0.05f;

    switch(key)
    {
        case GLUT_KEY_LEFT:  analyzer.Pan(-step_x, 0); break;
        case GLUT_KEY_RIGHT: analyzer.Pan(step_x, 0); break;
        case GLUT_KEY_DOWN:  analyzer.Pan(0, -step_y); break;
        case GLUT_KEY_UP:    analyzer.Pan(0, step_y); break;
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
        Matrix3f rotation;
        slice_gizmo.UpdateDrag(mx, my, win_width, win_height, translation, rotation);
        ApplySliceGizmoDelta(translation, rotation);
    }

    GLUTBaseApp::MouseDrag(mx, my);
}

void MotionApp::MouseMotion(int mx, int my)
{
    GLUTBaseApp::MouseMotion(mx, my);
}

void MotionApp::Animation(float delta)
{
    if (!on_animation || drag_mouse_l || !motion || !motion2) return;
    animation_time += delta * animation_speed;
    float max_duration = max(motion->GetDuration(), motion2->GetDuration());
    if (animation_time >= max_duration) animation_time = 0.0f;
    if (animation_time < 0.0f) animation_time = 0.0f;
    motion->GetPosture(animation_time, *curr_posture);
    motion2->GetPosture(animation_time, *curr_posture2);
}

void MotionApp::Display()
{
    GLUTBaseApp::Display();
    
    // 1. 3Dモデルの描画
    if (curr_posture) {
        // MODIFIED: 部位別表示モードの場合は選択的描画
        if (analyzer.show_segment_mode && analyzer.selected_segment_index >= 0) {
            // 選択部位のみ赤色、他は灰色
            Color3f highlight_color(1.0f, 0.0f, 0.0f);  // 赤
            Color3f base_color(0.7f, 0.7f, 0.7f);       // 灰色
            DrawPostureSelective(*curr_posture, analyzer.selected_segment_index, 
                                 highlight_color, base_color);
        } else {
            // 通常表示：全体を赤色
            glColor3f(1.0f, 0.0f, 0.0f); 
            DrawPosture(*curr_posture);
        }
        glColor3f(0.0f, 0.0f, 0.0f);
        DrawPostureShadow(*curr_posture, shadow_dir, shadow_color); 
    }
    if (curr_posture2) {
        // MODIFIED: 部位別表示モードの場合は選択的描画
        if (analyzer.show_segment_mode && analyzer.selected_segment_index >= 0) {
            // 選択部位のみ青色、他は灰色
            Color3f highlight_color(0.0f, 0.0f, 1.0f);  // 青
            Color3f base_color(0.7f, 0.7f, 0.7f);       // 灰色
            DrawPostureSelective(*curr_posture2, analyzer.selected_segment_index, 
                                 highlight_color, base_color);
        } else {
            // 通常表示：全体を青色
            glColor3f(0.0f, 0.0f, 1.0f);
            DrawPosture(*curr_posture2);
        }
        glColor3f(0.0f, 0.0f, 0.0f);
        DrawPostureShadow(*curr_posture2, shadow_dir, shadow_color); 
    }

    // 2. Analyzerによる3D描画
    if (analyzer.show_planes) {
        analyzer.DrawSlicePlanes();
    }
    UpdateVoxelDataWrapper();
    analyzer.DrawVoxels3D();

    // 2.5 ギズモの描画
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

    // 3. 2D UIの描画（CTマップ）
    if (analyzer.show_maps) {
        analyzer.DrawCTMaps(win_width, win_height);
    }

    // 4. 情報テキストの表示
    char title[512];
    int d_axis = 3 - analyzer.h_axis - analyzer.v_axis;
    const char* slice_mode_str = analyzer.projection_mode ? "Projection" : "Slice";
    const char* feature_mode_str = (analyzer.feature_mode == 0) ? "Occupancy" : "Speed";
    const char* norm_mode_str = (analyzer.norm_mode == 0) ? "CurrentFrame" : "Accumulated";
    const char* planes_on_str = (analyzer.show_planes) ? "ON" : "OFF";
    const char* maps_on_str = (analyzer.show_maps) ? "ON" : "OFF";
    const char* voxels_on_str = (analyzer.show_voxels) ? "ON" : "OFF";
    const char* segment_mode_str = (analyzer.show_segment_mode) ? "ON" : "OFF";
    const char* rotated_slice_str = (analyzer.use_rotated_slice) ? "ON" : "OFF";
    const char* gizmo_str = (use_slice_gizmo && analyzer.use_rotated_slice) ? "ON" : "OFF";
    const char* gizmo_mode_str = (slice_gizmo.GetMode() == GIZMO_TRANSLATE) ? "Move" : "Rotate";
    
    float s1 = (analyzer.slice_positions.size() > 0) ? analyzer.slice_positions[0] : 0.0f;
    float s2 = (analyzer.slice_positions.size() > 1) ? analyzer.slice_positions[1] : 0.0f;

    // 部位情報を追加
    char segment_info[128] = "All";
    if (analyzer.show_segment_mode && analyzer.selected_segment_index >= 0 && motion) {
        sprintf(segment_info, "%s[%d]", 
                motion->body->segments[analyzer.selected_segment_index]->name.c_str(),
                analyzer.selected_segment_index);
    }
    
    // 回転情報
    char rotation_info[64] = "";
    if (analyzer.use_rotated_slice) {
        sprintf(rotation_info, " | Rot:X%.0f Y%.0f Z%.0f", 
                analyzer.slice_rotation_x, analyzer.slice_rotation_y, analyzer.slice_rotation_z);
    }

    sprintf(title, "CT-Scan | H:%c V:%c | RotSlice:%s%s | Gizmo:%s(%s) | Feature:%s | Data:%s | Seg:%s | Planes:%s | Maps:%s | Voxels:%s",
        'X' + analyzer.h_axis, 'X' + analyzer.v_axis, 
        rotated_slice_str, rotation_info, gizmo_str, gizmo_mode_str,
        feature_mode_str, norm_mode_str, segment_info,
        planes_on_str, maps_on_str, voxels_on_str);
    
    DrawTextInformation(0, title);
    if (motion) { 
        char msg[64]; 
        sprintf(msg, "Time:%.2f(%d)", animation_time, (int)(animation_time / motion->interval)); 
        DrawTextInformation(1, msg); 
    }
}

void MotionApp::LoadBVH(const char* file_name) {
	Motion* new_motion = LoadAndCoustructBVHMotion(file_name);
	if (!new_motion) return;
	if (motion) { if (motion->body) delete motion->body; delete motion; }
	if (curr_posture) delete curr_posture;
	motion = new_motion;
	curr_posture = new Posture(motion->body);
	Start();
}

void MotionApp::LoadBVH2(const char* file_name)
{ 
	if (!motion) return;
	Motion* m2 = LoadAndCoustructBVHMotion(file_name);
	if (!m2) return;
	if (motion2) delete motion2; 
	if (curr_posture2) delete curr_posture2;
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
	if (!motion || !motion2 || motion->num_frames == 0 || motion2->num_frames == 0) return;
	
    float y1 = motion->frames[0].root_pos.y;
    float y2 = motion2->frames[0].root_pos.y;
    float target_y = min(y1, y2);

    Point3f target_pos(0.0f, target_y, 0.0f);
    Point3f offset1 = motion->frames[0].root_pos - target_pos;
    Point3f offset2 = motion2->frames[0].root_pos - target_pos;

	for (int i = 0; i < motion->num_frames; i++) { 
        motion->frames[i].root_pos -= offset1; 
    }
	for (int i = 0; i < motion2->num_frames; i++) { 
        motion2->frames[i].root_pos -= offset2; 
    }

	printf("Initial positions aligned.\n");
}

void MotionApp::AlignInitialOrientations() {
	if (!motion || !motion2 || motion->num_frames == 0 || motion2->num_frames == 0) return;
	
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
	if (!motion || !motion2) return;

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
    if (!motion || !motion2) return;
    AlignInitialPositions();
    AlignInitialOrientations();
    CalculateWorldBounds();
    
    bool cache_loaded = analyzer.LoadVoxelCache(motion->name.c_str(), motion2->name.c_str());
    
    if (!cache_loaded) {
        std::cout << "Cache not found. Calculating accumulated voxels for entire motion..." << std::endl;
        analyzer.AccumulatePresenceAllFrames(motion, motion2);
        
        std::cout << "Calculating accumulated speed for entire motion..." << std::endl;
        analyzer.AccumulateSpeedAllFrames(motion, motion2);
        
        std::cout << "Calculating accumulated voxels by segment..." << std::endl;
        analyzer.AccumulatePresenceBySegmentAllFrames(motion, motion2);
        
        std::cout << "Calculating accumulated speed by segment..." << std::endl;
        analyzer.AccumulateSpeedBySegmentAllFrames(motion, motion2);
        
        std::cout << "Saving voxel cache..." << std::endl;
        analyzer.SaveVoxelCache(motion->name.c_str(), motion2->name.c_str());
    } else {
        std::cout << "Voxel cache loaded successfully. Skipping calculation." << std::endl;
    }
    
    UpdateVoxelDataWrapper();
}

void MotionApp::UpdateVoxelDataWrapper() {
    if(!motion || !motion2) return;
    analyzer.UpdateVoxels(motion, motion2, animation_time);
}

void MotionApp::DrawText(int x, int y, const char *text, void *font)
{
    glRasterPos2i(x, y);
    for (const char* c = text; *c != '\0'; c++) 
        glutBitmapCharacter(font, *c);
}

// ボクセル情報を持つ部位かチェック（指などの細部を除外）
bool MotionApp::HasVoxelData(int segment_index)
{
    if (!motion || !motion->body) return false;
    if (segment_index < 0 || segment_index >= motion->body->num_segments) return false;
    
    // MODIFIED: 体節名ベースで指かどうか判定（BVHファイルによる体節数の違いに対応）
    const Segment* segment = motion->body->segments[segment_index];
    return !IsFingerSegment(segment);
}

// 次の有効な部位を取得（ボクセル情報を持つ部位のみ）
int MotionApp::GetNextValidSegment(int current_segment, int direction)
{
    if (!motion || !motion->body) return -1;
    
    int num_segments = motion->body->num_segments;
    int next_segment = current_segment;
    
    // 最大num_segments回ループして次の有効な部位を探す（無限ループ防止）
    for (int i = 0; i < num_segments; i++) {
        next_segment += direction;
        
        // 範囲外になったらラップアラウンド
        if (next_segment < 0) {
            next_segment = num_segments - 1;
        } else if (next_segment >= num_segments) {
            next_segment = 0;
        }
        
        // 有効な部位が見つかったら返す
        if (HasVoxelData(next_segment)) {
            return next_segment;
        }
    }
    
    // 有効な部位が見つからなかった場合は現在の部位を返す
    return current_segment;
}

void MotionApp::ToggleSliceGizmo()
{
    use_slice_gizmo = !use_slice_gizmo;
    if (use_slice_gizmo && !analyzer.use_rotated_slice) {
        analyzer.ToggleRotatedSliceMode();
    }
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
    Vector3f u = analyzer.slice_plane_u;
    Vector3f v = analyzer.slice_plane_v;
    Vector3f n = analyzer.slice_plane_normal;

    if (u.length() < 1e-4f || v.length() < 1e-4f || n.length() < 1e-4f) {
        ori.setIdentity();
        return ori;
    }

    u.normalize();
    v.normalize();
    n.normalize();

    // 列ベクトルとして設定
    ori.m00 = u.x; ori.m10 = u.y; ori.m20 = u.z;
    ori.m01 = v.x; ori.m11 = v.y; ori.m21 = v.z;
    ori.m02 = n.x; ori.m12 = n.y; ori.m22 = n.z;
    return ori;
}

Point3f MotionApp::GetSliceGizmoPosition() const
{
    Point3f pos = analyzer.slice_plane_center;

    // 回転スライスモード: スライス中心 + パン
    if (analyzer.use_rotated_slice) {
        pos.x += analyzer.slice_plane_u.x * analyzer.pan_center.x + analyzer.slice_plane_v.x * analyzer.pan_center.y;
        pos.y += analyzer.slice_plane_u.y * analyzer.pan_center.x + analyzer.slice_plane_v.y * analyzer.pan_center.y;
        pos.z += analyzer.slice_plane_u.z * analyzer.pan_center.x + analyzer.slice_plane_v.z * analyzer.pan_center.y;
        return pos;
    }

    // 非回転モード: アクティブスライスの位置を使用
    if (!analyzer.slice_positions.empty()) {
        int depth_axis = 3 - analyzer.h_axis - analyzer.v_axis;
        float depth = analyzer.slice_positions[analyzer.active_slice_index];
        pos.x = (analyzer.world_bounds[0][0] + analyzer.world_bounds[0][1]) * 0.5f;
        pos.y = (analyzer.world_bounds[1][0] + analyzer.world_bounds[1][1]) * 0.5f;
        pos.z = (analyzer.world_bounds[2][0] + analyzer.world_bounds[2][1]) * 0.5f;
        if (depth_axis == 0) pos.x = depth;
        if (depth_axis == 1) pos.y = depth;
        if (depth_axis == 2) pos.z = depth;
    }

    return pos;
}

float MotionApp::ComputeGizmoScale() const
{
    float extent_x = analyzer.world_bounds[0][1] - analyzer.world_bounds[0][0];
    float extent_y = analyzer.world_bounds[1][1] - analyzer.world_bounds[1][0];
    float extent_z = analyzer.world_bounds[2][1] - analyzer.world_bounds[2][0];
    float max_extent = (std::max)(extent_x, (std::max)(extent_y, extent_z));
    if (max_extent <= 0.0f) max_extent = 1.0f;
    // enlarged scale for better UI visibility
    return max_extent * 0.6;
}

static float ExtractAxisAngleDeg(const Matrix3f& rot, GizmoAxis axis)
{
    float angle_rad = 0.0f;
    switch (axis) {
        case GIZMO_X:
            angle_rad = atan2(rot.m21, rot.m22);
            break;
        case GIZMO_Y:
            angle_rad = atan2(rot.m02, rot.m00);
            break;
        case GIZMO_Z:
            angle_rad = atan2(rot.m10, rot.m00);
            break;
        default:
            break;
    }
    return angle_rad * 180.0f / kPi;
}

void MotionApp::ApplySliceGizmoDelta(const Point3f& translation, const Matrix3f& rotation)
{
    // 平行移動
    analyzer.slice_plane_center.x += translation.x;
    analyzer.slice_plane_center.y += translation.y;
    analyzer.slice_plane_center.z += translation.z;

    // 回転（回転ギズモのみ）
    if (slice_gizmo.GetMode() == GIZMO_ROTATE) {
        Vector3f u = analyzer.slice_plane_u;
        Vector3f v = analyzer.slice_plane_v;
        Vector3f n = analyzer.slice_plane_normal;
        rotation.transform(&u);
        rotation.transform(&v);
        rotation.transform(&n);
        u.normalize();
        v.normalize();
        n.normalize();
        analyzer.slice_plane_u = u;
        analyzer.slice_plane_v = v;
        analyzer.slice_plane_normal = n;

        float delta_deg = ExtractAxisAngleDeg(rotation, slice_gizmo.GetSelectedAxis());
        if (slice_gizmo.GetSelectedAxis() == GIZMO_X) analyzer.slice_rotation_x += delta_deg;
        if (slice_gizmo.GetSelectedAxis() == GIZMO_Y) analyzer.slice_rotation_y += delta_deg;
        if (slice_gizmo.GetSelectedAxis() == GIZMO_Z) analyzer.slice_rotation_z += delta_deg;
    }

    analyzer.UpdateSlicePlaneVectors();
}

void MotionApp::SyncGizmoToSliceState()
{
    if (!analyzer.use_rotated_slice) {
        slice_gizmo.SetMode(GIZMO_TRANSLATE);
        return;
    }

    // use rotation gizmo by default when rotated slice mode is on
    if (slice_gizmo.GetMode() == GIZMO_TRANSLATE && analyzer.use_rotated_slice) {
        // keep current mode; user can toggle with 'u'
    }
}
#include "MotionApp.h"
#include "BVH.h"
#include <string>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <iostream>

using namespace std;

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
        
        // スライスの移動
        case 'w': 
            if(!analyzer.slice_positions.empty()) 
                analyzer.slice_positions[0] += 0.01f; 
            break;
        case 's': 
            if(!analyzer.slice_positions.empty()) 
                analyzer.slice_positions[0] -= 0.01f; 
            break;
        case 't': 
            if(analyzer.slice_positions.size() > 1) 
                analyzer.slice_positions[1] += 0.01f; 
            break;
        case 'g': 
            if(analyzer.slice_positions.size() > 1) 
                analyzer.slice_positions[1] -= 0.01f; 
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
        
        // 部位別表示モード
        case 'e': 
            analyzer.show_segment_mode = !analyzer.show_segment_mode; 
            break;
        case '[': 
            if (motion && analyzer.show_segment_mode) {
                analyzer.selected_segment_index--;
                if (analyzer.selected_segment_index < 0) {
                    analyzer.selected_segment_index = motion->body->num_segments - 1;
                }
            }
            break;
        case ']': 
            if (motion && analyzer.show_segment_mode) {
                analyzer.selected_segment_index++;
                if (analyzer.selected_segment_index >= motion->body->num_segments) {
                    analyzer.selected_segment_index = 0;
                }
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
        DrawPosture(*curr_posture); 
        DrawPostureShadow(*curr_posture, shadow_dir, shadow_color); 
    }
	if (curr_posture2) { 
        DrawPosture(*curr_posture2); 
        DrawPostureShadow(*curr_posture2, shadow_dir, shadow_color); 
    }

	// 2. Analyzerによる3D描画
    if (analyzer.show_planes) {
        analyzer.DrawSlicePlanes();
    }
    UpdateVoxelDataWrapper();
    analyzer.DrawVoxels3D();

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
    
    float s1 = (analyzer.slice_positions.size() > 0) ? analyzer.slice_positions[0] : 0.0f;
    float s2 = (analyzer.slice_positions.size() > 1) ? analyzer.slice_positions[1] : 0.0f;

    // 部位情報を追加
    char segment_info[128] = "All";
    if (analyzer.show_segment_mode && analyzer.selected_segment_index >= 0 && motion) {
        sprintf(segment_info, "%s[%d]", 
                motion->body->segments[analyzer.selected_segment_index]->name.c_str(),
                analyzer.selected_segment_index);
    }

    sprintf(title, "CT-Scan Mode | H:%c V:%c | Mode:%s | Feature:%s | Data:%s | SegMode:%s | Seg:%s | Planes:%s | Maps:%s | Voxels:%s | S1:%.2f S2:%.2f",
        'X' + analyzer.h_axis, 'X' + analyzer.v_axis, slice_mode_str, 
        feature_mode_str, norm_mode_str, segment_mode_str, segment_info,
        planes_on_str, maps_on_str, voxels_on_str, s1, s2);
	
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
        analyzer.AccumulateVoxelsAllFrames(motion, motion2);
        
        std::cout << "Calculating accumulated voxels by segment..." << std::endl;
        analyzer.AccumulateVoxelsBySegmentAllFrames(motion, motion2);
        
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
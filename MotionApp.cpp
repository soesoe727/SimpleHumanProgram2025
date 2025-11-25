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

// 軸の値を取得するヘルパー (プロット用)
static float get_axis_value(const Point3f& p, int axis_index) 
{
	if (axis_index == 0) return p.x; 
	if (axis_index == 1) return p.y;
	return p.z; 
}

static const char* get_axis_name(int axis_index)
{ 
    if (axis_index == 0) return "X"; 
	if (axis_index == 1) return "Y";
	if (axis_index == 2) return "Z";
	return "Frame"; 
}

static Color3f GetHeatmapColor(float value)
{ 
	Color3f color; 
	value = max(0.0f, min(1.0f, value));
	if (value < 0.5f) 
		color.set(0.0f, value * 2.0f, 1.0f - (value * 2.0f)); 
	else 
		color.set((value - 0.5f) * 2.0f, 1.0f - ((value - 0.5f) * 2.0f), 0.0f); 
	return color; 
}

MotionApp::MotionApp() {
	g_app_instance = this;
	app_name = "Motion Analysis App";
    motion = NULL; motion2 = NULL; curr_posture = NULL; curr_posture2 = NULL;
    on_animation = true; animation_time = 0.0f; animation_speed = 1.0f;
    view_mode = 2; 
    
    plot_segment_index = 0; 
    plot_vertical_axis = 1; 
    plot_horizontal_axis = 3;
    cmap_min_diff = 0.0f; 
    cmap_max_diff = 1.0f;

    // Analyzerは SpatialAnalyzer のコンストラクタで初期化されるため
    // ここでの追加設定は不要です。
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
	
    // ビューモードの切り替えなど
	switch (key) {
		case ' ': on_animation = !on_animation; break;
		case 'c': view_mode = (view_mode + 1) % 3; break;
		case 'v': plot_vertical_axis = (plot_vertical_axis + 1) % 3; break;
		case 'h': plot_horizontal_axis = (plot_horizontal_axis + 1) % 4; break;
		case 'p': if (motion) { plot_segment_index = (plot_segment_index + 1) % motion->body->num_segments; PreparePlotData(); } break;
        case 'q': if (motion) { plot_segment_index = (plot_segment_index - 1 + motion->body->num_segments) % motion->body->num_segments; PreparePlotData(); } break;
		case 'l': {OpenNewBVH(); OpenNewBVH2();} break;
	}

    // CTスキャンモード時の操作 (analyzer に委譲)
	if (view_mode == 2) {
		switch(key) {
            // 軸の切り替え
			case 'x': analyzer.h_axis = (analyzer.h_axis + 1) % 3; if (analyzer.h_axis == analyzer.v_axis) analyzer.h_axis = (analyzer.h_axis + 1) % 3; analyzer.ResetView(); break;
			case 'y': analyzer.v_axis = (analyzer.v_axis + 1) % 3; if (analyzer.v_axis == analyzer.h_axis) analyzer.v_axis = (analyzer.v_axis + 1) % 3; analyzer.ResetView(); break;
			case 'z': analyzer.projection_mode = !analyzer.projection_mode; break;
			
            // スライスの移動 (Active sliceの概念がない場合は要調整)
			case 'w': if(!analyzer.slice_positions.empty()) analyzer.slice_positions[0] += 0.01f; break;
			case 's': if(!analyzer.slice_positions.empty()) analyzer.slice_positions[0] -= 0.01f; break;
            case 't': if(analyzer.slice_positions.size() > 1) analyzer.slice_positions[1] += 0.01f; break;
            case 'g': if(analyzer.slice_positions.size() > 1) analyzer.slice_positions[1] -= 0.01f; break;

            // 特徴量・正規化モード
            case 'f': analyzer.feature_mode = (analyzer.feature_mode + 1) % 2; break;
			case 'n': analyzer.norm_mode = (analyzer.norm_mode + 1) % 2; break;
            
            // 表示切り替え
			case 'b': analyzer.show_planes = !analyzer.show_planes; break;
			case 'm': analyzer.show_maps = !analyzer.show_maps; break;
			case 'k': analyzer.show_voxels = !analyzer.show_voxels; break;
			
            // NEW: 部位別表示モード
            case 'e': analyzer.show_segment_mode = !analyzer.show_segment_mode; break;
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
            case 'i': analyzer.Zoom(0.9f); break;
            case 'o': analyzer.Zoom(1.1f); break;
            case 'r': analyzer.ResetView(); break;
		}
	}
}

void MotionApp::Special(int key, int mx, int my)
{
    if (view_mode != 2) return;
    
    // パン操作 (analyzerに委譲)
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
	if (curr_posture) { DrawPostureSelective(*curr_posture, plot_segment_index, Color3f(0.8f,0.2f,0.2f), Color3f(0.7f,0.7f,0.7f)); DrawPostureShadow(*curr_posture, shadow_dir, shadow_color); }
	if (curr_posture2) { DrawPostureSelective(*curr_posture2, plot_segment_index, Color3f(0.2f,0.2f,0.8f), Color3f(0.6f,0.6f,0.6f)); DrawPostureShadow(*curr_posture2, shadow_dir, shadow_color); }

	// 2. Analyzerによる3D描画
    if (view_mode == 2) {
        // 2-1. スライス平面
        if (analyzer.show_planes) {
            analyzer.DrawSlicePlanes();
        }
        // 2-2. 3Dボクセル描画（NEW）
        UpdateVoxelDataWrapper();
        analyzer.DrawVoxels3D();
    }

    // 3. 2D UIの描画
	if (view_mode == 0) DrawPositionPlot();
	else if (view_mode == 1) DrawColormap();
	else if (view_mode == 2) { 
        // 2Dマップ描画
        if (analyzer.show_maps) {
            analyzer.DrawCTMaps(win_width, win_height);
        }
	}

    // 4. 情報テキストの表示
	char title[512]; const char* mode_str[] = {"Plot", "Colormap", "CT-Scan"};
	if (view_mode == 2) {
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

        // NEW: 部位情報を追加
        char segment_info[128] = "All";
        if (analyzer.show_segment_mode && analyzer.selected_segment_index >= 0 && motion) {
            sprintf(segment_info, "%s[%d]", 
                    motion->body->segments[analyzer.selected_segment_index]->name.c_str(),
                    analyzer.selected_segment_index);
        }

		sprintf(title, "View:%s|H:%s V:%s|Mode:%s|Feature:%s|Data:%s|SegMode:%s|Seg:%s|Planes:%s|Maps:%s|Voxels:%s|'e':SegMode '['/']':SelectSeg '\\':ResetSeg|S1:%.2f S2:%.2f",
            mode_str[view_mode], get_axis_name(analyzer.h_axis), get_axis_name(analyzer.v_axis), slice_mode_str, 
            feature_mode_str, norm_mode_str, segment_mode_str, segment_info,
            planes_on_str, maps_on_str, voxels_on_str,
            s1, s2);
	} else {
		sprintf(title, "View:%s('c')|Plotting:%s('p'/'q')|V-Axis:%s('v')|H-Axis:%s('h')", 
			mode_str[view_mode], motion ? motion->body->segments[plot_segment_index]->name.c_str() : "N/A",
			get_axis_name(plot_vertical_axis), get_axis_name(plot_horizontal_axis)); 
    }
	
	DrawTextInformation(0, title);
	if (motion) { char msg[64]; sprintf(msg, "Time:%.2f(%d)", animation_time, (int)(animation_time / motion->interval)); DrawTextInformation(1, msg); }
}

//
//  BVH動作ファイルの読み込み
//
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

//
// ２つのモーションの初期位置を合わせる
//
void MotionApp::AlignInitialPositions() {
	if (!motion || !motion2 || motion->num_frames == 0 || motion2->num_frames == 0) return;
	
    // 低い方のY座標に合わせてXZ位置をリセット
    float y1 = motion->frames[0].root_pos.y;
    float y2 = motion2->frames[0].root_pos.y;
    float target_y = min(y1, y2);

    Point3f target_pos(0.0f, target_y, 0.0f);
    Point3f offset1 = motion->frames[0].root_pos - target_pos;
    Point3f offset2 = motion2->frames[0].root_pos - target_pos;

	for (int i = 0; i < motion->num_frames; i++) { motion->frames[i].root_pos -= offset1; }
	for (int i = 0; i < motion2->num_frames; i++) { motion2->frames[i].root_pos -= offset2; }

	printf("Initial positions aligned.\n");
}

//
//２つのモーションの初期の向きを合わせる
//
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

    // 一時的なバウンズ配列
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

    // 計算結果をAnalyzerに渡す
    analyzer.SetWorldBounds(bounds);
	printf("World bounds calculated and set to Analyzer.\n");
}

void MotionApp::PrepareAllData() {
    if (!motion || !motion2) return;
    AlignInitialPositions();
    AlignInitialOrientations();
    PreparePlotData();
    PrepareColormapData();
    CalculateWorldBounds();
    
    // 動作全体の累積ボクセルを事前計算
    std::cout << "Calculating accumulated voxels for entire motion..." << std::endl;
    analyzer.AccumulateVoxelsAllFrames(motion, motion2);
    
    // NEW: 部位ごとの累積ボクセルも計算
    std::cout << "Calculating accumulated voxels by segment..." << std::endl;
    analyzer.AccumulateVoxelsBySegmentAllFrames(motion, motion2);
    
    UpdateVoxelDataWrapper();
}

// ボクセルデータ更新のラッパー
void MotionApp::UpdateVoxelDataWrapper() {
    if(!motion || !motion2) return;
    analyzer.UpdateVoxels(motion, motion2, animation_time);
}

//
// プロット用データを事前に計算
//
void MotionApp::PreparePlotData()
{
	if (!motion || !motion2) return;
	
	plot_data1.clear();
	plot_data2.clear();

	if (plot_segment_index >= motion->body->num_segments) {
		plot_segment_index = 0;
	}
	
	vector<Matrix4f> seg_frames;
	for (int i = 0; i < motion->num_frames; ++i) {
		ForwardKinematics(motion->frames[i], seg_frames);
		const Matrix4f& frame = seg_frames[plot_segment_index];
		plot_data1.push_back(Point3f(frame.m03, frame.m13, frame.m23));
	}
	for (int i = 0; i < motion2->num_frames; ++i) {
		ForwardKinematics(motion2->frames[i], seg_frames);
		const Matrix4f& frame = seg_frames[plot_segment_index];
		plot_data2.push_back(Point3f(frame.m03, frame.m13, frame.m23));
	}
}

//
// 2Dプロットを描画
//
void MotionApp::DrawPositionPlot()
{
	if (plot_data1.empty() || plot_data2.empty()) return;

	BeginScreenMode();

	vector<float> h1, v1, h2, v2;
	for(size_t i = 0; i < plot_data1.size(); ++i) {
		v1.push_back(get_axis_value(plot_data1[i], plot_vertical_axis));
		h1.push_back( (plot_horizontal_axis == 3) ? (float)i : get_axis_value(plot_data1[i], plot_horizontal_axis) );
	}
	for(size_t i = 0; i < plot_data2.size(); ++i) {
		v2.push_back(get_axis_value(plot_data2[i], plot_vertical_axis));
		h2.push_back( (plot_horizontal_axis == 3) ? (float)i : get_axis_value(plot_data2[i], plot_horizontal_axis) );
	}

	float h_min = h1[0], h_max = h1[0], v_min = v1[0], v_max = v1[0];
	for(float val : h1) { if(val < h_min) h_min = val; if(val > h_max) h_max = val; }
	for(float val : v1) { if(val < v_min) v_min = val; if(val > v_max) v_max = val; }
	for(float val : h2) { if(val < h_min) h_min = val; if(val > h_max) h_max = val; }
	for(float val : v2) { if(val < v_min) v_min = val; if(val > v_max) v_max = val; }
	
	int plot_size = min(win_width / 2, win_height / 2);
	
	int plot_w, plot_h;
	if (plot_horizontal_axis == 3) { // 横軸がフレームの場合
		plot_h = plot_size;
		plot_w = (int)((float)plot_size * 2.0f);
	} else { // 横軸が座標の場合
		plot_h = plot_size;
		plot_w = plot_size;
	}
	int plot_margin = 50;
	int plot_x = plot_margin;
	int plot_y = win_height - plot_margin - plot_h;

	float padding_factor = 0.01f;

	if (plot_horizontal_axis == 3) {
		float v_range = v_max - v_min; if (v_range < 0.01f) v_range = 0.01f;
		h_min = 0.0f;
		h_max += (h_max - h_min) * padding_factor;
		v_min -= v_range * padding_factor;
		v_max += v_range * padding_factor;
	} else {
		float h_range_data = h_max - h_min; if (h_range_data < 1e-5f) h_range_data = 1e-5f;
		float v_range_data = v_max - v_min; if (v_range_data < 1e-5f) v_range_data = 1e-5f;
		float screen_aspect = (float)plot_w / (float)plot_h;
		float data_aspect = h_range_data / v_range_data;
		float h_center = (h_min + h_max) / 2.0f;
		float v_center = (v_min + v_max) / 2.0f;
		if (data_aspect > screen_aspect) { v_range_data = h_range_data / screen_aspect; } 
        else { h_range_data = v_range_data * screen_aspect; }
		h_min = h_center - h_range_data / 2.0f; h_max = h_center + h_range_data / 2.0f;
		v_min = v_center - v_range_data / 2.0f; v_max = v_center + v_range_data / 2.0f;
		float h_padding = (h_max - h_min) * padding_factor;
		float v_padding = (v_max - v_min) * padding_factor;
		h_min -= h_padding; h_max += h_padding;
		v_min -= v_padding; v_max += v_padding;
	}

	float h_range = h_max - h_min; if(h_range < 1e-5f) h_range = 1e-5f;
	float v_range = v_max - v_min; if(v_range < 1e-5f) v_range = 1e-5f;
	
	DrawPlotAxes(plot_x, plot_y, plot_w, plot_h, h_min, h_max, v_min, v_max, get_axis_name(plot_horizontal_axis), get_axis_name(plot_vertical_axis));

	auto draw_line_strip = [&](const vector<float>& h_vals, const vector<float>& v_vals) {
		glBegin(GL_LINE_STRIP);
		for (size_t i = 0; i < h_vals.size(); ++i) {
			float sx = plot_x + ((h_vals[i] - h_min) / h_range) * plot_w;
			float sy = (plot_y + plot_h) - ((v_vals[i] - v_min) / v_range) * plot_h;
			glVertex2f(sx, sy);
		}
		glEnd();
	};

	glColor3f(0.8f, 0.2f, 0.2f); draw_line_strip(h1, v1);
	glColor3f(0.2f, 0.2f, 0.8f); draw_line_strip(h2, v2);

	int frame_idx = static_cast<int>(animation_time / motion->interval);
	
	if (plot_horizontal_axis == 3) {
		float line_x = plot_x + (( (float)frame_idx - h_min) / h_range) * plot_w;
		if (line_x >= plot_x && line_x <= plot_x + plot_w) {
			glLineWidth(2.0f);
			glColor3f(1.0f, 1.0f, 0.2f);
			glBegin(GL_LINES); glVertex2f(line_x, plot_y); glVertex2f(line_x, plot_y + plot_h); glEnd();
		}
	} else {
		glPointSize(8.0f);
		glColor3f(1.0f, 1.0f, 0.2f);
		glBegin(GL_POINTS);
		if (frame_idx < h1.size()) {
			float sx = plot_x + ((h1[frame_idx] - h_min) / h_range) * plot_w;
			float sy = (plot_y + plot_h) - ((v1[frame_idx] - v_min) / v_range) * plot_h;
			glVertex2f(sx, sy);
		}
		if (frame_idx < h2.size()) {
			float sx = plot_x + ((h2[frame_idx] - h_min) / h_range) * plot_w;
			float sy = (plot_y + plot_h) - ((v2[frame_idx] - v_min) / v_range) * plot_h;
			glVertex2f(sx, sy);
		}
		glEnd();
	}
	EndScreenMode();
}

void MotionApp::DrawPlotAxes(int x, int y, int w, int h, float h_min, float h_max, float v_min, float v_max, const char* h_label, const char* v_label)
{
	glColor4f(0.2f, 0.2f, 0.2f, 0.5f);
	glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBegin(GL_QUADS); glVertex2f(x, y); glVertex2f(x, y + h); glVertex2f(x + w, y + h); glVertex2f(x + w, y); glEnd();
	glDisable(GL_BLEND);

	glColor3f(0.8f, 0.8f, 0.8f);
	glBegin(GL_LINE_LOOP); glVertex2f(x, y); glVertex2f(x, y + h); glVertex2f(x + w, y + h); glVertex2f(x + w, y); glEnd();

	char label[64];
	glColor3f(0.0f, 0.0f, 0.0f);
	sprintf(label, "%s: %.2f", h_label, h_max); DrawText(x + w - 70, y + h + 15, label, GLUT_BITMAP_HELVETICA_10);
	sprintf(label, "%.2f", h_min); DrawText(x, y + h + 15, label, GLUT_BITMAP_HELVETICA_10);
	sprintf(label, "%s: %.2f", v_label, v_max); DrawText(x - 45, y + 12, label, GLUT_BITMAP_HELVETICA_10);
	sprintf(label, "%.2f", v_min); DrawText(x - 45, y + h, label, GLUT_BITMAP_HELVETICA_10);
}

void MotionApp::BeginScreenMode()
{
	glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
	gluOrtho2D(0.0, win_width, win_height, 0.0);
	glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT);
	glDisable(GL_DEPTH_TEST); glDisable(GL_LIGHTING);
}

void MotionApp::EndScreenMode()
{
	glPopAttrib();
	glMatrixMode(GL_PROJECTION); glPopMatrix();
	glMatrixMode(GL_MODELVIEW); glPopMatrix();
}

void MotionApp::DrawText(int x, int y, const char *text, void *font)
{
    glRasterPos2i(x, y);
    for (const char* c = text; *c != '\0'; c++) glutBitmapCharacter(font, *c);
}

void MotionApp::PrepareColormapData()
{
	if (!motion || !motion2) return;
	int num_segments = motion->body->num_segments;
	int num_frames = min(motion->num_frames, motion2->num_frames);
	if (num_frames == 0) { colormap_data.clear(); return; }

	colormap_data.assign(num_segments, vector<float>(num_frames, 0.0f));
	vector<Matrix4f> seg_frames1, seg_frames2;
	cmap_min_diff = 1e6; cmap_max_diff = -1e6;

	for (int f = 0; f < num_frames; ++f) {
		ForwardKinematics(motion->frames[f], seg_frames1);
		ForwardKinematics(motion2->frames[f], seg_frames2);
		for (int s = 0; s < num_segments; ++s) {
			Point3f p1(seg_frames1[s].m03, seg_frames1[s].m13, seg_frames1[s].m23);
			Point3f p2(seg_frames2[s].m03, seg_frames2[s].m13, seg_frames2[s].m23);
			float diff = p1.distance(p2);
			colormap_data[s][f] = diff;
			if (diff < cmap_min_diff) cmap_min_diff = diff;
			if (diff > cmap_max_diff) cmap_max_diff = diff;
		}
	}
}

void MotionApp::DrawColormap()
{
	if (colormap_data.empty() || colormap_data[0].empty()) {
		BeginScreenMode();
		DrawText(win_width / 2 - 100, win_height - 100, "Colormap data not available.", GLUT_BITMAP_HELVETICA_18);
		EndScreenMode();
		return;
	}

	BeginScreenMode();

	int cmap_margin = 50;
	int cmap_h = min(win_height / 3, 300);
	int cmap_w = win_width - 2 * cmap_margin;
	int cmap_x = cmap_margin;
	int cmap_y = win_height - cmap_margin - cmap_h;

	DrawPlotAxes(cmap_x, cmap_y, cmap_w, cmap_h, 0, colormap_data[0].size(), cmap_min_diff, cmap_max_diff, "Time (Frame)", "Body Part");
	
	int num_segments = colormap_data.size();
	int num_frames = colormap_data[0].size();
	float cell_w = (float)cmap_w / num_frames;
	float cell_h = (float)cmap_h / num_segments;
	float diff_range = cmap_max_diff - cmap_min_diff;
	if (diff_range < 1e-5f) diff_range = 1e-5f;

	glBegin(GL_QUADS);
	for (int s = 0; s < num_segments; ++s) {
		for (int f = 0; f < num_frames; ++f) {
			float value = (colormap_data[s][f] - cmap_min_diff) / diff_range;
			Color3f color = GetHeatmapColor(value);
			glColor3f(color.x, color.y, color.z);
			float x0 = cmap_x + f * cell_w;
			float y0 = cmap_y + s * cell_h;
			glVertex2f(x0, y0); glVertex2f(x0, y0 + cell_h);
			glVertex2f(x0 + cell_w, y0 + cell_h); glVertex2f(x0 + cell_w, y0);
		}
	}
	glEnd();

    int frame_idx = static_cast<int>(animation_time / motion->interval);
    if (frame_idx < num_frames) {
        float line_x = cmap_x + (frame_idx + 0.5f) * cell_w;
        glLineWidth(2.0f);
        glColor3f(1.0f, 1.0f, 1.0f);
        glBegin(GL_LINES); glVertex2f(line_x, cmap_y); glVertex2f(line_x, cmap_y + cmap_h); glEnd();
    }
	EndScreenMode();
}
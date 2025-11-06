#include "MotionApp.h"
#include "BVH.h"
#include <string>
#include <algorithm>
#include <cstdio>
#include <cmath>

static float get_axis_value(const Point3f& p, int axis_index) 
{
	if (axis_index == 0) return p.x; 
	if (axis_index == 1) return p.y;
	return p.z; 
}

static const char* get_axis_name(int axis_index)
{ if (axis_index == 0) return "X"; 
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
	app_name = "Motion Analysis App";
    motion = NULL; motion2 = NULL; curr_posture = NULL; curr_posture2 = NULL;
    on_animation = true; animation_time = 0.0f; animation_speed = 1.0f;
    view_mode = 0; plot_segment_index = 0; plot_vertical_axis = 1; plot_horizontal_axis = 3;
    cmap_min_diff = 0.0f; cmap_max_diff = 1.0f;
    ct_h_axis = 0; ct_v_axis = 2;
    projection_mode = false;
    
    // REVISED: 2つのスライス値を設定
    slice_values.push_back(1.0f); // スライス1
    slice_values.push_back(0.5f); // スライス2
    
    grid_resolution = 80;
    ct_feature_mode = 0;
    speed_norm_mode = 0;
    global_max_speed = 1.0f;

    // REVISED: 複数スライス用のデータ構造のサイズを確保
    size_t num_slices = slice_values.size();
    occupancy_grids1.resize(num_slices);
    occupancy_grids2.resize(num_slices);
    difference_grids.resize(num_slices);
    speed_grids1.resize(num_slices);
    speed_grids2.resize(num_slices);
    speed_diff_grids.resize(num_slices);
    
    max_occupancy_diffs.resize(num_slices, 1.0f);
    max_speeds.resize(num_slices, 1.0f);
    max_speed_diffs.resize(num_slices, 1.0f);
}

MotionApp::~MotionApp() 
{
	if ( motion )
	{ 
		if (motion->body) 
			delete motion->body;
		delete motion;
	} 
	if ( motion2 )
		delete motion2; 
	if ( curr_posture )
		delete curr_posture;
	if ( curr_posture2 )
		delete curr_posture2;
}

void MotionApp::Initialize() 
{
	GLUTBaseApp::Initialize();
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
	const char* modes[] = {"Line Plot", "Time vs Part Colormap", "CT-Scan View"};
	switch (key) {
		case ' ': on_animation = !on_animation; break;
		case 'c': view_mode = (view_mode + 1) % 3; break;
		case 'v': plot_vertical_axis = (plot_vertical_axis + 1) % 3; break;
		case 'h': plot_horizontal_axis = (plot_horizontal_axis + 1) % 4; break;
		case 'p': if (motion) { plot_segment_index = (plot_segment_index + 1) % motion->body->num_segments; PreparePlotData(); } break;
        case 'q': if (motion) { plot_segment_index = (plot_segment_index - 1 + motion->body->num_segments) % motion->body->num_segments; PreparePlotData(); } break;
	}
	if (view_mode == 2) {
		switch(key) {
			case 'x': ct_h_axis = (ct_h_axis + 1) % 3; if (ct_h_axis == ct_v_axis) ct_h_axis = (ct_h_axis + 1) % 3; break;
			case 'y': ct_v_axis = (ct_v_axis + 1) % 3; if (ct_v_axis == ct_h_axis) ct_v_axis = (ct_v_axis + 1) % 3; break;
			case 'z': projection_mode = !projection_mode; break;
			
            // REVISED: w/s がスライス1を操作
			case 'w': slice_values[0] += 0.01f; break;
			case 's': slice_values[0] -= 0.01f; break;
            
            // ADDED: t/g がスライス2を操作
            case 't': if (slice_values.size() > 1) slice_values[1] += 0.01f; break;
            case 'g': if (slice_values.size() > 1) slice_values[1] -= 0.01f; break;

            case 'f': // (変更なし)
                ct_feature_mode = (ct_feature_mode + 1) % 2;
                printf("CT-Scan Feature Mode: %s\n", (ct_feature_mode == 0) ? "Occupancy" : "Speed");
                break;
			case 'n': // (変更なし)
                if (ct_feature_mode == 1) { 
                    speed_norm_mode = (speed_norm_mode + 1) % 2;
                    printf("Speed Normalization Mode: %s\n", (speed_norm_mode == 0) ? "Per-Frame" : "Global");
                }
                break;
		}
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
	if (curr_posture) { DrawPostureSelective(*curr_posture, plot_segment_index, Color3f(0.8f,0.2f,0.2f), Color3f(0.7f,0.7f,0.7f)); DrawPostureShadow(*curr_posture, shadow_dir, shadow_color); }
	if (curr_posture2) { DrawPostureSelective(*curr_posture2, plot_segment_index, Color3f(0.2f,0.2f,0.8f), Color3f(0.6f,0.6f,0.6f)); DrawPostureShadow(*curr_posture2, shadow_dir, shadow_color); }

	//CT-Scanの"スライスモード"時に、分析平面を3Dビューに描画する
    if (view_mode == 2 && !projection_mode) {
        DrawSlicePlane();
    }

	if (view_mode == 0) DrawPositionPlot();
	else if (view_mode == 1) DrawColormap();
	else if (view_mode == 2) { UpdateOccupancyGrids(); DrawCtScanView(); }

	char title[256]; const char* mode_str[] = {"Plot", "Colormap", "CT-Scan"};
	if (view_mode == 2) {
		int d_axis = 3 - ct_h_axis - ct_v_axis;
		const char* slice_mode_str = projection_mode ? "Projection" : "Slice";
        const char* feature_mode_str = (ct_feature_mode == 0) ? "Occupancy" : "Speed";
        const char* norm_mode_str = (speed_norm_mode == 0) ? "Frame" : "Global";

		// REVISED: 2つのスライスの値を表示
		sprintf(title, "View:%s|H:%s V:%s|Mode:%s|Feature:%s|Norm:%s('n')|%s|S1:%.2f(w/s) S2:%.2f(t/g)",
        mode_str[view_mode], get_axis_name(ct_h_axis), get_axis_name(ct_v_axis), slice_mode_str, 
        feature_mode_str, norm_mode_str, get_axis_name(d_axis), 
        slice_values[0], (slice_values.size() > 1 ? slice_values[1] : 0.0f) );
	} else
		sprintf(title, "View:%s('c')|Plotting:%s('p'/'q')|V-Axis:%s('v')|H-Axis:%s('h')", 
			mode_str[view_mode], motion ? motion->body->segments[plot_segment_index]->name.c_str() : "N/A",
			get_axis_name(plot_vertical_axis), get_axis_name(plot_horizontal_axis)); 
	
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
	if (motion2) { AlignInitialPositions(); AlignInitialOrientations(); }
	Start();
}

void MotionApp::LoadBVH2(const char* file_name)
{ 
	if (!motion) return;
	Motion* m2 = LoadAndCoustructBVHMotion(file_name, motion->body);
	if (!m2) return;
	if (motion2) delete motion2; 
	if (curr_posture2) delete curr_posture2;
	motion2 = m2;
	curr_posture2 = new Posture(motion2->body); 
	AlignInitialPositions(); 
	AlignInitialOrientations(); 
	PrepareAllData();
	Start();
}

//
// ２つのモーションの初期位置を合わせる
//
void MotionApp::AlignInitialPositions() {
	if (!motion || !motion2 || motion->num_frames == 0 || motion2->num_frames == 0) return;
	Point3f offset1(motion->frames[0].root_pos.x, 0.0f, motion->frames[0].root_pos.z);
	for (int i = 0; i < motion->num_frames; i++) { motion->frames[i].root_pos -= offset1; }
	Point3f offset2 = motion2->frames[0].root_pos - motion->frames[0].root_pos;
	for (int i = 0; i < motion2->num_frames; i++) { motion2->frames[i].root_pos -= offset2; }
	printf("Initial positions aligned.\n");
	PreparePlotData();
	PrepareColormapData();
}

//
// ADDED: ２つのモーションの初期の向きを合わせる
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
	PreparePlotData();
	PrepareColormapData();
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

void MotionApp::PrepareAllData() {
    if (!motion || !motion2) return;
    AlignInitialPositions();
    AlignInitialOrientations();
    PreparePlotData();
    PrepareColormapData();
    CalculateWorldBounds();
    PrepareSpeedData();
    UpdateOccupancyGrids();
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
	printf("Plot data recalculated for segment: %s\n", motion->body->segments[plot_segment_index]->name.c_str());
}


//
// 2Dプロットを描画 (REVISED)
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
	
	// REVISED: ユーザー指定のプロット領域サイズ設定
	int plot_size = min(win_width / 2, win_height / 2);
	//if (plot_size < 300) plot_size = 300;
	
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
		
		if (data_aspect > screen_aspect) {
			v_range_data = h_range_data / screen_aspect;
		} else {
			h_range_data = v_range_data * screen_aspect;
		}
		
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

	// ハイライト機能の分離と改善
	int frame_idx = static_cast<int>(animation_time / motion->interval);
	
	if (plot_horizontal_axis == 3) { // 横軸がフレームの場合は黄色の縦線
		float line_x = plot_x + (( (float)frame_idx - h_min) / h_range) * plot_w;
		if (line_x >= plot_x && line_x <= plot_x + plot_w) {
			glLineWidth(2.0f);
			glColor3f(1.0f, 1.0f, 0.2f);
			glBegin(GL_LINES);
				glVertex2f(line_x, plot_y);
				glVertex2f(line_x, plot_y + plot_h);
			glEnd();
		}
	} else { // 横軸が座標の場合は黄色の点
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

// 2つのモーションが占める空間全体の境界を計算

void MotionApp::CalculateWorldBounds() {
	if (!motion || !motion2) return;

	// X,Y,Z各軸の最小/最大値を初期化
	for (int i = 0; i < 3; ++i) {
		world_bounds[i][0] = 1e6;  // min
		world_bounds[i][1] = -1e6; // max
	}

	auto update_bounds = [&](const Motion* m) {
		vector<Matrix4f> seg_frames;
		for (int f = 0; f < m->num_frames; ++f) {
			ForwardKinematics(m->frames[f], seg_frames);
			for (int s = 0; s < m->body->num_segments; ++s) {
				Point3f p(seg_frames[s].m03, seg_frames[s].m13, seg_frames[s].m23);
				if (p.x < world_bounds[0][0]) world_bounds[0][0] = p.x;
				if (p.x > world_bounds[0][1]) world_bounds[0][1] = p.x;
				if (p.y < world_bounds[1][0]) world_bounds[1][0] = p.y;
				if (p.y > world_bounds[1][1]) world_bounds[1][1] = p.y;
				if (p.z < world_bounds[2][0]) world_bounds[2][0] = p.z;
				if (p.z > world_bounds[2][1]) world_bounds[2][1] = p.z;
			}
		}
	};
	
	update_bounds(motion);
	update_bounds(motion2);

	// 各軸に持たせる余白を10%から5%に減らす
	for (int i = 0; i < 3; ++i) {
		float range = world_bounds[i][1] - world_bounds[i][0];
		// ここの値を調整することで、ズーム具合を変更できる
		world_bounds[i][0] -= range * 0.1f;
		world_bounds[i][1] += range * 0.1f;
	}

	printf("World bounds calculated:\n");
	printf("  X: [%.2f, %.2f]\n", world_bounds[0][0], world_bounds[0][1]);
	printf("  Y: [%.2f, %.2f]\n", world_bounds[1][0], world_bounds[1][1]);
	printf("  Z: [%.2f, %.2f]\n", world_bounds[2][0], world_bounds[2][1]);
}

//
// プロットの軸を描画 (MODIFIED)
//
void MotionApp::DrawPlotAxes(int x, int y, int w, int h, float h_min, float h_max, float v_min, float v_max, const char* h_label, const char* v_label)
{
	glColor4f(0.2f, 0.2f, 0.2f, 0.5f);
	glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBegin(GL_QUADS);
		glVertex2f(x, y); glVertex2f(x, y + h);
		glVertex2f(x + w, y + h); glVertex2f(x + w, y);
	glEnd();
	glDisable(GL_BLEND);

	glColor3f(0.8f, 0.8f, 0.8f);
	glBegin(GL_LINE_LOOP);
		glVertex2f(x, y); glVertex2f(x, y + h);
		glVertex2f(x + w, y + h); glVertex2f(x + w, y);
	glEnd();

	char label[64];
	glColor3f(0.0f, 0.0f, 0.0f); // テキスト色を黒に変更
	sprintf(label, "%s: %.2f", h_label, h_max); DrawText(x + w - 70, y + h + 15, label, GLUT_BITMAP_HELVETICA_10);
	sprintf(label, "%.2f", h_min); DrawText(x, y + h + 15, label, GLUT_BITMAP_HELVETICA_10);
	sprintf(label, "%s: %.2f", v_label, v_max); DrawText(x - 45, y + 12, label, GLUT_BITMAP_HELVETICA_10);
	sprintf(label, "%.2f", v_min); DrawText(x - 45, y + h, label, GLUT_BITMAP_HELVETICA_10);
}

//
// 2D描画モードを開始
//
void MotionApp::BeginScreenMode()
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0.0, win_width, win_height, 0.0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
}

//
// 2D描画モードを終了
//
void MotionApp::EndScreenMode()
{
	glPopAttrib();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

//
// テキスト描画
//
void MotionApp::DrawText(int x, int y, const char *text, void *font)
{
    glRasterPos2i(x, y);
    for (const char* c = text; *c != '\0'; c++)
        glutBitmapCharacter(font, *c);
}

void MotionApp::PrepareColormapData()
{
	printf("DEBUG: PrepareColormapData() called.\n");
	if (!motion || !motion2) {
		printf("DEBUG: Cannot prepare colormap data, one or both motions missing.\n");
		return;
	}

	int num_segments = motion->body->num_segments;
	int num_frames = min(motion->num_frames, motion2->num_frames);

	if (num_frames == 0) {
		printf("DEBUG: Cannot prepare colormap data, zero frames to compare.\n");
		colormap_data.clear();
		return;
	}

	colormap_data.assign(num_segments, vector<float>(num_frames, 0.0f));
	vector<Matrix4f> seg_frames1, seg_frames2;
	cmap_min_diff = 1e6;
	cmap_max_diff = -1e6;

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
	printf("DEBUG: Colormap data prepared. Frames=%d, Segments=%d. Diff range: [%.3f, %.3f]\n", 
		num_frames, num_segments, cmap_min_diff, cmap_max_diff);
}

//
// カラーマップを描画
//
void MotionApp::DrawColormap()
{
	if (colormap_data.empty() || colormap_data[0].empty()) {
		// データがない場合、ここにいることを知らせるメッセージを描画
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
			glVertex2f(x0, y0);
			glVertex2f(x0, y0 + cell_h);
			glVertex2f(x0 + cell_w, y0 + cell_h);
			glVertex2f(x0 + cell_w, y0);
		}
	}
	glEnd();

    int frame_idx = static_cast<int>(animation_time / motion->interval);
    if (frame_idx < num_frames) {
        float line_x = cmap_x + (frame_idx + 0.5f) * cell_w;
        glLineWidth(2.0f);
        glColor3f(1.0f, 1.0f, 1.0f);
        glBegin(GL_LINES);
            glVertex2f(line_x, cmap_y);
            glVertex2f(line_x, cmap_y + cmap_h);
        glEnd();
    }
	
	EndScreenMode();
}

// 全体基準の最大速度を事前に計算する
void MotionApp::PrepareSpeedData() {
    if (!motion || !motion2) return;

    printf("Calculating global max speed... ");
    global_max_speed = 0.0f;

    auto calculate_max_speed = [&](const Motion* m) {
        for (int f = 1; f < m->num_frames; ++f) {
            vector<Point3f> jp_curr, jp_prev;
			vector<Matrix4f> sf_curr, sf_prev; // REVISED: Added
            ForwardKinematics(m->frames[f], sf_curr, jp_curr);
            ForwardKinematics(m->frames[f-1], sf_prev, jp_prev);

            for (int s = 0; s < m->body->num_segments; ++s) {

                std::string seg_name = m->body->segments[s]->name;
                if (seg_name.find("Hand") != std::string::npos && seg_name != "RightHand" && seg_name != "LeftHand") {
                    continue;
                }

                const Segment* seg = m->body->segments[s];
			    Point3f p1, p2, p1_prev, p2_prev;
                
                // ==================================================================
			    // REVISED: Logic to handle both normal and terminal bones (like Head)
			    // ==================================================================
                if (seg->num_joints == 1) {
                    p1 = jp_curr[seg->joints[0]->index];
                    p1_prev = jp_prev[seg->joints[0]->index];
                    Matrix3f R_curr(sf_curr[s].m00, sf_curr[s].m01, sf_curr[s].m02, sf_curr[s].m10, sf_curr[s].m11, sf_curr[s].m12, sf_curr[s].m20, sf_curr[s].m21, sf_curr[s].m22);
                    Point3f offset_curr = seg->site_position;
                    R_curr.transform(&offset_curr);
                    p2 = p1 + offset_curr;
                    Matrix3f R_prev(sf_prev[s].m00, sf_prev[s].m01, sf_prev[s].m02, sf_prev[s].m10, sf_prev[s].m11, sf_prev[s].m12, sf_prev[s].m20, sf_prev[s].m21, sf_prev[s].m22);
                    Point3f offset_prev = seg->site_position;
                    R_prev.transform(&offset_prev);
                    p2_prev = p1_prev + offset_prev;
                } else if (seg->num_joints >= 2) {
                    p1 = jp_curr[seg->joints[0]->index]; p2 = jp_curr[seg->joints[1]->index];
                    p1_prev = jp_prev[seg->joints[0]->index]; p2_prev = jp_prev[seg->joints[1]->index];
                } else {
                    continue;
                }
                // ==================================================================

                Vector3f vel1 = p1 - p1_prev, vel2 = p2 - p2_prev;
                float speed = ((vel1.length() + vel2.length()) / 2.0f) / m->interval;
                if (speed > global_max_speed) global_max_speed = speed;
            }
        }
    };

    calculate_max_speed(motion);
    calculate_max_speed(motion2);

    if (global_max_speed < 1e-5f) global_max_speed = 1.0f;
    printf("Done. Global max speed (filtered) = %.2f\n", global_max_speed);
}

// 空間占有率の計算ロジック
void MotionApp::UpdateOccupancyGrids() {
	if (!motion || !motion2) return;
	
	int current_frame = (int)(animation_time / motion->interval);
	if (current_frame <= 0 || current_frame >= max(motion->num_frames, motion2->num_frames)) return;

	int d_axis = 3 - ct_h_axis - ct_v_axis;
	float h_world_range = world_bounds[ct_h_axis][1] - world_bounds[ct_h_axis][0];
	float v_world_range = world_bounds[ct_v_axis][1] - world_bounds[ct_v_axis][0];

	// REVISED: すべてのスライス平面について計算をループ
    for (int i = 0; i < slice_values.size(); ++i)
    {
        float current_slice_val = slice_values[i];

        // --- グリッドと最大値をスライスごとに初期化 ---
        occupancy_grids1[i].assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
        occupancy_grids2[i].assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
        difference_grids[i].assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
        speed_grids1[i].assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
        speed_grids2[i].assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
        speed_diff_grids[i].assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
        float max_occ_diff_local = 0.0f;
        float max_speed_local = 0.0f;
        float max_speed_diff_local = 0.0f;

        // --- 計算の本体 (process_motion ラムダ) ---
        auto process_motion = [&](const Motion* m, vector<vector<float>>& occ_grid, vector<vector<float>>& spd_grid) {
            // ... (このラムダ関数の内部は、slice_value -> current_slice_val に変更した以外は、以前のバージョンと全く同じです) ...
            vector<Point3f> joint_pos, joint_pos_prev;
            vector<Matrix4f> seg_frames, seg_frames_prev;
            ForwardKinematics(m->frames[min(current_frame, m->num_frames - 1)], seg_frames, joint_pos);
            ForwardKinematics(m->frames[min(current_frame, m->num_frames - 1) - 1], seg_frames_prev, joint_pos_prev);
            float bone_radius = 0.08f;
            for (int s = 0; s < m->body->num_segments; ++s) {
                std::string seg_name = m->body->segments[s]->name;
                if (seg_name.find("Hand") != std::string::npos && seg_name != "RightHand" && seg_name != "LeftHand") continue;
                const Segment* seg = m->body->segments[s];
                Point3f p1, p2, p1_prev, p2_prev;
                if (seg->num_joints == 1) {
					p1 = joint_pos[seg->joints[0]->index];
					p1_prev = joint_pos_prev[seg->joints[0]->index];
					Matrix3f R_curr(seg_frames[s].m00, seg_frames[s].m01, seg_frames[s].m02, seg_frames[s].m10, seg_frames[s].m11, seg_frames[s].m12, seg_frames[s].m20, seg_frames[s].m21, seg_frames[s].m22);
					Point3f offset_vec = seg->site_position; 
					R_curr.transform(&offset_vec);
					p2 = p1 + offset_vec;
					Matrix3f R_prev(seg_frames_prev[s].m00, seg_frames_prev[s].m01, seg_frames_prev[s].m02, seg_frames_prev[s].m10, seg_frames_prev[s].m11, seg_frames_prev[s].m12, seg_frames_prev[s].m20, seg_frames_prev[s].m21, seg_frames_prev[s].m22);
					Point3f offset_vec_prev = seg->site_position;
					R_prev.transform(&offset_vec_prev);
					p2_prev = p1_prev + offset_vec_prev;
				} else if (seg->num_joints >= 2) {
					p1 = joint_pos[seg->joints[0]->index];
					p2 = joint_pos[seg->joints[1]->index];
					p1_prev = joint_pos_prev[seg->joints[0]->index];
					p2_prev = joint_pos_prev[seg->joints[1]->index];
				} else
					continue;

				if (!projection_mode)
					if (min(get_axis_value(p1, d_axis), get_axis_value(p2, d_axis)) > current_slice_val + bone_radius || max(get_axis_value(p1, d_axis), get_axis_value(p2, d_axis)) < current_slice_val - bone_radius) continue;
				Vector3f vel1 = p1 - p1_prev; Vector3f vel2 = p2 - p2_prev;
				float speed = ((vel1.length() + vel2.length()) / 2.0f) / m->interval;
				float h_min = min(get_axis_value(p1, ct_h_axis), get_axis_value(p2, ct_h_axis)) - bone_radius;
				float h_max = max(get_axis_value(p1, ct_h_axis), get_axis_value(p2, ct_h_axis)) + bone_radius;
				float v_min = min(get_axis_value(p1, ct_v_axis), get_axis_value(p2, ct_v_axis)) - bone_radius;
				float v_max = max(get_axis_value(p1, ct_v_axis), get_axis_value(p2, ct_v_axis)) + bone_radius;
				int h_start = (int)(((h_min - world_bounds[ct_h_axis][0]) / h_world_range) * grid_resolution);
				int h_end   = (int)(((h_max - world_bounds[ct_h_axis][0]) / h_world_range) * grid_resolution);
				int v_start = (int)(((v_min - world_bounds[ct_v_axis][0]) / v_world_range) * grid_resolution);
				int v_end   = (int)(((v_max - world_bounds[ct_v_axis][0]) / v_world_range) * grid_resolution);
				h_start = max(0, h_start); h_end = min(grid_resolution - 1, h_end);
				v_start = max(0, v_start); v_end = min(grid_resolution - 1, v_end);
				for (int v_idx = v_start; v_idx <= v_end; ++v_idx) {
					for (int h_idx = h_start; h_idx <= h_end; ++h_idx) {
						float world_h = world_bounds[ct_h_axis][0] + (h_idx + 0.5f) * (h_world_range / grid_resolution);
						float world_v = world_bounds[ct_v_axis][0] + (v_idx + 0.5f) * (v_world_range / grid_resolution);
						float p1_h = get_axis_value(p1, ct_h_axis); float p1_v = get_axis_value(p1, ct_v_axis);
						float p2_h = get_axis_value(p2, ct_h_axis); float p2_v = get_axis_value(p2, ct_v_axis);
						float dx = p2_h - p1_h, dv = p2_v - p1_v;
						float len_sq = dx*dx + dv*dv;
						if (len_sq < 1e-6) continue;
						float t = ((world_h - p1_h) * dx + (world_v - p1_v) * dv) / len_sq;
						t = max(0.0f, min(1.0f, t));
						float closest_h = p1_h + t * dx; float closest_v = p1_v + t * dv;
						float dist_sq = pow(closest_h - world_h, 2) + pow(closest_v - world_v, 2);
						if (!projection_mode) {
							Point3f closest_p_3D = p1 + (p2 - p1) * t;
							float dist_sq_depth = pow(get_axis_value(closest_p_3D, d_axis) - current_slice_val, 2);
							dist_sq += dist_sq_depth;
						}
						if (dist_sq < bone_radius*bone_radius) {
							float presence = exp(-dist_sq / (2.0f * (bone_radius/2.0f)*(bone_radius/2.0f)));
							occ_grid[v_idx][h_idx] += presence;
							spd_grid[v_idx][h_idx] = max(spd_grid[v_idx][h_idx], speed);
						}
					}
				}
			}
		};// --- process_motion ラムダの終わり ---

		process_motion(motion, occupancy_grids1[i], speed_grids1[i]);
		process_motion(motion2, occupancy_grids2[i], speed_grids2[i]);

		// --- 差分と最大値を計算 ---
        for (int v = 0; v < grid_resolution; ++v) {
            for (int h = 0; h < grid_resolution; ++h) {
                float diff = abs(occupancy_grids1[i][v][h] - occupancy_grids2[i][v][h]);
                difference_grids[i][v][h] = diff;
                if (diff > max_occ_diff_local) max_occ_diff_local = diff;
                if (speed_grids1[i][v][h] > max_speed_local) max_speed_local = speed_grids1[i][v][h];
                if (speed_grids2[i][v][h] > max_speed_local) max_speed_local = speed_grids2[i][v][h];
                float speed_diff = abs(speed_grids1[i][v][h] - speed_grids2[i][v][h]);
                speed_diff_grids[i][v][h] = speed_diff;
                if (speed_diff > max_speed_diff_local) max_speed_diff_local = speed_diff;
            }
        }

		// --- 計算結果をグローバルなベクトルに格納 ---
        max_occupancy_diffs[i] = (max_occ_diff_local < 1e-5f) ? 1.0f : max_occ_diff_local;
        max_speeds[i] = (max_speed_local < 1e-5f) ? 1.0f : max_speed_local;
        max_speed_diffs[i] = (max_speed_diff_local < 1e-5f) ? 1.0f : max_speed_diff_local;
    }
}

// CTスキャン風カラーマップ描画
void MotionApp::DrawCtScanView() {
	if (occupancy_grids1.empty() || occupancy_grids1[0].empty()) return;

	BeginScreenMode();

	// 1. レイアウト計算 (2行3列)
	int margin = 50, gap = 15;
    int num_cols = 3;
    int num_rows = slice_values.size();

    // 利用可能な全幅と全高
    int total_w = win_width - 5 * margin - (num_cols - 1) * gap;
    int total_h = win_height - 5 * margin - (num_rows - 1) * gap;

    int map_w = total_w / num_cols;
    int map_h = total_h / num_rows;

    // 正方形に補正
    map_w = min(map_w, map_h);
    map_h = map_w;

    // 描画開始位置 (左上)
    int start_x = margin;
    int start_y = win_height - margin - (num_rows * map_h + (num_rows - 1) * gap);

	// 2. 描画範囲の計算 (変更なし)
	float final_h_min, final_h_max, final_v_min, final_v_max;
	CalculateCtScanBounds(final_h_min, final_h_max, final_v_min, final_v_max);
	float final_h_range = final_h_max - final_h_min;
	float final_v_range = final_v_max - final_v_min;
	float raw_h_min = world_bounds[ct_h_axis][0], raw_h_max = world_bounds[ct_h_axis][1];
	float raw_v_min = world_bounds[ct_v_axis][0], raw_v_max = world_bounds[ct_v_axis][1];
	float raw_h_range = raw_h_max - raw_h_min;
	float raw_v_range = raw_v_max - raw_v_min;
	
	// 3. 描画ヘルパー関数
	auto draw_single_map = [&](int x_pos, int y_pos, int w, int h, const vector<vector<float>>& source_grid, float max_val, const char* title) {
		
		bool invert_y = (ct_v_axis == 1);
		float v_min_label = invert_y ? final_v_max : final_v_min;
		float v_max_label = invert_y ? final_v_min : final_v_max;
		DrawPlotAxes(x_pos, y_pos, w, h, final_h_min, final_h_max, v_min_label, v_max_label, get_axis_name(ct_h_axis), get_axis_name(ct_v_axis));

		// ==================================================================
		// NEW: 描画直前にデータをリサンプリングして、スケールを補正する
		// ==================================================================
		vector<vector<float>> resampled_grid(grid_resolution, vector<float>(grid_resolution, 0.0f));
		if (raw_h_range > 1e-5f && raw_v_range > 1e-5f) {
			for (int v_new = 0; v_new < grid_resolution; ++v_new) {
				for (int h_new = 0; h_new < grid_resolution; ++h_new) {
					// 正方形の描画範囲に対応するワールド座標を計算
					float world_h = final_h_min + (h_new + 0.5f) * (final_h_range / grid_resolution);
					float world_v = final_v_min + (v_new + 0.5f) * (final_v_range / grid_resolution);
					
					// そのワールド座標が、元のデータ範囲（長方形）の内側にあるかチェック
					if (world_h >= raw_h_min && world_h <= raw_h_max && world_v >= raw_v_min && world_v <= raw_v_max) {
						// 内側にあれば、元のグリッドから値を取得して新しいグリッドにコピー
						int h_old = (int)(((world_h - raw_h_min) / raw_h_range) * grid_resolution);
						int v_old = (int)(((world_v - raw_v_min) / raw_v_range) * grid_resolution);
						resampled_grid[v_new][h_new] = source_grid[v_old][h_old];
					}
				}
			}
		}

		// ==================================================================
		// STABLE: シンプルで安定した描画ループ
		// ==================================================================
		glBegin(GL_QUADS);
		for (int v_idx = 0; v_idx < grid_resolution; ++v_idx) {
			for (int h_idx = 0; h_idx < grid_resolution; ++h_idx) {
				int read_v_idx = invert_y ? (grid_resolution - 1 - v_idx) : v_idx;
				float value = resampled_grid[read_v_idx][h_idx] / max_val; // 補正後のグリッドを使用
				if (value <= 0.01) continue;

				Color3f color = GetHeatmapColor(value);
				glColor3f(color.x, color.y, color.z);

				float cell_w = (float)w / grid_resolution;
				float cell_h = (float)h / grid_resolution;
				float sx0 = x_pos + h_idx * cell_w;
				float sy0 = y_pos + v_idx * cell_h;
				
				glVertex2f(sx0, sy0);
				glVertex2f(sx0, sy0 + cell_h);
				glVertex2f(sx0 + cell_w, sy0 + cell_h);
				glVertex2f(sx0 + cell_w, sy0);
			}
		}
		glEnd();

		glColor3f(0.0f, 0.0f, 0.0f);
		DrawText(x_pos + 5, y_pos - 5, title, GLUT_BITMAP_HELVETICA_10);
	};

	// 4. REVISED: 6つのマップを2行3列で描画
    for (int i = 0; i < slice_values.size(); ++i) // i が行 (スライス)
    {
        int y_pos = start_y + i * (map_h + gap);
        int x_pos_1 = start_x;
        int x_pos_2 = start_x + (map_w + gap);
        int x_pos_3 = start_x + 2 * (map_w + gap);

        char title1[64], title2[64], title3[64];
        sprintf(title1, "S%d Motion 1", i+1);
        sprintf(title2, "S%d Motion 2", i+1);
        sprintf(title3, "S%d Difference", i+1);

        if (ct_feature_mode == 0) {
            draw_single_map(x_pos_1, y_pos, map_w, map_h, occupancy_grids1[i], max_occupancy_diffs[i], title1);
            draw_single_map(x_pos_2, y_pos, map_w, map_h, occupancy_grids2[i], max_occupancy_diffs[i], title2);
            draw_single_map(x_pos_3, y_pos, map_w, map_h, difference_grids[i], max_occupancy_diffs[i], title3);
        } else {
            float norm_speed = (speed_norm_mode == 0) ? max_speeds[i] : global_max_speed;
            float norm_speed_diff = (speed_norm_mode == 0) ? max_speed_diffs[i] : global_max_speed;
            draw_single_map(x_pos_1, y_pos, map_w, map_h, speed_grids1[i], norm_speed, title1);
            draw_single_map(x_pos_2, y_pos, map_w, map_h, speed_grids2[i], norm_speed, title2);
            draw_single_map(x_pos_3, y_pos, map_w, map_h, speed_diff_grids[i], norm_speed_diff, title3);
        }
    }

	EndScreenMode();
}

// スライス平面の描画
void MotionApp::DrawSlicePlane()
{
    int d_axis = 3 - ct_h_axis - ct_v_axis;

    float h_min, h_max, v_min, v_max;
    CalculateCtScanBounds(h_min, h_max, v_min, v_max);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // REVISED: すべてのスライス平面をループで描画
    for (int i = 0; i < slice_values.size(); ++i)
    {
        float current_slice_val = slice_values[i];
        Point3f corners[4];

        if (d_axis == 0) { // Slicing along X
            corners[0].set(current_slice_val, v_min, h_min); corners[1].set(current_slice_val, v_max, h_min);
            corners[2].set(current_slice_val, v_max, h_max); corners[3].set(current_slice_val, v_min, h_max);
        } else if (d_axis == 1) { // Slicing along Y
            corners[0].set(h_min, current_slice_val, v_min); corners[1].set(h_max, current_slice_val, v_min);
            corners[2].set(h_max, current_slice_val, v_max); corners[3].set(h_min, current_slice_val, v_max);
        } else { // Slicing along Z
            corners[0].set(h_min, v_min, current_slice_val); corners[1].set(h_max, v_min, current_slice_val);
            corners[2].set(h_max, v_max, current_slice_val); corners[3].set(h_min, v_max, current_slice_val);
        }
        
        // REVISED: i (0 or 1) に基づいて色と太さを決定
        if (i == 0) {
            glColor4f(1.0f, 1.0f, 0.8f, 0.25f); // Slice 1 Face (Yellow)
            glLineWidth(1.5f);
        } else {
            glColor4f(0.7f, 0.7f, 1.0f, 0.15f); // Slice 2 Face (Blue)
            glLineWidth(1.0f);
        }

        // 面の描画
        glBegin(GL_QUADS);
        for (int j = 0; j < 4; ++j) glVertex3f(corners[j].x, corners[j].y, corners[j].z);
		for (int j = 3; j >=0; --j) glVertex3f(corners[j].x, corners[j].y, corners[j].z);
        glEnd();
        
        // 枠線の描画
		glBegin(GL_LINE_LOOP);
        if (i == 0) {
            glColor4f(1.0f, 1.0f, 0.8f, 0.7f); // Slice 1 Border
        } else {
            glColor4f(0.7f, 0.7f, 1.0f, 0.7f); // Slice 2 Border
        }
		for (int j = 0; j < 4; ++j) glVertex3f(corners[j].x, corners[j].y, corners[j].z);
        glEnd();
    }
    
    glDisable(GL_BLEND);
}

//
void MotionApp::CalculateCtScanBounds(float& h_min, float& h_max, float& v_min, float& v_max)
{
	// 選択された2軸の元の範囲を取得
	float h_min_raw = world_bounds[ct_h_axis][0];
	float h_max_raw = world_bounds[ct_h_axis][1];
	float v_min_raw = world_bounds[ct_v_axis][0];
	float v_max_raw = world_bounds[ct_v_axis][1];

	// 2軸の範囲の広い方を基準に、正方形の描画範囲を計算
	float h_range = h_max_raw - h_min_raw;
	float v_range = v_max_raw - v_min_raw;
	float max_range = max(h_range, v_range);

	float h_center = (h_min_raw + h_max_raw) / 2.0f;
	float v_center = (v_min_raw + v_max_raw) / 2.0f;

	h_min = h_center - max_range / 2.0f;
	h_max = h_center + max_range / 2.0f;
	v_min = v_center - max_range / 2.0f;
	v_max = v_center + max_range / 2.0f;
}
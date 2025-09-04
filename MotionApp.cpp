#include "MotionApp.h"
#include "BVH.h"
#include <string>
#include <algorithm>
#include <cstdio>
#include <cmath>

static float get_axis_value(const Point3f& p, int axis_index) { if (axis_index == 0) return p.x; if (axis_index == 1) return p.y; return p.z; }
static const char* get_axis_name(int axis_index) { if (axis_index == 0) return "X"; if (axis_index == 1) return "Y"; if (axis_index == 2) return "Z"; return "Frame"; }
static Color3f GetHeatmapColor(float value) { Color3f color; value = max(0.0f, min(1.0f, value)); if (value < 0.5f) { color.set(0.0f, value * 2.0f, 1.0f - (value * 2.0f)); } else { color.set((value - 0.5f) * 2.0f, 1.0f - ((value - 0.5f) * 2.0f), 0.0f); } return color; }

MotionApp::MotionApp() {
    app_name = "Motion Analysis App";
    motion = NULL; motion2 = NULL; curr_posture = NULL; curr_posture2 = NULL;
    on_animation = true; animation_time = 0.0f; animation_speed = 1.0f;
    view_mode = 0; plot_segment_index = 0; plot_vertical_axis = 1; plot_horizontal_axis = 3;
    cmap_min_diff = 0.0f; cmap_max_diff = 1.0f;
    ct_h_axis = 0; ct_v_axis = 2; // 初期値: XZ平面（上から）
    projection_mode = false;     // 初期値: スライスモード
    slice_value = 1.0f;          // 初期スライス高さ
    grid_resolution = 80; max_occupancy_diff = 1.0f;
}

MotionApp::~MotionApp() { if ( motion ) { if (motion->body) delete motion->body; delete motion; } if ( motion2 ) delete motion2; if ( curr_posture ) delete curr_posture; if ( curr_posture2 ) delete curr_posture2; }
void MotionApp::Initialize() { GLUTBaseApp::Initialize(); OpenNewBVH(); OpenNewBVH2(); }
void MotionApp::Start() { GLUTBaseApp::Start(); on_animation = true; animation_time = 0.0f; Animation(0.0f); }


void MotionApp::Keyboard(unsigned char key, int mx, int my) {
	GLUTBaseApp::Keyboard(key, mx, my);
	bool ct_updated = false;
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
			case 'w': slice_value += 0.05f; break;
			case 's': slice_value -= 0.05f; break;
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

	if (view_mode == 0) DrawPositionPlot();
	else if (view_mode == 1) DrawColormap();
	else if (view_mode == 2) { UpdateOccupancyGrids(); DrawCtScanView(); }

	char title[256]; const char* mode_str[] = {"Plot", "Colormap", "CT-Scan"};
	if (view_mode == 2) {
		int d_axis = 3 - ct_h_axis - ct_v_axis;
		const char* slice_mode_str = projection_mode ? "Projection" : "Slice";
		sprintf(title, "View:%s('c') | H:%s('x') V:%s('y') | Mode:%s('z') | Slice:%s %.2f('w'/'s')", mode_str[view_mode], get_axis_name(ct_h_axis), get_axis_name(ct_v_axis), slice_mode_str, get_axis_name(d_axis), slice_value);
	} else { sprintf(title, "View:%s('c')|Plotting:%s('p'/'q')|V-Axis:%s('v')|H-Axis:%s('h')", mode_str[view_mode], motion ? motion->body->segments[plot_segment_index]->name.c_str() : "N/A", get_axis_name(plot_vertical_axis), get_axis_name(plot_horizontal_axis)); }
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

void MotionApp::LoadBVH2(const char* file_name) { if (!motion) return; Motion* m2 = LoadAndCoustructBVHMotion(file_name, motion->body); if (!m2) return; if (motion2) delete motion2; if (curr_posture2) delete curr_posture2; motion2 = m2; curr_posture2 = new Posture(motion2->body); AlignInitialPositions(); AlignInitialOrientations(); PrepareAllData(); Start(); }

//
// ADDED: ２つのモーションの初期位置を合わせる
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

void MotionApp::PrepareAllData() { if (!motion || !motion2) return; PreparePlotData(); PrepareColormapData(); CalculateWorldBounds(); UpdateOccupancyGrids(); }

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

	// REVISED: ハイライト機能の分離と改善
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
				
				// X axis
				if (p.x < world_bounds[0][0]) world_bounds[0][0] = p.x;
				if (p.x > world_bounds[0][1]) world_bounds[0][1] = p.x;
				
				// Y axis
				if (p.y < world_bounds[1][0]) world_bounds[1][0] = p.y;
				if (p.y > world_bounds[1][1]) world_bounds[1][1] = p.y;

				// Z axis
				if (p.z < world_bounds[2][0]) world_bounds[2][0] = p.z;
				if (p.z > world_bounds[2][1]) world_bounds[2][1] = p.z;
			}
		}
	};
	
	update_bounds(motion);
	update_bounds(motion2);

	// 各軸に少し余白を持たせる
	for (int i = 0; i < 3; ++i) {
		float range = world_bounds[i][1] - world_bounds[i][0];
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
// カラーマップを描画 (完成版)
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

// REVISED: 空間占有率の計算ロジック
void MotionApp::UpdateOccupancyGrids() {
	if (!motion || !motion2) return;
	occupancy_grid1.assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
	occupancy_grid2.assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
	difference_grid.assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
	max_occupancy_diff = 0.0f;
	int current_frame = (int)(animation_time / motion->interval);
	if (current_frame >= min(motion->num_frames, motion2->num_frames)) return;
	
	int d_axis = 3 - ct_h_axis - ct_v_axis; // 0,1,2の合計が3になるように深度軸を決定

	auto process_motion = [&](const Motion* m, vector<vector<float>>& grid) {
		vector<Point3f> joint_pos; ForwardKinematics(m->frames[current_frame], vector<Matrix4f>(), joint_pos);
		float bone_radius = 0.08f;
		for (int s = 0; s < m->body->num_segments; ++s) {
			const Segment* seg = m->body->segments[s];
			if (seg->num_joints < 2) continue;
			Point3f p1 = joint_pos[seg->joints[0]->index]; Point3f p2 = joint_pos[seg->joints[1]->index];

			if (!projection_mode) { // スライスモードの場合のみ、深度チェック
				if (min(get_axis_value(p1, d_axis), get_axis_value(p2, d_axis)) > slice_value + bone_radius ||
					max(get_axis_value(p1, d_axis), get_axis_value(p2, d_axis)) < slice_value - bone_radius) continue;
			}
			
			for (int v_idx = 0; v_idx < grid_resolution; ++v_idx) {
				for (int h_idx = 0; h_idx < grid_resolution; ++h_idx) {
					float world_h = world_bounds[ct_h_axis][0] + (h_idx + 0.5f) * (world_bounds[ct_h_axis][1] - world_bounds[ct_h_axis][0]) / grid_resolution;
					float world_v = world_bounds[ct_v_axis][0] + (v_idx + 0.5f) * (world_bounds[ct_v_axis][1] - world_bounds[ct_v_axis][0]) / grid_resolution;
					
					// 点と線分の最短距離を計算
					Point3f bone_vec = p2 - p1;
					Point3f point_vec(world_h - get_axis_value(p1, ct_h_axis), world_v - get_axis_value(p1, ct_v_axis), 0.0f);
					float t = (get_axis_value(point_vec, ct_h_axis) * get_axis_value(bone_vec, ct_h_axis) + get_axis_value(point_vec, ct_v_axis) * get_axis_value(bone_vec, ct_v_axis)) / (get_axis_value(bone_vec, ct_h_axis)*get_axis_value(bone_vec, ct_h_axis) + get_axis_value(bone_vec, ct_v_axis)*get_axis_value(bone_vec, ct_v_axis));
					t = max(0.0f, min(1.0f, t));
					Point3f closest_p = p1 + bone_vec * t;

					float dist_sq = 0;
					if (projection_mode) {
						dist_sq = pow(get_axis_value(closest_p, ct_h_axis) - world_h, 2) + pow(get_axis_value(closest_p, ct_v_axis) - world_v, 2);
					} else {
						dist_sq = pow(get_axis_value(closest_p, ct_h_axis) - world_h, 2) + pow(get_axis_value(closest_p, ct_v_axis) - world_v, 2) + pow(get_axis_value(closest_p, d_axis) - slice_value, 2);
					}
					
					if (dist_sq < bone_radius*bone_radius) {
						float presence = exp(-dist_sq / (2.0f * (bone_radius/2.0f)*(bone_radius/2.0f)));
						grid[v_idx][h_idx] += presence;
					}
				}
			}
		}
	};
	
	process_motion(motion, occupancy_grid1); process_motion(motion2, occupancy_grid2);
	for (int v = 0; v < grid_resolution; ++v) for (int h = 0; h < grid_resolution; ++h) {
		float diff = abs(occupancy_grid1[v][h] - occupancy_grid2[v][h]);
		difference_grid[v][h] = diff;
		if (diff > max_occupancy_diff) max_occupancy_diff = diff;
	}
	if (max_occupancy_diff < 1e-5f) max_occupancy_diff = 1.0f;
}

// REVISED: CTスキャン風カラーマップ描画
void MotionApp::DrawCtScanView() {
	if (difference_grid.empty()) return;

	// 1. レイアウトを計算
	int margin = 50;
	int gap = 15;
	int map_w = (win_width - 2 * margin - 2 * gap) / 3;
	if (map_w < 100) map_w = 100; // 最小幅を保証
	int map_h = map_w; // 正方形
	int area_y = win_height - margin - map_h;
	if (area_y < margin) { // ウィンドウが縦に狭い場合
		map_h = win_height - 2 * margin;
		if (map_h < 100) map_h = 100;
		map_w = map_h;
		area_y = margin;
	}
	int map1_x = margin;
	int map2_x = margin + map_w + gap;
	int map3_x = margin + 2 * (map_w + gap);

	// 2. 描画するワールド座標の範囲を、縦横スケールが1:1になるように計算
	float h_min_raw = world_bounds[ct_h_axis][0];
	float h_max_raw = world_bounds[ct_h_axis][1];
	float v_min_raw = world_bounds[ct_v_axis][0];
	float v_max_raw = world_bounds[ct_v_axis][1];

	float h_range = h_max_raw - h_min_raw;
	float v_range = v_max_raw - v_min_raw;
	float max_range = max(h_range, v_range);

	float h_center = (h_min_raw + h_max_raw) / 2.0f;
	float v_center = (v_min_raw + v_max_raw) / 2.0f;
	
	float final_h_min = h_center - max_range / 2.0f;
	float final_h_max = h_center + max_range / 2.0f;
	float final_v_min = v_center - max_range / 2.0f;
	float final_v_max = v_center + max_range / 2.0f;

	// 3. 描画ヘルパー関数
	auto draw_single_map = [&](int x_pos, int y_pos, int w, int h, const vector<vector<float>>& grid, float max_val, const char* title) {
		// 描画のメイン部分
		BeginScreenMode();
		// 背景
		glColor4f(0.9f, 0.9f, 0.9f, 0.8f);
		glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glBegin(GL_QUADS);
			glVertex2f(x_pos, y_pos); glVertex2f(x_pos, y_pos + h);
			glVertex2f(x_pos + w, y_pos + h); glVertex2f(x_pos + w, y_pos);
		glEnd();
		glDisable(GL_BLEND);
		// カラーマップ本体
		float cell_w = (float)w / grid_resolution; float cell_h = (float)h / grid_resolution;
		glBegin(GL_QUADS);
		for (int v_idx = 0; v_idx < grid_resolution; ++v_idx) {
			for (int h_idx = 0; h_idx < grid_resolution; ++h_idx) {
				int read_v_idx = (ct_v_axis == 1) ? (grid_resolution - 1 - v_idx) : v_idx;
				float value = grid[read_v_idx][h_idx] / max_val;
				if (value > 0.01) { // 閾値以下の値は描画しない
					Color3f color = GetHeatmapColor(value); glColor3f(color.x, color.y, color.z);
					float sx0 = x_pos + h_idx * cell_w; float sy0 = y_pos + v_idx * cell_h;
					glVertex2f(sx0, sy0); glVertex2f(sx0, sy0 + cell_h); glVertex2f(sx0 + cell_w, sy0 + cell_h); glVertex2f(sx0 + cell_w, sy0);
				}
			}
		}
		glEnd();
		// 枠線とラベル
		DrawPlotAxes(x_pos, y_pos, w, h, final_h_min, final_h_max, (ct_v_axis == 1 ? final_v_max : final_v_min), (ct_v_axis == 1 ? final_v_min : final_v_max), get_axis_name(ct_h_axis), get_axis_name(ct_v_axis));
		glColor3f(0.0f, 0.0f, 0.0f); DrawText(x_pos + 5, y_pos - 5, title, GLUT_BITMAP_HELVETICA_10);
		EndScreenMode();
	};

	// 4. 3つのマップを描画
	draw_single_map(map1_x, area_y, map_w, map_h, occupancy_grid1, max_occupancy_diff, "Motion 1 Occupancy");
	draw_single_map(map2_x, area_y, map_w, map_h, occupancy_grid2, max_occupancy_diff, "Motion 2 Occupancy");
	draw_single_map(map3_x, area_y, map_w, map_h, difference_grid, max_occupancy_diff, "Difference");
}
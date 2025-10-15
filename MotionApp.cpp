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
    projection_mode = false; // 初期値: スライスモード
    slice_value = 1.0f; // 初期スライス高さ
    grid_resolution = 80; max_occupancy_diff = 1.0f;
    ct_feature_mode = 0;
    speed_norm_mode = 0; // 初期値はフレーム基準
    global_max_speed = 1.0f;
}

MotionApp::~MotionApp() { if ( motion ) { if (motion->body) delete motion->body; delete motion; } if ( motion2 ) delete motion2; if ( curr_posture ) delete curr_posture; if ( curr_posture2 ) delete curr_posture2; }
void MotionApp::Initialize() { GLUTBaseApp::Initialize(); OpenNewBVH(); OpenNewBVH2(); }
void MotionApp::Start() { GLUTBaseApp::Start(); on_animation = true; animation_time = 0.0f; Animation(0.0f); }


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
			case 'w': slice_value += 0.01f; break;
			case 's': slice_value -= 0.01f; break;
            case 'f': // ADDED: 特徴量モードの切り替え
                ct_feature_mode = (ct_feature_mode + 1) % 2;
                printf("CT-Scan Feature Mode: %s\n", (ct_feature_mode == 0) ? "Occupancy" : "Speed");
                break;
			case 'n': // ADDED: ノーマライゼーションモードの切り替え
                if (ct_feature_mode == 1) { // 速度モードの時のみ有効
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

	// ADDED: CT-Scanの"スライスモード"時に、分析平面を3Dビューに描画する
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

		sprintf(title, "View:%s|H:%s V:%s|Mode:%s|Feature:%s|Norm:%s('n')|Slice:%s %.2f",
        mode_str[view_mode], get_axis_name(ct_h_axis), get_axis_name(ct_v_axis), slice_mode_str, feature_mode_str, norm_mode_str, get_axis_name(d_axis), slice_value);
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

// ADDED: 全体基準の最大速度を事前に計算する
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

// REVISED: 空間占有率の計算ロジック
// 既存の UpdateOccupancyGrids 関数を、以下の内容に丸ごと置き換えてください。

void MotionApp::UpdateOccupancyGrids() {
	if (!motion || !motion2) return;

	// グリッド初期化 (変更なし)
	occupancy_grid1.assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
	occupancy_grid2.assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
	difference_grid.assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
    speed_grid1.assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
    speed_grid2.assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
    speed_diff_grid.assign(grid_resolution, vector<float>(grid_resolution, 0.0f));
	max_occupancy_diff = 0.0f;
    max_speed = 0.0f;
    max_speed_diff = 0.0f;
	
	int current_frame = (int)(animation_time / motion->interval);
	if (current_frame <= 0 || current_frame >= min(motion->num_frames, motion2->num_frames)) return;

	int d_axis = 3 - ct_h_axis - ct_v_axis;
	float h_world_range = world_bounds[ct_h_axis][1] - world_bounds[ct_h_axis][0];
	float v_world_range = world_bounds[ct_v_axis][1] - world_bounds[ct_v_axis][0];

	auto process_motion = [&](const Motion* m, vector<vector<float>>& occ_grid, vector<vector<float>>& spd_grid) {
		vector<Point3f> joint_pos, joint_pos_prev;
		vector<Matrix4f> seg_frames, seg_frames_prev;
		ForwardKinematics(m->frames[current_frame], seg_frames, joint_pos);
		ForwardKinematics(m->frames[current_frame - 1], seg_frames_prev, joint_pos_prev);
		float bone_radius = 0.08f;

		for (int s = 0; s < m->body->num_segments; ++s) {
			
			std::string seg_name = m->body->segments[s]->name;
			if (seg_name.find("Hand") != std::string::npos && seg_name != "RightHand" && seg_name != "LeftHand") {
				continue;
			}

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
			} else {
				continue;
			}

			if (!projection_mode) {
				if (min(get_axis_value(p1, d_axis), get_axis_value(p2, d_axis)) > slice_value + bone_radius || max(get_axis_value(p1, d_axis), get_axis_value(p2, d_axis)) < slice_value - bone_radius) continue;
			}
			
            Vector3f vel1 = p1 - p1_prev;
            Vector3f vel2 = p2 - p2_prev;
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
					
					// ==================================================================
					// REVISED: 選択された軸で正しく距離を計算するロジック
					// ==================================================================
					float p1_h = get_axis_value(p1, ct_h_axis); float p1_v = get_axis_value(p1, ct_v_axis);
					float p2_h = get_axis_value(p2, ct_h_axis); float p2_v = get_axis_value(p2, ct_v_axis);

					float dx = p2_h - p1_h, dv = p2_v - p1_v;
					float len_sq = dx*dx + dv*dv;
					if (len_sq < 1e-6) continue;

					float t = ((world_h - p1_h) * dx + (world_v - p1_v) * dv) / len_sq;
					t = max(0.0f, min(1.0f, t));

					float closest_h = p1_h + t * dx;
					float closest_v = p1_v + t * dv;
					
					float dist_sq_2D = pow(closest_h - world_h, 2) + pow(closest_v - world_v, 2);
					float dist_sq = dist_sq_2D;

					if (!projection_mode) {
						Point3f closest_p_3D = p1 + (p2 - p1) * t;
						float dist_sq_depth = pow(get_axis_value(closest_p_3D, d_axis) - slice_value, 2);
						dist_sq += dist_sq_depth;
					}
					// ==================================================================
					
					if (dist_sq < bone_radius*bone_radius) {
						float presence = exp(-dist_sq / (2.0f * (bone_radius/2.0f)*(bone_radius/2.0f)));
						occ_grid[v_idx][h_idx] += presence;
						spd_grid[v_idx][h_idx] = max(spd_grid[v_idx][h_idx], speed);
					}
				}
			}
		}
	};
	
	process_motion(motion, occupancy_grid1, speed_grid1);
	process_motion(motion2, occupancy_grid2, speed_grid2);

	for (int v = 0; v < grid_resolution; ++v) {
		for (int h = 0; h < grid_resolution; ++h) {
			float diff = abs(occupancy_grid1[v][h] - occupancy_grid2[v][h]);
			difference_grid[v][h] = diff;
			if (diff > max_occupancy_diff) max_occupancy_diff = diff;
            if (speed_grid1[v][h] > max_speed) max_speed = speed_grid1[v][h];
            if (speed_grid2[v][h] > max_speed) max_speed = speed_grid2[v][h];
            float speed_diff = abs(speed_grid1[v][h] - speed_grid2[v][h]);
            speed_diff_grid[v][h] = speed_diff;
            if (speed_diff > max_speed_diff) max_speed_diff = speed_diff;
		}
	}
	if (max_occupancy_diff < 1e-5f) max_occupancy_diff = 1.0f;
    if (max_speed < 1e-5f) max_speed = 1.0f;
    if (max_speed_diff < 1e-5f) max_speed_diff = 1.0f;
}

// REVISED: CTスキャン風カラーマップ描画
void MotionApp::DrawCtScanView() {
	if (occupancy_grid1.empty() || speed_grid1.empty()) return;
	BeginScreenMode();
	
	// レイアウト計算
	int margin = 50, gap = 15;
	int map_w = (win_width - 2 * margin - 2 * gap) / 3;
	if (map_w < 100) map_w = 100;
	int map_h = map_w;
	int area_y = win_height - margin - map_h;
	if (area_y < margin) { map_h = win_height - 2 * margin; if (map_h < 100) map_h = 100; map_w = map_h; area_y = margin; }
	int map1_x = margin, map2_x = margin + map_w + gap, map3_x = margin + 2 * (map_w + gap);

	// REVISED: 新しい補助関数で、統一された正方形の範囲を取得
	float final_h_min, final_h_max, final_v_min, final_v_max;
	CalculateCtScanBounds(final_h_min, final_h_max, final_v_min, final_v_max);

	auto draw_single_map = [&](int x_pos, const vector<vector<float>>& grid, float max_val, const char* title) {
		bool invert_y = (ct_v_axis == 1);
		float v_min_label = invert_y ? world_bounds[ct_v_axis][1] : world_bounds[ct_v_axis][0];
		float v_max_label = invert_y ? world_bounds[ct_v_axis][0] : world_bounds[ct_v_axis][1];
		DrawPlotAxes(x_pos, area_y, map_w, map_h, world_bounds[ct_h_axis][0], world_bounds[ct_h_axis][1], v_min_label, v_max_label, get_axis_name(ct_h_axis), get_axis_name(ct_v_axis));
		float cell_w = (float)map_w / grid_resolution; float cell_h = (float)map_h / grid_resolution;
		glBegin(GL_QUADS);
		for (int v_idx = 0; v_idx < grid_resolution; ++v_idx) {
			for (int h_idx = 0; h_idx < grid_resolution; ++h_idx) {
				int read_v_idx = invert_y ? (grid_resolution - 1 - v_idx) : v_idx;
				float value = grid[read_v_idx][h_idx] / max_val;
				if (value > 0.01) {
					Color3f color = GetHeatmapColor(value); glColor3f(color.x, color.y, color.z);
					float sx0 = x_pos + h_idx * cell_w; float sy0 = area_y + v_idx * cell_h;
					glVertex2f(sx0, sy0); glVertex2f(sx0, sy0 + cell_h); glVertex2f(sx0 + cell_w, sy0 + cell_h); glVertex2f(sx0 + cell_w, sy0);
				}
			}
		}
		glEnd();
		glColor3f(0.0f, 0.0f, 0.0f); DrawText(x_pos + 5, area_y - 5, title, GLUT_BITMAP_HELVETICA_10);
	};

	if (ct_feature_mode == 0) { // 占有率モード
		draw_single_map(map1_x, occupancy_grid1, max_occupancy_diff, "Motion 1 Occupancy");
		draw_single_map(map2_x, occupancy_grid2, max_occupancy_diff, "Motion 2 Occupancy");
		draw_single_map(map3_x, difference_grid, max_occupancy_diff, "Difference");
	} else { // 速度モード
		// REVISED: ノーマライゼーションモードに応じて最大値を切り替え
        float norm_speed = (speed_norm_mode == 0) ? max_speed : global_max_speed;
        float norm_speed_diff = (speed_norm_mode == 0) ? max_speed_diff : global_max_speed;

		draw_single_map(map1_x, speed_grid1, norm_speed, "Motion 1 Speed");
		draw_single_map(map2_x, speed_grid2, norm_speed, "Motion 2 Speed");
		draw_single_map(map3_x, speed_diff_grid, norm_speed_diff, "Speed Difference");
	}

	// ビューポートを全体に戻す
	glViewport(0, 0, win_width, win_height);

	EndScreenMode();
}

// REPLACE the existing DrawSlicePlane function
void MotionApp::DrawSlicePlane()
{
    int d_axis = 3 - ct_h_axis - ct_v_axis;

    // REVISED: 新しい補助関数で、統一された正方形の範囲を取得
    float h_min, h_max, v_min, v_max;
    CalculateCtScanBounds(h_min, h_max, v_min, v_max);

    Point3f corners[4];
    if (d_axis == 0) { // Slicing along X
        corners[0].set(slice_value, v_min, h_min); corners[1].set(slice_value, v_max, h_min);
        corners[2].set(slice_value, v_max, h_max); corners[3].set(slice_value, v_min, h_max);
    } else if (d_axis == 1) { // Slicing along Y
        corners[0].set(h_min, slice_value, v_min); corners[1].set(h_max, slice_value, v_min);
        corners[2].set(h_max, slice_value, v_max); corners[3].set(h_min, slice_value, v_max);
    } else { // Slicing along Z
        corners[0].set(h_min, v_min, slice_value); corners[1].set(h_max, v_min, slice_value);
        corners[2].set(h_max, v_max, slice_value); corners[3].set(h_min, v_max, slice_value);
    }
    
    // --- Drawing (no changes) ---
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(1.0f, 1.0f, 0.8f, 0.25f);
    glBegin(GL_QUADS);
    for (int i = 0; i < 4; ++i) glVertex3f(corners[i].x, corners[i].y, corners[i].z);
    glEnd();
    glLineWidth(1.5f);
    glColor4f(1.0f, 1.0f, 0.8f, 0.6f);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 4; ++i) glVertex3f(corners[i].x, corners[i].y, corners[i].z);
    glEnd();
    glDisable(GL_BLEND);
}

// ADD THIS NEW FUNCTION
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
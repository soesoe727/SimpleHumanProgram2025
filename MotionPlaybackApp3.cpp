/**
*** 動作再生＆詳細プロットアプリケーション
**/

#include "MotionPlaybackApp.h"
#include "BVH.h"
#include <string>
#include <algorithm>
#include <cstdio>
#include <cmath>

// ヘルパー関数: 軸インデックスから値を取得
static float get_axis_value(const Point3f& p, int axis_index) {
	if (axis_index == 0) return p.x;
	if (axis_index == 1) return p.y;
	return p.z; // axis_index == 2
}

// ヘルパー関数: 軸インデックスから軸名を取得
static const char* get_axis_name(int axis_index) {
	if (axis_index == 0) return "X";
	if (axis_index == 1) return "Y";
	if (axis_index == 2) return "Z";
	return "Frame"; // axis_index == 3
}

//
//  コンストラクタ
//
MotionPlaybackApp3::MotionPlaybackApp3()
{
	app_name = "Motion Playback 3 (Plotting)";
	motion = NULL;
	motion2 = NULL;
	curr_posture = NULL;
	curr_posture2 = NULL;
	on_animation = true;
	animation_time = 0.0f;
	animation_speed = 1.0f;
	
	// プロットの初期設定
	plot_segment_index = 0; // 0: ルート(Hips)
	plot_vertical_axis = 1;   // 1: Y軸
	plot_horizontal_axis = 3; // 3: フレーム（時間）
}

//
//  デストラクタ
//
MotionPlaybackApp3::~MotionPlaybackApp3()
{
	if ( motion ) {
		if (motion->body) delete motion->body;
		delete motion;
	}
	if ( motion2 ) delete motion2;
	if ( curr_posture ) delete curr_posture;
	if ( curr_posture2 ) delete curr_posture2;
}

//
//  初期化
//
void MotionPlaybackApp3::Initialize()
{
	GLUTBaseApp::Initialize();
	OpenNewBVH();
	OpenNewBVH2();
}

//
//  開始・リセット
//
void MotionPlaybackApp3::Start()
{
	GLUTBaseApp::Start();
	on_animation = true;
	animation_time = 0.0f;
	Animation(0.0f);
}

//
//  画面描画
//
void MotionPlaybackApp3::Display()
{
	GLUTBaseApp::Display();

	if (curr_posture) {
		// 選択部位を赤、その他を明るい灰色で描画
		Color3f highlight_color1(0.8f, 0.2f, 0.2f);
		Color3f base_color1(1.0f, 1.0f, 1.0f);
		DrawPostureSelective(*curr_posture, plot_segment_index, highlight_color1, base_color1);
		DrawPostureShadow(*curr_posture, shadow_dir, shadow_color);
	}
	if (curr_posture2) {
		// 選択部位を青、その他を少し暗い灰色で描画
		Color3f highlight_color2(0.2f, 0.2f, 0.8f);
		Color3f base_color2(0.6f, 0.6f, 0.6f);
		DrawPostureSelective(*curr_posture2, plot_segment_index, highlight_color2, base_color2);
		DrawPostureShadow(*curr_posture2, shadow_dir, shadow_color);
	}

	DrawPositionPlot();

	char title[256];
	sprintf(title, "Plotting: %s (%s vs %s)", 
		motion ? motion->body->segments[plot_segment_index]->name.c_str() : "N/A",
		get_axis_name(plot_vertical_axis), 
		get_axis_name(plot_horizontal_axis));
	DrawTextInformation(0, title);
	
	if (motion) {
		char message[64];
		sprintf(message, "Time: %.2f (%d)", animation_time, (int)(animation_time / motion->interval));
		DrawTextInformation(1, message);
	}
}

//
//  キーボードのキー押下
//
void MotionPlaybackApp3::Keyboard(unsigned char key, int mx, int my)
{
	GLUTBaseApp::Keyboard(key, mx, my);

	switch (key) {
		case ' ':
			on_animation = !on_animation;
			break;
		case 'v':
			plot_vertical_axis = (plot_vertical_axis + 1) % 3; // X, Y, Z
			printf("Plot Vertical Axis: %s\n", get_axis_name(plot_vertical_axis));
			break;
		case 'h':
			plot_horizontal_axis = (plot_horizontal_axis + 1) % 4; // X, Y, Z, Frame
			printf("Plot Horizontal Axis: %s\n", get_axis_name(plot_horizontal_axis));
			break;
		case 'p':
			if (motion) {
				plot_segment_index = (plot_segment_index + 1) % motion->body->num_segments;
				printf("Plotting Segment: %s (index: %d)\n", motion->body->segments[plot_segment_index]->name.c_str(), plot_segment_index);
				PreparePlotData(); // 部位が変わったのでプロットデータを再計算
			}
			break;
		case 'q':
			if (motion) {
				plot_segment_index = (plot_segment_index - 1) % motion->body->num_segments;
				if(plot_segment_index < 0) plot_segment_index += motion->body->num_segments; // 負のインデックスを回避
				printf("Plotting Segment: %s (index: %d)\n", motion->body->segments[plot_segment_index]->name.c_str(), plot_segment_index);
				PreparePlotData(); // 部位が変わったのでプロットデータを再計算
			}
			break;
	}
}

//
//  アニメーション処理
//
void MotionPlaybackApp3::Animation(float delta)
{
	if (!on_animation || drag_mouse_l || !motion || !motion2)
		return;

	animation_time += delta * animation_speed;
	float max_duration = max(motion->GetDuration(), motion2->GetDuration());
	if (animation_time > max_duration)
		animation_time = 0.0f;
	
	if (animation_time < 0.0f) animation_time = 0.0f;

	motion->GetPosture(animation_time, *curr_posture);
	motion2->GetPosture(animation_time, *curr_posture2);
}

//
//  BVH動作ファイルの読み込み
//
void MotionPlaybackApp3::LoadBVH(const char* file_name)
{
	Motion* new_motion = LoadAndCoustructBVHMotion(file_name);
	if (!new_motion) return;
	if (motion) {
		if (motion->body) delete motion->body;
		delete motion;
	}
	if (curr_posture) delete curr_posture;

	motion = new_motion;
	curr_posture = new Posture(motion->body);
	
	AlignInitialPositions();
	AlignInitialOrientations();
	Start();
}
void MotionPlaybackApp3::LoadBVH2(const char* file_name, bool is_first_load)
{
	if (!motion) return;
	Motion* new_motion2 = LoadAndCoustructBVHMotion(file_name, motion->body);
	if (!new_motion2) return;
	if (motion2) delete motion2;
	if (curr_posture2) delete curr_posture2;

	motion2 = new_motion2;
	curr_posture2 = new Posture(motion2->body);

	AlignInitialPositions();
	AlignInitialOrientations();
	Start();
}

//
// ADDED: ２つのモーションの初期位置を合わせる
//
void MotionPlaybackApp3::AlignInitialPositions()
{
	if (!motion || !motion2 || motion->num_frames == 0 || motion2->num_frames == 0) {
		return;
	}

	// 1. motion1の開始XZ座標を原点に移動
	Point3f offset1(motion->frames[0].root_pos.x, 0.0f, motion->frames[0].root_pos.z);
	for (int i = 0; i < motion->num_frames; i++) {
		motion->frames[i].root_pos -= offset1;
	}

	// 2. motion2の開始座標を、移動後のmotion1の開始座標に合わせる
	Point3f offset2 = motion2->frames[0].root_pos - motion->frames[0].root_pos;
	for (int i = 0; i < motion2->num_frames; i++) {
		motion2->frames[i].root_pos -= offset2;
	}
	
	printf("Initial positions aligned.\n");
	
	// 位置を変更したので、プロットデータを再計算
	PreparePlotData();
}

//
// ADDED: ２つのモーションの初期の向きを合わせる
//
void MotionPlaybackApp3::AlignInitialOrientations()
{
    if (!motion || !motion2 || motion->num_frames == 0 || motion2->num_frames == 0) {
        return;
    }

    // --- 1つ目のモーションの向きをZ軸正方向に合わせる ---
    Matrix3f ori1 = motion->frames[0].root_ori;
    // 水平方向(XZ平面)の向きを計算 (atan2(x, z) で Z軸からの角度が求まる)
    float angle1 = -atan2(ori1.m02, ori1.m22);
    Matrix3f rot1;
    rot1.rotY(angle1); // Z軸正方向へ向けるための補正回転

    // 全フレームに補正回転を適用
    for (int i = 0; i < motion->num_frames; i++) {
        // 位置をY軸中心に回転
        rot1.transform(&motion->frames[i].root_pos);
        // 向きを回転
        motion->frames[i].root_ori.mul(rot1, motion->frames[i].root_ori);
    }

    // --- 2つ目のモーションの向きも同様にZ軸正方向に合わせる ---
    Matrix3f ori2 = motion2->frames[0].root_ori;
    float angle2 = -atan2(ori2.m02, ori2.m22);
    Matrix3f rot2;
    rot2.rotY(angle2);

    for (int i = 0; i < motion2->num_frames; i++) {
        rot2.transform(&motion2->frames[i].root_pos);
        motion2->frames[i].root_ori.mul(rot2, motion2->frames[i].root_ori);
    }
    
    printf("Initial orientations aligned to face +Z axis.\n");
	
    // 向きの変更に伴い位置も変わったため、プロットデータを再計算
    PreparePlotData();
}

void MotionPlaybackApp3::OpenNewBVH()
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
void MotionPlaybackApp3::OpenNewBVH2()
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
// プロット用データを事前に計算
//
void MotionPlaybackApp3::PreparePlotData()
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
void MotionPlaybackApp3::DrawPositionPlot()
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

//
// プロットの軸を描画 (MODIFIED)
//
void MotionPlaybackApp3::DrawPlotAxes(int x, int y, int w, int h, float h_min, float h_max, float v_min, float v_max, const char* h_label, const char* v_label)
{
	glColor4f(0.2f, 0.2f, 0.2f, 0.8f);
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
void MotionPlaybackApp3::BeginScreenMode()
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
void MotionPlaybackApp3::EndScreenMode()
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
void MotionPlaybackApp3::DrawText(int x, int y, const char *text, void *font)
{
    glRasterPos2i(x, y);
    for (const char* c = text; *c != '\0'; c++)
        glutBitmapCharacter(font, *c);
}
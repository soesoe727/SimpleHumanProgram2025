/**
*** 描画モード切替機能を持つアプリケーション
**/

#include "MotionPlaybackApp.h"
#include <cstdio>
#include <algorithm>
#include <cmath>

// (ヘルパー関数は MotionPlaybackApp3.cpp と同じ)
static float get_axis_value(const Point3f& p, int axis_index) { if (axis_index == 0) return p.x; if (axis_index == 1) return p.y; return p.z; }
static const char* get_axis_name(int axis_index) { if (axis_index == 0) return "X"; if (axis_index == 1) return "Y"; if (axis_index == 2) return "Z"; return "Frame"; }

//
// コンストラクタ
//
MotionPlaybackApp4::MotionPlaybackApp4()
{
	app_name = "Motion Playback 5 (View Toggle)";
	view_mode = 0; // 初期表示は折れ線グラフ
	cmap_min_diff = 0.0f;
	cmap_max_diff = 1.0f;
}

//
// BVH読み込み (オーバーライド)
//
void MotionPlaybackApp4::LoadBVH(const char* file_name)
{
	MotionPlaybackApp3::LoadBVH(file_name);
	PrepareColormapData(); // カラーマップ用のデータも準備
}
void MotionPlaybackApp4::LoadBVH2(const char* file_name, bool is_first_load)
{
	MotionPlaybackApp3::LoadBVH2(file_name, is_first_load);
	PrepareColormapData(); // カラーマップ用のデータも準備
}

//
// キーボード入力 (オーバーライド)
//
void MotionPlaybackApp4::Keyboard(unsigned char key, int mx, int my)
{
	if (key == 'c') {
		view_mode = (view_mode + 1) % 2; // 0と1を交互に切り替え
		printf("Switched View Mode to: %s\n", (view_mode == 0) ? "Line Plot" : "Colormap");
	} else {
		MotionPlaybackApp3::Keyboard(key, mx, my); // 他のキーは親クラスの処理
	}
}

//
// 画面描画 (オーバーライド) - 描画モードを切り替える
//
void MotionPlaybackApp4::Display()
{
	// 3Dシーンの描画（親クラスのロジックを流用）
	GLUTBaseApp::Display();
	if (curr_posture) {
		Color3f highlight_color1(0.8f, 0.2f, 0.2f);
		Color3f base_color1(0.7f, 0.7f, 0.7f);
		DrawPostureSelective(*curr_posture, plot_segment_index, highlight_color1, base_color1);
		DrawPostureShadow(*curr_posture, shadow_dir, shadow_color);
	}
	if (curr_posture2) {
		Color3f highlight_color2(0.2f, 0.2f, 0.8f);
		Color3f base_color2(0.6f, 0.6f, 0.6f);
		DrawPostureSelective(*curr_posture2, plot_segment_index, highlight_color2, base_color2);
		DrawPostureShadow(*curr_posture2, shadow_dir, shadow_color);
	}

	// 2D描画（モードに応じて切り替え）
	//if (view_mode == 0) {
	//	DrawPositionPlot(); // 親クラスの折れ線グラフ描画を呼び出す
	//} else {
	//	DrawColormap();     // カラーマップを描画
	//}
	DrawColormap();
	// 情報テキスト
	char title[256];
	const char* mode_str = (view_mode == 0) ? "Plot" : "Colormap";
	sprintf(title, "Mode: %s ('c' key to toggle) | Plotting: %s (%s vs %s)",
		mode_str,
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
// カラーマップ用のデータを準備 (MotionPlaybackApp4から流用)
//
void MotionPlaybackApp4::PrepareColormapData()
{
	if (!motion || !motion2) return;
	int num_segments = motion->body->num_segments;
	int num_frames = min(motion->num_frames, motion2->num_frames);
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
	printf("Colormap data prepared. Positional difference range: [%.3f, %.3f]\n", cmap_min_diff, cmap_max_diff);
}

// 値を色に変換するヘルパー関数 (0:青 -> 0.5:緑 -> 1:赤)
static Color3f GetHeatmapColor(float value)
{
    Color3f color;
    value = max(0.0f, min(1.0f, value));
    if (value < 0.5f) { color.set(0.0f, value * 2.0f, 1.0f - (value * 2.0f)); }
    else { color.set((value - 0.5f) * 2.0f, 1.0f - ((value - 0.5f) * 2.0f), 0.0f); }
    return color;
}

//
// カラーマップを描画 (MotionPlaybackApp4から流用し、領域を調整)
//
void MotionPlaybackApp4::DrawColormap()
{
	if (colormap_data.empty()) return;

	BeginScreenMode();

	// 描画領域を画面下部全体に設定
	int cmap_margin = 50;
	int cmap_h = min(win_height / 3, 300); // 高さは最大300px
	int cmap_w = win_width - 2 * cmap_margin;
	int cmap_x = cmap_margin;
	int cmap_y = win_height - cmap_margin - cmap_h;

	// 背景と枠線
	glColor4f(0.2f, 0.2f, 0.2f, 0.8f);
	glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBegin(GL_QUADS);
		glVertex2f(cmap_x, cmap_y); glVertex2f(cmap_x, cmap_y + cmap_h);
		glVertex2f(cmap_x + cmap_w, cmap_y + cmap_h); glVertex2f(cmap_x + cmap_w, cmap_y);
	glEnd();
	glDisable(GL_BLEND);
	glColor3f(0.8f, 0.8f, 0.8f);
	glBegin(GL_LINE_LOOP);
		glVertex2f(cmap_x, cmap_y); glVertex2f(cmap_x, cmap_y + cmap_h);
		glVertex2f(cmap_x + cmap_w, cmap_y + cmap_h); glVertex2f(cmap_x + cmap_w, cmap_y);
	glEnd();

	// カラーマップ本体
	int num_segments = colormap_data.size();
	int num_frames = colormap_data[0].size();
	float cell_w = (float)cmap_w / num_frames;
	float cell_h = (float)cmap_h / num_segments;
	float diff_range = cmap_max_diff - cmap_min_diff; if (diff_range < 1e-5f) diff_range = 1e-5f;

	glBegin(GL_QUADS);
	for (int s = 0; s < num_segments; ++s) {
		// 手の細かい部位はスキップして見やすくする
		if(s > 16 && s < 36) continue;
		if(s > 39) continue;
		for (int f = 0; f < num_frames; ++f) {
			float value = (colormap_data[s][f] - cmap_min_diff) / diff_range;
			Color3f color = GetHeatmapColor(value);
			glColor3f(color.x, color.y, color.z);
			float x0 = cmap_x + f * cell_w;
			float y0 = cmap_y + s * cell_h; // 部位sがy座標に対応
			glVertex2f(x0, y0); glVertex2f(x0, y0 + cell_h);
			glVertex2f(x0 + cell_w, y0 + cell_h); glVertex2f(x0 + cell_w, y0);
		}
	}
	glEnd();

    // 現在時刻の縦線
    int frame_idx = static_cast<int>(animation_time / motion->interval);
    if (frame_idx < num_frames) {
        float line_x = cmap_x + (frame_idx + 0.5f) * cell_w;
        glLineWidth(2.0f); glColor3f(1.0f, 1.0f, 1.0f);
        glBegin(GL_LINES); glVertex2f(line_x, cmap_y); glVertex2f(line_x, cmap_y + cmap_h); glEnd();
    }
    
	// ラベル
	char label[64]; glColor3f(0.0f, 0.0f, 0.0f);
	sprintf(label, "Time (Frame)"); DrawText(cmap_x, cmap_y + cmap_h + 15, label, GLUT_BITMAP_HELVETICA_10);
	sprintf(label, "Body Part"); DrawText(cmap_x - 50, cmap_y, label, GLUT_BITMAP_HELVETICA_10);

	EndScreenMode();
}
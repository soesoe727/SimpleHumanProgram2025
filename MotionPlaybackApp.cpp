/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  動作再生アプリケーション
**/

//test3
// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "BVH.h"
#include "MotionPlaybackApp.h"
#include "Timeline.h"
#define  _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <iomanip>


//
//  コンストラクタ
//
MotionPlaybackApp::MotionPlaybackApp()
{
	app_name = "Motion Playback";

	motion = NULL;
	motion2 = NULL;
	curr_posture = NULL;
	curr_posture2 = NULL;
	on_animation = true;
	animation_time = 0.0f;
	animation_speed = 1.0f;
	DTWframe_no = 0;
	frame_no = 0;
	frames = 0;
	view_segment = 12;
	view_segments.resize(40,-1);
	DTWa = NULL;
	pattern = 0;
	timeline = NULL;
	show_color_matrix = true;
}


//
//  デストラクタ
//
MotionPlaybackApp::~MotionPlaybackApp()
{
	// 骨格・動作・姿勢の削除
	if ( motion && motion->body )
		delete  motion->body;
	if ( motion )
		delete  motion;
	if ( curr_posture )
		delete  curr_posture;

	// 骨格・動作・姿勢の削除
	if ( motion2 && motion2->body )
		delete  motion2->body;
	if ( motion2 )
		delete  motion2;
	if ( curr_posture2 )
		delete  curr_posture2;

	if(DTWa)
		delete DTWa;
	if(timeline)
		delete timeline;
}


//
//  初期化
//
void  MotionPlaybackApp::Initialize()
{
	// 基底クラスの処理を実行
	GLUTBaseApp::Initialize();
	DTWa = new DTWinformation();
	// サンプルBVH動作データを読み込み
	//LoadBVH( "sample_walking1.bvh" );
	OpenNewBVH();
	OpenNewBVH2();
	timeline = new Timeline();
	if(motion && motion2)
	{
		motion2->num_frames -= 20;
		//腰の位置を最初に一緒にする
		Point3f root = motion2->frames[0].root_pos - motion->frames[0].root_pos;
		for(int i = 0; i < motion2->num_frames; i++)
			motion2->frames[i].root_pos -= root;

		//DTWの作成
		DTWa->DTWinformation_init( motion->num_frames, motion2->num_frames, *motion, *motion2 );
		InitSegmentname(motion->body->num_segments);
		PatternTimeline(timeline, * motion, *motion2, DTWframe_no, DTWa);
	}
}

//
//  開始・リセット
//
void  MotionPlaybackApp::Start()
{
	// 基底クラスの処理を実行
	GLUTBaseApp::Start();

	// アニメーション再生のための変数の初期化
	on_animation = true;
	animation_time = 0.0f;
	DTWframe_no = 0;
	frame_no = 0;

	// アニメーション再生処理（姿勢の初期化）
	Animation( 0.0f );

	//カメラ位置の調整
	view_center = motion->frames[0].root_pos;
}


//
//  画面描画
//
void  MotionPlaybackApp::Display()
{
	// 基底クラスの処理を実行
	GLUTBaseApp::Display();


		// タイムラインを描画
	if ( timeline )
	{	
		//部位の集合ごとの色付け
		if(sabun_flag == 1)//ワーピング・パスによる再生
			PatternTimeline(timeline, * motion, *motion2, DTWframe_no, DTWa);
		else//ワーピング・パスによる再生から通常再生
			PatternTimeline(timeline, *motion, *motion2, frame_no, DTWa);

		//timeline->DrawTimeline();
	}
	// キャラクタを描画
	
	if(curr_posture && curr_posture2)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		
		if(error_flag == 1)//位置誤差による再生
		{
			if (sabun_flag == 1)//ワーピング・パスによる再生
			{
				//1人目の動作の可視化(色付け)
				Pass_posture1 = motion->frames[DTWa->DisPassAll[0][DTWframe_no]];
				DrawPostureColor(Pass_posture1, pattern, view_segment, view_segments, DTWa->Seg_Color, DTWa->Pa_Color);
				DrawPostureShadow(Pass_posture1, shadow_dir, shadow_color);

				//2人目の動作の可視化(白･灰)(パス対応)
				Pass_posture2 = motion2->frames[DTWa->DisPassAll[1][DTWframe_no]];
				DrawPostureGray(Pass_posture2, pattern, view_segment, view_segments);
				DrawPostureShadow(Pass_posture2, shadow_dir, shadow_color);
			}
			else//ワーピング・パスによる再生から通常再生
			{
				if (frame_no <= mf)//ワーピング・パスによる再生
				{
					//1人目の動作の可視化(色付け)
					Pass_posture1 = motion->frames[DTWa->DisPassAll[0][frame_no]];
					DrawPostureColor(Pass_posture1, pattern, view_segment, view_segments, DTWa->Seg_Color, DTWa->Pa_Color);
					DrawPostureShadow(Pass_posture1, shadow_dir, shadow_color);

					//2人目の動作の可視化(白･灰)(パス対応)
					Pass_posture2 = motion2->frames[DTWa->DisPassAll[1][frame_no]];
					DrawPostureGray(Pass_posture2, pattern, view_segment, view_segments);
					DrawPostureShadow(Pass_posture2, shadow_dir, shadow_color);
				}
				else//通常再生
				{
					//1人目の動作の可視化(色付け)
					Pass_posture1 = motion->frames[min(motion->num_frames - 1, m1f + frame_no - mf)];
					DrawPostureColor(Pass_posture1, pattern, view_segment, view_segments, DTWa->Seg_Color, DTWa->Pa_Color);
					DrawPostureShadow(Pass_posture1, shadow_dir, shadow_color);
					//2人目の動作の可視化(白･灰)(パス対応)
					Pass_posture2 = motion2->frames[min(motion2->num_frames - 1, m2f + frame_no - mf)];
					DrawPostureGray(Pass_posture2, pattern, view_segment, view_segments);
					DrawPostureShadow(Pass_posture2, shadow_dir, shadow_color);
				}
			}
		}
		else//角度誤差による再生
		{
			if (sabun_flag == 1)//ワーピング・パスによる再生
			{
				//1人目の動作の可視化(色付け)
				Pass_posture1 = motion->frames[DTWa->AngPassAll[0][DTWframe_no]];
				DrawPostureColor(Pass_posture1, pattern, view_segment, view_segments, DTWa->Seg_Color, DTWa->Pa_Color);
				DrawPostureShadow(Pass_posture1, shadow_dir, shadow_color);
				//2人目の動作の可視化(白･灰)(パス対応)
				Pass_posture2 = motion2->frames[DTWa->AngPassAll[1][DTWframe_no]];
				DrawPostureGray(Pass_posture2, pattern, view_segment, view_segments);
				DrawPostureShadow(Pass_posture2, shadow_dir, shadow_color);
			}
			else//ワーピング・パスによる再生から通常再生
			{
				if (frame_no <= mf)
				{
					//1人目の動作の可視化(色付け)
					Pass_posture1 = motion->frames[DTWa->AngPassAll[0][frame_no]];
					DrawPostureColor(Pass_posture1, pattern, view_segment, view_segments, DTWa->Seg_Color, DTWa->Pa_Color);
					DrawPostureShadow(Pass_posture1, shadow_dir, shadow_color);
					//2人目の動作の可視化(白･灰)(パス対応)
					Pass_posture2 = motion2->frames[DTWa->AngPassAll[1][frame_no]];
					DrawPostureGray(Pass_posture2, pattern, view_segment, view_segments);
					DrawPostureShadow(Pass_posture2, shadow_dir, shadow_color);
				}
				else//通常再生
				{
					//1人目の動作の可視化(色付け)
					Pass_posture1 = motion->frames[min(motion->num_frames - 1, m1f + frame_no - mf)];
					DrawPostureColor(Pass_posture1, pattern, view_segment, view_segments, DTWa->Seg_Color, DTWa->Pa_Color);
					DrawPostureShadow(Pass_posture1, shadow_dir, shadow_color);
					//2人目の動作の可視化(白･灰)(パス対応)
					Pass_posture2 = motion2->frames[min(motion2->num_frames - 1, m2f + frame_no - mf)];
					DrawPostureGray(Pass_posture2, pattern, view_segment, view_segments);
					DrawPostureShadow(Pass_posture2, shadow_dir, shadow_color);
				}
			}
		}
		glDisable(GL_BLEND);
	}

	// タイムラインを描画
	if ( show_color_matrix )
	{	
		//部位の集合ごとの色付け
		//if(sabun_flag == 1)//ワーピング・パスによる再生
		//	PatternTimeline(timeline, * motion, *motion2, DTWframe_no, DTWa);
		//else//ワーピング・パスによる再生から通常再生
		//	PatternTimeline(timeline, *motion, *motion2, frame_no, DTWa);

		timeline->DrawTimeline();
	}

	// 現在のモード、時間・フレーム番号を表示
	DrawTextInformation( 0, "Motion Playback" );
	char  message1[64];
	char  message2[64];
	char message3[64];
	char message4[64];
	char messages[5][64]{};
	if ( motion && motion2 )
	{
		if (sabun_flag == 1)
			sprintf(message1, "%.2f (%d)", animation_time, DTWframe_no);
		else
			sprintf(message1, "%.2f (%d)", animation_time, frame_no);
		sprintf( message2, "viewing %s", motion->body->segments[view_segment]->name.c_str() );
		for(int i = 0; i < 5; i++)
			if(error_flag == 1)
				sprintf(messages[i], "%s : %.2f percent", motion->body->segments[DTWa->DisOrder[i]]->name.c_str(), 100 * DTWa->ErrorDisTotalPart[DTWa->DisOrder[i]] / DTWa->ErrorDisTotalAll);
			else
				sprintf(messages[i], "%s : %.2f percent", motion->body->segments[DTWa->AngOrder[i]]->name.c_str(), 100 * DTWa->ErrorAngTotalPart[DTWa->AngOrder[i]] / DTWa->ErrorAngTotalAll);
		if (sabun_flag == 1)
			sprintf(message3, "DTW_reproduction");
		else
			sprintf(message3, "usual_reproduction");
		if (error_flag == 1)
			sprintf(message4, "position_error");
		else
			sprintf(message4, "angle_error");
		/*
		if( pattern == 0 )
			sprintf(message3, "leg:%.2f, chest:%.2f, arm:%.2f", DTWa->left_leg + DTWa->right_leg, DTWa->chest + DTWa->head, DTWa->left_arm + DTWa->right_arm);
		else if( pattern == 1 )
			sprintf(message3, "right_leg:%.2f, left_leg:%.2f", DTWa->right_leg, DTWa->left_leg);
		else if(pattern == 2)
			sprintf(message3, "chest:%.2f, head:%.2f", DTWa->chest, DTWa->head);
		else
			sprintf(message3, "right_arm:%.2f, left_arm:%.2f", DTWa->right_arm, DTWa->left_arm);
		*/
		DrawTextInformation( 1, message1 );
		//DrawTextInformation( 2, message2 );
		//for(int i = 0; i < 5; i++)
		//	DrawTextInformation( 3+i, messages[i] );
		//DrawTextInformation( 8, message3 );
		//DrawTextInformation( 9, message4 );
	}
	else
	{
		sprintf( message1, "Press 'L' key to Load BVH files" );
		DrawTextInformation( 1, message1 );
	}
	
}

//
//  ウィンドウサイズ変更
//
void MotionPlaybackApp::Reshape(int w, int h)
{
	GLUTBaseApp::Reshape( w, h );

	// タイムラインの描画領域の設定（画面の下部に配置）
	if ( timeline )
		timeline->SetViewAreaBottom(0, 1, 0, 26, 10, 2);
}

//
//  キーボードのキー押下
//
void  MotionPlaybackApp::Keyboard( unsigned char key, int mx, int my )
{
	GLUTBaseApp::Keyboard( key, mx, my );

	// space キーでアニメーションの停止・再開
	if ( key == ' ' )
		on_animation = !on_animation;

	// w キーでアニメーションの再生速度を変更
	if ( key == 'w' )
		animation_speed = ( animation_speed == 1.0f ) ? 0.1f : 1.0f;

	//d キーでview_segmentの変更+
	if (key == 'd')
		view_segment = ViewSegment(1, view_segment);

	//a キーでview_segmentの変更-
	if (key == 'a')
		view_segment = ViewSegment(0, view_segment);

	// s キーでview_segmentsの変更
	if (key == 's')
		view_segments[view_segment] *= -1;

	// x キーでview_segmentsの全削除
	if (key == 'x')
		for_each(view_segments.begin(), view_segments.end(), [](int& n) {n = -1; });

	// c キーでview_segmentsの全可視化
	if (key == 'c')
		for_each(view_segments.begin(), view_segments.end(), [](int& n) {n = 1; });

	// 0 キーでパターン0
	if ( key == '0' )
		pattern = 0;
	// 1 キーでパターン1
	if ( key == '1' )
		pattern = 1;
	// 2 キーでパターン2
	if ( key == '2' )
		pattern = 2;
	// 3 キーでパターン3
	if ( key == '3' )
		pattern = 3;
	// 4 キーでパターン4
	if ( key == '4' )
		pattern = 4;
	// 5 キーでパターン5
	if ( key == '5' )
		pattern = 5;

	//f キーで再生方法切り替え
	if(key == 'f')
	{
		if (error_flag == 1)//位置誤差
		{
			if (sabun_flag == 1)
			{
				m1f = DTWa->DisPassAll[0][DTWframe_no];
				m2f = DTWa->DisPassAll[1][DTWframe_no];
				mf = DTWframe_no;
				frames = mf + motion->num_frames - m1f;
				frame_no = 0;
				std::cout << m1f << " " << m2f << " " << mf << std::endl;
				sabun_flag *= -1;
			}
			else
			{
				m1f = -30;
				m2f = -30;
				mf = -30;
				DTWframe_no = 0;
				sabun_flag *= -1;
			}
		}
		else//角度誤差
		{
			if (sabun_flag == 1)
			{
				m1f = DTWa->AngPassAll[0][DTWframe_no];
				m2f = DTWa->AngPassAll[1][DTWframe_no];
				mf = DTWframe_no;
				frames = mf + motion->num_frames - m1f;
				frame_no = 0;
				std::cout << m1f << " " << m2f << " " << mf << std::endl;
				sabun_flag *= -1;
			}
			else
			{
				m1f = -30;
				m2f = -30;
				mf = -30;
				DTWframe_no = 0;
				sabun_flag *= -1;
			}
		}
	}

	// r キーで位置誤差と角度誤差の表示切り替え
	if (key == 'r')
	{
		error_flag *= -1;
		frame_no = 0;
		DTWframe_no = 0;
	}

	// m キーでカラーマトリックスの表示切り替え
	if (key == 'm')
	{
		if (show_color_matrix)
			show_color_matrix = false;
		else
			show_color_matrix = true;
	}
}

//
//  マウスドラッグ
//
void  MotionPlaybackApp::MouseDrag( int mx, int my )
{
	GLUTBaseApp::MouseDrag( mx, my );
	int name_space = 30.0f;

	// マウス座標に対応するタイムラインのトラック番号・時刻を取得
	int  selected_track_no = timeline->GetTrackByPosition( mx, my );
	float  selected_time = timeline->GetTimeByPosition( mx ) - name_space;

	if (error_flag == 1)//位置誤差
	{
		if (selected_time < 0)
			selected_time = 0.0f;
		else if (sabun_flag == 1 && selected_time > DTWa->DisFrame - 1)
			selected_time = DTWa->DisFrame - 1;
		else if (sabun_flag == -1 && selected_time > frames - 1)
			selected_time = frames - 1;
	}
	else//角度誤差
	{
		if (selected_time < 0)
			selected_time = 0.0f;
		else if (sabun_flag == 1 && selected_time > DTWa->AngFrame - 1)
			selected_time = DTWa->AngFrame - 1;
		else if (sabun_flag == -1 && selected_time > frames - 1)
			selected_time = frames - 1;
	}

	// 変形動作の再生時刻を変更
	if ( drag_mouse_l )
	{
		animation_time = selected_time * motion->interval;
		drag_mouse_l = false;
		Animation( 0.0f );
		drag_mouse_l = true;
	}
}

//
//  マウスクリック
//
void MotionPlaybackApp::MouseClick(int button, int state, int mx, int my)
{
	GLUTBaseApp::MouseClick( button, state, mx, my );
	int name_space = 30.0f;

	// マウス座標に対応するタイムラインのトラック番号・時刻を取得
	int  selected_track_no = timeline->GetTrackByPosition( mx, my );
	float  selected_time = timeline->GetTimeByPosition( mx ) - name_space;

	if (error_flag == 1)//位置誤差
	{
		if (selected_time < 0)
			selected_time = 0.0f;
		else if (sabun_flag == 1 && selected_time > DTWa->DisFrame - 1)
			selected_time = DTWa->DisFrame - 1;
		else if (sabun_flag == -1 && selected_time > frames - 1)
			selected_time = frames - 1;
	}
	else//角度誤差
	{
		if (selected_time < 0)
			selected_time = 0.0f;
		else if (sabun_flag == 1 && selected_time > DTWa->AngFrame - 1)
			selected_time = DTWa->AngFrame - 1;
		else if (sabun_flag == -1 && selected_time > frames - 1)
			selected_time = frames - 1;
	}

	// 入力動作トラック上のクリック位置に応じて、変形動作の再生時刻を変更
	if ( drag_mouse_l )
	{
		animation_time = selected_time * motion->interval;
		Animation( 0.0f );
	}
}

//
//  アニメーション処理
//
void  MotionPlaybackApp::Animation( float delta )
{
	// アニメーション再生中でなければ終了
	if ( !on_animation )
		return;

	// マウスドラッグ中はアニメーションを停止
	if ( drag_mouse_l )
		return;

	// 動作データが読み込まれていなければ終了
	if ( !motion || !motion2 )
		return;

	// 時間を進める
	animation_time += delta * animation_speed;
	//if ( animation_time > motion->GetDuration() )
	//	animation_time -= motion->GetDuration();
	
	if( sabun_flag == 1)
	{
		if(error_flag == 1 && animation_time >= DTWa->DisFrame * motion->interval)
			animation_time = 0.0f;
		else if (error_flag == -1 && animation_time >= DTWa->AngFrame * motion->interval)
			animation_time = 0.0f;

		DTWframe_no = animation_time / motion->interval;
	}
	else if( sabun_flag == -1)
	{
		if(animation_time >= frames * motion->interval)
			animation_time = 0.0f;
		frame_no = animation_time / motion->interval;
	}

	// 現在のフレーム番号を計算
	//DTWframe_no = animation_time / motion->interval;

	// 動作データから現在時刻の姿勢を取得
	//motion->GetPosture( animation_time, *curr_posture );
	//motion2->GetPosture( animation_time, *curr_posture2 );

}



//
//  補助処理
//

//
//view_segmentの順番
//
int  MotionPlaybackApp::ViewSegment(int i, int seg)
{
	if(i > 0)
	{
		if(seg == 7)
			return 0;
		else if(seg == 0)
			return 13;
		else if(seg == 16)
			return 36;
		else if(seg == 39)
			return 1;
		else if(seg == 6)
			return 12;
		else if(seg <= 12 && seg >= 8)
			return seg - 1;
		else
			return seg + 1;
	}
	else
	{
		if(seg == 12)
			return 6;
		else if(seg == 0)
			return 7;
		else if(seg == 13)
			return 0;
		else if(seg == 36)
			return 16;
		else if(seg == 1)
			return 39;
		else if(seg <= 11 && seg >= 7)
			return seg + 1;
		else
			return seg - 1;
	}
}

//
//  BVH動作ファイルの読み込み、動作・姿勢の初期化
//
void  MotionPlaybackApp::LoadBVH( const char * file_name )
{
	// BVHファイルを読み込んで動作データ（＋骨格モデル）を生成
	Motion *  new_motion = LoadAndCoustructBVHMotion( file_name );
	
	// BVHファイルの読み込みに失敗したら終了
	if ( !new_motion )
		return;
	
	// 現在使用している骨格・動作・姿勢を削除
	if ( motion && motion->body )
		delete  motion->body;
	if ( motion )
		delete  motion;
	if ( curr_posture )
		delete  curr_posture;

	// 動作再生に使用する動作・姿勢を初期化
	motion = new_motion;
	curr_posture = new Posture( motion->body );

	// 動作再生開始
	//Start();
}
void  MotionPlaybackApp::LoadBVH2( const char * file_name )
{
	// BVHファイルを読み込んで動作データ（＋骨格モデル）を生成
	Motion *  new_motion2 = LoadAndCoustructBVHMotion( file_name );
	
	// BVHファイルの読み込みに失敗したら終了
	if ( !new_motion2 )
		return;
	
	// 現在使用している骨格・動作・姿勢を削除
	if ( motion2 && motion2->body )
		delete  motion2->body;
	if ( motion2 )
		delete  motion2;
	if ( curr_posture2 )
		delete  curr_posture2;
	

	// 動作再生に使用する動作・姿勢を初期化
	motion2 = new_motion2;
	curr_posture2 = new Posture( motion2->body );
	
	// 動作再生の開始
	//Start();
}

//
//  ファイルダイアログを表示してBVH動作ファイルを選択・読み込み
//
void  MotionPlaybackApp::OpenNewBVH()
{
#ifdef  WIN32
	const int  file_name_len = 256;
	char  file_name[file_name_len] = "";

	// ファイルダイアログの設定
	OPENFILENAME	open_file;
	memset( &open_file, 0, sizeof( OPENFILENAME ) );
	open_file.lStructSize = sizeof( OPENFILENAME );
	open_file.hwndOwner = NULL;
	open_file.lpstrFilter = "BVH Motion Data (*.bvh)\0*.bvh\0All (*.*)\0*.*\0";
	open_file.nFilterIndex = 1;
	open_file.lpstrFile = file_name;
	open_file.nMaxFile = file_name_len;
	open_file.lpstrTitle = "Select first BVH file";
	open_file.lpstrDefExt = "bvh";
	open_file.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;

	// ファイルダイアログを表示
	BOOL  ret = GetOpenFileName( &open_file );

	// ファイルが指定されたら新しい動作を設定
	if ( ret )
	{
		// BVH動作データの読み込み、骨格・姿勢の初期化
		LoadBVH( file_name );

		// 動作再生の開始
		//Start();
	}
#endif // WIN32
}
void  MotionPlaybackApp::OpenNewBVH2()
{
#ifdef  WIN32
	const int  file_name_len = 256;
	char  file_name[file_name_len] = "";

	// ファイルダイアログの設定
	OPENFILENAME	open_file;
	memset( &open_file, 0, sizeof( OPENFILENAME ) );
	open_file.lStructSize = sizeof( OPENFILENAME );
	open_file.hwndOwner = NULL;
	open_file.lpstrFilter = "BVH Motion Data (*.bvh)\0*.bvh\0All (*.*)\0*.*\0";
	open_file.nFilterIndex = 1;
	open_file.lpstrFile = file_name;
	open_file.nMaxFile = file_name_len;
	open_file.lpstrTitle = "Select second BVH file";
	open_file.lpstrDefExt = "bvh";
	open_file.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;

	// ファイルダイアログを表示
	BOOL  ret = GetOpenFileName( &open_file );

	// ファイルが指定されたら新しい動作を設定
	if ( ret )
	{
		// BVH動作データの読み込み、骨格・姿勢の初期化
		LoadBVH2( file_name );

		// 動作再生の開始
		//Start();
	}
#endif // WIN32
}

//
//タイムラインのパターン毎読み込み
//
void MotionPlaybackApp::PatternTimeline(Timeline* timeline, Motion& motion, Motion& motion2, float curr_frame, DTWinformation* DTW)
{
	int Track_num = 0;
	
	// タイムラインの時間範囲を設定
	if(error_flag == 1)
		timeline->SetTimeRange( 0.0f, DTW->DisFrame + num_space );
	else
		timeline->SetTimeRange( 0.0f, DTW->AngFrame + num_space );
	// 全要素・縦棒の情報をクリア
	timeline->DeleteAllElements();
	timeline->DeleteAllLines();
	timeline->DeleteAllSubElements();

	// j番目の誤差
	if(error_flag == 1)
	{
		if (sabun_flag == 1)
			switch (pattern)
			{
			case 0:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[2]);
				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[2]);
				ColorBarElementPart(timeline, 0, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[2]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_arm, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[5]);
				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_arm, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[5]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_leg, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[8]);
				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_leg, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[8]);
				break;

			case 1:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_head, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[0]);
				Track_num++;

				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_chest, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[1]);
				ColorBarElementPart(timeline, 0, Track_num++, DTW->Dis_chest, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[1]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_arm, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[5]);
				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_arm, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[5]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_leg, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[8]);
				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_leg, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[8]);
				break;

			case 2:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[2]);
				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[2]);
				ColorBarElementPart(timeline, 0, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[2]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_right_arm, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[3]);
				Track_num++;

				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_left_arm, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[4]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_leg, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[8]);
				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_leg, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[8]);
				break;

			case 3:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[2]);
				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[2]);
				ColorBarElementPart(timeline, 0, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[2]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_arm, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[5]);
				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_arm, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[5]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_right_leg, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[6]);
				Track_num++;

				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_left_leg, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[7]);
				break;

			case 4:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_head, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[0]);
				Track_num++;

				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_chest, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[1]);
				ColorBarElementPart(timeline, 0, Track_num++, DTW->Dis_chest, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[1]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_right_arm, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[3]);
				Track_num++;

				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_left_arm, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[4]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_right_leg, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[6]);
				Track_num++;

				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Dis_left_leg, DTW->DisPassAll, motion, curr_frame, &DTW->Pa_Color[7]);
				break;

			case 5:
				for (int j = 12; j >= 11; j--)//頭部
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementGray(timeline, j, Track_num, DTW->DisPassAll, motion);
					else
						ColorBarElementPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->DisPassAll, motion, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 10; j >= 7; j--)//胸部
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementGray(timeline, j, Track_num, DTW->DisPassAll, motion);
					else
						ColorBarElementPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->DisPassAll, motion, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				if (view_segment != 0 && view_segments[0] != 1)//腰
					ColorBarElementGray(timeline, 0, Track_num, DTW->DisPassAll, motion);
				else
					ColorBarElementPart(timeline, 0, Track_num, DTW->DistancePart[0], DTW->DisPassAll, motion, curr_frame, &DTW->Seg_Color[0]);
				Track_num++;
				Track_num++;
				for (int j = 13; j <= 16; j++)//右腕
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementGray(timeline, j, Track_num, DTW->DisPassAll, motion);
					else
						ColorBarElementPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->DisPassAll, motion, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 36; j <= 39; j++)//左腕
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementGray(timeline, j, Track_num, DTW->DisPassAll, motion);
					else
						ColorBarElementPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->DisPassAll, motion, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 1; j <= 3; j++)//右脚
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementGray(timeline, j, Track_num, DTW->DisPassAll, motion);
					else
						ColorBarElementPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->DisPassAll, motion, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 4; j <= 6; j++)//左脚
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementGray(timeline, j, Track_num, DTW->DisPassAll, motion);
					else
						ColorBarElementPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->DisPassAll, motion, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				break;
			default:
				break;
			}
		else
			switch (pattern)
			{
			case 0:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);
				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);
				ColorBarElementRepPart(timeline, 0, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_arm, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[5]);
				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_arm, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[5]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_leg, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[8]);
				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_leg, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[8]);
				break;

			case 1:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_head, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[0]);
				Track_num++;

				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_chest, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[1]);
				ColorBarElementRepPart(timeline, 0, Track_num++, DTW->Dis_chest, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[1]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_arm, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[5]);
				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_arm, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[5]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_leg, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[8]);
				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_leg, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[8]);
				break;

			case 2:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);
				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);
				ColorBarElementRepPart(timeline, 0, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_right_arm, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[3]);
				Track_num++;

				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_left_arm, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[4]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_leg, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[8]);
				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_leg, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[8]);
				break;

			case 3:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);
				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);
				ColorBarElementRepPart(timeline, 0, Track_num++, DTW->Dis_head_chest, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_arm, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[5]);
				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_arm, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[5]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_right_leg, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[6]);
				Track_num++;

				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_left_leg, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[7]);
				break;

			case 4:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_head, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[0]);
				Track_num++;

				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_chest, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[1]);
				ColorBarElementRepPart(timeline, 0, Track_num++, DTW->Dis_chest, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[1]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_right_arm, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[3]);
				Track_num++;

				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_left_arm, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[4]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_right_leg, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[6]);
				Track_num++;

				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Dis_left_leg, DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[7]);
				break;

			case 5:
				for (int j = 12; j >= 11; j--)//頭部
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementRepGray(timeline, j, Track_num, DTW->DisPassAll, motion);
					else
						ColorBarElementRepPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 10; j >= 7; j--)//胸部
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementRepGray(timeline, j, Track_num, DTW->DisPassAll, motion);
					else
						ColorBarElementRepPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				if (view_segment != 0 && view_segments[0] != 1)//腰
					ColorBarElementRepGray(timeline, 0, Track_num, DTW->DisPassAll, motion);
				else
					ColorBarElementRepPart(timeline, 0, Track_num, DTW->DistancePart[0], DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Seg_Color[0]);
				Track_num++;
				Track_num++;
				for (int j = 13; j <= 16; j++)//右腕
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementRepGray(timeline, j, Track_num, DTW->DisPassAll, motion);
					else
						ColorBarElementRepPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 36; j <= 39; j++)//左腕
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementRepGray(timeline, j, Track_num, DTW->DisPassAll, motion);
					else
						ColorBarElementRepPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 1; j <= 3; j++)//右脚
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementRepGray(timeline, j, Track_num, DTW->DisPassAll, motion);
					else
						ColorBarElementRepPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 4; j <= 6; j++)//左脚
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementRepGray(timeline, j, Track_num, DTW->DisPassAll, motion);
					else
						ColorBarElementRepPart(timeline, j, Track_num, DTW->DistancePart[j], DTW->DisPassAll, motion, motion2, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				break;
			default:
				break;
			}
	}
	else
	{
		if (sabun_flag == 1)
			switch (pattern)
			{
			case 0:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[2]);
				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[2]);
				ColorBarElementPart(timeline, 0, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[2]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_arm, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[5]);
				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_arm, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[5]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_leg, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[8]);
				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_leg, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[8]);
				break;

			case 1:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_head, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[0]);
				Track_num++;

				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_chest, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[1]);
				ColorBarElementPart(timeline, 0, Track_num++, DTW->Ang_chest, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[1]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_arm, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[5]);
				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_arm, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[5]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_leg, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[8]);
				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_leg, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[8]);
				break;

			case 2:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[2]);
				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[2]);
				ColorBarElementPart(timeline, 0, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[2]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_right_arm, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[3]);
				Track_num++;

				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_left_arm, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[4]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_leg, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[8]);
				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_leg, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[8]);
				break;

			case 3:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[2]);
				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[2]);
				ColorBarElementPart(timeline, 0, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[2]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_arm, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[5]);
				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_arm, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[5]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_right_leg, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[6]);
				Track_num++;

				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_left_leg, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[7]);
				break;

			case 4:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_head, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[0]);
				Track_num++;

				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_chest, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[1]);
				ColorBarElementPart(timeline, 0, Track_num++, DTW->Ang_chest, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[1]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_right_arm, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[3]);
				Track_num++;

				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_left_arm, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[4]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_right_leg, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[6]);
				Track_num++;

				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementPart(timeline, j, Track_num++, DTW->Ang_left_leg, DTW->AngPassAll, motion, curr_frame, &DTW->Pa_Color[7]);
				break;

			case 5:
				for (int j = 12; j >= 11; j--)//頭部
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementGray(timeline, j, Track_num, DTW->AngPassAll, motion);
					else
						ColorBarElementPart(timeline, j, Track_num, DTW->AnglePart[j], DTW->AngPassAll, motion, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 10; j >= 7; j--)//胸部
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementGray(timeline, j, Track_num, DTW->AngPassAll, motion);
					else
						ColorBarElementPart(timeline, j, Track_num, DTW->AnglePart[j], DTW->AngPassAll, motion, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				if (view_segment != 0 && view_segments[0] != 1)//腰
					ColorBarElementGray(timeline, 0, Track_num, DTW->AngPassAll, motion);
				else
					ColorBarElementPart(timeline, 0, Track_num, DTW->AnglePart[0], DTW->AngPassAll, motion, curr_frame, &DTW->Seg_Color[0]);
				Track_num++;
				Track_num++;
				for (int j = 13; j <= 16; j++)//右腕
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementGray(timeline, j, Track_num, DTW->AngPassAll, motion);
					else
						ColorBarElementPart(timeline, j, Track_num, DTW->AnglePart[j], DTW->AngPassAll, motion, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 36; j <= 39; j++)//左腕
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementGray(timeline, j, Track_num, DTW->AngPassAll, motion);
					else
						ColorBarElementPart(timeline, j, Track_num, DTW->AnglePart[j], DTW->AngPassAll, motion, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 1; j <= 3; j++)//右脚
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementGray(timeline, j, Track_num, DTW->AngPassAll, motion);
					else
						ColorBarElementPart(timeline, j, Track_num, DTW->AnglePart[j], DTW->AngPassAll, motion, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 4; j <= 6; j++)//左脚
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementGray(timeline, j, Track_num, DTW->AngPassAll, motion);
					else
						ColorBarElementPart(timeline, j, Track_num, DTW->AnglePart[j], DTW->AngPassAll, motion, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				break;
			default:
				break;
			}
		else
			switch (pattern)
			{
			case 0:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);
				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);
				ColorBarElementRepPart(timeline, 0, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_arm, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[5]);
				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_arm, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[5]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_leg, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[8]);
				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_leg, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[8]);
				break;

			case 1:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_head, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[0]);
				Track_num++;

				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_chest, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[1]);
				ColorBarElementRepPart(timeline, 0, Track_num++, DTW->Ang_chest, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[1]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_arm, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[5]);
				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_arm, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[5]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_leg, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[8]);
				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_leg, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[8]);
				break;

			case 2:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);
				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);
				ColorBarElementRepPart(timeline, 0, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_right_arm, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[3]);
				Track_num++;

				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_left_arm, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[4]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_leg, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[8]);
				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_leg, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[8]);
				break;

			case 3:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);
				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);
				ColorBarElementRepPart(timeline, 0, Track_num++, DTW->Ang_head_chest, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[2]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_arm, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[5]);
				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_arm, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[5]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_right_leg, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[6]);
				Track_num++;

				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_left_leg, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[7]);
				break;

			case 4:
				for (int j = 12; j >= 11; j--)//頭部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_head, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[0]);
				Track_num++;

				for (int j = 10; j >= 7; j--)//胸部
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_chest, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[1]);
				ColorBarElementRepPart(timeline, 0, Track_num++, DTW->Ang_chest, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[1]);//腰
				Track_num++;

				for (int j = 13; j <= 16; j++)//右腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_right_arm, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[3]);
				Track_num++;

				for (int j = 36; j <= 39; j++)//左腕
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_left_arm, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[4]);
				Track_num++;

				for (int j = 1; j <= 3; j++)//右脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_right_leg, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[6]);
				Track_num++;

				for (int j = 4; j <= 6; j++)//左脚
					ColorBarElementRepPart(timeline, j, Track_num++, DTW->Ang_left_leg, DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Pa_Color[7]);
				break;

			case 5:
				for (int j = 12; j >= 11; j--)//頭部
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementRepGray(timeline, j, Track_num, DTW->AngPassAll, motion);
					else
						ColorBarElementRepPart(timeline, j, Track_num, DTW->AnglePart[j], DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 10; j >= 7; j--)//胸部
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementRepGray(timeline, j, Track_num, DTW->AngPassAll, motion);
					else
						ColorBarElementRepPart(timeline, j, Track_num, DTW->AnglePart[j], DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				if (view_segment != 0 && view_segments[0] != 1)//腰
					ColorBarElementRepGray(timeline, 0, Track_num, DTW->AngPassAll, motion);
				else
					ColorBarElementRepPart(timeline, 0, Track_num, DTW->AnglePart[0], DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Seg_Color[0]);
				Track_num++;
				Track_num++;
				for (int j = 13; j <= 16; j++)//右腕
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementRepGray(timeline, j, Track_num, DTW->AngPassAll, motion);
					else
						ColorBarElementRepPart(timeline, j, Track_num, DTW->AnglePart[j], DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 36; j <= 39; j++)//左腕
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementRepGray(timeline, j, Track_num, DTW->AngPassAll, motion);
					else
						ColorBarElementRepPart(timeline, j, Track_num, DTW->AnglePart[j], DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 1; j <= 3; j++)//右脚
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementRepGray(timeline, j, Track_num, DTW->AngPassAll, motion);
					else
						ColorBarElementRepPart(timeline, j, Track_num, DTW->AnglePart[j], DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				Track_num++;
				for (int j = 4; j <= 6; j++)//左脚
				{
					if (view_segment != j && view_segments[j] != 1)
						ColorBarElementRepGray(timeline, j, Track_num, DTW->AngPassAll, motion);
					else
						ColorBarElementRepPart(timeline, j, Track_num, DTW->AnglePart[j], DTW->AngPassAll, motion, motion2, curr_frame, &DTW->Seg_Color[j]);
					Track_num++;
				}
				break;
			default:
				break;
			}
	}


	// 動作再生時刻を表す縦線を設定
	timeline->AddLine( curr_frame + num_space, Color4f( 1.0f, 1.0f, 1.0f, 1.0f ) );

	// 動作再生の切り替え位置を表す縦線を設定
	timeline->AddLine( mf + num_space, Color4f( 0.0f, 0.0f, 0.0f, 1.0f ) );
}

//
//パターンによるカラーバーの部位名前の色変化
//
Color4f MotionPlaybackApp::Pattern_Color(int pattern, int num_segment)
{
	Color4f c;

	switch (pattern)
	{
		case 0: //全体3色
			if( 1 <= num_segment && num_segment <= 6 )
				c = Color4f( 0.8f, 0.2f, 0.2f, 1.0f );
			else if( num_segment <= 12 )
				c = Color4f( 0.2f, 0.8f, 0.2f, 1.0f );
			else
				c = Color4f( 0.2f, 0.2f, 0.8f, 1.0f );
			break;
		case 1: //頭部と胸部2色
			if( num_segment == 0 )
				c = Color4f( 0.5f, 0.8f, 0.2f, 1.0f );
			else if( 7 <= num_segment && num_segment <= 10 )
				c = Color4f( 0.5f, 0.8f, 0.2f, 1.0f );
			else if( num_segment == 11 || num_segment == 12 )
				c = Color4f( 0.2f, 0.8f, 0.5f, 1.0f );
			else
				c = Color4f( 0.4f, 0.4f, 0.4f, 1.0f );
			break;
		case 2: //腕2色
			if( 13 <= num_segment && num_segment <= 35 )
				c = Color4f( 0.4f, 0.2f, 0.8f, 1.0f );
			else if( 36 <= num_segment )
				c = Color4f( 0.2f, 0.4f, 0.8f, 1.0f );
			else
				c = Color4f( 0.4f, 0.4f, 0.4f, 1.0f );
			break;
		case 3: //脚2色
			if( 1 <= num_segment && num_segment <= 3 )
				c = Color4f( 0.8f, 0.4f, 0.2f, 1.0f );
			else if( 4 <= num_segment && num_segment <= 6 )
				c = Color4f( 0.8f, 0.2f, 0.4f, 1.0f );
			else
				c = Color4f( 0.4f, 0.4f, 0.4f, 1.0f );
			break;
		case 4: //全体6色
			if( num_segment == 0 )
				c = Color4f( 0.5f, 0.8f, 0.2f, 1.0f );
			else if( 1 <= num_segment && num_segment <= 3 )
				c = Color4f( 0.8f, 0.4f, 0.2f, 1.0f );
			else if( 4 <= num_segment && num_segment <= 6 )
				c = Color4f( 0.8f, 0.2f, 0.4f, 1.0f );
			else if( 7 <= num_segment && num_segment <= 10 )
				c = Color4f( 0.5f, 0.8f, 0.2f, 1.0f );
			else if( num_segment == 11 || num_segment == 12 )
				c = Color4f( 0.2f, 0.8f, 0.5f, 1.0f );
			else if( 13 <= num_segment && num_segment <= 35 )
				c = Color4f( 0.4f, 0.2f, 0.8f, 1.0f );
			else
				c = Color4f( 0.2f, 0.4f, 0.8f, 1.0f );
			break;
		case 5: //1部位1色
			if( num_segment == view_segment || view_segments[num_segment] == 1 )
				if( num_segment == 0 )
				c = Color4f( 0.5f, 0.8f, 0.2f, 1.0f );
				else if( 1 <= num_segment && num_segment <= 3 )
					c = Color4f( 0.8f, 0.4f, 0.2f, 1.0f );
				else if( 4 <= num_segment && num_segment <= 6 )
					c = Color4f( 0.8f, 0.2f, 0.4f, 1.0f );
				else if( 7 <= num_segment && num_segment <= 10 )
					c = Color4f( 0.5f, 0.8f, 0.2f, 1.0f );
				else if( num_segment == 11 || num_segment == 12 )
					c = Color4f( 0.2f, 0.8f, 0.5f, 1.0f );
				else if( 13 <= num_segment && num_segment <= 35 )
					c = Color4f( 0.4f, 0.2f, 0.8f, 1.0f );
				else
					c = Color4f( 0.2f, 0.4f, 0.8f, 1.0f );
			else
				c = Color4f( 0.4f, 0.4f, 0.4f, 1.0f );
			break;
		default:
			std::cout << "pattern isn't defined." << std::endl;
	}

	return c;
}

//
//体節の名前を入力
//
void MotionPlaybackApp::InitSegmentname(int num_segments)
{
	//体節の名前を入れる
	int i = 0;
	motion->body->segments[i++]->name = "Pelvis";
	motion->body->segments[i++]->name = "R-Thigh";
	motion->body->segments[i++]->name = "R-Calf";
	motion->body->segments[i++]->name = "R-Foof";
	motion->body->segments[i++]->name = "L-Thigh";
	motion->body->segments[i++]->name = "L-Calf";
	motion->body->segments[i++]->name = "L-Foof";
	motion->body->segments[i++]->name = "Back1";
	motion->body->segments[i++]->name = "Back2";
	motion->body->segments[i++]->name = "Back3";
	motion->body->segments[i++]->name = "Chest";
	motion->body->segments[i++]->name = "Neck";
	motion->body->segments[i++]->name = "Head";
	motion->body->segments[i++]->name = "R-Clavicle";
	motion->body->segments[i++]->name = "R-UpperArm";
	motion->body->segments[i++]->name = "R-ForeArm";
	motion->body->segments[i++]->name = "R-Hand";
	motion->body->segments[36]->name = "L-Clavicle";
	motion->body->segments[37]->name = "L-UpperArm";
	motion->body->segments[38]->name = "L-ForeArm";
	motion->body->segments[39]->name = "L-Hand";
}


//
//部位の集合毎のカラーバーの誤差による色の変化を設定(DTWframe)
//
//void MotionPlaybackApp::ColorBarElementSetPart(Timeline* timeline, int segment_num, int Track_num, vector<float> Distance, vector<vector<int>> PassAll, Motion & motion)
//{
//	float red_ratio = 0.0f;
//	float max_dep = 0.0f;
//	float max_frame = -1.0f;
//	float name_space = 30.0f;
//	//frame内での最大誤差値を求める
//	for(int f = 0; f < PassAll[0].size(); f++)
//	{
//		if(max_dep < Distance[PassAll[0][f]])
//		{
//			max_dep = Distance[PassAll[0][f]];
//			max_frame = f;
//		}
//	}
//	
//	//std::cout << max_frame << std::endl;
//
//	//誤差の大きさと色付けしてフレーム毎に表示
//	for(int i = 0; i < PassAll[0].size(); i++)
//	{
//		if(i == 0)
//			timeline->AddElement( 0.0f, name_space, Pattern_Color(pattern, segment_num),  motion.body->segments[segment_num]->name.c_str(), Track_num );
//		red_ratio = Distance[PassAll[0][i]] / max_dep;
//		if(red_ratio > 0.6f)
//			timeline->AddElement( i + name_space, i + 1.0f + name_space, Color4f( 1.0f, (1.0f - red_ratio) * (5.0f / 4.0f) * 2.0f, 0.0f, 1.0f ), "", Track_num );
//		else
//			timeline->AddElement( i + name_space, i + 1.0f + name_space, Color4f( red_ratio * (5.0f / 6.0f) * 2.0f, 1.0f, 0.0f, 1.0f ), "", Track_num );
//
//		if(i == max_frame && max_frame != -1)
//			timeline->AddElement( i + name_space, i + 1.0f + name_space, Color4f( 0.0f, 0.0f, 0.0f, 1.0f ), "", Track_num );
//	}
//}

//
//カラーバーの誤差による色の変化を設定(DTWframe)
//
void MotionPlaybackApp::ColorBarElementPart(Timeline* timeline, int segment_num, int Track_num, vector<vector<float>> Error, vector<vector<int>> PassAll, Motion& motion, float curr_frame, Color4f * curr_c)
{
	float red_ratio = 0.0f;
	float max_error = 0.0f;
	float min_error = 100.0f;
	float max_frame = -1.0f;
	float name_space = 30.0f;
	Color4f c;

	//frame内での最大誤差値を求める
	for(int f = 0; f < PassAll[0].size(); f++)
	{
		if(max_error < Error[PassAll[0][f]][PassAll[1][f]])
		{
			max_error = Error[PassAll[0][f]][PassAll[1][f]];
			max_frame = f;
		}
		else if (min_error > Error[PassAll[0][f]][PassAll[1][f]])
			min_error = Error[PassAll[0][f]][PassAll[1][f]];
	}

	max_error = max_error - min_error;

	//誤差の大きさと色付けしてフレーム毎に表示
	for(int f = 0; f < PassAll[0].size(); f++)
	{
		if(f == 0)
			timeline->AddElement( 0.0f, name_space, Pattern_Color(pattern, segment_num),  motion.body->segments[segment_num]->name.c_str(), Track_num );
		red_ratio = (Error[PassAll[0][f]][PassAll[1][f]] - min_error) / max_error;
		if (red_ratio == 1.0)
			c = Color4f( 0.0f, 0.0f, 0.0f, 1.0f );
		else if(red_ratio > 0.75)
			c = Color4f( 1.0f, (1.0f - red_ratio) * 4.0f, 0.0f, 1.0f );
		else if (red_ratio > 0.5)
			c = Color4f((red_ratio - 0.5f) * 4.0f, 1.0f, 0.0f, 1.0f);
		else if (red_ratio > 0.25)
			c = Color4f(0.0f, 1.0f, (0.5f - red_ratio) * 4.0f, 1.0f);
		else
			c = Color4f( 0.0f, red_ratio * 4.0f, 1.0f, 1.0f );

		timeline->AddElement( f + name_space, f + 1.0f + name_space, c, "", Track_num );
		if(f == curr_frame)
			*curr_c = c;
	}
}

//
//カラーバーの誤差による色の変化を設定(再生切り替え用)
//
void MotionPlaybackApp::ColorBarElementRepPart(Timeline* timeline, int segment_num, int Track_num, vector<vector<float>> Error, vector<vector<int>> PassAll, Motion& motion, Motion& motion2, float curr_frame, Color4f * curr_c)
{
	float red_ratio = 0.0f;
	float max_error = 0.0f;
	float min_error = 100.0f;
	float max_frame = -1.0f;
	float name_space = 30.0f;
	Color4f c;

	//frame内での最大誤差値を求める
	for(int f = 0; f <= mf; f++)
	{
		if(max_error < Error[PassAll[0][f]][PassAll[1][f]])
		{
			max_error = Error[PassAll[0][f]][PassAll[1][f]];
			max_frame = f;
		}
		else if (min_error > Error[PassAll[0][f]][PassAll[1][f]])
			min_error = Error[PassAll[0][f]][PassAll[1][f]];
	}

	for (int f = mf + 1; f < frames; f++)
	{
		if (max_error < Error[min(motion.num_frames - 1, f - mf + m1f)][min(motion2.num_frames - 1, f - mf + m2f)])
		{
			max_error = Error[min(motion.num_frames - 1, f - mf + m1f)][min(motion2.num_frames - 1, f - mf + m2f)];
			max_frame = f;
		}
		else if (min_error > Error[min(motion.num_frames - 1, f - mf + m1f)][min(motion2.num_frames - 1, f - mf + m2f)])
			min_error = Error[min(motion.num_frames - 1, f - mf + m1f)][min(motion2.num_frames - 1, f - mf + m2f)];
	}

	max_error = max_error - min_error;

	//誤差の大きさと色付けしてフレーム毎に表示
	for(int f = 0; f < frames; f++)
	{
		if(f == 0)
			timeline->AddElement( 0.0f, name_space, Pattern_Color(pattern, segment_num),  motion.body->segments[segment_num]->name.c_str(), Track_num );
		
		if (f <= mf)
			red_ratio = (Error[PassAll[0][f]][PassAll[1][f]] - min_error) / max_error;
		else
			red_ratio = (Error[min(motion.num_frames - 1, f - mf + m1f)][min(motion2.num_frames - 1, f - mf + m2f)] - min_error) / max_error;
		
		if (red_ratio == 1.0)
			c = Color4f( 0.0f, 0.0f, 0.0f, 1.0f );
		else if(red_ratio > 0.75)
			c = Color4f( 1.0f, (1.0f - red_ratio) * 4.0f, 0.0f, 1.0f );
		else if (red_ratio > 0.5)
			c = Color4f((red_ratio - 0.5f) * 4.0f, 1.0f, 0.0f, 1.0f);
		else if (red_ratio > 0.25)
			c = Color4f(0.0f, 1.0f, (0.5f - red_ratio) * 4.0f, 1.0f);
		else
			c = Color4f( 0.0f, red_ratio * 4.0f, 1.0f, 1.0f );

		timeline->AddElement( f + name_space, f + 1.0f + name_space, c, "", Track_num );
		if(f == curr_frame)
			*curr_c = c;
	}
}

//
//カラーバーを全て灰色に設定(DTWframe)
//
void MotionPlaybackApp::ColorBarElementGray(Timeline* timeline, int segment_num, int Track_num, vector<vector<int>> PassAll, Motion & motion)
{
	float name_space = 30.0f;
				
	//名前だけ表示
	for(int i = 0; i < PassAll[0].size(); i++)
		if(i == 0)
			timeline->AddElement( 0.0f, name_space, Pattern_Color(pattern, segment_num),  motion.body->segments[segment_num]->name.c_str(), Track_num );
}

//
//カラーバーを全て灰色に設定(再生切り替え用)
//
void MotionPlaybackApp::ColorBarElementRepGray(Timeline* timeline, int segment_num, int Track_num, vector<vector<int>> PassAll, Motion& motion)
{
	float name_space = 30.0f;
				
	//名前だけ表示
	for(int f = 0; f < frames; f++)
		if(f == 0)
			timeline->AddElement( 0.0f, name_space, Pattern_Color(pattern, segment_num),  motion.body->segments[segment_num]->name.c_str(), Track_num );
}


//
//DTW初期化
//
void DTWinformation::DTWinformation_init( int frames1, int frames2, const Motion & motion1, const Motion & motion2 )
{
	//iが体節、jがフレーム1、kがフレーム2
	int Num_segments = motion1.body->num_segments;
	this->Seg_Color = new Color4f[Num_segments];
	this->Pa_Color = new Color4f[9];

	this->DistanceAll.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->DistancePart.resize(Num_segments,vector<vector<float>>(frames1 + 1, vector<float>(frames2 + 1, 0.0f)));
	this->Dis_head.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Dis_chest.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Dis_head_chest.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Dis_right_arm.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Dis_left_arm.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Dis_arm.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Dis_right_leg.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Dis_left_leg.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Dis_leg.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->ErrorDisTotalAll = 0.0f;
	this->ErrorDisTotalPart = new float [Num_segments];
	this->DisOrder = new int[Num_segments];

	this->AngleAll.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->AnglePart.resize(Num_segments,vector<vector<float>>(frames1 + 1, vector<float>(frames2 + 1, 0.0f)));
	this->Ang_head.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Ang_chest.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Ang_head_chest.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Ang_right_arm.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Ang_left_arm.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Ang_arm.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Ang_right_leg.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Ang_left_leg.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->Ang_leg.resize(frames1 + 1, vector<float>(frames2 + 1, 0.0f));
	this->ErrorAngTotalAll = 0.0f;
	this->ErrorAngTotalPart = new float[Num_segments];
	this->AngOrder = new int[Num_segments];


	vector< Matrix4f >  seg_frame_array1, seg_frame_array2;
	vector< Point3f >  joi_pos_array1, joi_pos_array2;
	Matrix4f  mat1, mat2;
	vector< vector< Vector3f > > v1(frames1, vector< Vector3f >(Num_segments));
	vector< vector< Vector3f > > v2(frames2, vector< Vector3f >(Num_segments));
	vector< vector< Vector3f > > j1(frames1, vector< Vector3f >(Num_segments));
	vector< vector< Vector3f > > j2(frames2, vector< Vector3f >(Num_segments));

	for(int i = 0; i < Num_segments; i++)
	{
		this->DisOrder[i] = i;
		this->ErrorDisTotalPart[i] = 0.0f;
		this->AngOrder[i] = i;
		this->ErrorAngTotalPart[i] = 0.0f;
	}

	//std::cout << motion1.frames[0].root_ori << std::endl;
	//for (int i = 0; i < motion1.body->num_joints; i++)
	//{
	//	std::cout << i << std::endl;
	//	std::cout << motion1.frames[0].joint_rotations[i] << std::endl;
	//}

	//順運動学計算
	for(int j = 0; j < frames1; j++)
	{
		ForwardKinematics( motion1.frames[ j ], seg_frame_array1, joi_pos_array1 );
		for(int k = 0; k < frames2; k++)
		{
			ForwardKinematics( motion2.frames[ k ], seg_frame_array2, joi_pos_array2 );
			for(int i = 0; i < Num_segments; i++)
			{
				//手の部位は計算しない
				while(i > 16 && i < 36)
					i++;
				if(i > 39)
					break;

				// 体節の中心の位置・向きを基準とする変換行列を適用
				mat1.set( seg_frame_array1[ i ] );
				mat1.get(&v1[j][i]);

				mat2.set( seg_frame_array2[ i ] );
				mat2.get(&v2[k][i]);

				if(i == 0)
				{
					j1[j][i] = joi_pos_array1[6] - v1[j][i];
					j2[k][i] = joi_pos_array2[6] - v2[k][i];
				}
				else if(i == 10)
				{
					j1[j][i] = joi_pos_array1[10] - joi_pos_array1[9];
					j2[k][i] = joi_pos_array2[10] - joi_pos_array2[9];
				}
				else if(i == 3 || i == 6 || i == 12 || i == 16 || i == 39)
				{
					j1[j][i] = v1[j][i] - joi_pos_array1[i - 1];
					j2[k][i] = v2[k][i] - joi_pos_array2[i - 1];
				}
				else
				{
					j1[j][i] = joi_pos_array1[i] - joi_pos_array1[i - 1];
					j2[k][i] = joi_pos_array2[i] - joi_pos_array2[i - 1];
				}
			}
		}
	}

	//for(int i = 0; i < Num_segments; i++ )
	//{
	//	std::cout << "segment:" << i << "->" << v1[frames1-1][i] << std::endl;
	//	if (i < Num_segments - 1)
	//		std::cout << "joint:" << i << "->" <<joi_pos_array1[i] << std::endl;
	//}

	//位置誤差の計算
	for(int j = 0; j <= frames1; j++)
	{
		for(int k = 0; k <= frames2; k++)
		{
			for(int i = 0; i < Num_segments; i++)
			{
				//手の部位は計算しない
				while (i > 16 && i < 36)
					i++;
				if (i > 39)
					break;

				if(j == frames1 || k == frames2)
				{
					this->DistanceAll[j][k] =100.0f;
					this->DistancePart[i][j][k] = 100.0f;
					this->Dis_head[j][k] = 100.0f;
					this->Dis_chest[j][k] = 100.0f;
					this->Dis_head_chest[j][k] = 100.0f;
					this->Dis_right_arm[j][k] = 100.0f;
					this->Dis_left_arm[j][k] = 100.0f;
					this->Dis_leg[j][k] = 100.0f;
					this->Dis_right_leg[j][k] = 100.0f;
					this->Dis_left_leg[j][k] = 100.0f;
					this->Dis_arm[j][k] = 100.0f;
					break;
				}
				else
				{
					this->DistancePart[i][j][k] = 
						sqrt(pow(v1[j][i].x - v2[k][i].x, 2.0) +
						pow(v1[j][i].y - v2[k][i].y, 2.0) + 
						pow(v1[j][i].z - v2[k][i].z, 2.0));

					this->DistanceAll[j][k] += this->DistancePart[i][j][k];
					if (i == 11 || i == 12)
					{
						this->Dis_head[j][k] += this->DistancePart[i][j][k];
						this->Dis_head_chest[j][k] += this->DistancePart[i][j][k];
					}
					else if (i == 0 || (i >= 7 && i <= 10))
					{
						this->Dis_chest[j][k] += this->DistancePart[i][j][k];
						this->Dis_head_chest[j][k] += this->DistancePart[i][j][k];
					}
					else if (i >= 13 && i <= 16)
					{
						this->Dis_right_arm[j][k] += this->DistancePart[i][j][k];
						this->Dis_arm[j][k] += this->DistancePart[i][j][k];
					}
					else if (i >= 36 && i <= 39)
					{
						this->Dis_left_arm[j][k] += this->DistancePart[i][j][k];
						this->Dis_arm[j][k] += this->DistancePart[i][j][k];
					}
					else if (i >= 1 && i <= 3)
					{
						this->Dis_right_leg[j][k] += this->DistancePart[i][j][k];
						this->Dis_leg[j][k] += this->DistancePart[i][j][k];
					}
					else if (i >= 4 && i <= 6)
					{
						this->Dis_left_leg[j][k] += this->DistancePart[i][j][k];
						this->Dis_leg[j][k] += this->DistancePart[i][j][k];
					}
				}
			}
		}
	}

	//角度誤差の計算
	for(int j = 0; j <= frames1; j++)
	{
		for(int k = 0; k <= frames2; k++)
		{
			for(int i = 0; i < Num_segments; i++)
			{
				//手の部位は計算しない
				while (i > 16 && i < 36)
					i++;
				if (i > 39)
					break;

				if(j == frames1 || k == frames2)
				{
					this->AngleAll[j][k] = 100.0f;
					this->AnglePart[i][j][k] = 100.0f;
					this->Ang_head[j][k] = 100.0f;
					this->Ang_chest[j][k] = 100.0f;
					this->Ang_head_chest[j][k] = 100.0f;
					this->Ang_right_arm[j][k] = 100.0f;
					this->Ang_left_arm[j][k] = 100.0f;
					this->Ang_leg[j][k] = 100.0f;
					this->Ang_right_leg[j][k] = 100.0f;
					this->Ang_left_leg[j][k] = 100.0f;
					this->Ang_arm[j][k] = 100.0f;
					break;
				}
				else
				{
					this->AnglePart[i][j][k] = 
						(j1[j][i].z * j2[k][i].z + j1[j][i].z * j2[k][i].z + j1[j][i].z * j2[k][i].z) /
						(sqrt(pow(j1[j][i].x, 2.0) + pow(j1[j][i].y, 2.0) + pow(j1[j][i].z, 2.0)) *
						sqrt(pow(j2[k][i].x, 2.0) + pow(j2[k][i].y, 2.0) + pow(j2[k][i].z, 2.0)));
					//this->AnglePart[i][j][k] *= -0.5;
					//this->AnglePart[i][j][k] += 0.5f;
					this->AnglePart[i][j][k] += 1.0f;
					this->AnglePart[i][j][k] = 2.0f - this->AnglePart[i][j][k];
					this->AnglePart[i][j][k] /= 2.0f;

					this->AngleAll[j][k] += this->AnglePart[i][j][k];
					if (i == 11 || i == 12)
					{
						this->Ang_head[j][k] += this->AnglePart[i][j][k];
						this->Ang_head_chest[j][k] += this->AnglePart[i][j][k];
					}
					else if (i == 0 || (i >= 7 && i <= 10))
					{
						this->Ang_chest[j][k] += this->AnglePart[i][j][k];
						this->Ang_head_chest[j][k] += this->AnglePart[i][j][k];
					}
					else if (i >= 13 && i <= 16)
					{
						this->Ang_right_arm[j][k] += this->AnglePart[i][j][k];
						this->Ang_arm[j][k] += this->AnglePart[i][j][k];
					}
					else if (i >= 36 && i <= 39)
					{
						this->Ang_left_arm[j][k] += this->AnglePart[i][j][k];
						this->Ang_arm[j][k] += this->AnglePart[i][j][k];
					}
					else if (i >= 1 && i <= 3)
					{
						this->Ang_right_leg[j][k] += this->AnglePart[i][j][k];
						this->Ang_leg[j][k] += this->AnglePart[i][j][k];
					}
					else if (i >= 4 && i <= 6)
					{
						this->Ang_left_leg[j][k] += this->AnglePart[i][j][k];
						this->Ang_leg[j][k] += this->AnglePart[i][j][k];
					}
				}
			}
		}
	}

	//std::cout << "start"<< std::endl;
	//for(int k = 0; k <= frames2; k++)
	//{
	//		for (int j = 0; j <= frames1; j++)
	//			if(j != frames1)
	//				std::cout << std::fixed << std::setprecision(4) << this->DistanceAll[j][k] << ",";
	//			else
	//				std::cout << std::fixed << std::setprecision(4) << this->DistanceAll[j][k] << std::endl;
	//}

	//位置誤差の全体に対するパスの作成
	int f1 = frames1 - 1, f2 = frames2 - 1;
	this->DisPassAll.resize(2, vector<int>());
	float dis1, dis2, dis3;
	
	for(int j = 0; j < frames1; j++)
		for (int k = 0; k < frames2; k++)
			DistanceAll[j][k] += DisCostAcummurate(j, k);

	while (f1 >= 0 && f2 >= 0)
	{
		if (f1 == 0 && f2 == 0)
			break;
		else if(f1 == 0 && f2 != 0)
		{
			this->DisPassAll[0].push_back(f1);
			this->DisPassAll[1].push_back(--f2);
		}
		else if (f1 != 0 && f2 == 0)
		{
			this->DisPassAll[0].push_back(--f1);
			this->DisPassAll[1].push_back(f2);
		}
		else
		{
			dis1 = this->DistanceAll[f1 - 1][f2 - 1];
			dis2 = this->DistanceAll[f1 - 1][f2];
			dis3 = this->DistanceAll[f1][f2 - 1];
			if (dis1 > dis2 || dis1 > dis3)
				if (dis2 > dis3)
				{
					this->DisPassAll[0].push_back(f1);
					this->DisPassAll[1].push_back(--f2);
				}
				else
				{
					this->DisPassAll[0].push_back(--f1);
					this->DisPassAll[1].push_back(f2);
				}
			else
			{
				this->DisPassAll[0].push_back(--f1);
				this->DisPassAll[1].push_back(--f2);
			}
		}
	}
	std::reverse(this->DisPassAll[0].begin(), this->DisPassAll[0].end());
	std::reverse(this->DisPassAll[1].begin(), this->DisPassAll[1].end());
	this->DisFrame = DisPassAll[0].size();

	//角度誤差の全体に対するパスの作成
	f1 = frames1 - 1, f2 = frames2 - 1;
	this->AngPassAll.resize(2, vector<int>());

	for(int j = 0; j < frames1; j++)
		for (int k = 0; k < frames2; k++)
			AngleAll[j][k] += AngCostAcummurate(j, k);

	float ang1, ang2, ang3;
	while (f1 >= 0 && f2 >= 0)
	{
		if (f1 == 0 && f2 == 0)
			break;
		else if(f1 == 0 && f2 != 0)
		{
			this->AngPassAll[0].push_back(f1);
			this->AngPassAll[1].push_back(--f2);
		}
		else if (f1 != 0 && f2 == 0)
		{
			this->AngPassAll[0].push_back(--f1);
			this->AngPassAll[1].push_back(f2);
		}
		else
		{
			ang1 = this->AngleAll[f1 - 1][f2 - 1];
			ang2 = this->AngleAll[f1 - 1][f2];
			ang3 = this->AngleAll[f1][f2 - 1];
			if (ang1 > ang2 || ang1 > ang3)
				if (ang2 > ang3)
				{
					this->AngPassAll[0].push_back(f1);
					this->AngPassAll[1].push_back(--f2);
				}
				else
				{
					this->AngPassAll[0].push_back(--f1);
					this->AngPassAll[1].push_back(f2);
				}
			else
			{
				this->AngPassAll[0].push_back(--f1);
				this->AngPassAll[1].push_back(--f2);
			}
		}
	}
	std::reverse(this->AngPassAll[0].begin(), this->AngPassAll[0].end());
	std::reverse(this->AngPassAll[1].begin(), this->AngPassAll[1].end());
	this->AngFrame = AngPassAll[0].size();

	//位置誤差のパスの表示
	std::cout << "DistancePass"<< std::endl;
	for(int f = 0; f < this->DisFrame; f++)
		std::cout <<  f << "[" << this->DisPassAll[0][f] << "," <<
		this->DisPassAll[1][f] << "]" << std::endl;
	std::cout << "motion1.num_frames:" << frames1 << std::endl <<
		"motion2.num_frames:" << frames2 << std::endl;

	//角度誤差のパスの表示
	std::cout << "AnglePass" << std::endl;
	for (int f = 0; f < this->AngFrame; f++)
		std::cout << f << "[" << this->AngPassAll[0][f] << "," <<
		this->AngPassAll[1][f] << "]" << std::endl;
	std::cout << "motion1.num_frames:" << frames1 << std::endl <<
		"motion2.num_frames:" << frames2 << std::endl;

	//部位毎の位置誤差の合計値を取る
	for(int i = 0; i < Num_segments; i++)
	{
		//手の部位は計算しない
		while (i > 16 && i < 36)
			i++;
		if (i > 39)
			break;

		for(int f = 0; f < this->DisFrame; f++)
			this->ErrorDisTotalPart[i] += this->DistancePart[i][DisPassAll[0][f]][DisPassAll[1][f]];
	}

	//部位毎の角度誤差の合計値を取る
	for (int i = 0; i < Num_segments; i++)
	{
		//手の部位は計算しない
		while (i > 16 && i < 36)
			i++;
		if (i > 39)
			break;

		for (int f = 0; f < this->AngFrame; f++)
			this->ErrorAngTotalPart[i] += this->AnglePart[i][AngPassAll[0][f]][AngPassAll[1][f]];
	}

	//誤差の大きい順にDisorderを並べ替える
	for(int i = 0; i < Num_segments - 1; i++)
		for(int j = i; j < Num_segments; j++)
			if(this->ErrorDisTotalPart[i] < this->ErrorDisTotalPart[j])
				std::swap(this->DisOrder[i], this->DisOrder[j]);

	//誤差の大きい順にAngorderを並べ替える
	for (int i = 0; i < Num_segments - 1; i++)
		for (int j = i; j < Num_segments; j++)
			if (this->ErrorAngTotalPart[i] < this->ErrorAngTotalPart[j])
				std::swap(this->AngOrder[i], this->AngOrder[j]);

	//位置誤差の合計を加算
	std::cout << "ErrorDisTotalPart:" << std::endl;
	for(int i = 0; i < Num_segments; i++)
	{
		//手の部位は計算しない
		while (i > 16 && i < 36)
			i++;
		if (i > 39)
			break;

		//全部位の誤差の合計を計算
		this->ErrorDisTotalAll += this->ErrorDisTotalPart[i];

		std::cout << " [ " << this->DisOrder[i] << " segment: " << 
			this->ErrorDisTotalPart[this->DisOrder[i]] << " ] ";
	}
	std::cout << std::endl;

	//角度誤差の合計を加算
	std::cout << "ErrorAngTotalPart:" << std::endl;
	for (int i = 0; i < Num_segments; i++)
	{
		//手の部位は計算しない
		while (i > 16 && i < 36)
			i++;
		if (i > 39)
			break;
		//全部位の誤差の合計を計算
		this->ErrorAngTotalAll += this->ErrorAngTotalPart[i];
		std::cout << " [ " << this->AngOrder[i] << " segment: " <<
			this->ErrorAngTotalPart[this->AngOrder[i]] << " ] ";
	}
	std::cout << std::endl;
}

float DTWinformation::DisCostAcummurate(int j, int k)
{
	if(j == 0 && k == 0)
		return 0.0f;
	else if(j == 0 && k != 0)
		return DistanceAll[j][k - 1];
	else if(j != 0 && k == 0)
		return DistanceAll[j - 1][k];
	else
		return min(min(DistanceAll[j][k - 1], DistanceAll[j - 1][k]), DistanceAll[j - 1][k - 1]);
}

float DTWinformation::AngCostAcummurate(int j, int k)
{
	if(j == 0 && k == 0)
		return 0.0f;
	else if(j == 0 && k != 0)
		return AngleAll[j][k - 1];
	else if(j != 0 && k == 0)
		return AngleAll[j - 1][k];
	else
		return min(min(AngleAll[j][k - 1], AngleAll[j - 1][k]), AngleAll[j - 1][k - 1]);
}

//mat1.get(&rot1);
//e1[j][i].x = asin(rot1.m12);
//if (cos(e1[j][i].x) != 0.0f)
//{
//	e1[j][i].y = atan2(rot1.m02, rot1.m22);
//	e1[j][i].z = atan2(rot1.m10, rot1.m11);
//}
//else
//{
//	e1[j][i].y = atan2(-rot1.m20, rot1.m00);
//	e1[j][i].z = 0.0f;
//}

//mat2.get(&rot2);
//e2[k][i].x = asin(rot2.m12);
//if (cos(e2[k][i].x) != 0.0f)
//{
//	e2[k][i].y = atan2(rot2.m02, rot2.m22);
//	e2[k][i].z = atan2(rot2.m10, rot2.m11);
//}
//else
//{
//	e2[k][i].y = atan2(-rot2.m20, rot2.m00);
//	e2[k][i].z = 0.0f;
//}
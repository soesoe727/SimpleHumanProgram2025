/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  動作再生アプリケーション
**/

#ifndef  _MOTION_PLAYBACK_APP_H_
#define  _MOTION_PLAYBACK_APP_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"
#include "Timeline.h"


// プロトタイプ宣言
struct DTWinformation;

struct DTWinformation
{
	public:
		//対応パス
		int ** DTWpass;

		//パス対応された各フレームの誤差
		float ** DTWdepartment;

		//体節の順番を誤差の大きさ順に並べ替えた値
		int * DTWorder;

		//全体の誤差値の合計
		float DTWtotal;

		//frame毎の誤差値の合計
		int * DTWforder;

		//左脚の誤差
		float  * left_leg;

		//右脚の誤差
		float * right_leg;

		//脚全体の誤差
		float * leg;

		//胸部の誤差
		float * chest;

		//頭部の誤差
		float * head;

		//頭部と胸部の誤差
		float * head_chest;

		//左腕の誤差
		float * left_arm;

		//右腕の誤差
		float * right_arm;

		//腕全体の誤差
		float * arm;

	public:
		//DTW初期化
		void DTWinformation_init( int frames1, int frames2, const Motion & motion1, const Motion & motion2 );	

		//DTWフレーム毎の誤差
		void DTWinformation_frame(int now_frame, int num_segments);
};

//
//  動作再生アプリケーションクラス
//
class  MotionPlaybackApp : public GLUTBaseApp
{
  protected:
	// 動作再生の入力情報

	// 動作データ
	Motion *  motion;
	Motion *  motion2;

  protected:
	// 動作再生のための変数

	// 現在の姿勢
	Posture *  curr_posture;
	Posture *  curr_posture2;

	//パス対応された比較用の姿勢
	Posture  curr_posture3;

	// アニメーション再生中かどうかを表すフラグ
	bool  on_animation;

	// アニメーションの再生時間
	float  animation_time;

	// アニメーションの再生速度
	float  animation_speed;

	// 現在の表示フレーム番号
	int  frame_no;

	//領域の表示
	int area;

	//軌跡の表示
	int trace;

	//体節の番目
	int view_segment;

	//DTWの情報
	DTWinformation * DTWa;

	//全体誤差かフレーム毎誤差を表示するか
	int flag;

	//姿勢の色パターン
	int pattern;

	//Timeline
	Timeline * timeline;

  public:
	// コンストラクタ
	MotionPlaybackApp();

	// デストラクタ
	virtual ~MotionPlaybackApp();

  public:
	// イベント処理

	//  初期化
	virtual void  Initialize();

	//  開始・リセット
	virtual void  Start();

	//  画面描画
	virtual void  Display();

	// ウィンドウサイズ変更(pattern用)
	virtual void  Reshape(int w, int h);

	// キーボードのキー押下
	virtual void  Keyboard( unsigned char key, int mx, int my );

	// マウスドラッグ
	virtual void  MouseDrag( int mx, int my );

	// マウスクリック
	virtual void  MouseClick( int button, int state, int mx, int my );

	// アニメーション処理
	virtual void  Animation( float delta );

  public:
	// 補助処理

	int  ViewSegment(int i, int seg);

	// BVH動作ファイルの読み込み、動作・姿勢の初期化
	void  LoadBVH( const char * file_name );
	void  LoadBVH2( const char * file_name );

	// ファイルダイアログを表示してBVH動作ファイルを選択・読み込み
	void  OpenNewBVH();
	void  OpenNewBVH2();

	//タイムラインのパターン毎読み込み
	void  PatternTimeline( Timeline * timeline, Motion & motion, float curr_frame, DTWinformation * DTW);

	//パターンによるカラーバーの部位名前の色変化
	Color4f Pattern_Color(int pattern, int num_segment);

	//体節の名前を入力
	void InitSegmentname(int num_segments);

	//カラーバーの誤差による色の変化を設定
	void ColorBarElement(Timeline * timeline, int segment_num, int Track_num, float * frame_dep, Motion & motion);

	//カラーバーの誤差による色の変化を設定
	void ColorBarElementGray(Timeline * timeline, int segment_num, int Track_num, float * frame_dep, Motion & motion);

	//部位の集合による誤差の最大フレームに紫色を設定(パターン)
	void PartDepLine(Timeline * timeline, int part_num, int Track_num, DTWinformation * DTW, Motion & motion, int pattern);
};


#endif // _MOTION_PLAYBACK_APP_H_

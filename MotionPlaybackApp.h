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

		//パス作成後のフレーム数
		int DTWframe;

		//全体の対応パス
		vector< vector< int > > PassAll;

		//パス対応された各フレームの誤差(部位毎)
		vector< vector< vector<float > > > DistancePart;

		//パス対応された各フレームの誤差(全体)
		vector< vector< float > > DistanceAll;

		//体節の順番を誤差の大きさ順に並べ替えた値
		int * DTWorder;

		//全体の誤差値の合計
		float DisTotalAll;

		//部位毎の誤差値の合計
		float * DisTotalPart;

		//頭部の誤差
		vector< vector< float > > head;

		//胸部の誤差
		vector< vector< float > > chest;

		//頭部と胸部の誤差
		vector< vector< float > > head_chest;

		//右腕の誤差
		vector< vector< float > > right_arm;

		//左腕の誤差
		vector< vector< float > > left_arm;

		//腕全体の誤差
		vector< vector< float > > arm;

		//右脚の誤差
		vector< vector< float > > right_leg;

		//左脚の誤差
		vector< vector< float > > left_leg;

		//脚全体の誤差
		vector< vector< float > > leg;

	public:
		//DTW初期化
		void DTWinformation_init( int frames1, int frames2, const Motion & motion1, const Motion & motion2 );	

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
	Posture  Pass_posture1;
	Posture  Pass_posture2;

	// アニメーション再生中かどうかを表すフラグ
	bool  on_animation;

	// アニメーションの再生時間
	float  animation_time;

	// アニメーションの再生速度
	float  animation_speed;

	//再生切り替え後の全フレーム数
	int frames;

	// 現在の表示フレーム番号(frames)
	int  frame_no;

	// 現在の表示フレーム番号(DTWframes)
	int  DTWframe_no;

	//体節の番目
	int view_segment;

	//体節の色付け設定
	vector<int> view_segments;

	//DTWの情報
	DTWinformation * DTWa;

	//姿勢の色パターン
	int pattern;

	//Timeline
	Timeline * timeline;

	//num_space
	float num_space = 30.0f;

	//sabun
	int m1f = -30;
	int m2f = -30;
	int mf = -30;
	int sabun_flag = 1;

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
	void  PatternTimeline( Timeline * timeline, Motion & motion, Motion & motion2, float curr_frame, DTWinformation * DTW);

	//パターンによるカラーバーの部位名前の色変化
	Color4f Pattern_Color(int pattern, int num_segment);

	//体節の名前を入力
	void InitSegmentname(int num_segments);

	//部位の集合毎のカラーバーの誤差による色の変化を設定
	//void ColorBarElementSetPart(Timeline * timeline, int segment_num, int Track_num, vector<float> Distance, vector<vector<int>> PassAll, Motion & motion);

	//カラーバーの誤差による色の変化を設定(DTWframe)
	void ColorBarElementPart(Timeline * timeline, int segment_num, int Track_num, vector<vector<float>> Distance, vector<vector<int>> PassAll, Motion & motion);

	//カラーバーの誤差による色の変化を設定(再生切り替え用)
	void ColorBarElementRepPart(Timeline * timeline, int segment_num, int Track_num, vector<vector<float>> Distance, vector<vector<int>> PassAll, Motion & motion, Motion & motion2);

	//カラーバーを全て灰色に設定(DTWframe)
	void ColorBarElementGray(Timeline * timeline, int segment_num, int Track_num, vector<vector<int>> PassAll, Motion & motion);

	//カラーバーを全て灰色に設定(再生切り替え用)
	void ColorBarElementRepGray(Timeline * timeline, int segment_num, int Track_num, vector<vector<int>> PassAll, Motion & motion);
	
	//全部位との距離を取る
	//static float DistanceCalc( int num_segments, vector< Vector3f > v1, vector< Vector3f > v2);

	//1部位との距離を取る
	//static float DistanceCalcPart(int frame1, int frame2, int num_segment, vector< vector< Vector3f > > v1, vector< vector< Vector3f > > v2 );
};


#endif // _MOTION_PLAYBACK_APP_H_

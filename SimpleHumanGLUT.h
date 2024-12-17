/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  GLUTフレームワーク ＋ アプリケーション基底クラス
**/


#ifndef  SIMPLE_HUMAN_GLUT
#define  SIMPLE_HUMAN_GLUT


// Windows関数定義の読み込み
#include <windows.h>

// GLUT を使用
#include <GL/glut.h>

//
//  行列・ベクトルの表現には vecmath C++ライブラリ（http://objectclub.jp/download/vecmath1）を使用
//
#include <Vector3.h>
#include <Point3.h>
#include <Matrix3.h>
#include <Matrix4.h>
#include <Color3.h>
#include <Color4.h>

// STL を使用
#include <vector>
#include <string>
using namespace std;



///////////////////////////////////////////////////////////////////////////////
//
//  アプリケーション基底クラス
//


//
//  アプリケーション基底クラス
//
class  GLUTBaseApp
{
  protected:
	// アプリケーション情報

	// アプリケーション名
	string  app_name;

  protected:
	// 視点操作のための変数

	// 視点の方位角
	float  camera_yaw;

	// 視点の仰角
	float  camera_pitch;

	// 視点と注視点の距離
	float  camera_distance;

	// 注視点の位置
	Point3f  view_center;

  protected:
	// マウス入力処理のための変数

	// 右・左・中ボタンがドラッグ中かどうかのフラグ
	bool  drag_mouse_r;
	bool  drag_mouse_l;
	bool  drag_mouse_m;

	// 最後に記録されたマウスカーソルの座標
	int  last_mouse_x;
	int  last_mouse_y;

  protected:
	// 画面描画に関する変数

	// 光源位置
	Point4f  light_pos;

	// 影の方向・色
	Vector3f  shadow_dir;
	Color4f  shadow_color;

  protected:
	// アプリケーション状態の変数

	// ウィンドウのサイズ
	int  win_width;
	int  win_height;

	// 初期化フラグ
	bool  is_initialized;

	// 視点更新フラグ
	bool  is_view_updated;


  public:
	// コンストラクタ
	GLUTBaseApp();
	
	// デストラクタ
	virtual ~GLUTBaseApp() {}

  public:
	// アクセサ
	const string &  GetAppName() { return  app_name; }
	int  GetWindowWidth() { return  win_width; }
	int  GetWindowHeight() { return  win_height; }
	bool  IsInitialized() { return  is_initialized; }

  public:
	// イベント処理インターフェース

	//  初期化
	virtual void  Initialize();

	//  開始・リセット
	virtual void  Start();

	//  画面描画
	virtual void  Display();

	// ウィンドウサイズ変更
	virtual void  Reshape( int w, int h );

	// マウスクリック
	virtual void  MouseClick( int button, int state, int mx, int my );

	// マウスドラッグ
	virtual void  MouseDrag( int mx, int my );

	// マウス移動
	virtual void  MouseMotion( int mx, int my );

	// キーボードのキー押下
	virtual void  Keyboard( unsigned char key, int mx, int my );

	// キーボードの特殊キー押下
	virtual void  KeyboardSpecial( unsigned char key, int mx, int my );
	
	// アニメーション処理
	virtual void  Animation( float delta );

  protected:
	// 補助処理

	// 格子模様の床を描画
	void  DrawFloor( float tile_size, int num_x, int num_z, float r0, float g0, float b0, float r1, float g1, float b1 );

	// 文字情報を描画
	void  DrawTextInformation( int line_no, const char * message, Color3f color = Color3f( 1.0f, 0.0f, 0.0f ) );
};



///////////////////////////////////////////////////////////////////////////////
//
//  GLUTフレームワークのメイン関数
//


//
//  GLUTフレームワークのメイン関数（実行するアプリケーションのリストを指定）
//
int  SimpleHumanGLUTMain( const vector< class GLUTBaseApp * > & app_lists, int argc, char ** argv, const char * win_title = NULL, int win_width = 0, int win_height = 0 );


//
//  GLUTフレームワークのメイン関数（実行する一つのアプリケーションを指定）
//
int  SimpleHumanGLUTMain( class GLUTBaseApp * app, int argc, char ** argv, const char * win_title = NULL, int win_width = 0, int win_height = 0 );



#endif // SIMPLE_HUMAN_GLUT

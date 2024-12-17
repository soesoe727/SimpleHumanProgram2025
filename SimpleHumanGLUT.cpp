/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  GLUTフレームワーク ＋ アプリケーション基底クラス
**/


// ヘッダファイルのインクルード
#include "SimpleHumanGLUT.h"



///////////////////////////////////////////////////////////////////////////////
//
//  複数アプリケーションの管理・切替のための変数・関数
//

// 現在実行中のアプリケーション
class GLUTBaseApp *    app = NULL;

// 全アプリケーションのリスト
vector< class GLUTBaseApp * >    applications;

// ソフトウェア説明
const char *    software_description = "Human Animation Sample\nCopyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)";

// 実行アプリケーションの切替関数（プロトタイプ宣言）
void  ChangeApp( int app_no );



///////////////////////////////////////////////////////////////////////////////
//
//  アプリケーション基底クラス
//


//
//  コンストラクタ
//
GLUTBaseApp::GLUTBaseApp()
{
	app_name = "Unknown";

	win_width = 0;
	win_height = 0;
	is_initialized = false;
	is_view_updated = true;
}


//
//   初期化
//
void  GLUTBaseApp::Initialize()
{
	is_initialized = true;

	camera_yaw = 0.0f;
	camera_pitch = -20.0f;
	camera_distance = 5.0f;
	view_center.set( 0.0f, 0.0f, 0.0f );

	drag_mouse_r = false;
	drag_mouse_l = false;
	drag_mouse_m = false;
	last_mouse_x = 0;
	last_mouse_y = 0;

	light_pos.set( 0.0f, 10.0f, 0.0f, 1.0f );
	shadow_dir.set( 0.0f, 1.0f, 0.0f );
	shadow_color.set( 0.2f, 0.2f, 0.2f, 0.5f );
}


//
//   開始・リセット
//
void  GLUTBaseApp::Start()
{
}


//
//  画面描画
//
void  GLUTBaseApp::Display()
{
	// 画面をクリア
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );

	// 変換行列を設定（モデル座標系→カメラ座標系）
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	glTranslatef( 0.0, 0.0, -camera_distance );
	glRotatef( -camera_pitch, 1.0, 0.0, 0.0 );
	glRotatef( -camera_yaw, 0.0, 1.0, 0.0 );
	glTranslatef( -view_center.x, -0.5, -view_center.z );

	// 光源位置を再設定
	float  light0_position[] = { light_pos.x, light_pos.y, light_pos.z, light_pos.w };
	glLightfv( GL_LIGHT0, GL_POSITION, light0_position );

	// 格子模様の床を描画
	DrawFloor( 1.0f, 50, 50, 1.0f, 1.0f, 1.0f, 1.0f, 0.8f, 0.8f );
}


//
//  ウィンドウサイズ変更
//
void  GLUTBaseApp::Reshape( int w, int h )
{
	// ウィンドウのサイズを記録
	win_width = w;
	win_height = h;

	// 視点の更新フラグを設定
	is_view_updated = true;
}


//
//  マウスクリック
//
void  GLUTBaseApp::MouseClick( int button, int state, int mx, int my )
{
	// 左ボタンが押されたらドラッグ開始
	if ( ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_DOWN ) )
		drag_mouse_l = true;
	// 左ボタンが離されたらドラッグ終了
	else if ( ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_UP ) )
		drag_mouse_l = false;

	// 右ボタンが押されたらドラッグ開始
	if ( ( button == GLUT_RIGHT_BUTTON ) && ( state == GLUT_DOWN ) )
		drag_mouse_r = true;
	// 右ボタンが離されたらドラッグ終了
	else if ( ( button == GLUT_RIGHT_BUTTON ) && ( state == GLUT_UP ) )
		drag_mouse_r = false;

	// 中ボタンが押されたらドラッグ開始
	if ( ( button == GLUT_MIDDLE_BUTTON ) && ( state == GLUT_DOWN ) )
		drag_mouse_m = true;
	// 中ボタンが離されたらドラッグ終了
	else if ( ( button == GLUT_MIDDLE_BUTTON ) && ( state == GLUT_UP ) )
		drag_mouse_m = false;

	// 現在のマウス座標を記録
	last_mouse_x = mx;
	last_mouse_y = my;
}


//
//   マウスドラッグ
//
void  GLUTBaseApp::MouseDrag( int mx, int my )
{
	// SHIFTキーの押下状態を取得
	int  mod = glutGetModifiers();

	// 右ボタンのドラッグ中は視点を回転する
	if ( drag_mouse_r && !( mod & GLUT_ACTIVE_SHIFT ) )
	{
		// 前回のマウス座標と今回のマウス座標の差に応じて視点を回転

		// マウスの横移動に応じてＹ軸を中心に回転
		camera_yaw -= ( mx - last_mouse_x ) * 1.0;
		if ( camera_yaw < 0.0 )
			camera_yaw += 360.0;
		else if ( camera_yaw > 360.0 )
			camera_yaw -= 360.0;

		// マウスの縦移動に応じてＸ軸を中心に回転
		camera_pitch -= ( my - last_mouse_y ) * 1.0;
		if ( camera_pitch < -90.0 )
			camera_pitch = -90.0;
		else if ( camera_pitch > 90.0 )
			camera_pitch = 90.0;

		// 視点の更新フラグを設定
		is_view_updated = true;
	}

	// SHIFTキー ＋ 右ボタンのドラッグ中は視点とカメラの距離を変更する
	if ( drag_mouse_r && ( mod & GLUT_ACTIVE_SHIFT ) )
	{
		// 前回のマウス座標と今回のマウス座標の差に応じて視点を回転

		// マウスの縦移動に応じて距離を移動
		camera_distance += ( my - last_mouse_y ) * 0.2;
		if ( camera_distance < 2.0 )
			camera_distance = 2.0;

		// 視点の更新フラグを設定
		is_view_updated = true;
	}

	// 今回のマウス座標を記録
	last_mouse_x = mx;
	last_mouse_y = my;
}


//
//  マウス移動
//
void  GLUTBaseApp::MouseMotion( int mx, int my )
{
}


//
//  キーボードのキー押下
//
void  GLUTBaseApp::Keyboard( unsigned char key, int mx, int my )
{
}


//
//  キーボードの特殊キー押下
//
void  GLUTBaseApp::KeyboardSpecial( unsigned char key, int mx, int my )
{
}


//
//  アニメーション処理
//
void  GLUTBaseApp::Animation( float delta )
{
}


//
//  以下、補助処理
//


//
//  格子模様の床を描画
//
void  GLUTBaseApp::DrawFloor( float tile_size, int num_x, int num_z, float r0, float g0, float b0, float r1, float g1, float b1 )
{
	int  x, z;
	float  ox, oz;

	glBegin( GL_QUADS );
	glNormal3d( 0.0, 1.0, 0.0 );

	ox = - ( num_x * tile_size ) / 2;
	for ( x = 0; x < num_x; x++ )
	{
		oz = - ( num_z * tile_size ) / 2;
		for ( z = 0; z<num_z; z++ )
		{
			if ( ((x + z) % 2) == 0 )
				glColor3f( r0, g0, b0 );
			else
				glColor3f( r1, g1, b1 );

			glVertex3d( ox, 0.0, oz );
			glVertex3d( ox, 0.0, oz + tile_size );
			glVertex3d( ox + tile_size, 0.0, oz + tile_size );
			glVertex3d( ox + tile_size, 0.0, oz );

			oz += tile_size;
		}
		ox += tile_size;
	}
	glEnd();
}


//
//  文字情報を描画
//
void  GLUTBaseApp::DrawTextInformation( int line_no, const char * message, Color3f color )
{
	int   i;
	if ( message == NULL )
		return;

	// 射影行列を初期化（初期化の前に現在の行列を退避）
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D( 0.0, win_width, win_height, 0.0 );

	// モデルビュー行列を初期化（初期化の前に現在の行列を退避）
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();

	// Ｚバッファ・ライティングはオフにする
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );

	// メッセージの描画
	glColor3f( color.x, color.y, color.z );
	glRasterPos2i( 16, 28 + 24 * line_no );
	for ( i = 0; message[i] != '\0'; i++ )
		glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18, message[i] );

	// 設定を全て復元
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
	glMatrixMode( GL_PROJECTION );
	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );
	glPopMatrix();
}



///////////////////////////////////////////////////////////////////////////////
//
//  GLUTフレームワーク（イベント処理、初期化・メイン処理）
//


//
//  画面描画時に呼ばれるコールバック関数
//
void  DisplayCallback( void )
{
	// アプリケーションの描画処理
	if ( app )
		app->Display();

	// バックバッファに描画した画面をフロントバッファに表示
	glutSwapBuffers();
}


//
//  ウィンドウサイズ変更時に呼ばれるコールバック関数
//
void  ReshapeCallback( int w, int h )
{
	// ウィンドウ内の描画を行う範囲を設定（ここではウィンドウ全体に描画）
	glViewport(0, 0, w, h);
	
	// カメラ座標系→スクリーン座標系への変換行列を設定
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective( 45, (double)w/h, 1, 500 );

	// アプリケーションのウィンドウサイズ変更
	if ( app )
		app->Reshape( w, h );
}


//
//  マウスクリック時に呼ばれるコールバック関数
//
void  MouseClickCallback( int button, int state, int mx, int my )
{
	// アプリケーションのマウスクリック
	if ( app )
		app->MouseClick( button, state, mx, my );

	// 再描画
	glutPostRedisplay();
}


//
// マウスドラッグ時に呼ばれるコールバック関数
//
void  MouseDragCallback( int mx, int my )
{
	// アプリケーションのマウスドラッグ
	if ( app )
		app->MouseDrag( mx, my );

	// 再描画
	glutPostRedisplay();
}


//
// マウス移動時に呼ばれるコールバック関数
//
void  MouseMotionCallback( int mx, int my )
{
	// アプリケーションのマウスドラッグ
	if ( app )
		app->MouseMotion( mx, my );

	// 再描画
	glutPostRedisplay();
}


//
//  キーボードのキーが押されたときに呼ばれるコールバック関数
//
void  KeyboardCallback( unsigned char key, int mx, int my )
{
	// m キーでモードの切り替え
	if ( ( key == 'm' ) && app && ( applications.size() > 1 ) )
	{
		// 次のアプリケーションを選択
		int  app_no = 0;
		for ( int i = 0; i < applications.size(); i++ )
		{
			if ( app == applications[i] )
			{
				app_no = i;
				break;
			}
		}
		app_no = ( app_no + 1 ) % applications.size();

		// 実行アプリケーションの切替
		ChangeApp( app_no );
	}

	// r キーでアプリケーションのリセット
	if ( key == 'r' )
	{
		if ( app )
			app->Start();
	}

	// アプリケーションのキー押下
	if ( app )
		app->Keyboard( key, mx, my );

	// 再描画
	glutPostRedisplay();
}



//
//  キーボードの特殊キーが押されたときに呼ばれるコールバック関数
//
void  SpecialKeyboardCallback( int key, int mx, int my )
{
	// アプリケーションの特殊キー押下
	if ( app )
		app->KeyboardSpecial( key, mx, my );

	// 再描画
	glutPostRedisplay();
}


//
//  アイドル時に呼ばれるコールバック関数
//
void  IdleCallback( void )
{
	// アニメーション処理
	if ( app )
	{
		// アニメーションの時間変化（Δｔ）を計算
#ifdef  WIN32
		// システム時間を取得し、前回からの経過時間に応じてΔｔを決定
		static DWORD  last_time = 0;
		DWORD  curr_time = timeGetTime();
		float  delta = ( curr_time - last_time ) * 0.001f;
		if ( delta > 0.03f )
			delta = 0.03f;
		last_time = curr_time;
#else
		// 固定のΔｔを使用
		float  delta = 0.03f;
#endif

		// アプリケーションのアニメーション処理
		if ( app )
			app->Animation( delta );

		// 再描画の指示を出す（この後で再描画のコールバック関数が呼ばれる）
		glutPostRedisplay();
	}

#ifdef _WIN32
	// Windows環境では、CTRL＋右クリックでもメニューを呼び出せるようにする

	// 右クリックでのメニュー起動の状態
	static bool  menu_attached = false;

	// CTRLキーの押下状態を取得（Win32 API を使用）
	bool  ctrl = ( GetKeyState( VK_CONTROL ) & 0x80 );

	// 右クリックでのメニュー起動の登録・解除
	if ( ctrl && !menu_attached )
	{
		glutAttachMenu( GLUT_RIGHT_BUTTON );
		menu_attached = true;
	}
	else if ( !ctrl && menu_attached )
	{
		glutDetachMenu( GLUT_RIGHT_BUTTON );
		menu_attached = false;
	}
#endif
}


//
//  実行アプリケーションの切替
//
void  ChangeApp( int app_no )
{
	if ( ( app_no < 0 ) || ( app_no >= applications.size() ) )
		return;

	// 現在のウィンドウのサイズを取得
	int  win_width, win_height;
	GLUTBaseApp *  curr_app = app;
	if ( curr_app )
	{
		win_width = curr_app->GetWindowWidth();
		win_height = curr_app->GetWindowHeight();
	}

	// アプリケーションの初期化・開始
	app = applications[ app_no ];
	if ( !app->IsInitialized() )
		app->Initialize();
	app->Start();
	if ( curr_app )
		app->Reshape( win_width, win_height );
}


//
//  ポップアップメニュー選択時に呼ばれるコールバック関数
//
void  MenuCallback( int no )
{
	// 実行アプリケーションの切替
	if ( ( no >= 0 ) && ( no < applications.size() ) )
	{
		ChangeApp( no );
	}

	// ソフトウェア説明を表示（ダイアログボックスを使用）
	else if ( no == applications.size() )
	{
#ifdef _WIN32
		MessageBox( NULL, software_description, "About", MB_OK | MB_ICONINFORMATION );
#endif
	}
}


//
//  実行アプリケーション切替のためのポップアップメニューの初期化
//
void  InitAppMenu()
{
	// メニュー生成
	int  menu;
	menu = glutCreateMenu( MenuCallback );

	// 各アプリケーションのメニュー項目を追加
	for ( int i = 0; i < applications.size(); i++ )
	{
		glutAddMenuEntry( applications[ i ]->GetAppName().c_str(), i );
	}

	// ソフトウェア説明のためのメニュー項目を追加
	glutAddMenuEntry( "About...", applications.size() );

	// メニュー設定
	glutSetMenu( menu );

	// メニュー登録（マウスの中ボタンで表示されるように設定）
	glutAttachMenu( GLUT_MIDDLE_BUTTON );
}


//
//  レンダリング環境初期化関数
//
void  initEnvironment( void )
{
	// 光源を作成する
	float  light0_position[] = { 10.0, 10.0, 10.0, 1.0 };
	float  light0_diffuse[] = { 0.5, 0.5, 0.5, 1.0 };
	float  light0_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	float  light0_ambient[] = { 0.4, 0.4, 0.4, 1.0 };
	glLightfv( GL_LIGHT0, GL_POSITION, light0_position );
	glLightfv( GL_LIGHT0, GL_DIFFUSE, light0_diffuse );
	glLightfv( GL_LIGHT0, GL_SPECULAR, light0_specular );
	glLightfv( GL_LIGHT0, GL_AMBIENT, light0_ambient );
	glEnable( GL_LIGHT0 );

	// 光源計算を有効にする
	glEnable( GL_LIGHTING );

	// 物体の色情報を有効にする
	glEnable( GL_COLOR_MATERIAL );

	// Ｚテストを有効にする
	glEnable( GL_DEPTH_TEST );

	// 背面除去を有効にする
	glCullFace( GL_BACK );
	glEnable( GL_CULL_FACE );

	// 背景色を設定
	glClearColor( 0.5, 0.5, 0.8, 0.0 );
}



///////////////////////////////////////////////////////////////////////////////
//
//  GLUTフレームワークのメイン関数
//


//
//  GLUTフレームワークのメイン関数（実行するアプリケーションのリストを指定）
//
int  SimpleHumanGLUTMain( const vector< class GLUTBaseApp * > & app_lists, int argc, char ** argv, const char * win_title, int win_width, int win_height )
{
	// GLUTウィンドウのパラメタの決定（引数で指定されなかった場合はデフォルト値を設定）
	if ( !win_title || ( strlen( win_title ) == 0 ) )
		win_title = "Human Animation Sample";
	if ( win_width <= 0 )
		win_width = 640;
	if ( win_height <= 0 )
		win_height = 640;
	
	// GLUTの初期化
	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_STENCIL );
	glutInitWindowSize( win_width, win_height );
	glutInitWindowPosition( 0, 0 );
	glutCreateWindow( win_title );
	
	// コールバック関数の登録
	glutDisplayFunc( DisplayCallback );
	glutReshapeFunc( ReshapeCallback );
	glutMouseFunc( MouseClickCallback );
	glutMotionFunc( MouseDragCallback );
	glutPassiveMotionFunc( MouseMotionCallback );
	glutKeyboardFunc( KeyboardCallback );
	glutSpecialFunc( SpecialKeyboardCallback );
	glutIdleFunc( IdleCallback );

	// レンダリング環境初期化
	initEnvironment();

	// 全アプリケーションを登録
	applications = app_lists;

	// 最初のアプリケーションを実行開始
	ChangeApp( 0 );

	// 実行アプリケーション切替のためのポップアップメニューの初期化
	InitAppMenu();

	// GLUTのメインループに処理を移す
	glutMainLoop();
	return 0;
}


//
//  GLUTフレームワークのメイン関数（実行する一つのアプリケーションを指定）
//
int  SimpleHumanGLUTMain( class GLUTBaseApp * app, int argc, char ** argv, const char * win_title, int win_width, int win_height )
{
	vector< class GLUTBaseApp * >    app_lists;
	app_lists.push_back( app );
	
	return  SimpleHumanGLUTMain( app_lists, argc, argv, win_title, win_width,  win_height );
}




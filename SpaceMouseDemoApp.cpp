// ライブラリ・クラス定義の読み込み
#include "SpaceMouseDemoApp.h"
#include "SpaceMouseGLUTHelper.hpp"

#include <iostream>

using namespace SpaceMouseHelper;


//
//  コンストラクタ
//
SpaceMouseDemoApp::SpaceMouseDemoApp()
{
	app_name = "SpaceMouse Demo - Box Rotation";
}


//
//  デストラクタ
//
SpaceMouseDemoApp::~SpaceMouseDemoApp()
{
	// SpaceMouseのシャットダウン
	CSpaceMouseGLUT::GetInstance().Shutdown();
}


//
//  初期化
//
void  SpaceMouseDemoApp::Initialize()
{
	// 基底クラスの処理を実行
	GLUTBaseApp::Initialize();

	// 座標変換行列を初期化（単位行列）
	m_boxTransform.setIdentity();

	// SpaceMouseの初期化
	if (CSpaceMouseGLUT::GetInstance().Initialize("SpaceMouse Demo App")) {
		// 感度の設定
		CSpaceMouseGLUT::GetInstance().SetSensitivity(1.0f);
	}
}


//
//  開始・リセット
//
void  SpaceMouseDemoApp::Start()
{
	// 基底クラスの処理を実行
	GLUTBaseApp::Start();

	// 座標変換行列をリセット
	m_boxTransform.setIdentity();
	
	// SpaceMouse変換をリセット
	CSpaceMouseGLUT::GetInstance().Reset();
}


//
//  画面描画
//
void  SpaceMouseDemoApp::Display()
{
	// SpaceMouseの状態を更新（navlibからイベントをポーリング）
	CSpaceMouseGLUT::GetInstance().Update();

	// 基底クラスの処理を実行
	GLUTBaseApp::Display();

	// SpaceMouse変換を適用して直方体を描画
	glPushMatrix();
	
	// SpaceMouseによる変換行列を取得
	CSpaceMouseGLUT::GetInstance().GetTransformMatrix(m_boxTransform);

	// Y軸正の方向にオフセットを適用する（直方体が地面にめり込まないようにするため）
    glTranslatef(0.0f, 0.5f, 0.0f);

	float glMatrix[16] = {
		m_boxTransform.m00, m_boxTransform.m10, m_boxTransform.m20, m_boxTransform.m30,
		m_boxTransform.m01, m_boxTransform.m11, m_boxTransform.m21, m_boxTransform.m31,
		m_boxTransform.m02, m_boxTransform.m12, m_boxTransform.m22, m_boxTransform.m32,
		m_boxTransform.m03, m_boxTransform.m13, m_boxTransform.m23, m_boxTransform.m33
	};
	glMultMatrixf(glMatrix);
	
	// 直方体を描画
	DrawBox();
	
	glPopMatrix();

	// 現在のモード、時間・フレーム番号を表示
	DrawTextInformation( 0, "SpaceMouse Demo" );
	DrawTextInformation( 1, "Press 'R' to reset transformation" );
}


//
//  キーボードのキー押下
//
void  SpaceMouseDemoApp::Keyboard( unsigned char key, int mx, int my )
{
	const float translateStep = 0.1f;
	const float rotateStep = 0.1f;  // ラジアン

	switch (key) {
	case 'r':
	case 'R':
		// 変換をリセット
		m_boxTransform.setIdentity();
		CSpaceMouseGLUT::GetInstance().Reset();
		break;

	// 平行移動のキーボードコントロール
	case 'w':
	case 'W': // 前進 (Z+)
		CSpaceMouseGLUT::GetInstance().ApplyTranslation(0, 0, translateStep);
		break;
	case 's':
	case 'S': // 後退 (Z-)
		CSpaceMouseGLUT::GetInstance().ApplyTranslation(0, 0, -translateStep);
		break;
	case 'a':
	case 'A': // 左 (X-)
		CSpaceMouseGLUT::GetInstance().ApplyTranslation(-translateStep, 0, 0);
		break;
	case 'd':
	case 'D': // 右 (X+)
		CSpaceMouseGLUT::GetInstance().ApplyTranslation(translateStep, 0, 0);
		break;
	case 'q':
	case 'Q': // 上 (Y+)
		CSpaceMouseGLUT::GetInstance().ApplyTranslation(0, translateStep, 0);
		break;
	case 'e':
	case 'E': // 下 (Y-)
		CSpaceMouseGLUT::GetInstance().ApplyTranslation(0, -translateStep, 0);
		break;

	// 回転のキーボードコントロール
	case 'j':
	case 'J': // Y軸周りの回転
		CSpaceMouseGLUT::GetInstance().ApplyLocalRotation(rotateStep, 0, 1, 0);
		break;
	case 'l':
	case 'L': // Y軸周りの回転（反対方向）
          CSpaceMouseGLUT::GetInstance().ApplyLocalRotation(-rotateStep, 0, 1, 0);
		break;
	case 'i':
	case 'I': // X軸周りの回転
          CSpaceMouseGLUT::GetInstance().ApplyLocalRotation(rotateStep, 1, 0, 0);
		break;
	case 'k':
	case 'K': // X軸周りの回転（反対方向）
          CSpaceMouseGLUT::GetInstance().ApplyLocalRotation(-rotateStep, 1, 0, 0);
		break;
	case 'u':
	case 'U': // Z軸周りの回転
          CSpaceMouseGLUT::GetInstance().ApplyLocalRotation(rotateStep, 0, 0, 1);
		break;
	case 'o':
	case 'O': // Z軸周りの回転（反対方向）
          CSpaceMouseGLUT::GetInstance().ApplyLocalRotation(-rotateStep, 0, 0, 1);
		break;

	default:
		// その他のキーは基底クラスに処理させる
		GLUTBaseApp::Keyboard( key, mx, my );
		break;
	}
}


//
//  アニメーション処理
//
void  SpaceMouseDemoApp::Animation( float delta )
{
	// 特に処理なし（SpaceMouseによる更新は自動的に反映される）
}


//
//  直方体を描画
//
void  SpaceMouseDemoApp::DrawBox()
{
	// 直方体のサイズ
	float width = 1.0f;
	float height = 1.0f;
	float depth = 1.0f;

	// 色の設定
	glEnable(GL_LIGHTING);
	
	// 面ごとに異なる色を設定して描画
	glBegin(GL_QUADS);
	
	// 前面（赤）
	glColor3f(1.0f, 1.0f, 1.0f);
	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(-width/2, -height/2, depth/2);
	glVertex3f(width/2, -height/2, depth/2);
	glVertex3f(width/2, height/2, depth/2);
	glVertex3f(-width/2, height/2, depth/2);
	
	// 背面（緑）
	glNormal3f(0.0f, 0.0f, -1.0f);
	glVertex3f(-width/2, -height/2, -depth/2);
	glVertex3f(-width/2, height/2, -depth/2);
	glVertex3f(width/2, height/2, -depth/2);
	glVertex3f(width/2, -height/2, -depth/2);
	
	// 上面（青）
	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(-width/2, height/2, -depth/2);
	glVertex3f(-width/2, height/2, depth/2);
	glVertex3f(width/2, height/2, depth/2);
	glVertex3f(width/2, height/2, -depth/2);
	
	// 底面（黄）
	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(-width/2, -height/2, -depth/2);
	glVertex3f(width/2, -height/2, -depth/2);
	glVertex3f(width/2, -height/2, depth/2);
	glVertex3f(-width/2, -height/2, depth/2);
	
	// 右面（マゼンタ）
	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(width/2, -height/2, -depth/2);
	glVertex3f(width/2, height/2, -depth/2);
	glVertex3f(width/2, height/2, depth/2);
	glVertex3f(width/2, -height/2, depth/2);
	
	// 左面（シアン）
	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(-width/2, -height/2, -depth/2);
	glVertex3f(-width/2, -height/2, depth/2);
	glVertex3f(-width/2, height/2, depth/2);
	glVertex3f(-width/2, height/2, -depth/2);
	
	glEnd();

	// ワイヤーフレームを描画（エッジを強調）
	glDisable(GL_LIGHTING);
	glColor3f(0.0f, 0.0f, 0.0f);
	glLineWidth(2.0f);
	
	glBegin(GL_LINE_LOOP);
	glVertex3f(-width/2, -height/2, depth/2);
	glVertex3f(width/2, -height/2, depth/2);
	glVertex3f(width/2, height/2, depth/2);
	glVertex3f(-width/2, height/2, depth/2);
	glEnd();
	
	glBegin(GL_LINE_LOOP);
	glVertex3f(-width/2, -height/2, -depth/2);
	glVertex3f(width/2, -height/2, -depth/2);
	glVertex3f(width/2, height/2, -depth/2);
	glVertex3f(-width/2, height/2, -depth/2);
	glEnd();
	
	glBegin(GL_LINES);
	glVertex3f(-width/2, -height/2, depth/2);
	glVertex3f(-width/2, -height/2, -depth/2);
	glVertex3f(width/2, -height/2, depth/2);
	glVertex3f(width/2, -height/2, -depth/2);
	glVertex3f(width/2, height/2, depth/2);
	glVertex3f(width/2, height/2, -depth/2);
	glVertex3f(-width/2, height/2, depth/2);
	glVertex3f(-width/2, height/2, -depth/2);
	glEnd();
	
	glLineWidth(1.0f);

	// ローカル座標系の軸を描画（Zバッファをオフにして常に手前に表示）
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glLineWidth(3.0f);

    float axisLength = 0.5f;

    glBegin(GL_LINES);
    // X軸（赤）
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(axisLength, 0.0f, 0.0f);

    // Y軸（緑）
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, axisLength, 0.0f);

    // Z軸（青）
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, axisLength);
    glEnd();

    glLineWidth(1.0f);
    glEnable(GL_DEPTH_TEST);
}

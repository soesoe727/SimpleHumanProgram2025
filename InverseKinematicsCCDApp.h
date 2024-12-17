/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  逆運動学計算（CCD法）アプリケーション
**/

#ifndef  _INVERSE_KINEMATICS_CCD_APP_H_
#define  _INVERSE_KINEMATICS_CCD_APP_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"


//
//  逆運動学計算（CCD法）アプリケーションクラス
//
class  InverseKinematicsCCDApp : public GLUTBaseApp
{
  protected:
	// 姿勢＋逆運動学計算による姿勢変形情報

	// キャラクタの姿勢
	Posture *  curr_posture;

	// 支点・末端関節
	int  base_joint_no;
	int  ee_joint_no;

  protected:
	// IK計算のための変数

	// 関節点の位置
	vector< Point3f >  joint_world_positions;
	vector< Point3f >  joint_screen_positions;

  protected:
	// 描画設定

	// 関節点の描画設定
	bool   draw_joints;


  public:
	// コンストラクタ
	InverseKinematicsCCDApp();

	// デストラクタ
	virtual ~InverseKinematicsCCDApp();

  public:
	// イベント処理

	//  初期化
	virtual void  Initialize();

	//  開始・リセット
	virtual void  Start();

	//  画面描画
	virtual void  Display();

	// マウスクリック
	virtual void  MouseClick( int button, int state, int mx, int my );

	//  マウスドラッグ
	virtual void  MouseDrag( int mx, int my );

	// キーボードのキー押下
	virtual void  Keyboard( unsigned char key, int mx, int my );

  public:
	// Inverse Kinematics 処理

	//  Inverse Kinematics 計算（CCD法）
	virtual void  ApplyInverseKinematics( Posture & posture, int base_joint_no, int ee_joint_no, Point3f ee_joint_position );

  public:
	// 関節点の選択・移動のための補助処理

	// 関節点の位置の更新
	void  UpdateJointPositions( const Posture & posture );

	// 関節点の描画
	void  DrawJoint();

	// 関節点の選択
	void  SelectJoint( int mouse_x, int mouse_y, bool ee_or_base );

	// 関節点の移動（視線に垂直な平面上で上下左右に移動する）
	void  MoveJoint( int mouse_dx, int mouse_dy );
};


// 補助処理（グローバル関数）のプロトタイプ宣言

// 末端関節から支点関節へのパス（関節の配列と各関節における末端関節の方向）を探索
void  FindJointPath( const Skeleton * body, int base_joint_no, int ee_joint_no, vector< int > & joint_path, vector< int > & joint_path_signs );

// Inverse Kinematics 計算（CCD法）
void  ApplyInverseKinematicsCCD( Posture & posture, int base_joint_no, int ee_joint_no, Point3f ee_joint_position );

// 順運動学計算（※レポート課題）
void  MyForwardKinematics( const Posture & posture, vector< Matrix4f > & seg_frame_array, vector< Point3f > & joi_pos_array );


#endif // _INVERSE_KINEMATICS_CCD_APP_H_

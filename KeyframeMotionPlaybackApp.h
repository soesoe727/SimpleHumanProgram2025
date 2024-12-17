/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理 ライブラリ・サンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
***  Released under the MIT license http://opensource.org/licenses/mit-license.php
**/

/**
***  キーフレーム動作再生アプリケーション
**/

#ifndef  _KEYFRAME_MOTION_PLAYBACK_APP_H_
#define  _KEYFRAME_MOTION_PLAYBACK_APP_H_


// ライブラリ・クラス定義の読み込み
#include "SimpleHuman.h"
#include "SimpleHumanGLUT.h"
#include "MotionPlaybackApp.h"


//
//  キーフレーム動作再生アプリケーションクラス
//
class  KeyframeMotionPlaybackApp : public MotionPlaybackApp
{
  protected:
	// キーフレーム動作再生のための変数

	// キーフレーム動作データ
	KeyframeMotion *  keyframe_motion;

	// キーフレーム動作からの取得姿勢
	Posture *  keyframe_posture;

	// キーフレーム動作と元の動作を同期して再生するための時間のオフセット
	float  motion_time_offset;

  protected:
	// 描画設定

	// 元の動作を並べて再生表示
	bool  draw_original_motion;

	// キーフレームの姿勢を並べて表示
	bool  draw_key_poses;


  public:
	// コンストラクタ
	KeyframeMotionPlaybackApp();

	// デストラクタ
	virtual ~KeyframeMotionPlaybackApp();

  public:
	// イベント処理

	//  初期化
	virtual void  Initialize();

	//  画面描画
	virtual void  Display();

	// キーボードのキー押下
	virtual void  Keyboard( unsigned char key, int mx, int my );

	// アニメーション処理
	virtual void  Animation( float delta );
};


// 補助処理（グローバル関数）のプロトタイプ宣言

// キーフレーム動作から姿勢を取得
void  GetKeyframeMotionPosture( const KeyframeMotion & motion, float time, Posture & p );


#endif // _KEYFRAME_MOTION_PLAYBACK_APP_H_

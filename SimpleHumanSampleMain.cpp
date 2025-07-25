/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理のサンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
**/

/**
***  メイン関数
**/


// GLUTフレームワーク＋アプリケーション基底クラスの定義を読み込み
#include "SimpleHumanGLUT.h"

// アプリケーションの定義を読み込み
#include "MotionPlaybackApp.h"
#include "KeyframeMotionPlaybackApp.h"
#include "ForwardKinematicsApp.h"
#include "PostureInterpolationApp.h"
#include "MotionInterpolationApp.h"
#include "MotionTransitionApp.h"
#include "MotionDeformationEditApp.h"
#include "InverseKinematicsCCDApp.h"



//
//  メイン関数（プログラムはここから開始）
//
int  main( int argc, char ** argv )
{
	// 全アプリケーションのリスト
	vector< class GLUTBaseApp * >    applications;

	// 全アプリケーションを登録
	//applications.push_back( new MotionPlaybackApp() );
	applications.push_back( new MotionPlaybackApp2() );
	//applications.push_back( new KeyframeMotionPlaybackApp() );
	//applications.push_back( new ForwardKinematicsApp() );
	//applications.push_back( new PostureInterpolationApp() );
	//applications.push_back( new MotionInterpolationApp() );
	//applications.push_back( new MotionTransitionApp() );
	//applications.push_back( new MotionDeformationEditApp() );
	//applications.push_back( new InverseKinematicsCCDApp() );

	// GLUTフレームワークのメイン関数を呼び出し（実行するアプリケーションのリストを指定）
	SimpleHumanGLUTMain( applications, argc, argv );
}



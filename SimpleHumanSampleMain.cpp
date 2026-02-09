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
#include "MotionApp.h"



//
//  メイン関数（プログラムはここから開始）
//
int  main( int argc, char ** argv )
{
	// 全アプリケーションのリスト
	vector< class GLUTBaseApp * >    applications;

	// アプリケーションを登録
	applications.push_back( new MotionApp() );

	// GLUTフレームワークのメイン関数を呼び出し（実行するアプリケーションのリストを指定）
	SimpleHumanGLUTMain( applications, argc, argv );
}



// *************************************************************************
//   ロボット名	： 電研ベーシックDCマウス、サンシャイン
//   概要		： サンシャインのsearch2ヘッダファイル
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato			新規（ファイルのインクルード）
// *************************************************************************/
// 多重コンパイル抑止
#ifndef _SEARCH2_H
#define _SEARCH2_H

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <hal.h>

//**************************************************
// 定義（define）
//**************************************************
/* 迷路サイズ */
#define GOAL_MAP_X_def					( 0 )				// ゴールのX区画数（横方向） [区画]
#define GOAL_MAP_Y_def					( 7 )				// ゴールのY区画数（縦方向） [区画]
#define MAP_X_SIZE					( 16 )				// 迷路のX区画数（横方向） [区画]
#define MAP_Y_SIZE					( 16 )				// 迷路のY区画数（縦方向） [区画]

#define MAP_X_SIZE_REAL				( 16 )					// 迷路の実X区画数（横方向） [区画]
#define MAP_Y_SIZE_REAL				( 16 )					// 迷路の実Y区画数（縦方向） [区画]

//**************************************************
// 列挙体（enum）
//**************************************************
/* 探索方法 */
typedef enum{
	CONTOUR_SYSTEM =0,			// 等高線MAP法
	MAP_SEARCH_TYPE_MAX,
}enMAP_SEARCH_TYPE;

/* 探索方法 */
typedef enum{
	SEARCH =0,			// 探索
	BEST_WAY,			// 最短
	MAP_ACT_MODE_MAX,
}enMAP_ACT_MODE;

/* 探索動作 */
typedef enum{
	SEARCH_TURN =0,		// 超信地旋回探索
	SEARCH_SURA,		// スラローム探索
	SEARCH_SKEW,		// 斜め探索
	SEARCH_MAX,
}enSEARCH_MODE;

/* 方位 */
typedef enum{
	NORTH =0,
	EAST,
	SOUTH,
	WEST,
	MAP_HEAD_DIR_MAX,
}enMAP_HEAD_DIR;



//**************************************************
// 構造体（struct）
//**************************************************


//**************************************************
// グローバル変数
//**************************************************
extern PUBLIC UCHAR		g_sysMap[MAP_Y_SIZE][MAP_X_SIZE];		///< 迷路情報
extern PUBLIC USHORT		us_cmap[MAP_Y_SIZE][MAP_X_SIZE];		///< 等高線 データ

extern PUBLIC UCHAR		GOAL_MAP_X;					//ゴール座標変更プログラム用ｘ
extern PUBLIC UCHAR		GOAL_MAP_Y;					//ゴール座標変更プログラム用ｙ

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
PUBLIC void MAP_init( void );
PUBLIC void MAP_Goal_init( void );
PUBLIC void MAP_showLog( void );
PUBLIC void MAP_clearMap( void );
PUBLIC void MAP_setPos( UCHAR uc_x, UCHAR uc_y, enMAP_HEAD_DIR en_dir );
PUBLIC void MAP_searchGoal( UCHAR uc_trgX, UCHAR uc_trgY, enMAP_ACT_MODE en_type, enSEARCH_MODE en_search );
PUBLIC void MAP_makeContourMap( UCHAR uc_goalX, UCHAR uc_goalY, enMAP_ACT_MODE en_type );
PUBLIC void MAP_actGoalLED( void );

PUBLIC void MAP_ClearMapData( void );



#endif //_SEARCH_H
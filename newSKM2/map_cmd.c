// *************************************************************************
//   ロボット名	： 電研ベーシックDCマウス、サンシャイン
//   概要		： サンシャインのmodeファイル
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.08.25			sato		新規（ファイルのインクルード）
// *************************************************************************/


//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <hal.h>						// HAL
#include <stdio.h>
#include <search2.h>
#include <parameters.h>
#include <init.h>
#include <hal_dist.h>
#include <map_cmd.h>

//**************************************************
// 定義（define）
//**************************************************
#define LIST_NUM			( 4096 )				// コマンド走行のリスト数


//**************************************************
// 列挙体（enum）
//**************************************************

//**************************************************
// 構造体（struct）
//**************************************************
/* シミュレーション */
typedef struct{
	enMAP_CMD	en_cmd;			// コマンド
	FLOAT		f_x0_x1;		// [0]/[1]のX座標加算値
	FLOAT		f_y0_y1;		// [0]/[1]のy座標加算値
	FLOAT		f_x2_x3;		// [2]/[3]のX座標加算値
	FLOAT		f_y2_y3;		// [2]/[3]のy座標加算値
	FLOAT		f_x4_x5;		// [4]/[5]のX座標加算値
	FLOAT		f_y4_y5;		// [4]/[5]のy座標加算値
	FLOAT		f_x6_x7;		// [6]/[7]のX座標加算値
	FLOAT		f_y6_y7;		// [6]/[7]のy座標加算値
	SHORT		s_dir;			// 進行方向（[0]北 [1]北東 [2]東 [3]南東 [4]南 [5]南西 [6]西 [7]北西 ）
}stMAP_SIM;


//**************************************************
// グローバル変数
//**************************************************
/* コマンドリスト */
PUBLIC	UCHAR		dcom[LIST_NUM];					// 超地信旋回用
PUBLIC	UCHAR		scom[LIST_NUM];					// スラローム用
PUBLIC	UCHAR		tcom[LIST_NUM];					// 斜め走行用
PRIVATE	USHORT		us_totalCmd;					// トータルコマンド量

PRIVATE	FLOAT		f_PosX;							// X座標
PRIVATE	FLOAT		f_PosY;							// Y座標
PRIVATE	SHORT		s_PosDir;						// 進行方向（[0]北 [1]北東 [2]東 [3]南東 [4]南 [5]南西 [6]西 [7]北西 ）

/* コマンドに応じた座標更新データ */
PRIVATE CONST stMAP_SIM st_PosData[] = {
	
	//	コマンド	[0]/[1]のX	[0]/[1]のY	[2]/[3]のX	[2]/[3]のY	[4]/[5]のX	[4]/[5]のY	[6]/[7]のX	[6]/[7]のY	方向
	{ R90,			0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-0.5,		0.5,		+2 },		// [0]
	{ L90,			-0.5,		0.5,		0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-2 },		// [1]
	{ R90S,			0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-0.5,		0.5,		+2 },		// [2]
	{ L90S,			-0.5,		0.5,		0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-2 },		// [3]
	{ RS45N,		0.25,		0.75,		0.75,		-0.25,		-0.25,		-0.75,		-0.75,		0.25,		+1 },		// [4]
	{ LS45N,		-0.25,		0.75,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-1 },		// [5]
	{ RS135N,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-0.25,		0.75,		+3 },		// [6]
	{ LS135N,		-0.75,		0.25,		0.25,		0.75,		0.75,		-0.25,		-0.25,		-0.75,		-3 },		// [7]
	{ RN45S,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-0.25,		0.75,		+1 },		// [8]
	{ LN45S,		0.25,		0.75,		0.75,		-0.25,		-0.25,		-0.75,		-0.75,		0.25,		-1 },		// [9]
	{ RN135S,		0.75,		-0.25,		-0.25,		-0.75,		-0.75,		0.25,		0.25,		0.75,		+3 },		// [10]
	{ LN135S,		-0.25,		0.75,		0.75,		0.25,		0.25,		-0.75,		-0.75,		-0.25,		-3 },		// [11]
	{ RN90N,		0.5,		0,			0,			-0.5,		-0.5,		0,			0,			0.5,		+2 },		// [12]
	{ LN90N,		0,			0.5,		0.5,		0,			0,			-0.5,		-0.5,		0,			-2 },		// [13]
	{ GO1,			0,			0.5,		0.5,		0,			0,			-0.5,		-0.5,		0,			0  },		// [14]
	{ NGO1,			0.5,		0.5,		0.5,		-0.5,		-0.5,		-0.5,		-0.5,		0.5,		0  },		// [15]
	{ MAP_CMD_MAX,	0,			0,			0,			0,			0,			0,			0,			0,			0  },
};

PRIVATE FLOAT f_LogPosX[30];
PRIVATE FLOAT f_LogPosY[30];
PRIVATE USHORT us_LogIndex = 0;
PRIVATE USHORT us_LogWallCut[30];
PRIVATE USHORT us_LogIndexWallCut = 0;



//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************


// *************************************************************************
//   機能		： 位置を更新する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PRIVATE void MAP_refPos( UCHAR uc_cmd )
{
	UCHAR uc_index = 0;			// テーブルのインデックス番号
	
	/* ------------------------------------------ */
	/*  コマンドからテーブルのインデックスを取得  */
	/* ------------------------------------------ */
	/* 直進 */
	if( ( uc_cmd <=  GO71 ) && ( uc_cmd >=  GO1) ){
		
		uc_index = 14;		// st_PosDataテーブルの直進のインデックス番号
	}
	/* 斜め直進 */
	else if( ( uc_cmd <=  NGO71 ) && ( uc_cmd >=  NGO1) ){
		
		uc_index = 15;		// st_PosDataテーブルの斜め直進のインデックス番号
	}
	/* その他のコマンド */
	else{
		while(1){
			
			if( st_PosData[uc_index].en_cmd == uc_cmd )      break;			// コマンド発見
			if( st_PosData[uc_index].en_cmd == MAP_CMD_MAX ) return;		// コマンド未発見
			uc_index++;
		}
	}
	
	/* 位置更新 */
	switch( s_PosDir ){
		
		/* [0]北 [1]北東 */
		case 0:
		case 1:
		
			/* 直進 */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x0_x1 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y0_y1 * uc_cmd;
			}
			/* 斜め直進 */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x0_x1 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y0_y1 * ( uc_cmd - 81 );
			}
			/* その他のカーブ */
			else{
				f_PosX += st_PosData[uc_index].f_x0_x1;
				f_PosY += st_PosData[uc_index].f_y0_y1;
			}
			break;
		
		/* [2]東 [3]南東 */
		case 2:
		case 3:

			/* 直進 */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x2_x3 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y2_y3 * uc_cmd;
			}
			/* 斜め直進 */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x2_x3 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y2_y3 * ( uc_cmd - 81 );
			}
			/* その他のカーブ */
			else{
				f_PosX += st_PosData[uc_index].f_x2_x3;
				f_PosY += st_PosData[uc_index].f_y2_y3;
			}
			break;

		/* [4]南 [5]南西 */
		case 4:
		case 5:

			/* 直進 */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x4_x5 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y4_y5 * uc_cmd;
			}
			/* 斜め直進 */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x4_x5 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y4_y5 * ( uc_cmd - 81 );
			}
			/* その他のカーブ */
			else{
				f_PosX += st_PosData[uc_index].f_x4_x5;
				f_PosY += st_PosData[uc_index].f_y4_y5;
			}
			break;

		/* [6]西 [7]北西 */
		case 6:
		case 7:

			/* 直進 */
			if( uc_index == 14 ){
				
				f_PosX += st_PosData[uc_index].f_x6_x7 * uc_cmd;
				f_PosY += st_PosData[uc_index].f_y6_y7 * uc_cmd;
			}
			/* 斜め直進 */
			else if( uc_index == 15 ){
				
				f_PosX += st_PosData[uc_index].f_x6_x7 * ( uc_cmd - 81 );
				f_PosY += st_PosData[uc_index].f_y6_y7 * ( uc_cmd - 81 );
			}
			/* その他のカーブ */
			else{
				f_PosX += st_PosData[uc_index].f_x6_x7;
				f_PosY += st_PosData[uc_index].f_y6_y7;
			}
			break;
	}
	
	/* 進行方向更新 */
	s_PosDir += st_PosData[uc_index].s_dir;
	if( s_PosDir < 0 ) s_PosDir += 8;				// [0]～[7]にしたい
	else if( s_PosDir > 7 ) s_PosDir -= 8;
	
	f_LogPosX[us_LogIndex] = f_PosX;
	f_LogPosY[us_LogIndex] = f_PosY;
	
	us_LogIndex++;
	us_LogIndex %= 30;
}

// *************************************************************************
//   機能		： コーナ前に壁があったら切れ目補正を行う設定をする
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PRIVATE BOOL MAP_setWallCut( UCHAR uc_cmd )
{
	UCHAR uc_val = 0;			// 1区画前のコーナー側の壁があるか（0以外なら壁あり）
	UCHAR uc_valPrev = 0;		// 2区画前のコーナー側の壁があるか（0以外なら壁あり）
	BOOL bl_wallCut = FALSE;
	
	/* 位置更新 */
	switch( uc_cmd ){
		
		case R90S:
		case RS135N:
			
			/* 1区画前のコーナー側の壁があるか（0以外ならあり） */
			// s_PosDir：進行方向（[0]北 [1]北東 [2]東 [3]南東 [4]南 [5]南西 [6]西 [7]北西 ）
			switch( s_PosDir ){
				
				/* 柱基準で旋回するので、半区画手前が壁の有無を調べたい座標となる（注意：g_sysMapは2次元配列です） */
				case 0: 
					if( 0 < f_PosY-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY-0.5)][(UCHAR)(f_PosX)] & 0x02;		// 北を向いているので東側の壁があるか
					if( 0 < f_PosY-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY-1.5)][(UCHAR)(f_PosX)] & 0x02;		// 北を向いているので東側の壁があるか
					break;	
				case 2: 
					if( 0 < f_PosX-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-0.5)] & 0x04;		// 東を向いているので南側の壁があるか
					if( 0 < f_PosX-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-1.5)] & 0x04;		// 東を向いているので南側の壁があるか
					break;
				case 4: 
					if( MAP_Y_SIZE_REAL > f_PosY+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY+0.5)][(UCHAR)(f_PosX)] & 0x08;		// 南を向いているので西側の壁があるか
					if( MAP_Y_SIZE_REAL > f_PosY+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY+1.5)][(UCHAR)(f_PosX)] & 0x08;		// 南を向いているので西側の壁があるか
					break;
				case 6:
					if( MAP_X_SIZE_REAL > f_PosX+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+0.5)] & 0x01;		// 西を向いているので北側の壁があるか
					if( MAP_X_SIZE_REAL > f_PosX+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+1.5)] & 0x01;		// 西を向いているので北側の壁があるか
					break;
			}
			/* 壁があるため壁切れ補正を行う */
			if( ( uc_val != 0 ) || ( ( uc_val != 0 ) && ( uc_valPrev != 0 ) ) ){
				
				MOT_setWallEdgeType( MOT_WALL_EDGE_RIGHT );		// 壁切れ補正を実施する
				bl_wallCut = TRUE;
			}
			break;
			
		case L90S:
		case LS135N:
			/* 1区画前のコーナー側の壁があるか（0以外ならあり） */
			// s_PosDir：進行方向（[0]北 [1]北東 [2]東 [3]南東 [4]南 [5]南西 [6]西 [7]北西 ）
			switch( s_PosDir ){
				
				/* 柱基準で旋回するので、半区画手前が壁の有無を調べたい座標となる（注意：g_sysMapは2次元配列です） */
				case 0: 
					if( 0 < f_PosY-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY-0.5)][(UCHAR)(f_PosX)] & 0x08;			// 北を向いているので西側の壁があるか
					if( 0 < f_PosY-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY-1.5)][(UCHAR)(f_PosX)] & 0x08;			// 北を向いているので西側の壁があるか
					break;
				case 2: 
					if( 0 < f_PosX-0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-0.5)] & 0x01;			// 東を向いているので北側の壁があるか
					if( 0 < f_PosX-1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX-1.5)] & 0x01;			// 東を向いているので北側の壁があるか
					break;
				case 4: 
					if( MAP_Y_SIZE_REAL > f_PosY+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY+0.5)][(UCHAR)(f_PosX)] & 0x02;			// 南を向いているので東側の壁があるか
					if( MAP_Y_SIZE_REAL > f_PosY+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY+1.5)][(UCHAR)(f_PosX)] & 0x02;			// 南を向いているので東側の壁があるか
					break;
				case 6: 
					if( MAP_X_SIZE_REAL > f_PosX+0.5 ) uc_val     = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+0.5)] & 0x04;			// 西を向いているので南側の壁があるか
					if( MAP_X_SIZE_REAL > f_PosX+1.5 ) uc_valPrev = g_sysMap[(UCHAR)(f_PosY)][(UCHAR)(f_PosX+1.5)] & 0x04;			// 西を向いているので南側の壁があるか
					break;
			}
			/* 壁があるため壁切れ補正を行う */
			if( ( uc_val != 0 ) || ( ( uc_val != 0 ) && ( uc_valPrev != 0 ) ) ){
				
				MOT_setWallEdgeType( MOT_WALL_EDGE_LEFT );		// 壁切れ補正を実施する
				bl_wallCut = TRUE;
			}
			break;
			
		default:
			break;
	}
	
	return bl_wallCut;
}

// *************************************************************************
//   機能		： コマンド走行用の座標位置を設定する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PROTECTED void MAP_setCmdPos( UCHAR uc_x, UCHAR uc_y, enMAP_HEAD_DIR en_dir )
{
	f_PosX   = (FLOAT)uc_x;
	f_PosX   = (FLOAT)uc_y;
	s_PosDir = (SHORT)(en_dir * 2);	// 進行方向（[0]北 [1]北東 [2]東 [3]南東 [4]南 [5]南西 [6]西 [7]北西 ）、2倍すると丁度値が合致する
	
}

// *************************************************************************
//   機能		： 迷路コマンドデータを表示する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PUBLIC void MAP_showCmdLog( void )
{
	USHORT i=0;
	
	/* 超信地旋回コマンド */
	while(1){
		
		printf("dcom[%4d] = %02d  \n\r",i,dcom[i]);
		if( dcom[i] == CEND ) break;
		i++;
	}
	i=0;
	
	/* スラロームコマンド */
	while(1){
		
		printf("scom[%4d] = %02d  \n\r",i,scom[i]);
		if( scom[i] == CEND ) break;
		i++;
	}
	i=0;

	/* 斜め走行コマンド */
	while(1){
		
		printf("tcom[%4d] = %02d  \n\r",i,tcom[i]);
		if( tcom[i] == CEND ) break;
		i++;
	}

}

// *************************************************************************
//   機能		： 超信地動作コマンド作成
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PUBLIC void MAP_makeCmdList( 
	UCHAR uc_staX,					///< [in] 開始X座標
	UCHAR uc_staY,					///< [in] 開始Y座標
	enMAP_HEAD_DIR en_staDir,		///< [in] 開始時の方向
	UCHAR uc_endX,					///< [in] 終了X座標
	UCHAR uc_endY,					///< [in] 終了Y座標
	enMAP_HEAD_DIR* en_endDir		///< [out] 終了時の方向
){
	UCHAR			uc_goStep;									// 前進のステップ数
	USHORT			us_high;									// 等高線の高さ
	USHORT			us_pt;										// コマンドポインタ
	enMAP_HEAD_DIR	en_nowDir;									// 現在マウスの向いている絶対方向
	enMAP_HEAD_DIR	en_tempDir;									// 相対方向
//	USHORT			i;											// roop
	
	/* 前進ステップ数を初期化する */
	uc_goStep = 0;
	us_pt = 0;

	/* 迷路情報からコマンド作成 */
	while(1){	
		us_high = us_cmap[uc_staY][uc_staX]-1;
		if (en_staDir == NORTH){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else   while(1);
		}else if (en_staDir == EAST){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else   while(1);
		}else if (en_staDir == SOUTH){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else   while(1);
		}else if (en_staDir == WEST){
			if     (((g_sysMap[uc_staY][uc_staX] & 0x88)==0x80)&&(us_cmap[uc_staY][uc_staX-1]==us_high)) en_nowDir=WEST;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x11)==0x10)&&(us_cmap[uc_staY+1][uc_staX]==us_high)) en_nowDir=NORTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x44)==0x40)&&(us_cmap[uc_staY-1][uc_staX]==us_high)) en_nowDir=SOUTH;
			else if(((g_sysMap[uc_staY][uc_staX] & 0x22)==0x20)&&(us_cmap[uc_staY][uc_staX+1]==us_high)) en_nowDir=EAST;
			else   while(1);
		}
		
		en_tempDir = (enMAP_HEAD_DIR)( (en_nowDir - en_staDir) & (enMAP_HEAD_DIR)3 );		// 方向更新
		en_staDir = en_nowDir;

		if (en_tempDir == NORTH){
			uc_goStep = uc_goStep + 2;
		}
		else if (en_tempDir == EAST){
			dcom[us_pt] = uc_goStep;
			dcom[++us_pt] = R90;
			uc_goStep = 2;
			us_pt++;
		}
		else if (en_tempDir == WEST){
			dcom[us_pt] = uc_goStep;
			dcom[++us_pt] = L90;
			uc_goStep = 2;
			us_pt++;
		}
		else{
			dcom[us_pt] = uc_goStep;
			dcom[++us_pt] = R180;
			uc_goStep = 2;
			us_pt++;
		}

		if      (en_nowDir == NORTH) uc_staY = uc_staY + 1;
		else if (en_nowDir == EAST) uc_staX = uc_staX + 1;
		else if (en_nowDir == SOUTH) uc_staY = uc_staY - 1;
		else if (en_nowDir == WEST) uc_staX = uc_staX - 1;
		
		en_staDir = en_nowDir;
		
		if ((uc_staX == uc_endX) &&(uc_staY == uc_endY)) break;
	}
	
	/* 超地信旋回用のコマンドリスト作成 */
	dcom[us_pt] = uc_goStep;
	dcom[++us_pt] = STOP;
	dcom[++us_pt] = CEND;
	us_totalCmd = us_pt+1;			// コマンド総数


	/* 最終的に向いている方向 */
	*en_endDir = en_staDir;
	
#if 0
	/* debug */
	for( i = 0; i < us_totalCmd; i++){
		printf("dcom[%4d] = %02d  \n\r",i,dcom[i]);
	}
#endif
}

// *************************************************************************
//   機能		： スラローム動作コマンド作成
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PUBLIC void MAP_makeSuraCmdList( void )
{
	USHORT dcom_temp[4096];			// 半区画超信旋回コマンドリスト
	USHORT i=0,j=0;					// roop
	
	/* 超地信旋回コマンドをコピー */
	for( i=0; i<us_totalCmd; i++ ){
		dcom_temp[i] = dcom[i];
	}

	i = 0;

	/* 配列が旋回コマンドかをチェック */
	while(1)
	{
		if( dcom_temp[i] == R90 ){		// 右90°
			dcom_temp[i-1] -= 1;		// 1つ手前を引く
			dcom_temp[i+1] -= 1;		// 1つ手前を引く
			dcom_temp[i] = R90S;		// 右スラローム90°
		}
		else if( dcom_temp[i] == L90 ){	// 左90°
			dcom_temp[i-1] -= 1;		// 1つ手前を引く
			dcom_temp[i+1] -= 1;		// 1つ手前を引く
			dcom_temp[i] = L90S;		// 左スラローム90°
		}
		else{
			if( dcom_temp[i] == CEND ){
				break;
			}
		}
		i++;
	}

	i = j = 0;

	/* スラロームコマンド変換 */
	while(1)
	{
		if( dcom_temp[i+1] == CEND ){
			scom[j] = STOP;
			scom[j+1] = CEND;
			break;
		}
		else
		{
			/* データがストップコマンドだったら */
			if( dcom_temp[i] == 0 ){
				i++;
			}
			
			scom[j] = dcom_temp[i];
			
			i++;
			j++;
		}
	}
	
#if 0
	for( i = 0; i < us_totalCmd; i++)
	{
		printf("scom[%4d] = %02d  \n\r",i,scom[i]);
	}
#endif

}

// *************************************************************************
//   機能		： 斜め動作コマンド作成
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.10.23			sato		新規
// *************************************************************************/
PUBLIC void MAP_makeSkewCmdList( void )
{
	USHORT	scom_temp[4096];			// 半区画超信旋回コマンドリスト
	USHORT	i;							// roop
	USHORT	c1, c2, c3, c4;				// 計算用
	USHORT	x;
	USHORT	ct_n=0, ct_st=0;
	USHORT	flag = 3;					//	斜め走行用バッファ  0:複合コマンド　1:斜め  2:S135N → N135S  3:直進
	
	/* 超地信旋回コマンドをコピー */
	for( i=0; i<us_totalCmd; i++ )
	{
		scom_temp[i] = scom[i];
	}

	i=0;

	/* 配列が旋回コマンドかをチェック */
	while(1)
	{
		c1 = scom_temp[ct_st];
		c2 = scom_temp[ct_st+1];
		c3 = scom_temp[ct_st+2];
		c4 = scom_temp[ct_st+3];

		//	直進 → 右45度 → 斜め
		if( (c1<=GO32) && (c2==R90S) && (c3==L90S) && (ct_st != 0) )
		{
			if( c1-1 != 0 ) tcom[ ct_n++ ] = c1 - 1;		//	前の複合コマンドによって直線区間が消えない場合
			tcom[ ct_n++ ] = RS45N;
			ct_st ++;

			x = (USHORT)(NGO1 - 1);		//	斜めモード
			flag = 0;
		}
		//	直進 → 左45度 → 斜め
		else if( (c1<=GO32) && (c2==L90S) && (c3==R90S) && (ct_st != 0) )
		{
			if( c1-1 != 0 ) tcom[ ct_n++ ] = c1 - 1;		//	前の複合コマンドによって直線区間が消えない場合
			tcom[ ct_n++ ] = LS45N;
			ct_st ++;

			x = (USHORT)(NGO1 - 1);		//	斜めモード
			flag = 0;
		}

		//	直進 → 右90度 → 直進
		else if( (c1<=GO32) && (c2==R90S) && (c3<=GO32) )
		{
			tcom[ ct_n++ ] = c1;
			tcom[ ct_n++ ] = R90S;
			ct_st += 2;
			flag = 3;		//	直進
		}
		//	直進 → 左90度 → 直進
		else if( (c1<=GO32) && (c2==L90S) && (c3<=GO32) )
		{
			tcom[ ct_n++ ] = c1;
			tcom[ ct_n++ ] = L90S;
			ct_st += 2;
			flag = 3;		//	直進
		}
		//	直進 → 右135度 → 斜め
		else if( (c1<=GO32) && (c2==R90S) && (c3==R90S) && (c4==L90S) )
		{
			tcom[ ct_n++ ] = c1;
			tcom[ ct_n++ ] = RS135N;
			ct_st += 2;

			x = (USHORT)(NGO1 - 1);		//	斜めモード
			flag = 2;
		}
		//	直進 → 左135度 → 斜め
		else if( (c1<=GO32) && (c2==L90S) && (c3==L90S) && (c4==R90S) )
		{
			tcom[ ct_n++ ] = c1;
			tcom[ ct_n++ ] = LS135N;
			ct_st += 2;

			x = (USHORT)(NGO1 - 1);		//	斜めモード
			flag = 2;
		}

		//	直進 → 右180度 → 直進
		else if( (c1<=GO32) && (c2==R90S) && (c3==R90S) && (c4<=GO32) )
		{
			tcom[ ct_n++ ] = c1;
			tcom[ ct_n++ ] = R90S;
			tcom[ ct_n++ ] = R90S;
			ct_st += 3;
			flag = 3;		//	直進
		}
		//	直進 → 左180度 → 直進
		else if( (c1<=GO32) && (c2==L90S) && (c2==L90S) && (c4<=GO32) )
		{
			tcom[ ct_n++ ] = c1;
			tcom[ ct_n++ ] = L90S;
			tcom[ ct_n++ ] = L90S;
			ct_st += 3;
			flag = 3;		//	直進
		}

		//	斜め → 右45度 → 直進
		else if( (c1==R90S) && (c2<=GO32)  && (flag != 3 ) )
		{
			if( flag==1 ) tcom[ ct_n++ ] = x;
			tcom[ ct_n++ ] = RN45S;
			scom_temp[ct_st+1] = c2 - 1;		//	直線区間を1つ減らす
			ct_st ++;
			flag = 3;		//	直進
		}
		//	斜め → 左45度 → 直進
		else if( (c1==L90S) && (c2<=GO32)  && (flag != 3 ) )
		{
			if( flag==1 ) tcom[ ct_n++ ] = x;
			tcom[ ct_n++ ] = LN45S;
			scom_temp[ct_st+1] = c2 - 1;		//	直線区間を1つ減らす
			ct_st ++;
			flag = 3;		//	直進
		}
		//	斜め → 右90度 → 斜め
		else if( (c1==L90S) && (c2==R90S) && (c3==R90S) && (c4==L90S)  && (flag != 3 ) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45NからRN90N
			else if( flag==1 ) tcom[ ct_n++ ] = x+1;
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = RN90N;
			ct_st +=2;

			x = (USHORT)(NGO1 - 1);		//	斜めモード
			flag = 1;
		}
		//	斜め → 左90度 → 斜め
		else if( (c1==R90S) && (c2==L90S) && (c3==L90S) && (c4==R90S)  && (flag != 3 ) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45NからLN90N
			else if( flag==1 ) tcom[ ct_n++ ] = x+1;
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = LN90N;
			ct_st +=2;

			x = (USHORT)(NGO1 - 1);		//	斜めモード
			flag = 1;
		}
		//	斜め → 右135度 → 直進
		else if( (c1==L90S) && (c2==R90S) && (c3==R90S) && (c4<=GO32)  && (flag != 3 ) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45NからLN90N
			else if( flag==1 ) tcom[ ct_n++ ] = x+1;
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = RN135S;
			ct_st += 3;
			flag = 3;		//	直進
		}
		//	斜め → 左135度 → 直進
		else if( (c1==R90S) && (c2==L90S) && (c3==L90S) && (c4<=GO32)  && (flag != 3 ) )
		{
			if( flag==0 ) tcom[ ct_n++ ] = NGO1;		//	45NからLN90N
			else if( flag==1 ) tcom[ ct_n++ ] = x+1;
			else if( flag==2 ) tcom[ ct_n++ ] = NGO1;
			tcom[ ct_n++ ] = LN135S;
			ct_st += 3;
			flag = 3;		//	直進
		}
		//	斜め → 斜め
		else if( (c1==R90S) && (c2==L90S) && ( (c3==R90S) || (c3==L90S) || ( c3<=GO32 ) ) && (flag != 3 ) )
		{
			x++;
			ct_st ++;

			flag = 1;		//	斜め走行バッファあり
		}
		else if( (c1==L90S) && (c2==R90S) && ( (c3==L90S) || (c3==R90S) || ( c3<=GO32 ) ) && (flag != 3 ) )
		{
			//	コマンド出力
			x++;
			ct_st ++;

			flag = 1;		//	斜め走行バッファあり
		}
		else
		{
			tcom[ ct_n ] = scom_temp[ct_st];
			if( tcom[ ct_n ] == CEND ) break;
			ct_st ++;
			ct_n ++;
		}
	}
	
#if 0
	for( i = 0; i < us_totalCmd; i++)
	{
		printf("tcom[%4d] = %02d  \n\r",i,tcom[i]);
	}
#endif

}

// *************************************************************************
//   機能		： コマンド走行モジュール
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PUBLIC void MAP_drive( enMAP_DRIVE_TYPE en_driveType )
{
	USHORT			us_rp = 0;				// 現在の読み込み位置
	enMOT_TURN_CMD 		en_type;
	BOOL			bl_isWallCut = FALSE;
	
	/* 超信旋回モード*/
	if( en_driveType == MAP_DRIVE_TURN )
	{
		while(1)
		{
			if ( dcom[us_rp] == CEND  ) break;								//	コマンド終了
			
			else if ( dcom[us_rp] == STOP  ){
			 	CTRL_stop();			// 制御停止
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
			}
			else if ( ( dcom[us_rp] <=  GO71 ) && ( dcom[us_rp] >=  GO1) )
			{
				MOT_goBlock_FinSpeed( (FLOAT)dcom[us_rp]*0.5f, 0 );		// 直線走行コマンド、半区間前進後に停止
			}
			else{
				
				if( dcom[us_rp] == R90 ) en_type = MOT_R90;
				else 					 en_type = MOT_L90;
				
				TIME_wait(500);
				MOT_turn( en_type );		//	旋回
				TIME_wait(500);
			}
			us_rp++;
			
			/* 途中で制御不能になった */
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop();
				DCM_brakeMot(DCM_R);
				DCM_brakeMot(DCM_L);
				break;
			}
			
		}
	 	CTRL_stop();			// 制御停止
		DCM_brakeMot( DCM_R );		// ブレーキ
		DCM_brakeMot( DCM_L );		// ブレーキ
	}
	/* スラロームモード */
	else if( en_driveType == MAP_DRIVE_SURA )
	{
		while(1)
		{
			MAP_refPos( scom[us_rp] );									// 実行されるコマンドが終了した位置に更新

			if ( scom[us_rp] == CEND  ) break;							//	コマンド終了
			
			else if ( scom[us_rp] == STOP  )
			{
			 	CTRL_stop();			// 制御停止
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
			}
			else if ( ( scom[us_rp] <=  GO71 ) && ( scom[us_rp] >=  GO1) )
			{
				if( scom[us_rp+1] == STOP  ){
					MOT_goBlock_FinSpeed( (FLOAT)scom[us_rp]*0.5f, 0 );						// 直線走行コマンド、半区間前進（最終速度なし）
				}
				else{
					
					/* 壁の切れ目補正 */
					if( ( scom[us_rp+1] == R90S )   || ( scom[us_rp+1] == L90S ) ){
						bl_isWallCut = MAP_setWallCut( scom[us_rp+1] );		// コーナー前に壁があったら壁の切れ目補正を行う設定をする
						
						if( bl_isWallCut == TRUE ){
							
							bl_isWallCut = FALSE;
							us_LogWallCut[us_LogIndexWallCut] = us_rp;
							us_LogIndexWallCut++;
							us_LogIndexWallCut %= 30;
						}
					}
					MOT_goBlock_FinSpeed( (FLOAT)scom[us_rp]*0.5f, MOT_getSuraStaSpeed() );		// 直線走行コマンド、半区間前進（最終速度あり）
				}
			}
			else if( scom[us_rp] == R90S )
			{
				MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// 右スラローム
			}
			else if( scom[us_rp] == L90S )
			{
				MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );	// 左スラローム
			}
			us_rp++;
			
			/* 途中で制御不能になった */
			if( SYS_isOutOfCtrl() == TRUE){
				CTRL_stop();
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}
			
		}
	}
	/* 斜めモード */
	else if( en_driveType == MAP_DRIVE_SKEW )
	{
		while(1)
		{
			MAP_refPos( tcom[us_rp] );									// 実行されるコマンドが終了した位置に更新
			
			if ( tcom[us_rp] == CEND  ) break;							//	コマンド終了

			else if ( tcom[us_rp] == STOP  )
			{
			 	CTRL_stop();			// 制御停止
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
			}
			else if ( ( tcom[us_rp] <=  GO71 ) && ( tcom[us_rp] >=  GO1) )
			{
				if( tcom[us_rp+1] == STOP  ){
					MOT_goBlock_FinSpeed( (FLOAT)tcom[us_rp]*0.5f, 0 );						// 直線走行コマンド、半区間前進（最終速度なし）
				}
				else{
					
					/* 壁の切れ目補正 */
					if( ( tcom[us_rp+1] == R90S )   || ( tcom[us_rp+1] == L90S )   || 
					 	( tcom[us_rp+1] == RS135N ) || ( tcom[us_rp+1] == LS135N ) 
					 ){
						bl_isWallCut = MAP_setWallCut( tcom[us_rp+1] );		// コーナー前に壁があったら壁の切れ目補正を行う設定をする
						
						if( bl_isWallCut == TRUE ){
							
							bl_isWallCut = FALSE;
							us_LogWallCut[us_LogIndexWallCut] = us_rp;
							us_LogIndexWallCut++;
							us_LogIndexWallCut %= 30;
						}
					}
					MOT_goBlock_FinSpeed( (FLOAT)tcom[us_rp]*0.5f, MOT_getSuraStaSpeed() );		// 直線走行コマンド、半区間前進（最終速度あり）
				}
			}
			else if ( ( tcom[us_rp] <=  NGO71 ) && ( tcom[us_rp] >=  NGO1) )
			{
				MOT_goSkewBlock_FinSpeed( (FLOAT)(tcom[us_rp]-81)*0.5f, MOT_getSuraStaSpeed());	// 斜め直線走行コマンド、半区間前進（最終速度あり）
			}
			else
			{
				switch( tcom[us_rp] )
				{

					/* 直進 → 直進 */
					case R90S:		MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );			break;
					case L90S:		MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );			break;
					
					/* 直進 → 斜め */
					case RS45N:		MOT_goSla( MOT_R45S_S2N, PARAM_getSra( SLA_45 ) ); 		break;
					case LS45N:		MOT_goSla( MOT_L45S_S2N, PARAM_getSra( SLA_45 ) ); 		break;
					case RS135N:	MOT_goSla( MOT_R135S_S2N, PARAM_getSra( SLA_135 ) ); 	break;
					case LS135N:	MOT_goSla( MOT_L135S_S2N, PARAM_getSra( SLA_135 ) ); 	break;

					/* 斜め → 直進 */
					case RN45S:		MOT_goSla( MOT_R45S_N2S, PARAM_getSra( SLA_45 ) ); 		break;
					case LN45S:		MOT_goSla( MOT_L45S_N2S, PARAM_getSra( SLA_45 ) ); 		break;
					case RN135S:	MOT_goSla( MOT_R135S_N2S, PARAM_getSra( SLA_135 ) ); 	break;
					case LN135S:	MOT_goSla( MOT_L135S_N2S, PARAM_getSra( SLA_135 ) ); 	break;

					/* 斜め → 斜め */
					case RN90N:		MOT_goSla( MOT_R90S_N, PARAM_getSra( SLA_N90 ) ); 		break;
					case LN90N:		MOT_goSla( MOT_L90S_N, PARAM_getSra( SLA_N90 ) );		break;
				}
			}
			us_rp++;
			
			/* 途中で制御不能になった */
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop();
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}
		}
	}

}

// *************************************************************************
//   機能		： オートスタートセット
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
/*PUBLIC void MAP_autostart()
{
	enMAP_HEAD_DIR	en_headset;
	
	// 迷路探索 
//	POS_clr();			// debug
//	POS_sta();			// debug
	MAP_setPos( 0, 0, NORTH );							// スタート位置
	MAP_searchGoal( GOAL_MAP_X, GOAL_MAP_Y, SEARCH, SEARCH_SURA );			// ゴール設定
//	POS_stop();			// debug
			
	// 帰りのスラローム探索 
	TIME_wait(1000);
	LED4 = LED4_ALL_OFF;
	MAP_searchGoal( 0, 0, SEARCH, SEARCH_SURA );
	TIME_wait(1000);
			
	if ( SW_ON == SW_INC_PIN ){}
	else{
		//最短走行1回目
//		PARAM_setCntType( TRUE );								// 最短走行
		MAP_setPos( 0, 0, NORTH );								// スタート位置
		MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// 等高線マップを作る
		MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir );		// ドライブコマンド作成
		MAP_makeSuraCmdList();													// スラロームコマンド作成
//		MAP_makeSkewCmdList();
			
		MOT_goHitBackWall();
		MOT_goBlock_FinSpeed( 0.3, 0 );
		TIME_wait(1000);
			
		MOT_setTrgtSpeed(SEARCH_SPEED*2);
		MOT_setSuraStaSpeed( (FLOAT)SEARCH_SPEED );							// スラローム開始速度設定
		PARAM_setSpeedType( PARAM_ST,   PARAM_NORMAL );							// [直進] 速度普通
		PARAM_setSpeedType( PARAM_TRUN, PARAM_NORMAL );							// [旋回] 速度普通
		PARAM_setSpeedType( PARAM_SLA,  PARAM_NORMAL );							// [スラ] 速度普通
			
		MAP_drive( MAP_DRIVE_SURA );
		TIME_wait(500);
		MOT_turn(MOT_R180);
		MAP_actGoalLED();
		
		//復路
		en_headset = (enMAP_HEAD_DIR)( (&en_endDir + 2) & (MAP_HEAD_DIR_MAX-1) );
		
//		PARAM_setCntType( TRUE );								// 最短走行
		MAP_setPos( GOAL_MAP_X, GOAL_MAP_Y, en_headset );								// スタート位置
		MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// 等高線マップを作る
		MAP_makeCmdList( GOAL_MAP_X, GOAL_MAP_Y, en_headset, 0, 0, &en_endDir );		// ドライブコマンド作成
		MAP_makeSuraCmdList();													// スラロームコマンド作成
//		MAP_makeSkewCmdList();
			
		MOT_goHitBackWall();
		MOT_goBlock_FinSpeed( 0.3, 0 );
		TIME_wait(1000);
		
		//最短走行2回目

}
*/
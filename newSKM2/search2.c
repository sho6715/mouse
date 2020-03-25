// *************************************************************************
//   ロボット名	： 電研ベーシックDCマウス、サンシャイン
//   概要		： サンシャインのsearch2ファイル
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.09.27			sato		新規（ファイルのインクルード）
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
#include <DataFlash.h>
#include <map_cmd.h>

//**************************************************
// 定義（define）
//**************************************************
#define MAP_SMAP_MAX_VAL	( MAP_X_SIZE * MAP_Y_SIZE ) 			///< 等高線mapの最大値
#define MAP_SMAP_MAX_PRI_VAL	( MAP_SMAP_MAX_VAL * 4 )				///< 等高線mapの優先度最大値

#define MOVE_BACK_DIST		(0.3f)

//**************************************************
// 列挙体（enum）
//**************************************************

//**************************************************
// 構造体（struct）
//**************************************************
#define MAP_SMAP_MAX_VAL		( MAP_X_SIZE * MAP_Y_SIZE ) 			///< 等高線mapの最大値
#define MAP_SMAP_MAX_PRI_VAL	( MAP_SMAP_MAX_VAL * 4 )				///< 等高線mapの優先度最大値


//**************************************************
// グローバル変数
//**************************************************
PRIVATE enMAP_HEAD_DIR	en_Head;										///< マウスの進行方向 0:N 1:E 2:S 3:W
PRIVATE UCHAR		my;												///< マウスのＸ座標
PRIVATE UCHAR		mx;												///< マウスのＹ座標
PUBLIC USHORT		us_cmap[MAP_Y_SIZE][MAP_X_SIZE];				///< 等高線 データ
PUBLIC UCHAR		g_sysMap[ MAP_Y_SIZE ][ MAP_X_SIZE ];			///< 迷路情報
PRIVATE FLOAT		f_MoveBackDist;									///< 壁当て動作で後退した距離[区画]
PRIVATE UCHAR		uc_SlaCnt = 0;									// スラローム連続回数
PRIVATE UCHAR		uc_back[ MAP_Y_SIZE ][ MAP_X_SIZE ];			// 迷路データ

PUBLIC UCHAR		GOAL_MAP_X;					//ゴール座標変更プログラム用ｘ
PUBLIC UCHAR		GOAL_MAP_Y;					//ゴール座標変更プログラム用ｙ

PUBLIC BOOL			search_flag;

PUBLIC UCHAR		GOAL_SIZE;
//等高線マップを更新を止めるための移動区画規定変数
PRIVATE UCHAR		uc_max_x = GOAL_MAP_X_def;
PRIVATE UCHAR		uc_max_y = GOAL_MAP_Y_def;

//TKR
/* 既知区間加速 */
typedef struct
{
	UCHAR	uc_StrCnt;
	BOOL	bl_Known;
}stMAP_KNOWN;

PRIVATE stMAP_KNOWN		st_known = { 0,FALSE };

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
extern PUBLIC	UCHAR		dcom[];					// 超地信旋回用
extern PUBLIC	UCHAR		scom[];					// スラローム用
extern PUBLIC	UCHAR		tcom[];					// 斜め走行用


// *************************************************************************
//   機能		： 迷路モジュールを初期化する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PUBLIC void MAP_init( void )
{
	UCHAR uc_dummy[ MAP_Y_SIZE ][ MAP_X_SIZE ];			// 迷路データ

	/* 座標、向き、迷路情報を初期化 */
	en_Head		= NORTH;
	mx		= 0;
	my		= 0;
	MAP_clearMap();
	
	/* 走行用のパラメータ */
	f_MoveBackDist = 0;
	uc_SlaCnt = 0;
	
//	Storage_Load( (const void*)uc_dummy, sizeof(uc_dummy), ADR_MAP );			// データロード(dummy) これがないと次の１発目のsaveがうまくいかない

	/* バックアップ迷路の復帰 */
/*	Storage_Load( (const void*)uc_back, sizeof(uc_back), ADR_MAP );				// バックアップデータを復帰する
	if( ( uc_back[0][0] & 0xf0 ) == 0xf0  ){									// データがあったら
		
		printf("\n\r　　　　　　　　　　　　＼　│　／");
		printf("\n\r　　　　　　　　　　　　　／￣＼　　／￣￣￣￣￣￣￣￣￣");
		printf("\n\r　　　　　　　　　　　─（ﾟ ∀ ﾟ）＜　迷路データをぉぉぉ");
		printf("\n\r　　　　　　　　　　　　　＼＿／　　＼＿＿＿＿＿＿＿＿＿");
		printf("\n\r　　　　　　　　　　　　／　│　＼");
		printf("\n\r　　　　　　　　　　　　　　　 ∩ ∧　∧∩ ／￣￣￣￣￣￣￣￣￣￣");
		printf("\n\r　￣￣￣￣￣￣￣￣＼ ∩∧ ∧∩ ＼（ ﾟ∀ﾟ）＜　復帰だ復帰だ復帰だ！");
		printf("\n\r　　復帰だ～～～！ ＞（ﾟ∀ﾟ）/ 　｜　　　/　＼＿＿＿＿＿＿＿＿＿＿");
		printf("\n\r　＿＿＿＿＿＿＿＿／ ｜　　〈　　｜　　 ｜");
		printf("\n\r　　　　　　　　　　 /　／＼_」　/　／＼」");
		printf("\n\r　　　　　　　　　　 ￣　　　　 / ／");
		printf("\n\r　　　　　　　　　　　　　　　  ￣");
		Storage_Load( (const void*)g_sysMap, sizeof(g_sysMap), ADR_MAP );		// バックアップ迷路データで現在の迷路情報を上書き
	}

*/
}

// *************************************************************************
//   機能		： ゴール座標を初期化する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.08.05			sato		新規
// *************************************************************************/
PUBLIC void MAP_Goal_init( void )
{
	GOAL_MAP_X = GOAL_MAP_X_def;
	GOAL_MAP_Y = GOAL_MAP_Y_def;
}

// *************************************************************************
//   機能		： Xゴール座標変更プログラム
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.08.05			sato		新規
// *************************************************************************/
PUBLIC void MAP_Goal_change_x( void )
{
	LED4 = GOAL_MAP_X;
	while(1){
		if ( SW_ON == SW_INC_PIN ){
			GOAL_MAP_X = GOAL_MAP_X ++;
			if (GOAL_MAP_X == 16){
				GOAL_MAP_X = 0;
			}
			LED4 = GOAL_MAP_X;
			TIME_wait(SW_CHATTERING_WAIT);			// SWが離されるまで待つ
			printf("GOAL_x %d\r\n",GOAL_MAP_X);
		} 
		else if (( SW_ON == SW_EXE_PIN )||(TRUE == MODE_CheckExe())){
			LED4 = LED4_ALL_OFF;
			break;
		}
	}
}

// *************************************************************************
//   機能		： yゴール座標変更プログラム
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.08.05			sato		新規
// *************************************************************************/
PUBLIC void MAP_Goal_change_y( void )
{
	LED4 = GOAL_MAP_Y;
	while(1){
		if ( SW_ON == SW_INC_PIN ){
			GOAL_MAP_Y = GOAL_MAP_Y ++;
			if (GOAL_MAP_Y == 16){
				GOAL_MAP_Y = 0;
			}
			LED4 = GOAL_MAP_Y;
			TIME_wait(SW_CHATTERING_WAIT);			// SWが離されるまで待つ
			printf("GOAL_y %d\r\n",GOAL_MAP_Y);
		} 
		else if (( SW_ON == SW_EXE_PIN )||(TRUE == MODE_CheckExe())){
			LED4 = LED4_ALL_OFF;
			break;
		}
	}
}

// *************************************************************************
//   機能		： 迷路情報をクリアする
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PUBLIC void MAP_ClearMapData( void )
{
	/* 座標、向き、迷路情報を初期化 */
	en_Head		= NORTH;
	mx			= 0;
	my			= 0;
	MAP_clearMap();
	
	/* 走行用のパラメータ */
	f_MoveBackDist = 0;
	uc_SlaCnt = 0;

//	Storage_Clear( sizeof(g_sysMap), ADR_MAP );			// データセーブ

}

// *************************************************************************
//   機能		： 座標位置を設定する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PUBLIC void MAP_setPos( UCHAR uc_x, UCHAR uc_y, enMAP_HEAD_DIR en_dir )
{
	mx		= uc_x;
	my		= uc_y;
	en_Head		= en_dir;
	
	MAP_setCmdPos( uc_x, uc_y, en_dir );

}

// *************************************************************************
//   機能		： 迷路データを表示する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PUBLIC void MAP_showLog( void )
{
	SHORT	x,y;
	CHAR	c_data;
	
	/* ---------- */
	/*  通常迷路  */
	/* ---------- */
	printf("\n\r  /* ---------- */   ");
	printf("\n\r  /*  通常迷路  */   ");
	printf("\n\r  /* ---------- */   ");

	printf("\n\r     ");
	for( x=0; x<MAP_X_SIZE; x++){
		printf("._");
	}
	printf(".\n\r");
	
	for( y=MAP_Y_SIZE-1; y>-1; y-- ){
		
		printf("   %2d",y);
		for( x=0; x<MAP_X_SIZE; x++){
			c_data = (UCHAR)g_sysMap[y][x];
			if ( ( c_data & 0x08 ) == 0 ){
				printf(".");
			}
			else{
				printf("|");
			}
			if ( ( c_data & 0x04 ) == 0 ){
				printf(" ");
			}
			else{
				printf("_");
			}
		}
		printf("|   ");
		
		for( x=0; x<MAP_X_SIZE; x++ ){
			c_data = g_sysMap[y][x];
			c_data = c_data >> 4;
			printf("%x", c_data);
		}
		
		printf("\n\r");
	}
	
	printf("     ");
	for( x=0; x<MAP_X_SIZE; x++){
		printf("%2d",x%10);
	}
	printf("\n\r");

}

// *************************************************************************
//   機能		： 迷路データをクリアする
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PUBLIC void MAP_clearMap( void )
{
	USHORT	x, y;
	UCHAR	uc_data;

	/* すべてのマップデータを未探索状態にする */
	for ( y = 0; y < MAP_Y_SIZE; y++){
		for( x = 0; x < MAP_X_SIZE; x++){
			uc_data = 0x00;
			if ( ( x == 0) && ( y == 0 ) ) uc_data = 0xfe;
			else if ( ( x == 1 ) && ( y == 0 ) ) uc_data = 0xcc;
			else if ( ( x == (MAP_X_SIZE-1) ) && ( y == 0 ) ) uc_data = 0x66;
			else if ( ( x == 0 ) && ( y == (MAP_Y_SIZE-1) ) ) uc_data = 0x99;
			else if ( ( x == (MAP_X_SIZE-1) ) && ( y == (MAP_Y_SIZE-1) ) ) uc_data = 0x33;
			else if ( x == 0 ) uc_data = 0x88;
			else if ( x == (MAP_X_SIZE-1) ) uc_data = 0x22;
			else if ( y == 0 ) uc_data = 0x44;
			else if ( y == (MAP_Y_SIZE-1) ) uc_data = 0x11;
			g_sysMap[y][x] = uc_data;
		}
	}

}

// *************************************************************************
//   機能		： センサ情報から壁情報を取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PRIVATE UCHAR MAP_getWallData( void )
{
	UCHAR	 uc_wall;

//	LED_offAll();			// debug

	// センサ情報から壁情報作成
	uc_wall = 0;
	if( TRUE == DIST_isWall_FRONT() ){
		uc_wall = uc_wall | 0x11;
//		LED_on(LED3);			// debug
//		LED_on(LED2);			// debug
	}
	if( TRUE == DIST_isWall_L_SIDE() ){
//		LED_on(LED0);			// debug
		uc_wall = uc_wall | 0x88;
	}
	if( TRUE == DIST_isWall_R_SIDE() ){
//		LED_on(LED1);			// debug
		uc_wall = uc_wall | 0x22;
	}

	// マウスの進行方向にあわせてセンサデータを移動し壁データとする
	if		( en_Head == EAST ){
		uc_wall = uc_wall >> 3;
	}
	else if ( en_Head == SOUTH ){
		uc_wall = uc_wall >> 2;
	}
	else if ( en_Head == WEST ){
		uc_wall = uc_wall >> 1;
	}

	//	探索済みフラグを立てる
	return ( uc_wall | 0xf0 );
}

// *************************************************************************
//   機能		： 壁情報を作成する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PRIVATE void MAP_makeMapData( void )
{
	UCHAR uc_wall;

	//	走行時の壁情報を迷路情報に書込
	if ( ( mx == 0 ) && ( my == 0 ) ){
		uc_wall = 0xfe;
	}
	else{
		uc_wall = MAP_getWallData();
	}
	g_sysMap[my][mx] = uc_wall;

	//	隣の区間のＭＡＰデータも更新する
	if ( mx != (MAP_X_SIZE-1) ){
		g_sysMap[my][mx+1] = ( g_sysMap[my][mx+1] & 0x77 ) | 0x80 | ( ( uc_wall << 2 ) & 0x08 );
	}
	if ( mx !=  0 ){
		g_sysMap[my][mx-1] = ( g_sysMap[my][mx-1] & 0xdd ) | 0x20 | ( ( uc_wall >> 2 ) & 0x02 );
	}
	if ( my != (MAP_Y_SIZE-1) ){
		g_sysMap[my+1][mx] = ( g_sysMap[my+1][mx] & 0xbb ) | 0x40 | ( ( uc_wall << 2 ) & 0x04 );
	}
	if ( my !=  0 ){
		g_sysMap[my-1][mx] = ( g_sysMap[my-1][mx] & 0xee ) | 0x10 | ( ( uc_wall >> 2 ) & 0x01 );
	}

}

// *************************************************************************
//   機能		： 等高線マップを作成する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PUBLIC void  MAP_makeContourMap( 
	UCHAR uc_goalX, 			///< [in] ゴールX座標
	UCHAR uc_goalY, 			///< [in] ゴールY座標
	enMAP_ACT_MODE	en_type		///< [in] 計算方法（まだ未使用）
){
	USHORT		x, y, i;		// ループ変数
	UCHAR		uc_dase;		// 基準値
	UCHAR		uc_new;			// 新値
	UCHAR		uc_level;		// 等高線
	UCHAR		uc_wallData;	// 壁情報

	en_type = en_type;		// コンパイルワーニング回避（いずれ削除）

	/* 等高線マップを初期化する */
	for ( i = 0; i < MAP_SMAP_MAX_VAL; i++ ){
		us_cmap[ i / MAP_Y_SIZE][ i & (MAP_X_SIZE-1) ] = MAP_SMAP_MAX_VAL - 1;
	}
	/* 目標地点の等高線を0に設定 */
	us_cmap[uc_goalY][uc_goalX] = 0;

	/* 等高線マップを作成 */
	uc_dase = 0;
	do{
		uc_level = 0;
		uc_new = uc_dase + 1;
		for ( y = 0; y < MAP_Y_SIZE; y++ ){
			for ( x = 0; x < MAP_X_SIZE; x++ ){
				if ( us_cmap[y][x] == uc_dase ){
					uc_wallData = g_sysMap[y][x];
					
					/* 探索走行 */
					if( SEARCH == en_type ){
						if ( ( ( uc_wallData & 0x01 ) == 0x00 ) && ( y != (MAP_Y_SIZE-1) ) ){
							if ( us_cmap[y+1][x] == MAP_SMAP_MAX_VAL - 1 ){
								us_cmap[y+1][x] = uc_new;
								uc_level++;
							}
						}
						if ( ( ( uc_wallData & 0x02 ) == 0x00 ) && ( x != (MAP_X_SIZE-1) ) ){
							if ( us_cmap[y][x+1] == MAP_SMAP_MAX_VAL - 1 ){
								us_cmap[y][x+1] = uc_new;
								uc_level++;
							}
						}
						if ( ( ( uc_wallData & 0x04 ) == 0x00 ) && ( y != 0 ) ){
							if ( us_cmap[y-1][x] == MAP_SMAP_MAX_VAL - 1 ){
								us_cmap[y-1][x] = uc_new;
								uc_level++;
							}
						}
						if ( ( ( uc_wallData & 0x08 ) == 0x00 ) && ( x != 0 ) ){
							if ( us_cmap[y][x-1] == MAP_SMAP_MAX_VAL - 1 ){
								us_cmap[y][x-1] = uc_new;
								uc_level++;
							}
						}
					}
					/* 最短走行 */
					else{
						if ( ( ( uc_wallData & 0x11 ) == 0x10 ) && ( y != (MAP_Y_SIZE-1) ) ){
							if ( us_cmap[y+1][x] == MAP_SMAP_MAX_VAL - 1 ){
								us_cmap[y+1][x] = uc_new;
								uc_level++;
							}
						}
						if ( ( ( uc_wallData & 0x22 ) == 0x20 ) && ( x != (MAP_X_SIZE-1) ) ){
							if ( us_cmap[y][x+1] == MAP_SMAP_MAX_VAL - 1 ){
								us_cmap[y][x+1] = uc_new;
								uc_level++;
							}
						}
						if ( ( ( uc_wallData & 0x44 ) == 0x40 ) && ( y != 0 ) ){
							if ( us_cmap[y-1][x] == MAP_SMAP_MAX_VAL - 1 ){
								us_cmap[y-1][x] = uc_new;
								uc_level++;
							}
						}
						if ( ( ( uc_wallData & 0x88 ) == 0x80 ) && ( x != 0 ) ){
							if ( us_cmap[y][x-1] == MAP_SMAP_MAX_VAL - 1 ){
								us_cmap[y][x-1] = uc_new;
								uc_level++;
							}
						}
					}
				}
			}
		}
		uc_dase = uc_dase + 1;
	}
	while( uc_level != 0 );
	
#if 0
	/* debug */
	for( x=0; x<4; x++ ){
		
		for( y=0; y<4; y++ ){
			us_Log[y][x][us_LogPt] = us_cmap[y][x];
		}
	}
	us_LogPt++;
#endif
	
}

// *************************************************************************
//   機能		： マウスの進行方向を決定する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PRIVATE void MAP_calcMouseDir( 
	enMAP_SEARCH_TYPE	en_calcType,	///< [in] 計算方法
	enMAP_HEAD_DIR* 	p_head			///< [out] 進行方向（戻り値）
){
	UCHAR		uc_wall;				// 壁情報
	USHORT		us_base;				// 等高線優先度決定値
	USHORT		us_new;
	enMAP_HEAD_DIR	en_tmpHead;

	/* 方向計算 */
	// 等高線MAP法
	if( CONTOUR_SYSTEM == en_calcType ){
		// 周辺の4区画で一番目的地に近い移動方向を算出する。
		// ただし、移動できる一番近い区間が複数ある場合には、次の順で選択する。
		// ①未探索区間,直進 ②未探索区間,旋回 ③既探索区間,直進 ④既探索区間,旋回
		uc_wall = g_sysMap[my][mx];
		us_base = MAP_SMAP_MAX_PRI_VAL;					// 16[区画]×16[区画]×4[方向]

		/* 4方向を比較 */
		//	北方向の区画の確認
		if ( ( uc_wall & 1 ) == 0 ){
			us_new = us_cmap[my+1][mx] * 4 + 4;
			if ( ( g_sysMap[my+1][mx] & 0xf0 ) != 0xf0 ) us_new = us_new - 2;
			if ( en_Head == NORTH ) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = NORTH;
			}
		}
		//	東方向の区画の確認
		if ( ( uc_wall & 2 ) == 0 ){
			us_new = us_cmap[my][mx+1] * 4 + 4;
			if ( ( g_sysMap[my][mx+1] & 0xf0 ) != 0xf0 ) us_new = us_new - 2;
			if ( en_Head == EAST) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = EAST;
			}
		}
		//	南方向の区画の確認
		if ( ( uc_wall & 4 ) == 0 ){
			us_new = us_cmap[my-1][mx] * 4 + 4;
			if ( ( g_sysMap[my-1][mx] & 0xf0 ) != 0xf0) us_new = us_new - 2;
			if ( en_Head == SOUTH ) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = SOUTH;
			}
		}
		//	西方向の区画の確認
		if ( ( uc_wall & 8 ) == 0 ){
			us_new = us_cmap[my][mx-1] * 4 + 4;
			if ( ( g_sysMap[my][mx-1] & 0xf0 ) != 0xf0 ) us_new = us_new - 2;
			if ( en_Head == WEST ) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = WEST;
			}
		}
		
		*p_head = (enMAP_HEAD_DIR)( (en_tmpHead - en_Head) & 3 );		// 移動方向
		
	}
	// 制御方法指定なし
	else{
		*p_head = (enMAP_HEAD_DIR)0;
	}

}

// *************************************************************************
//   機能		： マウスの座標位置を更新する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PRIVATE void MAP_refMousePos( 
	enMAP_HEAD_DIR 			en_head			///< [in] 進行方向
){
	switch( en_head ){
		case NORTH:
			my = my + 1;
			break;
		case EAST:
			mx = mx + 1;
			break;
		case SOUTH:
			my = my - 1;
			break;
		case WEST:
			mx = mx - 1;
			break;
		default:
			break;
	}
	
}

// *************************************************************************
//   機能		： 次の区画に移動する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PRIVATE void MAP_moveNextBlock( 
	enMAP_HEAD_DIR 	en_head,		///< [in] 相対進行方向（マウス進行方向を北としている）
	BOOL*			p_type			///< [in] FALSE: １区間前進状態、TURE:半区間前進状態
){
	*p_type = TRUE;
	f_MoveBackDist = 0;				// 移動距離を加算値クリア
	
	/* 動作 */
	switch( en_head ){

		/* そのまま前進 */
		case NORTH:
			*p_type = FALSE;
			MOT_goBlock_Const( 1 );				// 1区画前進
			break;
		// 右に旋回する
		case EAST:
			MOT_goBlock_FinSpeed( 0.5, 0 );		// 半区画前進
//			TIME_wait( MAP_TURN_WAIT );
			MOT_turn(MOT_R90);									// 右90度旋回
//			TIME_wait( MAP_TURN_WAIT );
			break;
		// 左に旋回する
		case WEST:
			MOT_goBlock_FinSpeed( 0.5, 0 );		// 半区画前進
//			TIME_wait( MAP_TURN_WAIT );
			MOT_turn(MOT_L90);									// 右90度旋回
//			TIME_wait( MAP_TURN_WAIT );
			break;
		// 反転して戻る
		case SOUTH:
			MOT_goBlock_FinSpeed( 0.5, 0 );		// 半区画前進
//			TIME_wait( MAP_TURN_WAIT );
			MOT_turn(MOT_R180);									// 右180度旋回
//			TIME_wait( MAP_TURN_WAIT );
			
			/* 壁当て姿勢制御（後ろに壁があったらバック＋移動距離を加算する） */
			if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) )  ||		// 北を向いていて北に壁がある
				( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// 東を向いていて東に壁がある
				( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) )  ||		// 南を向いていて南に壁がある
				( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) ) 			// 西を向いていて西に壁がある
			){
				MOT_goHitBackWall();					// バックする
				f_MoveBackDist = MOVE_BACK_DIST;		// バックした分の移動距離[区画]を加算
//				TIME_wait( MAP_TURN_WAIT );				// 時間待ち
			}
			break;
		default:
			break;
	}
	
	/* 前進中にパワーリリース機能が働いてレジュームしなければならない */
/*	if( ( TRUE == DCMC_isPowerRelease() ) && ( en_head == NORTH ) ){
		
		MOT_goBack_Const( MOT_BACK_POLE );					// １つ前の柱まで後退
		MAP_makeMapData();									// 壁データから迷路データを作成			← ここでデータ作成をミスっている
		MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			// 等高線MAP法で進行方向を算出			← 誤ったMAPを作成
		MAP_moveNextBlock(en_head, p_type);					// もう１度呼び出し（次の区画へ移動）
	}
	else{*/
		/* 進行方向更新 */
		en_Head = (enMAP_HEAD_DIR)( (en_Head + en_head) & (MAP_HEAD_DIR_MAX-1) );
//	}

}

// *************************************************************************
//   機能		： スラロームにて次の区画に移動する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PRIVATE void MAP_moveNextBlock_Sura( 
	enMAP_HEAD_DIR 	en_head,		///< [in] 相対進行方向（マウス進行方向を北としている）
	BOOL*			p_type,			///< [in] FALSE: １区間前進状態、TURE:半区間前進状態
	BOOL			bl_resume		///< [in] FALSE: レジューム動作ではない、TURE:レジューム動作
){
	*p_type = FALSE;
	f_MoveBackDist = 0;				// 移動距離を加算値クリア
	
	/* 動作 */
	switch( en_head ){

		// そのまま前進
		case NORTH:
			
			/* レジューム動作ではない */
			if( bl_resume == FALSE ){
		
				MOT_goBlock_Const( 1 );					// 1区画前進
				uc_SlaCnt = 0;							// スラロームしていない
			}
			/* レジューム動作 */
			else{
				MOT_goBlock_FinSpeed( 1.0f, SEARCH_SPEED );		// 半区画前進(バックの移動量を含む)
				uc_SlaCnt = 0;										// スラロームしていない
			}
			break;

		// 右にスラロームする
		case EAST:
			if( uc_SlaCnt < 7 ){
				MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// 右スラローム
				uc_SlaCnt++;
			}
			else{
				/* 壁当て姿勢制御（左に壁があったらバック＋移動距離を加算する） */
				if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) )  ||		// 北を向いていて西に壁がある
					( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) )  ||		// 東を向いていて北に壁がある
					( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// 南を向いていて東に壁がある
					( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) ) 			// 西を向いていて南に壁がある
				){
					MOT_goBlock_FinSpeed( 0.5, 0 );			// 半区画前進
//					TIME_wait( MAP_TURN_WAIT );
					MOT_turn(MOT_R90);						// 右90度旋回
//					TIME_wait( MAP_TURN_WAIT );
					uc_SlaCnt = 0;
					MOT_goHitBackWall();					// バックする
					f_MoveBackDist = MOVE_BACK_DIST;		// バックした分の移動距離[区画]を加算
//					TIME_wait( MAP_SLA_WAIT );				// 時間待ち
					*p_type = TRUE;							// 次は半区間（＋バック）分進める
				}
				else{
					MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// 右スラローム
					uc_SlaCnt++;
				}
			}
			break;

		// 左にスラロームする
		case WEST:
			if( uc_SlaCnt < 7 ){
				MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );	// 左スラローム
				uc_SlaCnt++;
			}
			else{
				/* 壁当て姿勢制御（後ろに壁があったらバック＋移動距離を加算する） */
				if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// 北を向いていて東に壁がある
					( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) )  ||		// 東を向いていて南に壁がある
					( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) )  ||		// 南を向いていて西に壁がある
					( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) ) 			// 西を向いていて北に壁がある
				){
					MOT_goBlock_FinSpeed( 0.5, 0 );		// 半区画前進
//					TIME_wait( MAP_TURN_WAIT );
					MOT_turn(MOT_L90);					// 右90度旋回
//					TIME_wait( MAP_TURN_WAIT );
					uc_SlaCnt = 0;
					MOT_goHitBackWall();					// バックする
					f_MoveBackDist = MOVE_BACK_DIST;		// バックした分の移動距離[区画]を加算
//					TIME_wait( MAP_SLA_WAIT );				// 時間待ち
					*p_type = TRUE;							// 次は半区間（＋バック）分進める
				}
				else{
					MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );	// 左スラローム
					uc_SlaCnt++;
				}
			}
			break;

		// 反転して戻る
		case SOUTH:
			MOT_goBlock_FinSpeed( 0.5, 0 );			// 半区画前進
//			TIME_wait( MAP_SLA_WAIT );
			MOT_turn(MOT_R180);									// 右180度旋回
//			TIME_wait( MAP_SLA_WAIT );
			uc_SlaCnt = 0;
			
			/* 壁当て姿勢制御（後ろに壁があったらバック＋移動距離を加算する） */
			if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) )  ||		// 北を向いていて北に壁がある
				( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// 東を向いていて東に壁がある
				( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) )  ||		// 南を向いていて南に壁がある
				( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) ) 			// 西を向いていて西に壁がある
			){
				MOT_goHitBackWall();					// バックする
				f_MoveBackDist = MOVE_BACK_DIST;		// バックした分の移動距離[区画]を加算
//				TIME_wait( MAP_SLA_WAIT );				// 時間待ち
			}
			*p_type = TRUE;								// 次は半区間＋バック分進める
			break;
			
		default:
			break;
	}
	
	/* 前進中にパワーリリース機能が働いてレジュームしなければならない */
/*	if( ( TRUE == DCMC_isPowerRelease() ) && ( en_head == NORTH ) ){
		
		TIME_wait(1000);
		MOT_goBack_Const( MOT_BACK_POLE );					// １つ前の柱まで後退
		MAP_makeMapData();									// 壁データから迷路データを作成			← ここでデータ作成をミスっている
		MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			// 等高線MAP法で進行方向を算出			← 誤ったMAPを作成
		MAP_moveNextBlock_Sura(en_head, p_type, TRUE );		// もう１度呼び出し（次の区画へ移動）
	}
	else{*/
		/* 進行方向更新 */
		en_Head = (enMAP_HEAD_DIR)( (en_Head + en_head) & (MAP_HEAD_DIR_MAX-1) );
//	}

}

// *************************************************************************
//   機能		： ゴール時の動作
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PRIVATE void MAP_actGoal( void )
{	
	MOT_goBlock_FinSpeed( 0.5, 0 );			// 半区画前進
	TIME_wait(500);
	MOT_turn(MOT_R180);										// 右180度旋回
	TIME_wait(500);
	
//	MAP_SaveMapData();						// 迷路情報のバックアップ
	log_flag_off();
	MAP_actGoalLED();
	
	en_Head = (enMAP_HEAD_DIR)( (en_Head + 2) & (MAP_HEAD_DIR_MAX-1) );			//	進行方向更新

}

// *************************************************************************
//   機能		： ゴール時のLED動作
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PUBLIC void MAP_actGoalLED( void )
{
	int i;
	for(i = 0;i<2;i++)
	{
		LED4 = 0x01;
		
		TIME_wait(100);
		LED4 = 0x02;
		
		TIME_wait(100);
		LED4 = 0x04;
		
		TIME_wait(100);
		LED4 = 0x08;
		
		TIME_wait(100);
		LED4 = 0x08;
		
		TIME_wait(100);
		LED4 = 0x04;
		
		TIME_wait(100);
		LED4 = 0x02;
		
		TIME_wait(100);
		LED4 = 0x01;
		
		TIME_wait(100);
	}
		
	TIME_wait(100);
	map_write();
	LED4 = LED4_ALL_ON;
}

// *************************************************************************
//   機能		： ゴール座標サイズ変更
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.11.23			sato		新規
// *************************************************************************/
PUBLIC void MAP_Goalsize(int size)
{
	GOAL_SIZE= size;
	if (size == 4) {
		uc_max_x = uc_max_x + 1;
		uc_max_y = uc_max_y + 1;
	}
	else if (size == 9) {
		uc_max_x = uc_max_x + 2;
		uc_max_y = uc_max_y + 2;
	}
}

// *************************************************************************
//   機能		： 帰還探索用等高線マップ作成
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.11.23			sato		新規
// *************************************************************************/
PUBLIC void  MAP_makeReturnContourMap(UCHAR uc_staX,UCHAR uc_staY) 
{
	USHORT		x, y, i;		// ループ変数
	UCHAR		uc_dase;		// 基準値
	UCHAR		uc_new;			// 新値
	UCHAR		uc_level;		// 等高線
	UCHAR		uc_wallData;	// 壁情報

	/* 等高線マップを初期化する */
	for (i = 0; i < MAP_SMAP_MAX_VAL; i++) {
		us_cmap[i / MAP_Y_SIZE][i & (MAP_X_SIZE - 1)] = MAP_SMAP_MAX_VAL - 1;
	}
	/* 目標地点の等高線を0に設定 */
	us_cmap[0][0] = 0;

	/* 等高線マップを作成 */
	uc_dase = 0;
	do {
		uc_level = 0;
		uc_new = uc_dase + 1;
		for (y = 0; y < MAP_Y_SIZE; y++) {
			for (x = 0; x < MAP_X_SIZE; x++) {
				if ((us_cmap[uc_staY][uc_staX] != MAP_SMAP_MAX_VAL - 1) && (us_cmap[uc_staY][uc_staX] + 2 < uc_new))break;
				if (us_cmap[y][x] == uc_dase) {
					uc_wallData = g_sysMap[y][x];
					/* 探索走行 */
	
						if (((uc_wallData & 0x01) == 0x00) && (y != (MAP_Y_SIZE - 1))) {
							if (us_cmap[y + 1][x] == MAP_SMAP_MAX_VAL - 1) {
								us_cmap[y + 1][x] = uc_new;
								uc_level++;
							}
						}
						if (((uc_wallData & 0x02) == 0x00) && (x != (MAP_X_SIZE - 1))) {
							if (us_cmap[y][x + 1] == MAP_SMAP_MAX_VAL - 1) {
								us_cmap[y][x + 1] = uc_new;
								uc_level++;
							}
						}
						if (((uc_wallData & 0x04) == 0x00) && (y != 0)) {
							if (us_cmap[y - 1][x] == MAP_SMAP_MAX_VAL - 1) {
								us_cmap[y - 1][x] = uc_new;
								uc_level++;
							}
						}
						if (((uc_wallData & 0x08) == 0x00) && (x != 0)) {
							if (us_cmap[y][x - 1] == MAP_SMAP_MAX_VAL - 1) {
								us_cmap[y][x - 1] = uc_new;
								uc_level++;
							}
						}

				}
			}
		}
		uc_dase = uc_dase + 1;
	} while (uc_level != 0);

#if 0
	/* debug */
	for (x = 0; x < 4; x++) {

		for (y = 0; y < 4; y++) {
			us_Log[y][x][us_LogPt] = us_cmap[y][x];
		}
	}
	us_LogPt++;
#endif

}

//TKR
// *************************************************************************
//   機能		： 進む区画方向が探索済みか未探索かを判定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： TRUE:探索済み	FALSE:未探索
// **************************    履    歴    *******************************
// 		v1.0		2014.09.29			TKR			新規
// *************************************************************************/
PRIVATE BOOL MAP_KnownAcc(void) {

	BOOL	bl_acc = FALSE;
#if 0
	if ((g_sysMap[my][mx] & 0xf0) == 0xf0) {
		bl_acc = TRUE;
	}
	else {
		bl_acc = FALSE;
	}
#endif
#if 1
	switch (en_Head) {
	case NORTH:
		if ((g_sysMap[my + 1][mx] & 0xf1) == 0xf0) {
			bl_acc = TRUE;
		}

		break;

	case EAST:
		if ((g_sysMap[my][mx + 1] & 0xf2) == 0xf0) {
			bl_acc = TRUE;
		}
		break;

	case SOUTH:
		if ((g_sysMap[my - 1][mx] & 0xf4) == 0xf0) {
			bl_acc = TRUE;
		}
		break;

	case WEST:
		if ((g_sysMap[my][mx - 1] & 0xf8) == 0xf0) {
			bl_acc = TRUE;
		}
		break;

	default:
		break;
	}
#endif

	return	bl_acc;

}

// *************************************************************************
//   機能		： 次の区画に移動する（既知区間加速）
//   注意		： なし
//   メモ		： なし
//   引数		： 相対進行方向（マウス進行方向を北としている）、前進状態（FALSE: １区間前進状態、TURE:半区間前進状態）
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2014.09.30			外川			新規
// *************************************************************************/
PRIVATE void MAP_moveNextBlock_acc(enMAP_HEAD_DIR en_head, BOOL* p_type)
{
	*p_type = FALSE;
	f_MoveBackDist = 0;

	/* 動作 */
	switch (en_head) {

		/* そのまま前進 */
	case NORTH:
//		*p_type = FALSE;
//		LED = LED6;
		if (MAP_KnownAcc() == FALSE) {					// 次に進む区画が未探索のとき
			if (st_known.bl_Known == TRUE){
				if (st_known.uc_StrCnt < 2) {
					MOT_goBlock_Const(1);					// 1区画の場合は等速のまま
				}
				else {
					MOT_setTrgtSpeed(MAP_KNOWN_ACC_SPEED);									// 既知区間加速するときの目標速度	
					MOT_goBlock_FinSpeed((FLOAT)(st_known.uc_StrCnt), SEARCH_SPEED);				// n区画前進
					MOT_setTrgtSpeed(SEARCH_SPEED);										// 目標速度をデフォルト値に戻す
				}
			}
			MOT_goBlock_Const(1);	////////////////////
			st_known.uc_StrCnt = 0;
			st_known.bl_Known = FALSE;

		}
		else {

			st_known.uc_StrCnt++;			// 移動区画の加算
			st_known.bl_Known = TRUE;
		}

		break;

		/* 右に旋回する */
	case EAST:
//		LED = LED8;
		if (st_known.bl_Known == TRUE) {		// 直線分を消化
			if (st_known.uc_StrCnt < 2) {
				MOT_goBlock_Const(1);					// 1区画の場合は等速のまま
			}
			else {
//				LED = LED_ALL_ON;
				MOT_setTrgtSpeed(MAP_KNOWN_ACC_SPEED);									// 既知区間加速するときの目標速度	
				MOT_goBlock_FinSpeed((FLOAT)(st_known.uc_StrCnt), SEARCH_SPEED);				// n区画前進
				MOT_setTrgtSpeed(SEARCH_SPEED);										// 目標速度をデフォルト値に戻す
//				LED = LED_ALL_OFF;
			}
			st_known.uc_StrCnt = 0;		/////////////////////////////////////////
			st_known.bl_Known = FALSE;
		}

		if( uc_SlaCnt < 7 ){
				MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// 右スラローム
				uc_SlaCnt++;
			}
			else{
				f_MoveBackDist = 0;
				/* 壁当て姿勢制御（左に壁があったらバック＋移動距離を加算する） */
				if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) )  ||		// 北を向いていて西に壁がある
					( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) )  ||		// 東を向いていて北に壁がある
					( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// 南を向いていて東に壁がある
					( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) ) 			// 西を向いていて南に壁がある
				){
					MOT_goBlock_FinSpeed( 0.5, 0 );			// 半区画前進
//					TIME_wait( MAP_TURN_WAIT );
					MOT_turn(MOT_R90);						// 右90度旋回
//					TIME_wait( MAP_TURN_WAIT );
					uc_SlaCnt = 0;
					MOT_goHitBackWall();					// バックする
					f_MoveBackDist = MOVE_BACK_DIST;		// バックした分の移動距離[区画]を加算
//					TIME_wait( MAP_SLA_WAIT );				// 時間待ち
					*p_type = TRUE;							// 次は半区間（＋バック）分進める
				}
				else{
					MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// 右スラローム
					uc_SlaCnt++;
				}
			}
		break;

		/* 左に旋回する */
	case WEST:
//		LED = LED1;
		if (st_known.bl_Known == TRUE) {		// 直線分を消化
			if (st_known.uc_StrCnt < 2) {
				MOT_goBlock_Const(1);					// 1区画の場合は等速のまま
			}
			else {
//				LED = LED_ALL_ON;
				MOT_setTrgtSpeed(MAP_KNOWN_ACC_SPEED);									// 既知区間加速するときの目標速度	
				MOT_goBlock_FinSpeed((FLOAT)(st_known.uc_StrCnt), SEARCH_SPEED);				// n区画前進
				MOT_setTrgtSpeed(SEARCH_SPEED);										// 目標速度をデフォルト値に戻す
//				LED = LED_ALL_OFF;
			}
			st_known.uc_StrCnt = 0;			//////////////////////////////////////
			st_known.bl_Known = FALSE;
		}

		if( uc_SlaCnt < 7 ){
				MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );	// 左スラローム
				uc_SlaCnt++;
			}
			else{
				f_MoveBackDist = 0;
				/* 壁当て姿勢制御（後ろに壁があったらバック＋移動距離を加算する） */
				if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// 北を向いていて東に壁がある
					( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) )  ||		// 東を向いていて南に壁がある
					( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) )  ||		// 南を向いていて西に壁がある
					( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) ) 			// 西を向いていて北に壁がある
				){
					MOT_goBlock_FinSpeed( 0.5, 0 );		// 半区画前進
//					TIME_wait( MAP_TURN_WAIT );
					MOT_turn(MOT_L90);					// 右90度旋回
//					TIME_wait( MAP_TURN_WAIT );
					uc_SlaCnt = 0;
					MOT_goHitBackWall();					// バックする
					f_MoveBackDist = MOVE_BACK_DIST;		// バックした分の移動距離[区画]を加算
//					TIME_wait( MAP_SLA_WAIT );				// 時間待ち
					*p_type = TRUE;							// 次は半区間（＋バック）分進める
				}
				else{
					MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );	// 左スラローム
					uc_SlaCnt++;
				}
			}
		break;

		/* 反転して戻る */
	case SOUTH:
//		LED = LED_ALL_ON;
		MOT_goBlock_FinSpeed(0.5, 0);			// 半区画前進
//		TIME_wait(MAP_SLA_WAIT);
		MOT_turn(MOT_R180);									// 右180度旋回
//		TIME_wait(MAP_SLA_WAIT);
		uc_SlaCnt = 0;

		/* 壁当て姿勢制御（後ろに壁があったらバック＋移動距離を加算する） */
		if (((en_Head == NORTH) && ((g_sysMap[my][mx] & 0x01) != 0)) ||		// 北を向いていて北に壁がある
			((en_Head == EAST) && ((g_sysMap[my][mx] & 0x02) != 0)) ||		// 東を向いていて東に壁がある
			((en_Head == SOUTH) && ((g_sysMap[my][mx] & 0x04) != 0)) ||		// 南を向いていて南に壁がある
			((en_Head == WEST) && ((g_sysMap[my][mx] & 0x08) != 0)) 			// 西を向いていて西に壁がある
			) {
			MOT_goHitBackWall();					// バックする
			f_MoveBackDist = MOVE_BACK_DIST;	// バックした分の移動距離[区画]を加算
//			TIME_wait(MAP_SLA_WAIT);				// 時間待ち
		}
		*p_type = TRUE;								// 次は半区間＋バック分進める
		break;

	default:
		break;
	}

#ifndef POWER_RELESASE
	/* 進行方向更新 */
//	en_Head = (enMAP_HEAD_DIR)( (en_Head + en_head) & (MAP_HEAD_DIR_MAX-1) );
	en_Head = (enMAP_HEAD_DIR)(((UCHAR)en_Head + (UCHAR)en_head) & (MAP_HEAD_DIR_MAX - 1));
#else
	/* 前進中にパワーリリース機能が働いてレジュームしなければならない */
	if ((TRUE == DCMC_isPowerRelease()) && (en_head == NORTH)) {

		MOT_goBack_Const(MOT_BACK_POLE);					// １つ前の柱まで後退
		MAP_makeMapData();									// 壁データから迷路データを作成			← ここでデータ作成をミスっている
		MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			// 等高線MAP法で進行方向を算出			← 誤ったMAPを作成
		MAP_moveNextBlock(en_head, p_type);					// もう１度呼び出し（次の区画へ移動）
	}
	else {
		/* 進行方向更新 */
		en_Head = (enMAP_HEAD_DIR)((en_Head + en_head) & (MAP_HEAD_DIR_MAX - 1));
	}
#endif
}


// *************************************************************************
//   機能		： ゴールの探索動作
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.27			sato		新規
// *************************************************************************/
PUBLIC void MAP_searchGoal(
	UCHAR 			uc_trgX, 		///< [in] 目標x座標
	UCHAR 			uc_trgY, 		///< [in] 目標y座標 
	enMAP_ACT_MODE 	en_type, 		///< [in] 探索方法
	enSEARCH_MODE	en_search 		///< [in] 探索方法
){
	enMAP_HEAD_DIR	en_head = NORTH;
	BOOL		bl_type = TRUE;			// 現在位置、FALSE: １区間前進状態、TURE:半区間前進状態
	enMAP_HEAD_DIR		en_endDir;
	
	UCHAR uc_goalX;
	UCHAR uc_goalY;
	UCHAR uc_staX;
	UCHAR uc_staY;
	
	search_flag = TRUE;

	if (en_search == SEARCH_RETURN){
		uc_goalX = uc_trgX;
		uc_goalY = uc_trgY;
		uc_staX = mx;
		uc_staY = my;
//		printf("mx%d,my%d\n", mx, my);
		MAP_makeContourMap(uc_trgX, uc_trgY, en_type);
		MAP_searchCmdList(uc_staX, uc_staY, en_Head, uc_goalX, uc_goalX, &en_endDir);
		uc_trgX = Return_X;
		uc_trgY = Return_Y;
//		printf("goalx%d,goaly%d\n", Return_X, Return_Y);
//		MAP_showcountLog();
	}

//	SYS_setDisable( SYS_MODE );				// モード変更禁止

	MOT_setTrgtSpeed(SEARCH_SPEED);		// 目標速度
	MOT_setNowSpeed( 0.0f );
	f_MoveBackDist = 0;
	uc_SlaCnt = 0;
	if(uc_trgX == GOAL_MAP_X && uc_trgY == GOAL_MAP_Y){
		f_MoveBackDist = MOVE_BACK_DIST;
	}
	
	log_flag_on();	//ログ関数スタート（大会時削除）
	
	/* 迷路探索 */
	while(1){
		MAP_refMousePos( en_Head );								// 座標更新
//		MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// 等高線マップを作る
		
		/* 超信地旋回探索 */
		if( SEARCH_TURN == en_search ){
			MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// 等高線マップを作る
			if( TRUE == bl_type ){
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, SEARCH_SPEED );		// 半区画前進(バックの移動量を含む)
			}
			MAP_makeMapData();												// 壁データから迷路データを作成			← ここでデータ作成をミスっている
			MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);						// 等高線MAP法で進行方向を算出			← 誤ったMAPを作成
			
			/* 次の区画へ移動 */
			if(( mx == uc_trgX ) && ( my == uc_trgY )){
				MAP_actGoal();										// ゴール時の動作
				break;
			}
			else{
				MAP_moveNextBlock(en_head, &bl_type);				// 次の区画へ移動								← ここで改めてリリースチェック＋壁再度作成＋等高線＋超信地旋回動作
			}
		}
		/* スラローム探索 */
		else if( SEARCH_SURA == en_search ){
			MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// 等高線マップを作る
			if( TRUE == bl_type ){
				
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, SEARCH_SPEED );		// 半区画前進(バックの移動量を含む)
			}
			MAP_makeMapData();		// 壁データから迷路データを作成
			
			MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);				// 等高線MAP法で進行方向を算出			← 誤ったMAPを作成
			
			/* 次の区画へ移動 */
			if(( mx == uc_trgX ) && ( my == uc_trgY )){
				MAP_actGoal();										// ゴール時の動作
				break;
			}
			else{
				MAP_moveNextBlock_Sura(en_head, &bl_type, FALSE );	// 次の区画へ移動						← ここで改めてリリースチェック＋壁再度作成＋等高線＋超信地旋回動作
//				MAP_moveNextBlock_acc(en_head, &bl_type);
			}
		}
		/* 帰還探索 */
		else if (SEARCH_RETURN == en_search) {
			
			if( TRUE == bl_type ){
				
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, SEARCH_SPEED );		// 半区画前進(バックの移動量を含む)
			}
			MAP_makeMapData();		// 壁データから迷路データを作成
			MAP_makeReturnContourMap(uc_staX,uc_staY);
			MAP_searchCmdList(uc_staX, uc_staY, en_Head, uc_goalX, uc_goalX, &en_endDir);
			uc_trgX = Return_X;
			uc_trgY = Return_Y;
			MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// 等高線マップを作る
			MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			
			
			/* 次の区画へ移動 */
//			if ((us_cmap[my][mx] == 0)||((g_sysMap[uc_trgY][uc_trgX]&0xf0) == 0xf0)) {
			if ((mx == 0)&&(my == 0)){
				MAP_actGoal();
				break;
			}
//			}
			else {
				MAP_moveNextBlock_Sura(en_head, &bl_type, FALSE);	// 次の区画へ移動						← ここで改めてリリースチェック＋壁再度作成＋等高線＋超信地旋回動作
			}
//			LED_count(uc_trgY);
		}

		
		/* 途中で制御不能になった */
		if( SYS_isOutOfCtrl() == TRUE ){
			CTRL_stop();
			DCM_brakeMot( DCM_R );		// ブレーキ
			DCM_brakeMot( DCM_L );		// ブレーキ
			
			/* 迷路関連を初期化 */
			en_Head		= NORTH;
			mx			= 0;
			my			= 0;
			f_MoveBackDist = 0;
			
			// DCMCは下位モジュールで既にクリアと緊急停止を行っている。
			break;
		}
	}
	search_flag = FALSE;
	TIME_wait(1000);
//	SYS_setEnable( SYS_MODE );				// モード変更有効

}

// *************************************************************************
//   機能		： 探索（既知区間加速）
//   注意		： なし
//   メモ		： なし
//   引数		： 目標x座標、目標y座標、探索方法
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.11.3			TKR			searchGoal関数から移植
// *************************************************************************/
PUBLIC void MAP_searchGoalKnown(
	UCHAR 			uc_trgX, 		///< [in] 目標x座標
	UCHAR 			uc_trgY, 		///< [in] 目標y座標 
	enMAP_ACT_MODE 	en_type, 		///< [in] 探索方法
	enSEARCH_MODE	en_search 		///< [in] 探索方法
){
	enMAP_HEAD_DIR	en_head = NORTH;
	BOOL		bl_type = TRUE;			// 現在位置、FALSE: １区間前進状態、TURE:半区間前進状態
	enMAP_HEAD_DIR		en_endDir;
	
	UCHAR uc_goalX;
	UCHAR uc_goalY;
	UCHAR uc_staX;
	UCHAR uc_staY;
	
	search_flag = TRUE;

	if (en_search == SEARCH_RETURN){
		uc_goalX = uc_trgX;
		uc_goalY = uc_trgY;
		uc_staX = mx;
		uc_staY = my;
//		printf("mx%d,my%d\n", mx, my);
		MAP_makeContourMap(uc_trgX, uc_trgY, en_type);
		MAP_searchCmdList(uc_staX, uc_staY, en_Head, uc_goalX, uc_goalX, &en_endDir);
		uc_trgX = Return_X;
		uc_trgY = Return_Y;
//		printf("goalx%d,goaly%d\n", Return_X, Return_Y);
//		MAP_showcountLog();
	}

//	SYS_setDisable( SYS_MODE );				// モード変更禁止

	MOT_setTrgtSpeed(SEARCH_SPEED);		// 目標速度
	MOT_setNowSpeed( 0.0f );
	f_MoveBackDist = 0;
	uc_SlaCnt = 0;
	if(uc_trgX == GOAL_MAP_X && uc_trgY == GOAL_MAP_Y){
		f_MoveBackDist = MOVE_BACK_DIST;
	}
	
	log_flag_on();	//ログ関数スタート（大会時削除）
	
	/* 迷路探索 */
	while(1){
		MAP_refMousePos( en_Head );								// 座標更新
//		MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// 等高線マップを作る
		
		/* 超信地旋回探索 */
		if( SEARCH_TURN == en_search ){
			MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// 等高線マップを作る
			if( TRUE == bl_type ){
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, SEARCH_SPEED );		// 半区画前進(バックの移動量を含む)
			}
			MAP_makeMapData();												// 壁データから迷路データを作成			← ここでデータ作成をミスっている
			MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);						// 等高線MAP法で進行方向を算出			← 誤ったMAPを作成
			
			/* 次の区画へ移動 */
			if(( mx == uc_trgX ) && ( my == uc_trgY )){
				MAP_actGoal();										// ゴール時の動作
				break;
			}
			else{
				MAP_moveNextBlock(en_head, &bl_type);				// 次の区画へ移動								← ここで改めてリリースチェック＋壁再度作成＋等高線＋超信地旋回動作
			}
		}
		/* スラローム探索 */
		else if( SEARCH_SURA == en_search ){
			MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// 等高線マップを作る
			if( TRUE == bl_type ){
				
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, SEARCH_SPEED );		// 半区画前進(バックの移動量を含む)
			}
			if (st_known.bl_Known != TRUE) {
				MAP_makeMapData();		// 壁データから迷路データを作成
			}
			MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);				// 等高線MAP法で進行方向を算出			← 誤ったMAPを作成
			
			/* 次の区画へ移動 */
			if(( mx == uc_trgX ) && ( my == uc_trgY )){
				MAP_actGoal();										// ゴール時の動作
				break;
			}
			else{
//				MAP_moveNextBlock_Sura(en_head, &bl_type, FALSE );	// 次の区画へ移動						← ここで改めてリリースチェック＋壁再度作成＋等高線＋超信地旋回動作
				MAP_moveNextBlock_acc(en_head, &bl_type);
			}
		}
		/* 帰還探索 */
		else if (SEARCH_RETURN == en_search) {
			
			if( TRUE == bl_type ){
				
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, SEARCH_SPEED );		// 半区画前進(バックの移動量を含む)
			}
			if (st_known.bl_Known != TRUE) {
				MAP_makeMapData();		// 壁データから迷路データを作成
			}
			MAP_makeReturnContourMap(uc_staX,uc_staY);
			MAP_searchCmdList(uc_staX, uc_staY, en_Head, uc_goalX, uc_goalX, &en_endDir);
			uc_trgX = Return_X;
			uc_trgY = Return_Y;
			MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// 等高線マップを作る
			MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			
			
			/* 次の区画へ移動 */
//			if ((us_cmap[my][mx] == 0)||((g_sysMap[uc_trgY][uc_trgX]&0xf0) == 0xf0)) {
			if ((mx == 0)&&(my == 0)){
				MAP_actGoal();
				break;
			}
//			}
			else {
				MAP_moveNextBlock_acc(en_head, &bl_type);
			}
//			LED_count(uc_trgY);
		}

		
		/* 途中で制御不能になった */
		if( SYS_isOutOfCtrl() == TRUE ){
			CTRL_stop();
			DCM_brakeMot( DCM_R );		// ブレーキ
			DCM_brakeMot( DCM_L );		// ブレーキ
			
			/* 迷路関連を初期化 */
			en_Head		= NORTH;
			mx			= 0;
			my			= 0;
			f_MoveBackDist = 0;
			
			// DCMCは下位モジュールで既にクリアと緊急停止を行っている。
			break;
		}
	}
	search_flag = FALSE;
	TIME_wait(1000);
//	SYS_setEnable( SYS_MODE );				// モード変更有効

}
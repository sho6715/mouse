// *************************************************************************
//   ロボット名		： AEGIS
//   概要		： hal_distファイル
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.15			sato			新規（ファイルのインクルード）
// *************************************************************************/

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <hal.h>							// HAL
#include <stdio.h>
#include <math.h>
#include <init.h>
#include <parameters.h>
#include <hal_dist.h>
//#include <search.h>

//**************************************************
// 定義（define）
//**************************************************
#define DIST_NO_WALL_DIV_FILTER				( 200 )						// 壁なしとする差分値
#define DIST_REF_UP					( 50 )						// 壁なしと判断する際に基準値に加算する値
/*
#define R_FRONT_REF					( 350 )					// 右前壁、基準値　区画の中央において壁を見た値275
#define L_FRONT_REF					( 280 )					// 左前壁、基準値210
#define R_SIDE_REF					( 280 )						// 右横壁、基準値230
#define L_SIDE_REF					( 240 )						// 左横壁、基準値180
#define R_FRONT_WALL					( 70 )						// 右前壁、壁検知値　区画の中央からずらして見たときの値（壁を見るための値）45
#define L_FRONT_WALL					( 50 )					// 左前壁、壁検知値40
#define R_SIDE_WALL					( 115 )						// 右横壁、壁検知値100
#define L_SIDE_WALL					( 70 )					// 左横壁、壁検知値80
*/
#define R_FRONT_REF					( 750 )					// 右前壁、基準値　区画の中央において壁を見た値250
#define L_FRONT_REF					( 620 )					// 左前壁、基準値210
#define R_SIDE_REF					( 550 )						// 右横壁、基準値290
#define L_SIDE_REF					( 520 )						// 左横壁、基準値250
#define R_FRONT_WALL					( 230 )						// 右前壁、壁検知値　区画の中央からずらして見たときの値（壁を見るための値）55
#define L_FRONT_WALL					( 230 )					// 左前壁、壁検知値55
#define R_SIDE_WALL					( 450 )						// 右横壁、壁検知値140
#define L_SIDE_WALL					( 430 )					// 左横壁、壁検知値100
#define R_FRONT_SKEW_ERR1				( 80 )//仮
#define L_FRONT_SKEW_ERR1				( 70 )
#define R_FRONT_SKEW_ERR2				( 192 )
#define L_FRONT_SKEW_ERR2				( 160 )
#define R_FRONT_SKEW_ERR3				( 250 )
#define L_FRONT_SKEW_ERR3				( 250 )
#define R_FRONT_CTRL					( 800 )
#define L_FRONT_CTRL					( 680 )
#define R_FRONT_NOCTRL					( 850 )
#define L_FRONT_NOCTRL					( 770 )


#define		SEN_WAIT_CNT		(175)		//センサの発光安定待ち(実験値)
#define		DIST_REF_NUM		(400)

/* 距離センサ(環境変化) */
#define R_FRONT_WALL_GAIN			( 0.13f )							// 右前壁ゲイン（基準値に対して）、壁検知値
#define L_FRONT_WALL_GAIN			( 0.16f )							// 左前壁ゲイン（基準値に対して）、壁検知値
#define R_SIDE_WALL_GAIN			( 0.43f )							// 右横壁ゲイン（基準値に対して）、壁検知値
#define L_SIDE_WALL_GAIN			( 0.60f )							// 左横壁ゲイン（基準値に対して）、壁検知値
#define R_FRONT_WALL_CTRL_GAIN		( 0.74f )							// 右前壁ゲイン（基準値に対して）、これ以上近いと制御する値
#define L_FRONT_WALL_CTRL_GAIN		( 0.82f )							// 左前壁ゲイン（基準値に対して）、これ以上近いと制御する値
#define R_FRONT_WALL_NO_CTRL_GAIN	( 2.13f )							// 右前壁ゲイン（基準値に対して）、これ以上近いと制御しない値
#define L_FRONT_WALL_NO_CTRL_GAIN	( 1.95f )							// 左前壁ゲイン（基準値に対して）、これ以上近いと制御しない値
//#define R_FRONT_WALL_HIT_GAIN		( 2.73f )							// 右前壁ゲイン（基準値に対して）、壁に当たっていてもおかしくない値（前壁とマウス間が約2mmの時の値）
//#define L_FRONT_WALL_HIT_GAIN		( 3.41f )							// 左前壁ゲイン（基準値に対して）、壁に当たっていてもおかしくない値（前壁とマウス間が約2mmの時の値）
#define R_FRONT_SKEW_ERR1_GAIN		( 0.29f )							// 右前壁、斜め走行時の補正閾値1
#define R_FRONT_SKEW_ERR2_GAIN		( 0.7f )							// 右前壁、斜め走行時の補正閾値2
#define R_FRONT_SKEW_ERR3_GAIN		( 1.23f )							// 右前壁、斜め走行時の補正閾値3
#define L_FRONT_SKEW_ERR1_GAIN		( 0.28f )							// 左前壁、斜め走行時の補正閾値1
#define L_FRONT_SKEW_ERR2_GAIN		( 0.7f )							// 左前壁、斜め走行時の補正閾値2
#define L_FRONT_SKEW_ERR3_GAIN		( 1.23f )							// 左前壁、斜め走行時の補正閾値3
#define R_FRONT_CTRL_GAIN		( 1.5f )
#define L_FRONT_CTRL_GAIN		( 1.5f ) 
#define R_FRONT_NOCTRL_GAIN		( 0.5f )
#define L_FRONT_NOCTRL_GAIN		( 0.5f )

//**************************************************
// 列挙体（enum）
//**************************************************


//**************************************************
// 構造体（struct）
//**************************************************
/* 距離センサ情報（前壁のみ、データフラッシュ用構造体としても使用する） */
typedef struct{
	SHORT		s_wallHit;					///< @var : 壁に当たっていてもおかしくない値         ( AD 値 ) （前壁とマウス間が約2mmの時の値）					///< @var : 斜め走行時の補正閾値3                    ( AD 値 )
	SHORT		s_skewErr1;					///< @var : 斜め走行時の補正閾値1                    ( AD 値 )
	SHORT		s_skewErr2;					///< @var : 斜め走行時の補正閾値2                    ( AD 値 )
	SHORT		s_skewErr3;					///< @var : 斜め走行時の補正閾値3
}stDIST_FRONT_SEN;


/* 距離センサ情報（全センサ共通、データフラッシュ用構造体のみに使用） */
typedef struct{
	SHORT		s_ref;						///< @var : 区画の中心に置いた時の距離センサの基準値 ( AD 値 )
	SHORT		s_limit;					///< @var : 距離センサの閾値                         ( AD 値 ) ( この値より大きい場合、壁ありと判断する )
	SHORT		s_ctrl;						///< @var : 制御有効化する際の閾値                   ( AD 値 ) 主に前壁で使用
	SHORT		s_noCtrl;					///< @var : 壁に近すぎるため制御無効化する際の閾値   ( AD 値 ) 主に前壁で使用
}stDIST_SEN_DATA;

/* 距離センサ情報（全センサ共通） 注　hal.hに移行*/
typedef struct{
	SHORT		s_now;						// LED 点灯中の距離センサの現在値           ( AD 値 )
	SHORT		s_old;						// LED 点灯中の距離センサの1つ前の値        ( AD 値 )
	SHORT		s_limit;					// 距離センサの閾値                         ( AD 値 ) ( この値より大きい場合、壁ありと判断する )
	SHORT		s_ref;						// 区画の中心に置いた時の距離センサの基準値 ( AD 値 )
	SHORT		s_offset;					// LED 消灯中の距離センサの値               ( AD 値 )
	SHORT		s_ctrl;						// 制御有効化する際の閾値                   ( AD 値 ) 主に前壁で使用
	SHORT		s_noCtrl;					// 壁に近すぎるため制御無効化する際の閾値   ( AD 値 ) 主に前壁で使用
}stDIST_SEN;

//**************************************************
// グローバル変数
//**************************************************
/* 距離センサ */
PRIVATE stDIST_SEN		st_sen[DIST_SEN_NUM];					// 距離センサ
PRIVATE stDIST_FRONT_SEN	st_senF[DIST_SEN_L_FRONT+1];	// 距離センサ(前壁のみ)
PRIVATE LONG  l_Lside_DistRef; 									// 
PRIVATE LONG  l_Rside_DistRef; 
PRIVATE LONG  l_Lfront_DistRef;
PRIVATE LONG  l_Rfront_DistRef; 

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************


// *************************************************************************
//   機能		： HAL_distを初期化する。
//   注意		： なし
//   メモ		： 内部変数などをクリアする。
//   引数		： なし
//   返り値		： なし
//   その他     ：起動時に動作
// **************************    履    歴    *******************************
// 		v1.0		2018.09.15			sato			新規
// *************************************************************************/
PUBLIC void DIST_init( void )
{
	/* 距離センサ */
	l_Lside_DistRef = 0; 									// ジャイロセンサの基準値
	l_Rside_DistRef = 0; 
	l_Lfront_DistRef = 0;
	l_Rfront_DistRef = 0;

	memset( st_sen, 0, sizeof(st_sen) );				// 距離センサ(全センサ共通)
	memset( st_senF, 0, sizeof(st_senF) );				// 距離センサ(前センサのみ)
	st_sen[DIST_SEN_R_FRONT].s_ref       = R_FRONT_REF;
	st_sen[DIST_SEN_L_FRONT].s_ref       = L_FRONT_REF;
	st_sen[DIST_SEN_R_SIDE].s_ref        = R_SIDE_REF;
	st_sen[DIST_SEN_L_SIDE].s_ref        = L_SIDE_REF;
	st_sen[DIST_SEN_R_FRONT].s_limit     = R_FRONT_WALL;
	st_sen[DIST_SEN_L_FRONT].s_limit     = L_FRONT_WALL;
	st_sen[DIST_SEN_R_SIDE].s_limit      = R_SIDE_WALL;
	st_sen[DIST_SEN_L_SIDE].s_limit      = L_SIDE_WALL;
	st_senF[DIST_SEN_R_FRONT].s_skewErr1	= R_FRONT_SKEW_ERR1;
	st_senF[DIST_SEN_L_FRONT].s_skewErr1	= L_FRONT_SKEW_ERR1;
	st_senF[DIST_SEN_R_FRONT].s_skewErr2	= R_FRONT_SKEW_ERR2;
	st_senF[DIST_SEN_L_FRONT].s_skewErr2	= L_FRONT_SKEW_ERR2;
	st_senF[DIST_SEN_R_FRONT].s_skewErr3	= R_FRONT_SKEW_ERR3;
	st_senF[DIST_SEN_L_FRONT].s_skewErr3	= L_FRONT_SKEW_ERR3;
	st_sen[DIST_SEN_R_FRONT].s_noCtrl = R_FRONT_NOCTRL;//実験値
	st_sen[DIST_SEN_L_FRONT].s_noCtrl = L_FRONT_NOCTRL;
	st_sen[DIST_SEN_R_FRONT].s_ctrl = R_FRONT_CTRL;
	st_sen[DIST_SEN_L_FRONT].s_ctrl = L_FRONT_CTRL;

}

// *************************************************************************
//   機能		： 距離センサの値を取得する
//   注意		： なし
//   メモ		： ☆
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.02			外川			新規
// *************************************************************************/
PUBLIC SHORT DIST_getNowVal( enDIST_SEN_ID en_id )
{
	return st_sen[en_id].s_now;
}


// *************************************************************************
//   機能		： 基準値と現在値の偏差を取得する
//   注意		： なし
//   メモ		： 壁の切れ目補正対応実施
//   引数		： なし
//   返り値		： 偏差
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void DIST_getErr( LONG* p_err )
{
	VSHORT	s_threshold_R = 0;		// 右センサの閾値
	VSHORT	s_threshold_L = 0;		// 左センサの閾値
	SHORT	s_temp;
/*	
	st_sen[DIST_SEN_R_FRONT].s_noCtrl = 340;//実験値
	st_sen[DIST_SEN_L_FRONT].s_noCtrl = 260;
	st_sen[DIST_SEN_R_FRONT].s_ctrl = 170;
	st_sen[DIST_SEN_L_FRONT].s_ctrl = 130;
*/	
	/* ---------- */
	/*  右壁制御  */
	/* ---------- */
	/* 壁の切れ目対策 */
	// 急激にセンサの値が変化した場合は、壁の有無の基準値を閾値に変更する
	s_temp = st_sen[DIST_SEN_R_SIDE].s_now - st_sen[DIST_SEN_R_SIDE].s_old;
	if( ( s_temp < -1 * DIST_NO_WALL_DIV_FILTER ) || ( DIST_NO_WALL_DIV_FILTER < s_temp )
	){
		s_threshold_R = st_sen[DIST_SEN_R_SIDE].s_ref + DIST_REF_UP;		// 基準値＋αを壁の存在する閾値にする
	}
	else{
		s_threshold_R = st_sen[DIST_SEN_R_SIDE].s_limit;		// 通常通り
	}

	/* ---------- */
	/*  左壁制御  */
	/* ---------- */
	/* 壁の切れ目対策 */
	// 急激にセンサの値が変化した場合は、壁の有無の基準値を閾値に変更する
	s_temp = st_sen[DIST_SEN_L_SIDE].s_now - st_sen[DIST_SEN_L_SIDE].s_old;
	if( ( s_temp < -1 * DIST_NO_WALL_DIV_FILTER ) || ( DIST_NO_WALL_DIV_FILTER < s_temp )
	){
		s_threshold_L = st_sen[DIST_SEN_L_SIDE].s_ref + DIST_REF_UP;		// 基準値＋αを壁の存在する閾値にする
	}
	else{
		s_threshold_L = st_sen[DIST_SEN_L_SIDE].s_limit;		// 通常通り
	}

	/* ------------ */
	/*  制御値算出  */
	/* ------------ */
	*p_err = 0;		// クリア
	
	/* 前壁がものすごく近い時 */
	if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_sen[DIST_SEN_R_FRONT].s_noCtrl ) &&
		( st_sen[DIST_SEN_L_FRONT].s_now > st_sen[DIST_SEN_L_FRONT].s_noCtrl )
	){
//		printf("[Val]%6d 前壁がものすごい近い 	\n\r", *p_err);
		*p_err = 0;
	}
	/* 前壁 */
	else if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_sen[DIST_SEN_R_FRONT].s_ctrl ) &&
		( st_sen[DIST_SEN_L_FRONT].s_now > st_sen[DIST_SEN_L_FRONT].s_ctrl )
	){
		*p_err = ( st_sen[DIST_SEN_L_FRONT].s_now - st_sen[DIST_SEN_L_FRONT].s_ref ) -
				 ( st_sen[DIST_SEN_R_FRONT].s_now - st_sen[DIST_SEN_R_FRONT].s_ref );
//		printf("[Val]%6d 前壁制御 	\n\r", *p_err);
	}
	/* 右壁と左壁あり */
	else if( ( s_threshold_R < st_sen[DIST_SEN_R_SIDE].s_now ) && ( s_threshold_L < st_sen[DIST_SEN_L_SIDE].s_now )
	){
		*p_err = ( st_sen[DIST_SEN_R_SIDE].s_now - st_sen[DIST_SEN_R_SIDE].s_ref ) + 
				 ( st_sen[DIST_SEN_L_SIDE].s_ref - st_sen[DIST_SEN_L_SIDE].s_now );
//		printf("[Val]%6d 両壁制御 	\n\r", *p_err);
//		LED4 = 0x09;
	}
	/* 右壁あり */
	else if( s_threshold_R < st_sen[DIST_SEN_R_SIDE].s_now ){
		*p_err = ( st_sen[DIST_SEN_R_SIDE].s_now - st_sen[DIST_SEN_R_SIDE].s_ref ) * 2;
//		printf("[Val]%6d 右壁制御 	\n\r", *p_err);
//		LED4 = 0x08;
	}
	/* 左壁あり */
	else if( s_threshold_L < st_sen[DIST_SEN_L_SIDE].s_now ){
		*p_err = ( st_sen[DIST_SEN_L_SIDE].s_ref - st_sen[DIST_SEN_L_SIDE].s_now ) * 2;
//		printf("[Val]%6d 左壁制御 	\n\r", *p_err);
//		LED4 = 0x01;
	}
}

// *************************************************************************
//   機能		： 基準値からの偏差状態を取得する
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.10.15			佐藤			新規
// *************************************************************************/
PUBLIC void DIST_getErrSkew( LONG* p_err )
{
	*p_err =0;
	
	/* 進行方向に壁が存在する場合によける動作を行う */
	if( st_sen[DIST_SEN_R_FRONT].s_now > st_senF[DIST_SEN_R_FRONT].s_skewErr3 ){
		*p_err = 150;
//		LED_on(LED0);
//		LED_off(LED1);
		
//		printf("右前が超近い  [NOW]%d > [ERR3]%d", st_sen[DIST_SEN_R_FRONT].s_now, st_senF[DIST_SEN_R_FRONT].s_skewErr3 );
	}
	else if( st_sen[DIST_SEN_L_FRONT].s_now > st_senF[DIST_SEN_L_FRONT].s_skewErr3 ){
		*p_err = -150;
//		LED_on(LED1);
//		LED_off(LED0);
//		printf("左前が超近い  [NOW]%d > [ERR3]%d", st_sen[DIST_SEN_L_FRONT].s_now, st_senF[DIST_SEN_L_FRONT].s_skewErr3 );
	}
	else if( st_sen[DIST_SEN_R_FRONT].s_now > st_senF[DIST_SEN_R_FRONT].s_skewErr2 ){
		*p_err = 100;
//		LED_on(LED0);
//		LED_off(LED1);
//		printf("右前が多少近い  [NOW]%d > [ERR2]%d", st_sen[DIST_SEN_R_FRONT].s_now, st_senF[DIST_SEN_R_FRONT].s_skewErr2 );
	}
	else if( st_sen[DIST_SEN_L_FRONT].s_now > st_senF[DIST_SEN_L_FRONT].s_skewErr2 ){
		*p_err = -100;
//		LED_on(LED1);
//		LED_off(LED0);
//		printf("左前が多少近い  [NOW]%d > [ERR2]%d", st_sen[DIST_SEN_L_FRONT].s_now, st_senF[DIST_SEN_L_FRONT].s_skewErr2 );
	}
	else if( st_sen[DIST_SEN_R_FRONT].s_now > st_senF[DIST_SEN_R_FRONT].s_skewErr1 ){
		*p_err = 50;
//		LED_on(LED0);
//		LED_off(LED1);
//		printf("右前が近い  [NOW]%d > [ERR1]%d", st_sen[DIST_SEN_R_FRONT].s_now, st_senF[DIST_SEN_R_FRONT].s_skewErr1 );
	}
	else if( st_sen[DIST_SEN_L_FRONT].s_now > st_senF[DIST_SEN_L_FRONT].s_skewErr1 ){
		*p_err = -50;
//		LED_on(LED1);
//		LED_off(LED0);
//		printf("左前が近い  [NOW]%d > [ERR1]%d", st_sen[DIST_SEN_L_FRONT].s_now, st_senF[DIST_SEN_L_FRONT].s_skewErr1 );
	}
	else{
//		LED_off(LED0);
//		LED_off(LED1);
	}
	
}

// *************************************************************************
//   機能		： 距離センサ用（前壁）ポーリング関数
//   注意		： なし
//   メモ		： 割り込みから実行される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.02			外川			新規
// *************************************************************************/
PUBLIC void DIST_Pol_Front( void )
{
	/* 無発光時の値取得 */
	S12AD.ADANS0.WORD 		= 0x0012;		// AN1/4 変換対象設定
	S12AD.ADCSR.BIT.ADST 		= 1;		// AD変換開始
	while( S12AD.ADCSR.BIT.ADST == 1);		// AD変換待ち
	st_sen[DIST_SEN_R_FRONT].s_offset = (SHORT)S12AD.ADDR4;
	st_sen[DIST_SEN_L_FRONT].s_offset = (SHORT)S12AD.ADDR1;

	/* 前壁LED点灯 */
	LED_DIST_RF = ON;
	LED_DIST_LF = ON;
	
	/* 発光安定待ち */
	TIME_waitFree( SEN_WAIT_CNT );

	/* 発光時の値と無発光時の値で差分を取得 */
	S12AD.ADANS0.WORD 		= 0x0012;		// AN1/2 変換対象設定
	S12AD.ADCSR.BIT.ADST 		= 1;		// AD変換開始
	while( S12AD.ADCSR.BIT.ADST == 1);		// AD変換待ち
	st_sen[DIST_SEN_R_FRONT].s_old = st_sen[DIST_SEN_R_FRONT].s_now;		// バッファリング
	st_sen[DIST_SEN_L_FRONT].s_old = st_sen[DIST_SEN_L_FRONT].s_now;		// バッファリング
	st_sen[DIST_SEN_R_FRONT].s_now = (SHORT)S12AD.ADDR4 - st_sen[DIST_SEN_R_FRONT].s_offset;		// 現在値書き換え
	st_sen[DIST_SEN_L_FRONT].s_now = (SHORT)S12AD.ADDR1 - st_sen[DIST_SEN_L_FRONT].s_offset;		// 現在値書き換え
	
	/* 前壁LED消灯 */
	LED_DIST_RF = OFF;
	LED_DIST_LF = OFF;
}


// *************************************************************************
//   機能		： 距離センサ用（横壁）ポーリング関数
//   注意		： なし
//   メモ		： 割り込みから実行される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.02			外川			新規
// *************************************************************************/
PUBLIC void DIST_Pol_Side( void )
{
	/* 無発光時の値取得 */
	S12AD.ADANS0.WORD 		= 0x000C;		// AN2/3 変換対象設定
	S12AD.ADCSR.BIT.ADST 		= 1;		// AD変換開始
	while( S12AD.ADCSR.BIT.ADST == 1);		// AD変換待ち
	st_sen[DIST_SEN_R_SIDE].s_offset = (SHORT)S12AD.ADDR3;
	st_sen[DIST_SEN_L_SIDE].s_offset = (SHORT)S12AD.ADDR2;

	/* 横壁LED点灯 */
	LED_DIST_RS = ON;
	LED_DIST_LS = ON;
	
	/* 発光安定待ち */
	TIME_waitFree( SEN_WAIT_CNT );

	/* 発光時の値と無発光時の値で差分を取得 */
	S12AD.ADANS0.WORD 		= 0x000C;		// AN0/3 変換対象設定
	S12AD.ADCSR.BIT.ADST 		= 1;		// AD変換開始
	while( S12AD.ADCSR.BIT.ADST == 1);		// AD変換待ち
	st_sen[DIST_SEN_R_SIDE].s_old = st_sen[DIST_SEN_R_SIDE].s_now;		// バッファリング
	st_sen[DIST_SEN_L_SIDE].s_old = st_sen[DIST_SEN_L_SIDE].s_now;		// バッファリング
	st_sen[DIST_SEN_R_SIDE].s_now = (SHORT)S12AD.ADDR3 - st_sen[DIST_SEN_R_SIDE].s_offset;		// 現在値書き換え
	st_sen[DIST_SEN_L_SIDE].s_now = (SHORT)S12AD.ADDR2 - st_sen[DIST_SEN_L_SIDE].s_offset;		// 現在値書き換え
	
	/* 横壁LED消灯 */
	LED_DIST_RS = OFF;
	LED_DIST_LS = OFF;
}

// *************************************************************************
//   機能		： 前壁の有無を判断する
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.02			外川			新規
// *************************************************************************/
PUBLIC BOOL DIST_isWall_FRONT( void )
{
	BOOL bl_res 		= FALSE;
//	printf("DIST_SEN_R_FRONT %5d \r\n",st_sen[DIST_SEN_R_FRONT].s_limit);
	if( ( st_sen[DIST_SEN_R_FRONT].s_now > st_sen[DIST_SEN_R_FRONT].s_limit ) ||
		( st_sen[DIST_SEN_L_FRONT].s_now > st_sen[DIST_SEN_L_FRONT].s_limit )
	){
		bl_res = TRUE;
	}
	
	return bl_res;
}

// *************************************************************************
//   機能		： 右壁の有無を判断する
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.02			外川			新規
// *************************************************************************/
PUBLIC BOOL DIST_isWall_R_SIDE( void )
{
	BOOL bl_res 		= FALSE;
	
	if( st_sen[DIST_SEN_R_SIDE].s_now > st_sen[DIST_SEN_R_SIDE].s_limit ){
		bl_res = TRUE;
	}
	
	return bl_res;
}

// *************************************************************************
//   機能		： 左壁の有無を判断する
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.02			外川			新規
// *************************************************************************/
PUBLIC BOOL DIST_isWall_L_SIDE( void )
{
	BOOL bl_res 		= FALSE;
	
	if( st_sen[DIST_SEN_L_SIDE].s_now > st_sen[DIST_SEN_L_SIDE].s_limit ){
		bl_res = TRUE;
	}
	
	return bl_res;
}

// *************************************************************************
//   機能		： 横壁オートキャリブレーション
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.10.28			sato			新規
// *************************************************************************/
PRIVATE void Dist_side_auto_calibration (void)
{
	USHORT i;
	ULONG ul_Lref = 0;
	ULONG ul_Rref = 0;
	
	/* データサンプリング */
	for( i=0; i<DIST_REF_NUM; i++){			// 100回サンプリングした平均値を基準の値とする。
		ul_Lref += st_sen[DIST_SEN_L_SIDE].s_now;
		ul_Rref += st_sen[DIST_SEN_R_SIDE].s_now;
		TIME_wait(1);
	}
	
	/* 基準値算出（平均値） */
	l_Lside_DistRef = ul_Lref / DIST_REF_NUM ;		// 精度を100倍にする
	l_Rside_DistRef = ul_Rref / DIST_REF_NUM ;
	
	st_sen[DIST_SEN_R_SIDE].s_ref        = l_Rside_DistRef;
	st_sen[DIST_SEN_L_SIDE].s_ref        = l_Lside_DistRef;
	st_sen[DIST_SEN_R_SIDE].s_limit      = l_Rside_DistRef * R_SIDE_WALL_GAIN;
	st_sen[DIST_SEN_L_SIDE].s_limit      = l_Lside_DistRef * L_SIDE_WALL_GAIN;
	st_senF[DIST_SEN_R_FRONT].s_skewErr1	= l_Rside_DistRef * R_FRONT_SKEW_ERR1_GAIN;
	st_senF[DIST_SEN_L_FRONT].s_skewErr1	= l_Lside_DistRef * L_FRONT_SKEW_ERR1_GAIN;
	st_senF[DIST_SEN_R_FRONT].s_skewErr2	= l_Rside_DistRef * R_FRONT_SKEW_ERR2_GAIN;
	st_senF[DIST_SEN_L_FRONT].s_skewErr2	= l_Lside_DistRef * L_FRONT_SKEW_ERR2_GAIN;
	st_senF[DIST_SEN_R_FRONT].s_skewErr3	= l_Rside_DistRef * R_FRONT_SKEW_ERR3_GAIN;
	st_senF[DIST_SEN_L_FRONT].s_skewErr3	= l_Lside_DistRef * L_FRONT_SKEW_ERR3_GAIN;
}

// *************************************************************************
//   機能		： 前壁オートキャリブレーション
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.10.28			sato			新規
// *************************************************************************/
PRIVATE void Dist_front_auto_calibration (void)
{
	USHORT i;
	ULONG ul_Lref = 0;
	ULONG ul_Rref = 0;
	
	/* データサンプリング */
	for( i=0; i<DIST_REF_NUM; i++){			// 100回サンプリングした平均値を基準の値とする。
		ul_Lref += st_sen[DIST_SEN_L_FRONT].s_now;
		ul_Rref += st_sen[DIST_SEN_R_FRONT].s_now;
		TIME_wait(1);
	}
	
	/* 基準値算出（平均値） */
	l_Lfront_DistRef = ul_Lref / DIST_REF_NUM ;		// 精度を100倍にする
	l_Rfront_DistRef = ul_Rref / DIST_REF_NUM ;
	
	st_sen[DIST_SEN_R_FRONT].s_ref       = l_Rfront_DistRef;
	st_sen[DIST_SEN_L_FRONT].s_ref       = l_Lfront_DistRef;
	st_sen[DIST_SEN_R_FRONT].s_limit     = l_Rfront_DistRef * R_FRONT_WALL_GAIN;
	st_sen[DIST_SEN_L_FRONT].s_limit     = l_Lfront_DistRef * L_FRONT_WALL_GAIN;
	st_sen[DIST_SEN_R_FRONT].s_noCtrl = l_Rfront_DistRef * R_FRONT_CTRL;//実験値
	st_sen[DIST_SEN_L_FRONT].s_noCtrl = l_Lfront_DistRef * L_FRONT_CTRL;
	st_sen[DIST_SEN_R_FRONT].s_ctrl = st_sen[DIST_SEN_R_FRONT].s_noCtrl * R_FRONT_NOCTRL_GAIN;
	st_sen[DIST_SEN_L_FRONT].s_ctrl = st_sen[DIST_SEN_L_FRONT].s_noCtrl * L_FRONT_NOCTRL_GAIN;
}

// *************************************************************************
//   機能		： オートキャリブレーション
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.10.28			sato			新規
// *************************************************************************/
PUBLIC void Dist_autocalibration(void)
{
	TIME_wait(2000);
	Dist_side_auto_calibration ();
	TIME_wait(100);
	MOT_goBlock_FinSpeed( 0.3, 0 );
	MOT_turn(MOT_R90);
	MOT_goHitBackWall();
	MOT_goBlock_FinSpeed( 0.3, 0 );
	TIME_wait(200);
	Dist_front_auto_calibration ();
	MOT_turn(MOT_L90);
	MOT_goHitBackWall();
//	printf("RF %5.2f LF %5.2f RS %5.2f LS %5.2 \n",st_sen[DIST_SEN_R_FRONT].s_ref,st_sen[DIST_SEN_L_FRONT].s_ref,st_sen[DIST_SEN_R_SIDE].s_ref,st_sen[DIST_SEN_L_SIDE].s_ref);
}

PUBLIC void calibration(void)
{
printf("RF %4df LF %4d RS %4d LS %4d \n",st_sen[DIST_SEN_R_FRONT].s_ref,st_sen[DIST_SEN_L_FRONT].s_ref,st_sen[DIST_SEN_R_SIDE].s_ref,st_sen[DIST_SEN_L_SIDE].s_ref);
printf("RF %4df LF %4d RS %4d LS %4d \n",st_sen[DIST_SEN_R_FRONT].s_limit,st_sen[DIST_SEN_L_FRONT].s_limit,st_sen[DIST_SEN_R_SIDE].s_limit,st_sen[DIST_SEN_L_SIDE].s_limit);
}
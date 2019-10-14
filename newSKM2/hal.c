// *************************************************************************
//   ロボット名	： 電研ベーシックDCマウス、サンシャイン
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.28			sato			新規（ファイルのインクルード）
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
#define		BAT_GOOD			(2135)			// 残量減ってきた（黄色）、1セル3.7V以上： 2135 = ( 3700mV * 2セル ) / 4.3(分圧) / 3300 * 4096 - 1
#define		BAT_LOW				(1961)			// 残量やばい！！（赤色）、1セル3.4V以上： 1961 = ( 3400mV * 2セル ) / 4.3(分圧) / 3300 * 4096 - 1
#define		GYRO_REF_NUM		(200)		//ジャイロのリファレンス値をサンプリングする数
#define		ACCEL_REF_NUM		(50)		//加速度のリファレンス値をサンプリングする数
#define		ENC_RESET_VAL		(32768)			// エンコーダの中間値
#define		ENC_R_TSTR			(TPUA.TSTR.BIT.CST1)	// 右エンコーダパルスカウント開始
#define		ENC_L_TSTR			(TPUA.TSTR.BIT.CST2)	// 左エンコーダパルスカウント開始
#define		DCM_ENA				(PORTB.PODR.BIT.B7)		// DCMの有効/無効
#define		DCM_R_IN1			(PORTB.PODR.BIT.B1)		// DCM右IN1
#define		DCM_R_IN2			(PORTB.PODR.BIT.B3)		// DCM右IN2
#define		DCM_L_IN1			(PORTB.PODR.BIT.B5)		// DCM左IN1
#define		DCM_L_IN2			(PORTB.PODR.BIT.B6)		// DCM左IN2
#define		DCM_R_TIMER			(TPUA.TSTR.BIT.CST0)	// DCM右タイマ開始
#define		DCM_L_TIMER			(TPUA.TSTR.BIT.CST3)	// DCM左タイマ開始
#define		DCM_R_TIORA			(TPU0.TIORH.BIT.IOA)	// DCM右ピン出力設定A
#define		DCM_R_TIORB			(TPU0.TIORH.BIT.IOB)	// DCM右ピン出力設定B
#define		DCM_L_TIORA			(TPU3.TIORH.BIT.IOA)	// DCM左ピン出力設定A
#define		DCM_L_TIORB			(TPU3.TIORH.BIT.IOB)	// DCM左ピン出力設定B
#define		DCM_R_TCNT			(TPU0.TCNT)				// DCM右カウント値
#define		DCM_L_TCNT			(TPU3.TCNT)				// DCM左カウント値
#define		DCM_R_GRA			(TPU0.TGRA)				// DCM右周期
#define		DCM_R_GRB			(TPU0.TGRB)				// DCM右Duty比
#define		DCM_L_GRA			(TPU3.TGRA)				// DCM左周期
#define		DCM_L_GRB			(TPU3.TGRB)				// DCM左Duty比

/* 非調整パラメータ */
#define PI							( 3.14159f )								// π

/* 調整パラメータ */
#define VCC_MAX						( 8.4f )									// バッテリ最大電圧[V]、4.2[V]×2[セル]
#define TIRE_R						( 22.1f )									// タイヤ直径 [mm]
#define GEAR_RATIO					( 36 / 8 )									// ギア比(スパー/ピニオン)
#define ROTATE_PULSE					( 2048 )									// 1周のタイヤパルス数
#define DIST_1STEP					( PI * TIRE_R / GEAR_RATIO / ROTATE_PULSE )				// 1パルスで進む距離 [mm]
#define F_CNT2MM(cnt)					( (FLOAT)cnt * DIST_1STEP )				// [カウント値]から[mm]へ換算
#define MOT_MOVE_ST_THRESHOLD				( 25 )							// 直進移動距離の閾値[mm]
#define MOT_MOVE_ST_MIN					( 20 )							// 直進移動距離の最低移動量[mm]
//#define MOT_ACC						( 1800 )						// 直進移動の加速度[mm/s2]
//#define MOT_DEC						( 1800 )						// 直進移動の減速度[mm/s2]
//#define MOT_ACC_ANGLE					( 1800 )		//旋回の角加速度[mm/s2]
//#define MOT_DEC_ANGLE					( 1800 )		//旋回の角減速度[mm/s2]

//20170815 超信地旋回実装時に追加
#define A1_MIN					( 25 )						// 第1最低移動角度
#define A2_MIN					( 30 )						// 第2最低移動角度
#define A3_MIN					( 20 )						// 第3最低移動角度

#define ANGLE_OFFSET1_R				( 0 )	//-12					// 角度のオフセット値（バッファリングによる誤差を埋めるための値）
#define ANGLE_OFFSET1				( 0 )	//-12					// 角度のオフセット値（バッファリングによる誤差を埋めるための値）
#define ANGLE_OFFSET2_R				( 0 )	//3
#define ANGLE_OFFSET2				( 0 )						// 角度のオフセット値（バッファリングによる誤差を埋めるための値）
#define ANGLE_OFFSET3				( 0 )					// 角度のオフセット値（バッファリングによる誤差を埋めるための値）

//#define MOT_MOVE_ST_THRESHOLD			( 25 )						// 直進移動距離の閾値[mm]
//#define MOT_MOVE_ST_MIN				( 20 )						// 直進移動距離の最低移動量[mm]


#define log_num			(1000)					//ログ取得数（変更時はこちらを変更）

//**************************************************
// 列挙体（enum）
//**************************************************
/* 制御動作タイプ */
typedef enum{
	CTRL_ACC,				// [00] 加速中(直進)
	CTRL_CONST,				// [01] 等速中(直進)
	CTRL_DEC,				// [02] 減速中(直進)

	CTRL_SKEW_ACC,			// [03] 斜め加速中(直進)
	CTRL_SKEW_CONST,		// [04] 斜め等速中(直進)
	CTRL_SKEW_DEC,			// [05] 斜め減速中(直進)
	
	CTRL_HIT_WALL,			// [06]壁当て動作
	
	CTRL_ACC_TRUN,			// [07] 加速中(超信地旋回)
	CTRL_CONST_TRUN,		// [08] 等速中(超信地旋回)
	CTRL_DEC_TRUN,			// [09] 減速中(超信地旋回)
	
	CTRL_ENTRY_SURA,		// [10]スラローム前前進
	CTRL_ACC_SURA,			// [11] 加速中(スラ)
	CTRL_CONST_SURA,		// [12] 等速中(スラ)
	CTRL_DEC_SURA,			// [13] 減速中(スラ)
	CTRL_EXIT_SURA,			// [14] スラローム後前進

	CTRL_MAX,

}enCTRL_TYPE;


/* 動作タイプ */
typedef enum{
	MOT_ST_NC    =  0,
	MOT_ACC_CONST_DEC,			// [01] 台形加速
	MOT_ACC_CONST_DEC_CUSTOM,	// [02] 台形加速（等速値変更）
	MOT_ACC_CONST,				// [03] 加速＋等速
	MOT_ACC_CONST_CUSTOM,		// [04] 加速＋等速（加速値変更）
	MOT_CONST_DEC,				// [05] 等速＋減速
	MOT_CONST_DEC_CUSTOM,		// [06] 等速＋減速（減速値変更）
	MOT_ST_MAX,
}enMOT_ST_TYPE;

/* 直進タイプ */
typedef enum{
	MOT_GO_ST_NORMAL    =  0,	// 通常の直進
	MOT_GO_ST_SKEW,				// 斜めの直進
	MOT_GO_ST_MAX,
}enMOT_GO_ST_TYPE;


//**************************************************
// 構造体（struct）
//**************************************************
/* 動作情報 */
typedef struct{

	FLOAT			f_time;			// 時間					[msec]

	/* 速度制御 */
	FLOAT			f_acc1;			// 加速度1				[mm/s2]
	FLOAT			f_acc3;			// 加速度3				[mm/s2]
	FLOAT			f_now;			// 現在速度				[mm/s]
	FLOAT			f_trgt;			// 加速後の目標速度		[mm/s]
	FLOAT			f_last;			// 減速後の最終速度		[mm/s]

	/* 距離制御 */
	FLOAT			f_dist;			// 移動距離				[mm]
	FLOAT			f_l1;			// 第1移動距離			[mm]
	FLOAT			f_l1_2;			// 第1+2移動距離		[mm]

	/* 角速度制御 */
	FLOAT			f_accAngleS1;	// 角加速度1			[rad/s2]
	FLOAT			f_accAngleS3;	// 角加速度3			[rad/s2]
	FLOAT			f_nowAngleS;	// 現在角速度			[rad/s]
	FLOAT			f_trgtAngleS;	// 加速後の目標角速度	[rad/s]
	FLOAT			f_lastAngleS;	// 減速後の最終角速度	[rad/s]

	/* 角度制御 */
	FLOAT			f_angle;		// 移動角度				[rad]
	FLOAT			f_angle1;		// 第1移動角度			[rad]
	FLOAT			f_angle1_2;		// 第1+2移動角度		[rad]
}stMOT_DATA;

/* 制御データ */
typedef struct{
	enCTRL_TYPE		en_type;		// 動作タイプ
	FLOAT			f_time;			// 目標時間 [sec]
	FLOAT			f_acc;			// [速度制御]   加速度[mm/s2]
	FLOAT			f_now;			// [速度制御]   現在速度[mm/s]
	FLOAT			f_trgt;			// [速度制御]   最終速度[mm/s]
	FLOAT			f_nowDist;		// [距離制御]   現在距離[mm]
	FLOAT			f_dist;			// [距離制御]   最終距離[mm]
	FLOAT			f_accAngleS;	// [角速度制御] 角加速度[rad/s2]
	FLOAT			f_nowAngleS;	// [角速度制御] 現在角速度[rad/s]
	FLOAT			f_trgtAngleS;	// [角速度制御] 最終角速度[rad/s]
	FLOAT			f_nowAngle;		// [角度制御]   現在角度[rad]
	FLOAT			f_angle;		// [角速制御]   最終角度[rad]
}stCTRL_DATA;


//**************************************************
// グローバル変数
//**************************************************
/* バッテリ監視 */
PRIVATE USHORT	us_BatLvAve = 4095;							// バッテリ平均値（AD変換の最大値で初期化）

/*ジャイロセンサ*/
PRIVATE SHORT s_GyroVal; 					  				// ジャイロセンサの現在値
//PRIVATE SHORT s_GyroValBuf[8];								// ジャイロセンサのバッファ値
PUBLIC FLOAT  f_GyroNowAngle;		 						// ジャイロセンサの現在角度
PRIVATE LONG  l_GyroRef; 									// ジャイロセンサの基準値

/*角速度取得*/
PRIVATE SHORT s_AccelVal; 					  				// 加速度の取得値
PRIVATE FLOAT f_NowAccel;										// 加速度の現在地
PRIVATE LONG  l_AccelRef; 									// 加速度の基準値

/* 制御  */
PRIVATE enCTRL_TYPE		en_Type;						// 制御方式
PRIVATE UCHAR 			uc_CtrlFlag			= FALSE;	// フィードバック or フィードフォワード 制御有効フラグ（FALSE:無効、1：有効）
PRIVATE LONG			l_CntR;							// 右モータのカウント変化量						（1[msec]毎に更新される）
PRIVATE LONG			l_CntL;							// 左モータのカウント変化量						（1[msec]毎に更新される）
// 制御
PUBLIC  FLOAT			f_Time 				= 0;		// 動作時間[sec]								（1[msec]毎に更新される）
PUBLIC  FLOAT			f_TrgtTime 			= 1000;		// 動作目標時間 [msec]							（設定値）
// 速度制御//////////////////////////////////////////
PRIVATE FLOAT 			f_Acc			= 0;		// [速度制御]   加速度							（設定値）
PRIVATE FLOAT			f_BaseSpeed		= 0;		// [速度制御]   初速度							（設定値）
PRIVATE FLOAT			f_LastSpeed 		= 0;		// [速度制御]   最終目標速度					（設定値）
PRIVATE FLOAT			f_NowSpeed		= 0;		// [速度制御]   現在の速度 [mm/s]				（1[msec]毎に更新される）
PUBLIC FLOAT			f_TrgtSpeed 		= 0;		// [速度制御]   目標移動速度 [mm/s]				（1[msec]毎に更新される）
PRIVATE FLOAT			f_ErrSpeedBuf		= 0;		// [速度制御] 　速度エラー値のバッファ	（1[msec]毎に更新される）
PUBLIC FLOAT			f_SpeedErrSum 		= 0;		// [速度制御]   速度積分制御のサム値			（1[msec]毎に更新される）
// 距離制御
PRIVATE FLOAT			f_BaseDist		= 0;		// [距離制御]   初期位置						（設定値）
PRIVATE FLOAT			f_LastDist 		= 0;		// [距離制御]   最終移動距離					（設定値）
PUBLIC FLOAT			f_TrgtDist 		= 0;		// [距離制御]   目標移動距離					（1[msec]毎に更新される）
PUBLIC volatile FLOAT 		f_NowDist		= 0;		// [距離制御]   現在距離						（1[msec]毎に更新される）
PRIVATE FLOAT			f_NowDistR		= 0;		// [距離制御]   現在距離（右）					（1[msec]毎に更新される）
PRIVATE FLOAT 			f_NowDistL		= 0;		// [距離制御]   現在距離（左）					（1[msec]毎に更新される）
PUBLIC FLOAT			f_DistErrSum 		= 0;		// [距離制御]   距離積分制御のサム値			（1[msec]毎に更新される）
// 角速度制御
PRIVATE FLOAT 			f_AccAngleS		= 0;		// [角速度制御] 角加速度						（設定値）
PRIVATE FLOAT			f_BaseAngleS		= 0;		// [角速度制御] 初期角速度						（設定値）
PRIVATE FLOAT			f_LastAngleS 		= 0;		// [角速度制御] 最終目標角速度					（設定値）
PUBLIC FLOAT			f_TrgtAngleS 		= 0;		// [角速度制御] 目標角速度 [rad/s]				（1[msec]毎に更新される）
PRIVATE FLOAT			f_ErrAngleSBuf		= 0;		// [角速度制御] 角速度エラー値のバッファ	（1[msec]毎に更新される）
PUBLIC FLOAT			f_AngleSErrSum 		= 0;		// [角速度制御]   角度積分制御のサム値			（1[msec]毎に更新される）
// 角度制御
PRIVATE FLOAT			f_BaseAngle		= 0;		// [角度制御]   初期角度						（設定値）
PRIVATE FLOAT			f_LastAngle 		= 0;		// [角度制御]   最終目標角度					（設定値）
PUBLIC volatile FLOAT 		f_NowAngle		= 0;		// [角度制御]   現在角度　	volatileをつけないとwhileから抜けられなくなる（最適化のせい）（1[msec]毎に更新される）
PUBLIC FLOAT			f_TrgtAngle 		= 0;		// [角度制御]   目標角度						（1[msec]毎に更新される）
PUBLIC FLOAT			f_AngleErrSum 		= 0;		// [角度制御]   角度積分制御のサム値			（1[msec]毎に更新される）
// 壁制御
PRIVATE LONG 			l_WallErr 		= 0;		// [壁制御]     壁との偏差						（1[msec]毎に更新される）
PRIVATE FLOAT			f_ErrDistBuf		= 0;		// [壁制御]     距離センサーエラー値のバッファ	（1[msec]毎に更新される）

/* 動作 */
PRIVATE FLOAT 			f_MotNowSpeed 		= 0.0f;		// 現在速度
PRIVATE FLOAT 			f_MotTrgtSpeed 		= 0.0f;		// 目標速度
PRIVATE	stMOT_DATA 		st_Info;				// シーケンスデータ
PRIVATE FLOAT			f_MotSuraStaSpeed	= 0.0f;
PRIVATE FLOAT			sliplengs		= 0.0f;		//スリップ距離

PRIVATE enMOT_WALL_EDGE_TYPE	en_WallEdge = MOT_WALL_EDGE_NONE;	// 壁切れ補正
PRIVATE BOOL			bl_IsWallEdge = FALSE;				// 壁切れ検知（TRUE:検知、FALSE：非検知）
PRIVATE FLOAT			f_WallEdgeAddDist = 0;				// 壁切れ補正の移動距離

//フェイルセーフ
PUBLIC FLOAT  f_ErrChkAngle; 			  // ジャイロセンサのエラー検出用の角度
PUBLIC BOOL   bl_ErrChk; 				  // ジャイロセンサのエラー検出（FALSE：検知しない、TRUE：検知する）
PRIVATE BOOL			bl_failsafe		= FALSE;	// マウスがの制御不能（TRUE：制御不能、FALSE：制御可能）

//ログプログラム群（取得数変更はdefineへ）
PRIVATE	FLOAT	Log_1[log_num];
PRIVATE FLOAT	Log_2[log_num];
PRIVATE FLOAT	Log_3[log_num];
PRIVATE FLOAT	Log_4[log_num];
PRIVATE FLOAT	Log_5[log_num];
PRIVATE FLOAT	Log_6[log_num];
PRIVATE FLOAT	Log_7[log_num];
PRIVATE FLOAT	Log_8[log_num];
PRIVATE FLOAT	Log_9[log_num];
PRIVATE FLOAT	Log_10[log_num];
PRIVATE FLOAT	Log_11[log_num];
PRIVATE FLOAT	Log_12[log_num];

PRIVATE	USHORT	log_count = 0;
PUBLIC	BOOL	b_logflag = FALSE;

PRIVATE FLOAT	templog1	= 0;
PRIVATE FLOAT	templog2	= 0;

//ログ用デューティー
PRIVATE	FLOAT	f_Duty_R;
PRIVATE	FLOAT	f_Duty_L;

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************

//ログ専用変数
//PRIVATE FLOAT LogBuf[1000];
// *************************************************************************
//   機能		： ログ取り関数
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.5.10		翔	新規
// *************************************************************************/
/*PUBLIC void log_in(FLOAT log_input)
{

	int i=0;
	//バッファをシフト
	while(i < 1000){
		LogBuf[i] = LogBuf[i+1];
		i++;
	}
	LogBuf[999]=log_input;

}
*/
// *************************************************************************
//   機能		： ログ出力関数
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.5.10		翔	新規
// *************************************************************************/
/*PUBLIC void log_out(void)
{
	int i=0;
	while(i<1000){
		printf("%5.2f\n\r",LogBuf[i]);
		i++;
	}
}
*/
// *************************************************************************
//   機能		： HALを初期化する。
//   注意		： なし
//   メモ		： 内部変数などをクリアする。
//   引数		： なし
//   返り値		： なし
//   その他     ：起動時に動作
// **************************    履    歴    *******************************
// 		v1.0		2013.11.27			外川			新規
// 		v1.1		2013.11.27			外川			センサの基準値とリミット値を設定
// *************************************************************************/
PUBLIC void HAL_init( void )
{
	/* ジャイロセンサ */
	f_GyroNowAngle = 0;			// ジャイロセンサの現在角度(0にしても探索他は動くが、宴会とかtestrunとかは動かない)修正済みと思われる
	l_GyroRef  = 0;				// ジャイロセンサの基準値
	
	f_ErrChkAngle = 0;
	bl_ErrChk = FALSE;
	
	/* エンコーダ */
	ENC_Sta();	//左側のピンがLEFT(突起は前方方向)
	
	/* DCM*/
	DCM_ENA = ON;
	
	/* [暫定] warningを消すだけ（削除してOK） */
	f_AccAngleS = f_AccAngleS;
	f_BaseAngleS = f_BaseAngleS;
	f_LastAngleS = f_LastAngleS;
	f_BaseAngle = f_BaseAngle;
	f_LastAngle = f_LastAngle;

}

// *************************************************************************
//   機能		： 1文字出力
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC void SCI_putch(char data)
{
	while(SCI1.SSR.BIT.TEND == 0) {};
//		if (SCI1.SSR.BYTE & 0x80) {			// 送信バッファの空きチェック
			SCI1.TDR = data;
//			SCI1.SSR.BYTE = SCI1.SSR.BYTE & 0x40;
			SCI1.SSR.BIT.TEND = 0;
//			break;
//		}
//	}
}


// *************************************************************************
//   機能		： 文字列出力
//   注意		： なし
//   メモ		： "\n"のみで"CR+LF"出力を行う。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC void SCI_puts(char *buffer)
{
	char data;
	
	/* nullまで出力 */
	while( (data = *( buffer++ ) ) != 0 ){
		
		/* データの値に応じて出力を変える */
		if (data == 0x0a) {
			SCI_putch(0x0d);		// CR出力
			SCI_putch(0x0a);		// LF出力
		} else {
			SCI_putch(data);		// 1文字出力
		}
	}
}


// *************************************************************************
//   機能		： 文字列出力
//   注意		： なし
//   メモ		： 文字列長さ指定付き"/\n"のみ "CR+LF"出力を行う。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC void SCI_putsL(char *buffer, int len)
{
	int i;
	char data;
	
	for( i=0; i<len; i++ ){
		data=*(buffer++);
		
		/* データの値に応じて出力を変える */
		if (data == 0x0a) {
			SCI_putch(0x0d);		// CR出力
			SCI_putch(0x0a);		// LF出力
		} else {
			SCI_putch(data);		// 1文字出力
		}
	}
}


// *************************************************************************
//   機能		： 1文字出力用ラッパー関数
//   注意		： なし
//   メモ		： printfなどの低レベル出力に使用される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC void charput(unsigned char data)
{
	SCI_putch(data);
}


// *************************************************************************
//   機能		： 入力バッファチェック
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC int SCI_chkRecv(void)
{
	/* データの受信チェック */
	if (IR(SCI1,RXI1) == 1) {
		return 1;		// 受信データあり
	}
	else {
		return 0;		// 受信データなし
	}
}


// *************************************************************************
//   機能		： 1文字入力（char型版）
//   注意		： なし
//   メモ		： エコーバックなし。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC char SCI_getch(void)
{
	char data;
	
	while(1){
		/* データの受信チェック */
		if ( SCI_chkRecv() ) {
			data = SCI1.RDR;						// データ受信
//			SCI1.SSR.BYTE = SCI1.SSR.BYTE & 0x80;
			IR(SCI1,RXI1) = 0;
			break;
		}
	}
	return data;
}


// *************************************************************************
//   機能		： 1文字入力（unsigned char型版）
//   注意		： なし
//   メモ		： エコーバックなし。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC unsigned char SCI_getch_uc(void)
{
	unsigned char data;
	while(1){
		/* データの受信チェック */
		if ( SCI_chkRecv() ) {
			data = SCI1.RDR;						// データ受信
//			SCI1.SSR.BYTE = SCI1.SSR.BYTE & 0x80;
			IR(SCI1,RXI1) = 0;
			break;
		}
	}
	return data;
}


// *************************************************************************
//   機能		： 文字列入力
//   注意		： なし
//   メモ		： CRコードまで/最大255文字/エコーバックあり
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC int SCI_gets(char *buffer)
{
	char data;
	int  i = 0;

	while(1){
		data = SCI_getch();		// 1文字入力
		*buffer = data;
		SCI_putch(data);		// 1文字出力(エコーバック)
		buffer++;
		i++;
		if (i > 255)      break;	// 最大文字数に到達
		if (data == 0x0D) break;	// CRコードまで受信した
	}
	*buffer = (unsigned char)0;		// null
	
	return i;						// 入力文字数を返却
}


// *************************************************************************
//   機能		： 1文字入力用ラッパー関数
//   注意		： なし
//   メモ		： scanfなどの低レベル出力に使用される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC unsigned char charget(void)
{
	return SCI_getch_uc();
}


// *************************************************************************
//   機能		： バッテリ電圧を取得する
//   注意		： なし
//   メモ		： 直前5回の平均値
//   引数		： なし
//   返り値		： 電圧[mV]
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC FLOAT BAT_getLv(void)
{
	FLOAT f_val = (FLOAT)( us_BatLvAve + 1 );		// 値は0から始まるから1を加算
	
	return ( f_val / 4096 * 3300 * 4.4f );
}


// *************************************************************************
//   機能		： バッテリ監視用ポーリング関数
//   注意		： なし
//   メモ		： 割り込みから実行される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.16			外川			新規
// *************************************************************************/
PUBLIC void BAT_Pol( void )
{
	static USHORT 	us_batLv[5] = { 4095, 4095, 4095, 4095, 4095 };		// バッテリレベル（AD変換の最大値で初期化）

	/* ================================================== */
	/*  平均値を取得するため、データのバッファ処理を行う  */
	/* ================================================== */
	/* バッファをシフト */
	us_batLv[4] = us_batLv[3];
	us_batLv[3] = us_batLv[2];
	us_batLv[2] = us_batLv[1];
	us_batLv[1] = us_batLv[0];

	/* 最新の値を格納 */
	S12AD.ADANS0.WORD 		= 0x0001;		// AN0 変換対象設定
	S12AD.ADCSR.BIT.ADST 		= 1;		// AD変換開始
	while( S12AD.ADCSR.BIT.ADST == 1);		// AD変換待ち
	us_batLv[0] = S12AD.ADDR0;				// AN0 変換データ取得

	/* 電圧平均化 */
	us_BatLvAve = ( us_batLv[0] + us_batLv[1] + us_batLv[2] + us_batLv[3] + us_batLv[4] ) / 5;
	
	/*  残量に応じてLEDを表示  */
	/* ======================= */
	if( us_BatLvAve < BAT_LOW ) {			// 残量やばい！！（赤色）
		LEDG = OFF;
		LEDR = ON;
	}
	else if( us_BatLvAve < BAT_GOOD ) {		// 残量減ってきた（黄色）
		LEDG = ON;
		LEDR = ON;
	}
	else{									// 残量問題なし（緑色）
		LEDG = ON;
		LEDR = OFF;
	}
}

// *************************************************************************
//   機能		： ジャイロセンサのリファレンス値（基準の値）を設定する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
//   その他　　：　起動時に動作
// **************************    履    歴    *******************************
// 		v1.0		2018.08.07			sato			新規
//		v1.1		2018.08.25			sato			Refの取得が正確にできていないように見えたため強引にRefを設定している（緊急措置）
// *************************************************************************/
PUBLIC void GYRO_SetRef( void )
{
	USHORT i;
	ULONG ul_ref = 0;
	
	/* データサンプリング */
	for( i=0; i<GYRO_REF_NUM; i++){			// 100回サンプリングした平均値を基準の値とする。
		ul_ref += (ULONG)s_GyroVal;
		TIME_wait(1);
	}
	
	/* 基準値算出（平均値） */
	l_GyroRef = ul_ref / GYRO_REF_NUM * 100;		// 精度を100倍にする
//	l_GyroRef = 0x1304*100;
}

// *************************************************************************
//   機能		： ジャイロの角速度に関する制御偏差を取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
//
// **************************    履    歴    *******************************
// 		v1.0		2013.11.26		外川			新規
// *************************************************************************/
PUBLIC FLOAT GYRO_getSpeedErr( void )
{
	LONG  l_val = (LONG)s_GyroVal * 100 ;				// 100倍の精度にする
	LONG  l_err = l_val - l_GyroRef ;
	FLOAT f_res;
	
	/* 角速度の偏差算出 */
	if( ( l_err < -20 * 100 ) || ( 20 * 100 < l_err ) ){
		f_res = (FLOAT)l_err /32.768 / 100;		//32.768 = 2^16(16bit)/2000(+-1000度) LSB/(°/s)
													// 100倍の精度
	}
	else{
		f_res = 0;									// [deg/s]
	}
	
	return f_res;
}

// *************************************************************************
//   機能		： ジャイロの現在の角度を取得する
//   注意		： なし
//   メモ		： ☆
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.26			外川			新規
// *************************************************************************/
PUBLIC FLOAT GYRO_getNowAngle( void )
{
	return f_GyroNowAngle;
}

// *************************************************************************
//   機能		： ジャイロの現在の角度を取得する
//   注意		： なし
//   メモ		： ☆
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.26			外川			新規
// *************************************************************************/
PUBLIC FLOAT GYRO_getRef( void )
{
	return l_GyroRef;
}

// *************************************************************************
//   機能		： ジャイロセンサ用ポーリング関数
//   注意		： なし
//   メモ		： 割り込みから実行される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.26			外川			新規
//		v2.0		2018.08.16			sato		SPIによるジャイロ取得設定
// *************************************************************************/
PUBLIC void GYRO_Pol( void )
{
	FLOAT f_speed;
	
	/* バッファシフト（[7]に新しいデータを入れるため、[0]のデータを捨てて、1つずつ詰める） */
/*	s_GyroValBuf[0]	= s_GyroValBuf[1];
	s_GyroValBuf[1]	= s_GyroValBuf[2];
	s_GyroValBuf[2]	= s_GyroValBuf[3];
	s_GyroValBuf[3]	= s_GyroValBuf[4];
	s_GyroValBuf[4]	= s_GyroValBuf[5];
	s_GyroValBuf[5]	= s_GyroValBuf[6];
	s_GyroValBuf[6]	= s_GyroValBuf[7];
*/	
	/* 最新のジャイロセンサ値を取得 */
//	s_GyroValBuf[7] = (SHORT)recv_spi_gyro();
	
	/* ジャイロの値を平滑する（平滑数は8つ） */
	s_GyroVal = (SHORT)recv_spi_gyro();//( s_GyroValBuf[0] + s_GyroValBuf[1] + s_GyroValBuf[2] + s_GyroValBuf[3] +
				//  s_GyroValBuf[4] + s_GyroValBuf[5] + s_GyroValBuf[6] + s_GyroValBuf[7] ) / 8;
	
	/* 現在の角度を更新する */
	f_speed = GYRO_getSpeedErr();			// 角速度取得 (0.001sec毎の角速度)
	f_GyroNowAngle += f_speed / 1000;		// 角度設定   (0.001sec毎に加算するため)

	/* エラーチェック */
	if( bl_ErrChk == TRUE ){
		
		f_ErrChkAngle += f_speed/1000;		// 角度設定   (0.001sec毎に加算するため)
		
		if( ( f_ErrChkAngle < -500 ) || ( 500 < f_ErrChkAngle )||(f_speed <-1000)||(1000<f_speed) ){
			
			Failsafe_flag();
//			printf("fail\n\r");
		}
	}
}

// *************************************************************************
//   機能		： 加速度のリファレンス値（基準の値）を設定する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
//   その他　　：　起動時に動作
// **************************    履    歴    *******************************
// 		v1.0		2019.10.14			sato			新規
// *************************************************************************/
PUBLIC void ACCEL_SetRef( void )
{
	USHORT i;
	LONG ul_ref = 0;
	
	/* データサンプリング */
	for( i=0; i<ACCEL_REF_NUM; i++){			// 100回サンプリングした平均値を基準の値とする。
		ul_ref += (LONG)s_AccelVal;
		TIME_wait(1);
	}
	
	/* 基準値算出（平均値） */
	l_AccelRef = ul_ref / ACCEL_REF_NUM ;		
//	l_GyroRef = 0x1304*100;
}

// *************************************************************************
//   機能		： 加速度の値を取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
//
// **************************    履    歴    *******************************
// 		v1.0		2019.10.13		sato			新規
// *************************************************************************/
PUBLIC FLOAT Accel_getSpeedErr( void )
{
	LONG  l_val = (LONG)s_AccelVal ;				// 100倍の精度にする
	LONG  l_err = l_val - l_AccelRef ;
	FLOAT f_res;

	f_res= (FLOAT)l_err/2048*9800;
	return f_res;
}

// *************************************************************************
//   機能		： ジャイロセンサ用ポーリング関数
//   注意		： なし
//   メモ		： 割り込みから実行される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.26			外川			新規
//		v2.0		2018.08.16			sato		SPIによるジャイロ取得設定
// *************************************************************************/
PUBLIC void ACCEL_Pol( void )
{	
	/* 加速度の値を取得する */
	s_AccelVal = (SHORT)recv_spi_accel();
	
	/* 現在の加速度を更新する */
	f_NowAccel = Accel_getSpeedErr();			// 角速度取得 (0.001sec毎の加速度)

	/* エラーチェック */
/*	if( bl_ErrChk == TRUE ){
		
		f_ErrChkAngle += f_speed/1000;		// 角度設定   (0.001sec毎に加算するため)
		
		if( ( f_ErrChkAngle < -500 ) || ( 500 < f_ErrChkAngle )||(f_speed <-1000)||(1000<f_speed) ){
			
			Failsafe_flag();
//			printf("fail\n\r");
		}

	}
*/
}

// *************************************************************************
//   機能		： エラー検出用を開始する
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.10.10			sato			新規
// *************************************************************************/
PUBLIC void GYRO_staErrChkAngle( void )
{
	f_ErrChkAngle = 0;
	bl_ErrChk = TRUE;

}


// *************************************************************************
//   機能		： エラー検出用を終了する
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.10.10			sato			新規
// *************************************************************************/
PUBLIC void GYRO_endErrChkAngle( void )
{
	f_ErrChkAngle = 0;
	bl_ErrChk = FALSE;

}

// *************************************************************************
//   機能		： SPI関数
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.08.06			sato			新規
// *************************************************************************/
PUBLIC USHORT recv_spi(USHORT spi_ad)
{
	USHORT recv;
	RSPI0.SPDR.WORD.H = spi_ad;
	
	while(!RSPI0.SPSR.BIT.IDLNF);	//送信開始を確認
	while(RSPI0.SPSR.BIT.IDLNF);		//RSPI0ｱｲﾄﾞﾙ状態か確認
		recv = RSPI0.SPDR.WORD.H ;

	return(recv);
}

// *************************************************************************
//   機能		： SPI(whoami)
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.08.04			外川			新規
// *************************************************************************/
PUBLIC USHORT recv_spi_who(void)
{
	USHORT recv;
	USHORT whoami = (0x75|0x80);
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(whoami);
	recv = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	return(recv);
		
}

// *************************************************************************
//   機能		： SPI_init
//   注意		： なし
//   メモ		： 初回実行
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.08.05			sato		新規
//		v1.1		2019.10.13			sato		acc追加
// *************************************************************************/
PUBLIC void recv_spi_init(void)
{
	USHORT recv;
	USHORT register107 = (0x6B|0x00);		//power management1
	USHORT register106 = (0x6A|0x00);
	USHORT register112 = (0x70|0x00);
	USHORT register27 = (0x1B|0x00);
	USHORT register28 = (0x1C|0x00);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register107);
	recv = recv_spi(0x80);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(100);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register106);
	recv = recv_spi(0x01);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(100);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register112);
	recv = recv_spi(0x40);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(1);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register27);
	recv = recv_spi(0x30);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(1);

	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register28);
	recv = recv_spi(0x18);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(1);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register107);
	recv = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(1);
	
	/*read*/
/*
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register27|0x80);
	recv = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	return(recv);
*/
}

// *************************************************************************
//   機能		： SPI_gyro_read
//   注意		： なし
//   メモ		： 初回実行
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.08.05			sato		新規
// *************************************************************************/
PUBLIC USHORT recv_spi_gyro(void)
{
	USHORT recv = 0;
	USHORT recv1;
	USHORT recv2;
	USHORT gyro_H = (0x47|0x80);	//register71
	USHORT gyro_L = (0x48|0x80);	//register72
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv1 = recv_spi(gyro_H);
	recv1 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv2 = recv_spi(gyro_L);
	recv2 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	RSPI0.SPSR.BYTE = 0xA0;
	
	recv = (recv1<<8)+(recv2&0xFF);
	
	return(recv);
		
}

// *************************************************************************
//   機能		： SPI_accelerometer_read
//   注意		： なし
//   メモ		： 初回実行
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.10.13			sato		新規
// *************************************************************************/
PUBLIC USHORT recv_spi_accel(void)
{
	USHORT recv = 0;
	USHORT recv1;
	USHORT recv2;
	USHORT accel_H = (0x3D|0x80);	//register71
	USHORT accel_L = (0x3E|0x80);	//register72
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv1 = recv_spi(accel_H);
	recv1 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv2 = recv_spi(accel_L);
	recv2 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	RSPI0.SPSR.BYTE = 0xA0;
	
	recv = (recv1<<8)+(recv2&0xFF);
	
	return(recv);
		
}

// *************************************************************************
//   機能		： SPI_gyro_read
//   注意		： なし
//   メモ		： 初回実行
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.08.25			sato		新規
// *************************************************************************/
PUBLIC USHORT recv_spi_gyrooffset(void)
{
	USHORT recv = 0;
	USHORT recv1;
	USHORT recv2;
	USHORT gyro_H = (0x23|0x80);	//register23
	USHORT gyro_L = (0x24|0x80);	//register24
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv1 = recv_spi(gyro_H);
	recv1 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv2 = recv_spi(gyro_L);
	recv2 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	RSPI0.SPSR.BYTE = 0xA0;
	
	recv = (recv1<<8)+(recv2&0xFF);
	
	return(recv);
		
}

// *************************************************************************
//   機能		： エンコーダのカウントを開始する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void ENC_Sta( void )
{
	ENC_R_TSTR = ON;		// カウント開始
	ENC_L_TSTR = ON;		// カウント開始
}


// *************************************************************************
//   機能		： エンコーダのカウントを停止する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void ENC_Stop( void )
{
	ENC_R_TSTR = OFF;		// カウント停止
	ENC_L_TSTR = OFF;		// カウント停止
}


// *************************************************************************
//   機能		： エンコーダのカウントをクリア
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PRIVATE void ENC_clr( void )
{
	ENC_R_TCNT = ENC_RESET_VAL;
	ENC_L_TCNT = ENC_RESET_VAL;
}


// *************************************************************************
//   機能		： エンコーダのカウント値（パルス数）を取得する
//   注意		： なし
//   メモ		： 中間値からの差分
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void ENC_GetDiv( LONG* p_r, LONG* p_l )
{
	LONG l_cntR = (LONG)ENC_R_TCNT;
	LONG l_cntL = (LONG)ENC_L_TCNT;
	
	ENC_clr();		// カウント値リセット
	
	*p_r = l_cntR - ENC_RESET_VAL;		// 右モータ
	*p_l = ENC_RESET_VAL - l_cntL;		// 左モータ
}


// *************************************************************************
//   機能		： DCMの回転方向をCW（時計回り）にする
//   注意		： なし
//   メモ		： なし
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.08.19			sato	回転方向は後でチェック	新規
// *************************************************************************/
PUBLIC void DCM_setDirCw( enDCM_ID en_id )
{
	/* 回転方向設定 */
	if( en_id == DCM_R ){			// 右
		DCM_R_IN1 = OFF;				// BIN1
		DCM_R_IN2 = ON;			// BIN2
	}
	else{							// 左
		DCM_L_IN1 = ON;			// AIN1
		DCM_L_IN2 = OFF;				// AIN2
	}
}


// *************************************************************************
//   機能		： DCMの回転方向をCCW（反時計回り）にする
//   注意		： なし
//   メモ		： なし
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void DCM_setDirCcw( enDCM_ID en_id )
{
	/* 回転方向設定 */
	if( en_id == DCM_R ){			// 右
		DCM_R_IN1 = ON;			// BIN1
		DCM_R_IN2 = OFF;				// BIN2
	}
	else{							// 左
		DCM_L_IN1 = OFF;				// AIN1
		DCM_L_IN2 = ON;			// AIN2
	}
}


// *************************************************************************
//   機能		： DCMを停止する
//   注意		： なし
//   メモ		： PWMのHI出力中に本関数を実行すると、ピンが100%出力状態なるため、関数内でピンをクリア（Lo）する。
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void DCM_stopMot( enDCM_ID en_id )
{
	/* 停止設定 */
	if( en_id == DCM_R ){			// 右
		DCM_R_IN1 = OFF;			// BIN1
		DCM_R_IN2 = OFF;			// BIN2
		DCM_R_TIMER = OFF;			// タイマ停止
		DCM_R_TIORA = 1;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
		DCM_R_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	}
	else{							// 左
		DCM_L_IN1 = OFF;			// AIN1
		DCM_L_IN2 = OFF;			// AIN2
		DCM_L_TIMER = OFF;			// タイマ停止
		DCM_L_TIORA = 1;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
		DCM_L_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	}
}


// *************************************************************************
//   機能		： DCMをブレーキングする
//   注意		： なし
//   メモ		： なし
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void DCM_brakeMot( enDCM_ID en_id )
{
	/* 停止設定 */
	if( en_id == DCM_R ){			// 右
		DCM_R_IN1 = ON;				// BIN1
		DCM_R_IN2 = ON;				// BIN2
		DCM_R_TIMER = OFF;			// タイマ停止
		DCM_R_TIORA = 1;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
		DCM_R_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	}
	else{							// 左
		DCM_L_IN1 = ON;				// AIN1
		DCM_L_IN2 = ON;				// AIN2
		DCM_L_TIMER = OFF;			// タイマ停止
		DCM_L_TIORA = 1;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	    	DCM_L_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	}
}


// *************************************************************************
//   機能		： DCMを動作開始する
//   注意		： なし
//   メモ		： 動作開始前にPWMと回転方向を指定しておくこと
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void DCM_staMot( enDCM_ID en_id )
{
	/* タイマスタート */
	if( en_id == DCM_R ){			// 右
		DCM_R_TIORA = 2;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 1 出力
		DCM_R_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
		DCM_R_TIMER = ON;			// タイマ開始
	}
	else{							// 左
		DCM_L_TIORA = 2;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 1 出力
	    	DCM_L_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
		DCM_L_TIMER = ON;			// タイマ開始
	}
}


// *************************************************************************
//   機能		： 全DCMを動作開始する
//   注意		： なし
//   メモ		： 動作開始前にPWMと回転方向を指定しておくこと
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void DCM_staMotAll( void )
{
	DCM_staMot(DCM_R);									// 右モータON
	DCM_staMot(DCM_L);									// 左モータON
}


// *************************************************************************
//   機能		： DCMのPWM-Dutyを設定する
//   注意		： 割り込み外から設定すると、ダブルバッファでないと歯抜けになる場合がある。
//   メモ		： 割り込みハンドラから実行すること。Duty0%の場合モータを停止させる（PWMにひげが出る）
//   引数		： モータID、？？？？？
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void DCM_setPwmDuty( enDCM_ID en_id, USHORT us_duty10 )
{
	USHORT	us_cycle;							// 周期
	USHORT	us_onReg;							// 設定するON-duty
	
	/* PWM設定 */
	if( en_id == DCM_R ){				// 右
	
		if( 0 == us_duty10 ){			// Duty0%設定
			DCM_brakeMot( en_id );
		}
		else if( 1000 <= us_duty10 ){	// Duty100%

			DCM_R_TIMER = OFF;			// タイマ停止
			DCM_R_TCNT = 0;				// TCNT カウンタをクリア
			DCM_R_GRB = 5000;			// タイマ値変更
			DCM_R_TIORA = 6;			// TIOCA 端子の機能 : 初期出力は 1 出力。コンペアマッチで 1 出力
		    	DCM_R_TIORB = 6;			// TIOCB 端子の機能 : 初期出力は 1 出力。コンペアマッチで 1 出力
			DCM_R_TIMER = ON;			// タイマ開始
			us_duty10 = 1000;
		}
		else{
			us_cycle = DCM_R_GRA;		// 周期
			us_onReg = (USHORT)( (ULONG)us_cycle * (ULONG)us_duty10 / (ULONG)1000 );	// Duty2Reg 計算式
			DCM_R_TIMER = OFF;			// タイマ停止
			DCM_R_TCNT = 0;				// TCNT カウンタをクリア
			DCM_R_GRB = us_onReg;		// onDuty
			DCM_staMot( en_id );		// 回転開始
		}
	}
	else{								// 左

		if( 0 == us_duty10 ){			// Duty0%
			DCM_brakeMot( en_id );
		}
		else if( 1000 <= us_duty10 ){	// Duty100%

			DCM_L_TIMER = OFF;			// タイマ停止
			DCM_L_TCNT = 0;				// TCNT カウンタをクリア
			DCM_L_GRB = 5000;			// タイマ値変更
			DCM_L_TIORA = 6;			// TIOCA 端子の機能 : 初期出力は 1 出力。コンペアマッチで 1 出力
		    	DCM_L_TIORB = 6;			// TIOCB 端子の機能 : 初期出力は 1 出力。コンペアマッチで 1 出力
			DCM_L_TIMER = ON;			// タイマ開始
			us_duty10 = 1000;
		}
		else{
			us_cycle = DCM_L_GRA;		// 周期
			us_onReg = (USHORT)( (ULONG)us_cycle * (ULONG)us_duty10 / (ULONG)1000 );	// Duty2Reg 計算式
			DCM_L_TIMER = OFF;			// タイマ停止
			DCM_L_TCNT = 0;				// TCNT カウンタをクリア
			DCM_L_GRB = us_onReg;		// タイマ値変更
			DCM_staMot( en_id );		// 回転開始
		}
	}
}

// *************************************************************************
//   機能		： 制御を開始する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void CTRL_sta( void )
{
	uc_CtrlFlag = TRUE;
}

// *************************************************************************
//   機能		： 制御を停止する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void CTRL_stop( void )
{
	uc_CtrlFlag = FALSE;
	DCM_brakeMot( DCM_R );		// ブレーキ
	DCM_brakeMot( DCM_L );		// ブレーキ
}

// *************************************************************************
//   機能		： 制御データをクリアする
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void CTRL_clrData( void )
{
	ENC_clr();								// エンコーダモジュール初期化
	l_CntR			= 0;						// カウンタクリア
	l_CntL			= 0;						// カウンタクリア
	
	/* 現在値 */
	f_NowDist 		= 0;						// 移動距離リセット
	f_NowDistR 		= 0;
	f_NowDistL 		= 0;
	f_NowSpeed		= 0;						// [速度制御]   現在の速度 [mm/s]			（1[msec]毎に更新される）
	f_NowAngle		= 0;						// [角度制御]   現在角度					（1[msec]毎に更新される）
	s_GyroVal		= 0;						// ジャイロ値クリア
	f_GyroNowAngle	= 0;							// ジャイロ値クリア
	
	/* 目標値 */
	f_TrgtSpeed		= 0;						// [速度制御]   目標移動速度 [mm/s]			（1[msec]毎に更新される）
	f_TrgtDist 		= 0;						// [距離制御]   目標移動距離				（1[msec]毎に更新される）
	f_TrgtAngleS	= 0;							// [角速度制御] 目標角速度 [rad/s]			（1[msec]毎に更新される）
	f_TrgtAngle		= 0;						// [角度制御]   目標角度					（1[msec]毎に更新される）
	
	/* 制御データ */
	f_DistErrSum 	= 0;						// [距離制御]   距離積分制御のサム値			（1[msec]毎に更新される）
	f_AngleErrSum 	= 0;						// [角度制御]   角度積分制御のサム値			（1[msec]毎に更新される）
	f_ErrDistBuf	= 0;						// [壁制御]     距離センサーエラー値のバッファ		（1[msec]毎に更新される）
	f_ErrAngleSBuf  = 0;
}


// *************************************************************************
//   機能		： 制御データをセットする
//   注意		： なし
//   メモ		： なし
//   引数		： 制御データ
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void CTRL_setData( stCTRL_DATA* p_data )
{
	/* 制御方法 */
	en_Type					= p_data->en_type;

	/* 速度制御 */
	f_Acc 					= p_data->f_acc;
	f_BaseSpeed				= p_data->f_now;
	f_LastSpeed				= p_data->f_trgt;

	/* 距離制御 */
	f_BaseDist 				= p_data->f_nowDist;
	f_LastDist 				= p_data->f_dist;

	/* 角速度制御 */
	f_AccAngleS 			= p_data->f_accAngleS;
	f_BaseAngleS			= p_data->f_nowAngleS;
	f_LastAngleS			= p_data->f_trgtAngleS;

	/* 角度制御 */
	f_BaseAngle 			= p_data->f_nowAngle;
	f_LastAngle 			= p_data->f_angle;
	
	f_Time 					= 0;
	f_TrgtTime				= p_data->f_time;

	CTRL_sta();				// 制御開始
	
#if 0
	/* debug */
	printf(" [Type]%2d [Acc]%05.2f [BaseSpeed]%05.2f [LastSpeed]%05.2f [BaseDist]%05.2f [TrgtDist]%05.2f \n\r" ,
		en_Type,
		f_Acc,
		f_BaseSpeed,
		f_LastSpeed,
		f_BaseDist,
		f_LastDist
	);
	printf(" [AccA]%05.2f [BaseAngleS]%05.2f [LastAngleS]%05.2f [BaseAngle]%05.2f [LastAngle]%05.2f [Time]%05.4f\n\r\n\r" ,
		f_AccAngleS,
		f_BaseAngleS,
		f_LastAngleS,
		f_BaseAngle,
		f_LastAngle,
		f_TrgtTime
	);
#endif	
}

// *************************************************************************
//   機能		： 制御データを現在の状態に更新する 
//   注意		： CTRL_polからのみ実行可能。
//   メモ		： 1msec毎に実行される。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PRIVATE void CTRL_refNow( void )
{
	FLOAT f_speedR		= 0;							// 右モータ現在速度 [mm/s]
	FLOAT f_speedL		= 0;							// 左モータ現在速度 [mm/s]
	FLOAT f_r 			= F_CNT2MM(l_CntR);				// 右モータの進んだ距離 [mm]
	FLOAT f_l 			= F_CNT2MM(l_CntL);				// 左モータの進んだ距離 [mm]

	/* 速度更新 */
	f_speedR = f_r * 1000;								// 右モータ速度 [mm/s] ( 移動距離[カウント] * 1パルスの移動量(0.0509[mm]) * 1000(msec→sec) 
	f_speedL = f_l * 1000;								// 左モータ速度 [mm/s] ( 移動距離[カウント] * 1パルスの移動量(0.0509[mm]) * 1000(msec→sec) 
	f_NowSpeed  = ( f_speedR + f_speedL ) / 2;			// マウス（進行方向中心軸） [1mm/s] 
	
	/* 距離更新 */
	f_NowDistR += f_r;									// カウント更新
	f_NowDistL += f_l;									// カウント更新
	f_NowDist  = ( f_NowDistR + f_NowDistL ) / 2;		// 平均値更新
	
	
}


// *************************************************************************
//   機能		： 制御データを目標値に更新する
//   注意		： CTRL_polからのみ実行可能。
//   メモ		： 1msec毎に実行される。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
//		v1.3		2017.11.12			sato	スラローム追加
// *************************************************************************/
PUBLIC void CTRL_refTarget( void )
{
	/* 動作モードに応じる */
	switch( en_Type ){
	
		/* 加速中(直進) */
		case CTRL_ACC:
		case CTRL_SKEW_ACC:
			if( f_TrgtSpeed < f_LastSpeed ){												// 加速目標更新区間
				f_TrgtSpeed = f_BaseSpeed + f_Acc * f_Time;									// 目標速度
			}
			break;
		
		/* 等速中(直進) */
		case CTRL_CONST:
		case CTRL_SKEW_CONST:
			f_TrgtSpeed = f_BaseSpeed;														// 目標速度
			break;
		
		/* 減速中(直進) */
		case CTRL_DEC:
		case CTRL_SKEW_DEC:
			/* 速度制御 ＋ 位置制御 */
			if( f_TrgtSpeed > f_LastSpeed ){												// 減速目標更新区間
				f_TrgtSpeed = f_BaseSpeed - f_Acc * f_Time;									// 目標速度
				f_TrgtDist  = f_BaseDist + ( f_BaseSpeed + f_TrgtSpeed ) * f_Time / 2;		// 目標距離
			}
			/* 位置制御 */
			else{
				f_TrgtDist  = f_LastDist;													// 目標距離
			}
			break;
			
		/* 加速中(超信地旋回) */
		case CTRL_ACC_TRUN:
			
			/* 反時計回り */
			if( ( f_LastAngle > 0 ) && ( f_TrgtAngleS < f_LastAngleS ) ){
				f_TrgtAngleS = 0 + f_AccAngleS * f_Time;									// 目標角速度
			}
			/* 時計回り */
			else if( ( f_LastAngle < 0 ) && ( f_TrgtAngleS > f_LastAngleS ) ){
				f_TrgtAngleS = 0 - f_AccAngleS * f_Time;									// 目標角速度
			}
			break;

		/* 等速中(超信地旋回) */
		case CTRL_CONST_TRUN:
//			f_TrgtAngleS =f_BaseAngleS;
			break;

		/* 減速中(超信地旋回) */
		case CTRL_DEC_TRUN:
		
			/* 反時計回り */
			if( f_LastAngle > 0 ){ 
			
				/* 角速度制御 ＋ 角度制御 */
				if( f_TrgtAngleS > f_LastAngleS ){												// 減速目標更新区間
					f_TrgtAngleS = f_BaseAngleS - f_AccAngleS * f_Time;							// 目標角速度
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS + f_TrgtAngleS ) * f_Time / 2;	// 目標角度
				}
				/* 角度制御 */
				else{
					f_TrgtAngle  = f_LastAngle;													// 目標距離
				}
			}
			/* 時計回り */
			else{
				
				/* 角速度制御 ＋ 角度制御 */
				if( f_TrgtAngleS < f_LastAngleS ){												// 減速目標更新区間
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;							// 目標角速度
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS + f_TrgtAngleS ) * f_Time / 2;	// 目標角度
				}
				/* 角度制御 */
				else{
					f_TrgtAngle  = f_LastAngle;													// 目標距離
				}
			}
			break;
			
		/* スラローム前の前進動作(スラローム) */
		case CTRL_ENTRY_SURA:
			f_TrgtSpeed = f_BaseSpeed;
			if( f_TrgtDist <= f_LastDist ){
				f_TrgtDist  = f_BaseDist + f_TrgtSpeed * f_Time;								// 目標距離
			}
			break;

		/* 加速中(スラローム) */
		case CTRL_ACC_SURA:
			f_TrgtSpeed = f_BaseSpeed;

			/* 反時計回り */
			if( f_LastAngle > 0 ){ 
				/* 反時計回り */
				if( f_TrgtAngleS < f_LastAngleS ){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;							// 目標角速度
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS + f_TrgtAngleS ) * f_Time / 2;	// 目標角度
				}
				else{
					f_TrgtAngle  = f_LastAngle;													// 目標距離
				}
			}
			else{
				/* 時計回り */
				if( f_TrgtAngleS > f_LastAngleS ){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;							// 目標角速度
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS + f_TrgtAngleS ) * f_Time / 2;	// 目標角度
//					printf("%5.2f %5.2f %5.4f %5.2f\n\r",f_TrgtAngleS,f_AccAngleS,f_Time,f_TrgtAngle);
				}
				else{
					f_TrgtAngle  = f_LastAngle;													// 目標距離
				}
			}

			/* 位置制御 */
			if( f_LastDist > f_TrgtDist ){													// 目標更新区間
				f_TrgtDist  = f_BaseDist + f_TrgtSpeed * f_Time;							// 目標位置
			}
			else{
				f_TrgtDist  = f_LastDist;													// 目標距離
			}
			break;

		/* 等速中(スラローム) */
		case CTRL_CONST_SURA:
			f_TrgtSpeed = f_BaseSpeed;
			f_TrgtAngleS = f_BaseAngleS;							// 目標角速度

			/* 反時計回り */
			if( f_LastAngle > 0 ){ 
				/* 反時計回り */
				if( f_TrgtAngle < f_LastAngle ){
					f_TrgtAngle  = f_BaseAngle + f_TrgtAngleS * f_Time;			// 目標角度
				}
				else{
					f_TrgtAngle  = f_LastAngle;									// 目標角度
				}
			}
			else{
				/* 時計回り */
				if( f_TrgtAngle > f_LastAngle ){
					f_TrgtAngle  = f_BaseAngle + f_TrgtAngleS * f_Time;			// 目標角度
				}
				else{
					f_TrgtAngle  = f_LastAngle;									// 目標角度
				}
			}

			/* 位置制御 */
			if( f_LastDist > f_TrgtDist ){													// 目標更新区間
				f_TrgtDist  = f_BaseDist + f_TrgtSpeed * f_Time;							// 目標位置
			}
			else{
				f_TrgtDist  = f_LastDist;													// 目標距離
			}
			break;

		/* 減速中(スラローム) */
		case CTRL_DEC_SURA:
			f_TrgtSpeed = f_BaseSpeed;

			/* 反時計回り */
			if( f_LastAngle > 0 ){ 
				/* 反時計回り */
				if( f_TrgtAngleS > f_LastAngleS ){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;							// 目標角速度
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS + f_TrgtAngleS ) * f_Time / 2;	// 目標角度
				}
				else{
					f_TrgtAngle  = f_LastAngle;													// 目標距離
				}
			}
			else{
				/* 時計回り */
				if( f_TrgtAngleS < f_LastAngleS ){
					f_TrgtAngleS = f_BaseAngleS + f_AccAngleS * f_Time;							// 目標角速度
					f_TrgtAngle  = f_BaseAngle + ( f_BaseAngleS + f_TrgtAngleS ) * f_Time / 2;	// 目標角度
				}
				else{
					f_TrgtAngle  = f_LastAngle;													// 目標距離
				}
			}
			
			/* 速度制御 ＋ 位置制御 */
			if( f_LastDist > f_TrgtDist ){													// 目標更新区間
				f_TrgtDist  = f_BaseDist + f_TrgtSpeed * f_Time;							// 目標位置
			}
			else{
				f_TrgtDist  = f_LastDist;													// 目標距離
			}
			break;
			
		/* スラローム後の前進動作(スラローム) */
		case CTRL_EXIT_SURA:
			f_TrgtSpeed = f_BaseSpeed;
			f_TrgtAngleS = 0;
			if( f_TrgtDist <= f_LastDist ){
				f_TrgtDist  = f_BaseDist + f_TrgtSpeed * f_Time;								// 目標距離
			}
			else{
				f_TrgtDist  = f_LastDist;														// 目標距離
			}
			break;
		
		/* 上記以外のコマンド */
		default:
			break;
	}
}

// *************************************************************************
//   機能		： 制御方式からパラメータIDに変換する
//   注意		： 
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.04		sato			新規
// *************************************************************************/
PRIVATE enPARAM_MODE Chg_ParamID( enCTRL_TYPE en_type )
{
	switch( en_type ){
		case CTRL_ACC:			return PARAM_ACC;				// 加速中(直進)
		case CTRL_CONST:		return PARAM_CONST;				// 等速中(直進)
		case CTRL_DEC:			return PARAM_DEC;				// 減速中(直進)
		case CTRL_HIT_WALL:		return PARAM_HIT_WALL;			// 壁あて制御
//		case DCMC_BACK_ACC:		return PARAM_BACK_ACC;			// 加速中(後進)
//		case DCMC_BACK_CONST:		return PARAM_BACK_CONST;		// 等速中(後進)
//		case DCMC_BACK_DEC:		return PARAM_BACK_DEC;			// 減速中(後進)
		case CTRL_SKEW_ACC:		return PARAM_SKEW_ACC;			// 加速中(直進)
		case CTRL_SKEW_CONST:		return PARAM_SKEW_CONST;		// 等速中(直進)
		case CTRL_SKEW_DEC:		return PARAM_SKEW_DEC;			// 減速中(直進)
		case CTRL_ACC_TRUN:		return PARAM_ACC_TRUN;			// 加速中(超地信旋回)
		case CTRL_CONST_TRUN:		return PARAM_CONST_TRUN;		// 等速中(超地信旋回)
		case CTRL_DEC_TRUN:		return PARAM_DEC_TRUN;			// 減速中(超地信旋回)
		case CTRL_ENTRY_SURA:		return PARAM_ENTRY_SURA;		// スラローム前の前進動作(スラローム)
		case CTRL_ACC_SURA:		return PARAM_ACC_SURA;			// 加速中(スラローム)
		case CTRL_CONST_SURA:		return PARAM_CONST_SURA;		// 等速中(スラローム)
		case CTRL_DEC_SURA:		return PARAM_DEC_SURA;			// 減速中(スラローム)
		case CTRL_EXIT_SURA:		return PARAM_EXIT_SURA;			// スラローム後の前進動作(スラローム)
		default:			return PARAM_NC;
	}
}


// *************************************************************************
//   機能		： フィードフォアード量を取得する。
//   注意		： CTRL_polからのみ実行可能。
//   メモ		： 1msec毎に実行される。
//   			   機体の質量と加速度から必要なOnDutyのオフセットを演算できるようにする。
//   引数		： [出力] フィードフォアード制御量
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void CTRL_getFF_speed( FLOAT* p_err )
{
	FLOAT f_ff_speed_acc = 0.0f;
	FLOAT f_ff_speed = 0.0f;

	f_ff_speed_acc	= PARAM_getGain( Chg_ParamID(en_Type) )->f_FF_speed_acc;
	f_ff_speed		= PARAM_getGain( Chg_ParamID(en_Type) )->f_FF_speed;

	/* 動作モードに応じる */
	switch( en_Type ){
	
		// 加速 
		case CTRL_ACC:
		case CTRL_HIT_WALL:
		case CTRL_SKEW_ACC:
		case CTRL_ACC_TRUN:
		case CTRL_ACC_SURA:
			*p_err = f_Acc * f_ff_speed_acc + f_TrgtSpeed * f_ff_speed ;
			break;
			
		case CTRL_CONST:
		case CTRL_SKEW_CONST:
		case CTRL_CONST_TRUN:
		case CTRL_ENTRY_SURA:
		case CTRL_EXIT_SURA:
		case CTRL_CONST_SURA:
			*p_err = f_TrgtSpeed * f_ff_speed ;
			break;
		
		case CTRL_DEC:
		case CTRL_SKEW_DEC:
		case CTRL_DEC_TRUN:
		case CTRL_DEC_SURA:
			*p_err = f_Acc * f_ff_speed_acc * (-1) + f_TrgtSpeed * f_ff_speed;
			break;

		// 加速以外 
		default:
			*p_err = 0;
			break;										// 何もしない
	}
	
}

// *************************************************************************
//   機能		： フィードフォアード量を取得する。
//   注意		： CTRL_polからのみ実行可能。
//   メモ		： 1msec毎に実行される。
//   			   機体の質量と加速度から必要なOnDutyのオフセットを演算できるようにする。
//   引数		： [出力] フィードフォアード制御量
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void CTRL_getFF_angle( FLOAT* p_err )
{
	FLOAT f_ff_angleS_acc = 0.0f;
	FLOAT f_ff_angleS = 0.0f;

	f_ff_angleS_acc = PARAM_getGain( Chg_ParamID(en_Type) )->f_FF_angleS_acc;
	f_ff_angleS 	= PARAM_getGain( Chg_ParamID(en_Type) )->f_FF_angleS;

	/* 動作モードに応じる */
	switch( en_Type ){
	
		// 加速 
		case CTRL_ACC:
		case CTRL_HIT_WALL:
		case CTRL_SKEW_ACC:
		case CTRL_ACC_TRUN:
		case CTRL_ACC_SURA:
			*p_err =FABS(f_AccAngleS) * f_ff_angleS_acc + FABS(f_TrgtAngleS) * f_ff_angleS;
			break;
			
		case CTRL_CONST:
		case CTRL_SKEW_CONST:
		case CTRL_CONST_TRUN:
		case CTRL_ENTRY_SURA:
		case CTRL_EXIT_SURA:
		case CTRL_CONST_SURA:
			*p_err = FABS(f_TrgtAngleS) * f_ff_angleS;
			break;
		
		case CTRL_DEC:
		case CTRL_SKEW_DEC:
		case CTRL_DEC_TRUN:
		case CTRL_DEC_SURA:
			*p_err = FABS(f_AccAngleS) * f_ff_angleS_acc *(-1) + FABS(f_TrgtAngleS) * f_ff_angleS;
			break;

		// 加速以外 
		default:
			*p_err = 0;
			break;										// 何もしない
	}
	
}

// *************************************************************************
//   機能		： 速度フィードバック量を取得する。
//   注意		： CTRL_polからのみ実行可能。
//   メモ		： 1msec毎に実行される。
//   			   P制御を行う
//   引数		： [出力] 速度フィードバック制御量
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void CTRL_getSpeedFB( FLOAT* p_err )
{
	FLOAT		f_speedErr;					// [速度制御] 速度偏差
	FLOAT		f_kp = 0.0f;
	FLOAT		f_ki = 0.0f;
	FLOAT		f_kd = 0.0f;
	/* 速度制御 */
	f_speedErr  = f_TrgtSpeed - f_NowSpeed;					// 速度偏差[mm/s]
	f_kp = PARAM_getGain( Chg_ParamID(en_Type))->f_FB_speed_kp;
	f_ki = PARAM_getGain( Chg_ParamID(en_Type))->f_FB_speed_ki;
	f_kd = PARAM_getGain( Chg_ParamID(en_Type))->f_FB_speed_kd;
	
	/* I成分演算 */
	f_SpeedErrSum += f_speedErr * f_ki;			// I成分更新
	if( f_SpeedErrSum > 120 ){
		f_SpeedErrSum = 120;			// 上限リミッター
	}
	
	/* PID制御 */
	*p_err = f_speedErr * f_kp + f_SpeedErrSum + ( f_speedErr - f_ErrSpeedBuf ) * f_kd;				// PI制御量算出
	
	f_ErrSpeedBuf = f_speedErr;		// 偏差をバッファリング	
	
	/* 累積偏差クリア */
	if( FABS( f_speedErr ) < 0.5 ){
		f_SpeedErrSum = 0;
	}
	
}


// *************************************************************************
//   機能		： 距離フィードバック量を取得する。
//   注意		： CTRL_polからのみ実行可能。
//   メモ		： 1msec毎に実行される。
//   			   PI制御を行う
//   引数		： [出力] 距離フィードバック制御量
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void CTRL_getDistFB( FLOAT* p_err )
{
	FLOAT				f_distErr;					// [距離制御] 距離偏差
//	PRIVATE FLOAT		f_limTime = 0;				// 飽和状態維持時間[sec]
	FLOAT 				f_kp = 0.0f;				// 比例ゲイン
	FLOAT 				f_ki = 0.0f;				// 積分ゲイン
	
	*p_err = 0;

	/* 加速/等速の位置制御 */
	if( ( en_Type == CTRL_ACC ) || ( en_Type == CTRL_CONST )||
		( en_Type == CTRL_SKEW_ACC ) || ( en_Type == CTRL_SKEW_CONST ))
	{
		// なにもしない
	}
	/* 減速のみ位置制御 */
	else if(( en_Type == CTRL_DEC )|| ( en_Type == CTRL_SKEW_DEC ) ||
			 ( en_Type == CTRL_ENTRY_SURA ) || ( en_Type == CTRL_EXIT_SURA ) ||
			 ( en_Type == CTRL_ACC_SURA ) || ( en_Type == CTRL_CONST_SURA ) || ( en_Type == CTRL_DEC_SURA ) ){
		
		f_kp = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_dist_kp;
		f_ki = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_dist_ki;
		
		/* 位置制御 */
		f_distErr  = f_TrgtDist - f_NowDist;					// 距離偏差[mm]

		/* I成分演算 */
		f_DistErrSum += f_distErr * f_ki;			// I成分更新
		if( f_DistErrSum > 100 ){
			f_DistErrSum = 100;			// 上限リミッター
		}
		
		/* PI制御 */
		*p_err = f_distErr * f_kp + f_DistErrSum;				// PI制御量算出
		
		/* 累積偏差クリア */
		if( FABS( f_TrgtDist - f_NowDist ) < 0.05 ){
			f_DistErrSum = 0;
		}
	}

/* 超信地旋回 */
	else if( ( en_Type == CTRL_ACC_TRUN ) || ( en_Type == CTRL_CONST_TRUN ) || ( en_Type == CTRL_DEC_TRUN ) ){
		f_distErr  = f_TrgtDist - f_NowDist;					// 距離偏差[mm]
		*p_err = f_distErr * 0.1;								// P制御量算出
	}

}


// *************************************************************************
//   機能		： 角速度フィードバック量を取得する。
//   注意		： CTRL_polからのみ実行可能。
//   メモ		： 1msec毎に実行される。
//   			   P制御を行う
//   引数		： [出力] 角速度フィードバック制御量
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void CTRL_getAngleSpeedFB( FLOAT* p_err )
{
	FLOAT f_err;					// [入力] ジャイロセンサーエラー値
	FLOAT f_kp = 0.0f;				// 比例ゲイン
	FLOAT f_ki = 0.0f;				
	FLOAT f_kd = 0.0f;
	
	
	f_err = f_TrgtAngleS - GYRO_getSpeedErr();			// 目標角度 - ジャイロセンサ[deg/s]
	f_kp = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_angleS_kp;
	f_ki = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_angleS_ki;
	f_kd = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_angleS_kd;
	
	f_AngleSErrSum += f_err*f_ki;
	
	if(f_AngleSErrSum > 100){
		f_AngleSErrSum = 100;			//上限リミッター
	}
	else if(f_AngleSErrSum <-100){
		f_AngleSErrSum = -100;
	}
	
	templog2 = f_AngleSErrSum;
	*p_err = f_err * f_kp + f_AngleSErrSum + ( f_err - f_ErrAngleSBuf ) * f_kd;		// PID制御
		
	f_ErrAngleSBuf = f_err;		// 偏差をバッファリング	
	
	// 累積偏差クリア 
	if( FABS( f_err ) < 1 ){
		f_AngleSErrSum = 0;
	}

	// 累積偏差クリア 
//	if( en_Type == CTRL_DEC_SURA ){
//		f_AngleSErrSum = 0;
//	}
}


// *************************************************************************
//   機能		： 角度フィードバック量を取得する。
//   注意		： CTRL_polからのみ実行可能。
//   メモ		： 1msec毎に実行される。
//   			   P制御を行う
//   引数		： [出力] 角度フィードバック制御量
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
//      v1.1        2017.4.20     佐藤(I制御追加)
// *************************************************************************/
PUBLIC void CTRL_getAngleFB( FLOAT* p_err )
{
	FLOAT f_err;					// [入力] 角度偏差[deg]
	FLOAT f_kp = 0.0f;				// 比例ゲイン
	FLOAT f_ki = 0.0f;				// 積分ゲイン
	
	*p_err = 0;

	f_NowAngle = GYRO_getNowAngle();					// 現在角度[deg]

	f_err = f_TrgtAngle - f_NowAngle;
	/* 直進時 */
	if( ( en_Type == CTRL_ACC ) || ( en_Type == CTRL_CONST ) || ( en_Type == CTRL_DEC )|| 
		( en_Type == CTRL_ENTRY_SURA ) || ( en_Type == CTRL_EXIT_SURA )||
		( en_Type == CTRL_SKEW_ACC ) || ( en_Type == CTRL_SKEW_CONST ) || ( en_Type == CTRL_SKEW_DEC )
	){
		f_kp = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_angle_kp;
		f_ki = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_angle_ki;
		
		f_AngleErrSum += f_err*f_ki;	//I成分更新
		if(f_AngleErrSum > 200){
			f_AngleErrSum = 200;			//上限リミッター
		}
		else if(f_AngleErrSum <-200){
			f_AngleErrSum = -200;
		}
		
		//*p_err = f_err * FB_ANG_KP_GAIN;					// P制御量算出
		*p_err = f_err * f_kp + f_AngleErrSum;					// PI制御量算出
//		templog2 = f_AngleErrSum;

		/* 累積偏差クリア */
		if( FABS( f_TrgtAngle - f_NowAngle ) < 0.3 ){
			f_AngleErrSum = 0;
		}
		
	}
	
	/* 超信地旋回時減速 */
	else if(( en_Type == CTRL_DEC_TRUN )||
			 ( en_Type == CTRL_ACC_SURA ) || ( en_Type == CTRL_CONST_SURA ) || ( en_Type == CTRL_DEC_SURA ))
	{
		f_kp = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_angle_kp;
		f_ki = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_angle_ki;
		
		f_AngleErrSum += f_err*f_ki;	//I成分更新
		if(f_AngleErrSum > 200){
			f_AngleErrSum = 200;			//上限リミッター
		}
		else if(f_AngleErrSum <-200){
			f_AngleErrSum = -200;
		}
		
		//*p_err = f_err * FB_ANG_KP_GAIN;					// P制御量算出
		*p_err = f_err * f_kp + f_AngleErrSum;					// PI制御量算出
//		templog2 = f_AngleErrSum;

		/* 累積偏差クリア */
		if( FABS( f_TrgtAngle - f_NowAngle ) < 0.3 ){
			f_AngleErrSum = 0;
		}
	}

}


// *************************************************************************
//   機能		： 壁制御のフィードバック量を取得する。
//   注意		： CTRL_polからのみ実行可能。
//   メモ		： 1msec毎に実行される。
//   			   PD制御を行う
//   引数		： [出力] 壁制御のフィードバック制御量
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void CTRL_getSenFB( FLOAT* p_err )
{
	FLOAT f_err 	= 0;
	FLOAT f_kp 		= 0.0f;				// 比例ゲイン
	FLOAT f_kd 		= 0.0f;				// 微分ゲイン
	FLOAT gyro		= 0.0f;
	
	/* 直進時 */
	if( ( en_Type == CTRL_ACC ) || ( en_Type == CTRL_CONST ) || ( en_Type == CTRL_DEC )|| 
			 ( en_Type == CTRL_ENTRY_SURA ) || ( en_Type == CTRL_EXIT_SURA ) ){
		
		f_kp = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_wall_kp;
		f_kd = PARAM_getGain( Chg_ParamID(en_Type) )->f_FB_wall_kd;
		
		/* 偏差取得 */
		DIST_getErr( &l_WallErr );
		f_err = (FLOAT)l_WallErr;
//		templog2 = f_err;
		/* PD制御 */
		*p_err = f_err * f_kp + ( f_err - f_ErrDistBuf ) * f_kd;		// PD制御
		
		f_ErrDistBuf = f_err;		// 偏差をバッファリング
	}
	else if( ( en_Type == CTRL_SKEW_ACC ) || ( en_Type == CTRL_SKEW_CONST ) || ( en_Type == CTRL_SKEW_DEC ) ){
		
		DIST_getErrSkew( &l_WallErr );
		f_err = (FLOAT)l_WallErr;
		
//		*p_err = f_err * f_kp + ( f_err - f_ErrDistBuf ) * f_kd;		// PD制御
		*p_err = f_err;
	}

}


// *************************************************************************
//   機能		： FF/FB制御量からDCMに出力する。
//   注意		： CTRL_polからのみ実行可能。
//   メモ		： 1msec毎に実行される。
//   			   現在のバッテリ電圧に合わせたON-Dutyを演算して出力する。
//   引数		： 右モータのDuty比、左モータのDuty比
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PRIVATE void CTRL_outMot( FLOAT f_duty10_R, FLOAT f_duty10_L )
{
	FLOAT	f_temp;			// 計算用
	
	f_Duty_R = f_duty10_R;
	f_Duty_L = f_duty10_L;
	
	/* 電圧に応じてPWM出力を変更する */
	f_duty10_R = f_duty10_R * VCC_MAX / (BAT_getLv()/1000);
	f_duty10_L = f_duty10_L * VCC_MAX / (BAT_getLv()/1000);
	
//	f_Duty_R = f_duty10_R;
//	f_Duty_L = f_duty10_L;
	
//	log_in(f_duty10_R);
	/* 右モータ */
	if( 20 < f_duty10_R ){									// 前進
		DCM_setDirCw( DCM_R );
		DCM_setPwmDuty( DCM_R, (USHORT)f_duty10_R );
	}
	else if( f_duty10_R < -20 ){							// 後退
		f_temp = f_duty10_R * -1;
		DCM_setDirCcw( DCM_R );
		DCM_setPwmDuty( DCM_R, (USHORT)f_temp );
	}
	else{
		DCM_brakeMot( DCM_R );								// ブレーキ
	}

	/* 左モータ */
	if( 20 < f_duty10_L ){									// 前進
		DCM_setDirCcw( DCM_L );
		DCM_setPwmDuty( DCM_L, (USHORT)f_duty10_L );
	}
	else if( f_duty10_L < -20 ){							// 後退
		f_temp = f_duty10_L * -1;
		DCM_setDirCw( DCM_L );
		DCM_setPwmDuty( DCM_L, (USHORT)f_temp );
	}
	else{
		DCM_brakeMot( DCM_L );								// ブレーキ
	}
}


// *************************************************************************
//   機能		： 制御のポーリング関数
//   注意		： なし
//   メモ		： 割り込みから実行される。1msec毎に割り込み処理を行う。
//				： 主にDCMのフィードバックとフィードフォワード制御処理を行う。
//				： 制御は、直進方向の制御（速度/距離）と回転方向の制御（角速度/角度/距離センサ）の２つを実施する。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void CTRL_pol( void )
{
	FLOAT f_feedFoard_speed		= 0;		// [制御] フィードフォワード制御
	FLOAT f_feedFoard_angle		= 0;
	FLOAT f_speedCtrl			= 0;		// [制御] 速度制御量
	FLOAT f_distCtrl			= 0;		// [制御] 距離制御量
	FLOAT f_angleSpeedCtrl			= 0;		// [制御] 角速度制御量
	FLOAT f_angleCtrl			= 0;		// [制御] 角度制御量
	FLOAT f_distSenCtrl			= 0;		// [制御] 距離センサー制御量
	FLOAT f_duty10_R;						// [出力] 右モータPWM-DUTY比[0.1%]
	FLOAT f_duty10_L;						// [出力] 左モータPWM-DUTY比[0.1%]
	
	/* 制御を行うかのチェック */
	if( uc_CtrlFlag != TRUE ){
		 return;		// 制御無効状態
	}
	if(SW_ON == SW_INC_PIN){
		Failsafe_flag();
	}
	
	/* 制御不能 */
	if( SYS_isOutOfCtrl() == TRUE ){

		f_DistErrSum = 0;				// 累積偏差クリア
		f_NowDist = f_LastDist;			// 強制的に最終目標位置に変更
		f_NowAngle = f_LastAngle;		// 強制的に最終目標時間に変更
		f_Time = f_TrgtTime;			// 強制的に最終目標時間に変更
		
	 	CTRL_stop();				// 制御停止
		CTRL_clrData();					// データクリア
		DCM_brakeMot( DCM_R );			// ブレーキ
		DCM_brakeMot( DCM_L );			// ブレーキ
	}
	
	/* 各種センサ入力 */
	ENC_GetDiv( &l_CntR, &l_CntL );					// 移動量[カウント値]を取得
	CTRL_refNow();									// 制御に使用する値を現在の状態に更新
	CTRL_refTarget();								// 制御に使用する値を目標値に更新

	/* 制御値取得 */
	CTRL_getFF_speed( &f_feedFoard_speed );					// [制御] フィードフォワード
	CTRL_getFF_angle( &f_feedFoard_angle );					// [制御] フィードフォワード
	CTRL_getSpeedFB( &f_speedCtrl );				// [制御] 速度
	CTRL_getDistFB( &f_distCtrl );					// [制御] 距離
	CTRL_getAngleSpeedFB( &f_angleSpeedCtrl );			// [制御] 角速度
	CTRL_getAngleFB( &f_angleCtrl );				// [制御] 角度
	CTRL_getSenFB( &f_distSenCtrl );				// [制御] 壁
	
	templog1 = f_angleSpeedCtrl;
//	templog1 = f_distSenCtrl;
	
	/* 直進制御 */
	if( ( en_Type == CTRL_ACC ) || ( en_Type == CTRL_CONST ) || ( en_Type == CTRL_DEC ) ||( en_Type == CTRL_ENTRY_SURA ) || ( en_Type == CTRL_EXIT_SURA ) ||
		( en_Type == CTRL_SKEW_ACC ) || ( en_Type == CTRL_SKEW_CONST ) || ( en_Type == CTRL_SKEW_DEC )
	){
		f_duty10_R = f_feedFoard_speed * FF_BALANCE_R +  f_distCtrl + f_speedCtrl + f_angleCtrl + f_angleSpeedCtrl + f_distSenCtrl;	// 右モータPWM-DUTY比[0.1%]
		f_duty10_L = f_feedFoard_speed * FF_BALANCE_L +  f_distCtrl + f_speedCtrl - f_angleCtrl - f_angleSpeedCtrl - f_distSenCtrl;	// 左モータPWM-DUTY比[0.1%]
	}
	
	/* 壁あて制御 */
	else if( en_Type == CTRL_HIT_WALL ){
		f_duty10_R = f_feedFoard_speed * FF_HIT_BALANCE_R * (-1);																		// 右モータPWM-DUTY比[0.1%]
		f_duty10_L = f_feedFoard_speed * FF_HIT_BALANCE_L * (-1);
	}
	
	/* スラローム制御 */
	else if( ( en_Type == CTRL_ACC_SURA ) || (en_Type == CTRL_CONST_SURA)||( en_Type == CTRL_DEC_SURA ) ){
		/* 左旋回 */
		if( f_LastAngle > 0 ){
			f_duty10_R = f_feedFoard_speed * FF_BALANCE_R + f_feedFoard_angle * FF_BALANCE_R + f_angleCtrl + f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// 右モータPWM-DUTY比[0.1%]
			f_duty10_L = f_feedFoard_speed * FF_BALANCE_L + f_feedFoard_angle * FF_BALANCE_L * (-1) - f_angleCtrl - f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// 左モータPWM-DUTY比[0.1%]
		}
		/*右旋回 */
		else{
			f_duty10_R = f_feedFoard_speed * FF_BALANCE_R + f_feedFoard_angle * FF_BALANCE_R * (-1) + f_angleCtrl + f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// 右モータPWM-DUTY比[0.1%]
			f_duty10_L = f_feedFoard_speed * FF_BALANCE_L + f_feedFoard_angle * FF_BALANCE_L - f_angleCtrl - f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// 左モータPWM-DUTY比[0.1%]
		}
	}
		
	
	/* 超信地旋回 */
	else{
		/* 左旋回 */
		if( f_LastAngle > 0 ){
//			f_duty10_R = f_feedFoard * FF_BALANCE_R        + f_angleCtrl + f_angleSpeedCtrl;									// 右モータPWM-DUTY比[0.1%]
//			f_duty10_L = f_feedFoard * FF_BALANCE_L * (-1) - f_angleCtrl - f_angleSpeedCtrl;									// 左モータPWM-DUTY比[0.1%]
			f_duty10_R = f_feedFoard_angle * FF_BALANCE_R        + f_angleCtrl + f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// 右モータPWM-DUTY比[0.1%]
			f_duty10_L = f_feedFoard_angle * FF_BALANCE_L * (-1) - f_angleCtrl - f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// 左モータPWM-DUTY比[0.1%]
		}
		/* 右旋回 */
		else{
//			f_duty10_R = f_feedFoard * FF_BALANCE_R * (-1) + f_angleCtrl + f_angleSpeedCtrl;									// 右モータPWM-DUTY比[0.1%]
//			f_duty10_L = f_feedFoard * FF_BALANCE_L        - f_angleCtrl - f_angleSpeedCtrl;									// 左モータPWM-DUTY比[0.1%]
			f_duty10_R = f_feedFoard_angle * FF_BALANCE_R * (-1) + f_angleCtrl + f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// 右モータPWM-DUTY比[0.1%]
			f_duty10_L = f_feedFoard_angle * FF_BALANCE_L        - f_angleCtrl - f_angleSpeedCtrl +  f_distCtrl + f_speedCtrl;		// 左モータPWM-DUTY比[0.1%]
		}
	}
	
	CTRL_outMot( f_duty10_R, f_duty10_L );				// モータへ出力

	f_Time += 0.001;
	
	/* 壁切れチェック */
	if( MOT_getWallEdgeType() == MOT_WALL_EDGE_RIGHT ){
		
		/* 壁抜け */
		if( DIST_isWall_R_SIDE() == FALSE ){
			
			MOT_setWallEdge( TRUE );		// 壁の切れ目を検知
		}
	}
	else if( MOT_getWallEdgeType() == MOT_WALL_EDGE_LEFT ){
		
		/* 壁抜け */
		if( DIST_isWall_L_SIDE() == FALSE ){
			
			MOT_setWallEdge( TRUE );		// 壁の切れ目を検知
		}
	}
//	log_write(GYRO_getSpeedErr(),f_NowAngle,CTRL_getAngleSpeedFB,f_duty10_R);
//	log_write(GYRO_getSpeedErr());
}


// *************************************************************************
//   機能		： 加速度を取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 加速度[mm/s2]
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC FLOAT MOT_getAcc1( void )
{
	return PARAM_getSpeed( PARAM_ST )->f_acc;
}


// *************************************************************************
//   機能		： 減速度を取得する
//   注意		： プラスで指定
//   メモ		： なし
//   引数		： なし
//   返り値		： 加速度[mm/s2]
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC FLOAT MOT_getAcc3( void )
{
	return PARAM_getSpeed( PARAM_ST )->f_dec;
}


// *************************************************************************
//   機能		： 直進する　
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PRIVATE void MOT_goBlock_AccConstDec( FLOAT f_fin, enMOT_ST_TYPE en_type, enMOT_GO_ST_TYPE en_goType )
{

	stCTRL_DATA		st_data;					// 制御データ
//	printf("目標速度 %f 目標位置 %f\r\n",st_Info.f_trgt,st_Info.f_dist);
	GYRO_staErrChkAngle();
	
	/* ================ */
	/*      実動作      */
	/* ================ */
	/* ------ */
	/*  加速  */
	/* ------ */
	if( ( en_type != MOT_CONST_DEC ) && ( en_type != MOT_CONST_DEC_CUSTOM ) ){

		if( MOT_GO_ST_NORMAL == en_goType ){
			st_data.en_type		= CTRL_ACC;
		}
		else{
			st_data.en_type		= CTRL_SKEW_ACC;
		}
		st_data.f_acc			= st_Info.f_acc1;		// 加速度指定
		st_data.f_now			= st_Info.f_now;		// 現在速度
		st_data.f_trgt			= st_Info.f_trgt;		// 目標速度
		st_data.f_nowDist		= 0;				// 進んでいない
		st_data.f_dist			= st_Info.f_l1;			// 加速距離
		st_data.f_accAngleS		= 0;				// 角加速度
		st_data.f_nowAngleS		= 0;				// 現在角速度
		st_data.f_trgtAngleS		= 0;				// 目標角度
		st_data.f_nowAngle		= 0;				// 現在角度
		st_data.f_angle			= 0;				// 目標角度
		st_data.f_time 			= 0;				// 目標時間 [sec] ← 指定しない
		CTRL_clrData();								// 設定データをクリア
		CTRL_setData( &st_data );						// データセット
//		printf("目標速度 %f 目標位置 %f \r\n",st_data.f_trgt,st_data.f_dist);
		DCM_staMotAll();							// モータON
		while( f_NowDist < st_Info.f_l1 ){					// 指定距離到達待ち
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
			MOT_setWallEdgeDist();
		}
//		LED4 = 0x01;
//		printf("現在位置 %f \r\n",f_NowDist);
	}
	
	/* ------ */
	/*  等速  */
	/* ------ */
	if( MOT_GO_ST_NORMAL == en_goType ){
		st_data.en_type		= CTRL_CONST;
	}
	else{
		st_data.en_type		= CTRL_SKEW_CONST;
	}
	st_data.f_acc			= 0;					// 加速度指定
	st_data.f_now			= st_Info.f_trgt;			// 現在速度
	st_data.f_trgt			= st_Info.f_trgt;			// 目標速度
	st_data.f_nowDist		= st_Info.f_l1;				// 現在位置
	st_data.f_dist			= st_Info.f_l1_2;			// 等速完了位置
	st_data.f_accAngleS		= 0;					// 角加速度
	st_data.f_nowAngleS		= 0;					// 現在角速度
	st_data.f_trgtAngleS		= 0;					// 目標角度
	st_data.f_nowAngle		= 0;					// 現在角度
	st_data.f_angle			= 0;					// 目標角度
	st_data.f_time 			= 0;					// 目標時間 [sec] ← 指定しない
	if( ( en_type == MOT_CONST_DEC ) || ( en_type == MOT_CONST_DEC_CUSTOM ) ){
		CTRL_clrData();										// 設定データをクリア
	}
	CTRL_setData( &st_data );						// データセット
//	printf("目標速度 %f 目標位置 %f \r\n",st_data.f_trgt,st_data.f_dist);
	while( f_NowDist < st_Info.f_l1_2 ){				// 指定距離到達待ち
		if( SYS_isOutOfCtrl() == TRUE ){
			CTRL_stop(); 
			DCM_brakeMot( DCM_R );		// ブレーキ
			DCM_brakeMot( DCM_L );		// ブレーキ
			break;
		}				// 途中で制御不能になった
		MOT_setWallEdgeDist();
	}
//	printf("現在位置 %f \r\n",f_NowDist);
//	LED4 = 0x02;

	/* ------ */
	/*  減速  */
	/* ------ */
	if( ( en_type != MOT_ACC_CONST ) && ( en_type != MOT_ACC_CONST_CUSTOM ) ){

		if( MOT_GO_ST_NORMAL == en_goType ){
			st_data.en_type		= CTRL_DEC;
		}
		else{
			st_data.en_type		= CTRL_SKEW_DEC;
		}
		st_data.f_acc			= st_Info.f_acc3;			// 減速
		st_data.f_now			= st_Info.f_trgt;			// 現在速度
		st_data.f_trgt			= st_Info.f_last;			// 最終速度
		st_data.f_nowDist		= st_Info.f_l1_2;			// 等速完了位置
		st_data.f_dist			= st_Info.f_dist;			// 全移動完了位置
		st_data.f_accAngleS		= 0;						// 角加速度
		st_data.f_nowAngleS		= 0;						// 現在角速度
		st_data.f_trgtAngleS		= 0;						// 目標角度
		st_data.f_nowAngle		= 0;						// 現在角度
		st_data.f_angle			= 0;						// 目標角度
		st_data.f_time 			= 0;						// 目標時間 [sec] ← 指定しない
		CTRL_setData( &st_data );							// データセット
//		printf("目標速度 %f 目標位置 %f \r\n",st_data.f_trgt,st_data.f_dist);
		while( f_NowDist < ( st_Info.f_dist - 0.2 ) ){		// 指定距離到達待ち
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
			MOT_setWallEdgeDist();
		}
//		printf("現在位置 %f \r\n",f_NowDist);
//		LED4 = 0x04;
	}
	
	/* -------------------- */
	/*  等速（壁の切れ目）  */
	/* -------------------- */
	/* 壁切れがまだ見つからない状態（壁切れ設定をしているのに、エッジを見つけていない） */
	if( ( en_WallEdge != MOT_WALL_EDGE_NONE ) && ( bl_IsWallEdge == FALSE )  ){
	
		st_data.en_type			= CTRL_CONST;
		st_data.f_acc			= 0;						// 加速度指定
		st_data.f_now			= st_Info.f_last;			// 現在速度
		st_data.f_trgt			= st_Info.f_last;			// 目標速度
		st_data.f_nowDist		= f_NowDist;				// 現在位置
		st_data.f_dist			= f_NowDist + 180.0f;		// 等速完了位置（180.0f：壁切れをどこまで救うかの距離）、ここではf_NowDistをクリアしてはいけない。
		st_data.f_accAngleS		= 0;						// 角加速度
		st_data.f_nowAngleS		= 0;						// 現在角速度
		st_data.f_trgtAngleS	= 0;						// 目標角度
		st_data.f_nowAngle		= 0;						// 現在角度
		st_data.f_angle			= 0;						// 目標角度
		st_data.f_time 			= 0;						// 目標時間 [sec] ← 指定しない
		CTRL_clrData();										// マウスの現在位置/角度をクリア
		CTRL_setData( &st_data );							// データセット
		while( f_NowDist < st_data.f_dist ){				// 指定距離到達待ち
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
			if( MOT_setWallEdgeDist_LoopWait() == TRUE ) break;	// 壁切れ補正を実行する距離を設定
		}
	}
	/* 壁切れまで直進動作を行う */
	if( ( MOT_GO_ST_NORMAL == en_goType ) &&				// 直進時に追加動作が必要な場合にしか実施しない
		( f_WallEdgeAddDist != 0.0f ) &&
		( f_fin != 0.0f )
	){
		st_data.en_type			= CTRL_CONST;
		st_data.f_acc			= 0;						// 加速度指定
		st_data.f_now			= st_Info.f_last;			// 現在速度
		st_data.f_trgt			= st_Info.f_last;			// 目標速度
		st_data.f_nowDist		= 0;						// 現在位置
		st_data.f_dist			= f_WallEdgeAddDist;		// 等速完了位置
		st_data.f_accAngleS		= 0;						// 角加速度
		st_data.f_nowAngleS		= 0;						// 現在角速度
		st_data.f_trgtAngleS	= 0;						// 目標角度
		st_data.f_nowAngle		= 0;						// 現在角度
		st_data.f_angle			= 0;						// 目標角度
		st_data.f_time 			= 0;						// 目標時間 [sec] ← 指定しない
		CTRL_clrData();										// マウスの現在位置/角度をクリア
		CTRL_setData( &st_data );							// データセット
		while( f_NowDist < st_data.f_dist ){				// 指定距離到達待ち
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
		}
	}
	
	/* 停止 */
	if( 0.0f == f_fin ){
		TIME_wait(100);			// 安定待ち
	 	CTRL_stop();				// 制御停止
		DCM_brakeMot( DCM_R );		// ブレーキ
		DCM_brakeMot( DCM_L );		// ブレーキ
	}
	
	f_MotNowSpeed = f_fin;			// 現在速度更新
	GYRO_endErrChkAngle();
}


// *************************************************************************
//   機能		： 区画 前進、動作データ作成（台形加速）
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PRIVATE void MOT_setData_ACC_CONST_DEC( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_l3;						// 第3移動距離[mm]
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}
	
	/* 加速度 */
	st_Info.f_acc1 		= MOT_getAcc1();								// 加速度1[mm/s^2]
	st_Info.f_acc3 		= MOT_getAcc3();								// 加速度3[mm/s^2]

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;								// 現在速度	
	st_Info.f_trgt		= f_MotTrgtSpeed;								// 目標速度
	st_Info.f_last		= f_fin;									// 最終速度
	
	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist - f_num * sliplengs;												// 移動距離[mm]
	st_Info.f_l1		= ( f_MotTrgtSpeed * f_MotTrgtSpeed - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// 第1移動距離[mm]
	f_l3			= ( f_fin * f_fin - f_MotTrgtSpeed * f_MotTrgtSpeed ) / ( ( st_Info.f_acc3 * -1 ) * 2 );			// 第3移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist - f_l3;											// 第1+2移動距離[mm]

//	printf("1 %f,%f\r",st_Info.f_trgt,st_Info.f_l1);
}


// *************************************************************************
//   機能		： 区画 前進、動作データ作成（台形加速（等速））
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST_DEC_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_l3;						// 第3移動距離[mm]
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 加速度 */
	st_Info.f_acc1 		= MOT_getAcc1();								// 加速度1[mm/s^2]
	st_Info.f_acc3 		= MOT_getAcc3();								// 加速度3[mm/s^2]


	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist - f_num * sliplengs;												// 移動距離[mm]

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;												// 現在速度	
	st_Info.f_last		= f_fin;													// 最終速度
	st_Info.f_trgt		= sqrt( 1 / ( ( st_Info.f_acc3 * -1 ) - st_Info.f_acc1 ) *					
					( 2 * st_Info.f_acc1 * ( st_Info.f_acc3 * -1 ) * ( st_Info.f_dist - MOT_MOVE_ST_MIN ) + 
					( st_Info.f_acc3 * -1 ) * f_MotNowSpeed * f_MotNowSpeed - st_Info.f_acc1 * f_fin * f_fin ) );

	st_Info.f_l1		= ( st_Info.f_trgt * st_Info.f_trgt - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// 第1移動距離[mm]
	f_l3			= ( f_fin * f_fin - st_Info.f_trgt * st_Info.f_trgt ) / ( ( st_Info.f_acc3  * -1 ) * 2 );			// 第3移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist - f_l3;											// 第1+2移動距離[mm]

//	printf("2 %f,%f,%f,%f\r",st_Info.f_trgt,st_Info.f_l1,f_fin,f_MotNowSpeed);
}


// *************************************************************************
//   機能		： 区画 前進、動作データ作成（加速＋等速）
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 加速度 */
	st_Info.f_acc1 		= MOT_getAcc1();													// 加速度1[mm/s^2]
	st_Info.f_acc3 		= 0;																// 加速度3[mm/s^2](未使用)

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_trgt		= f_fin;															// 目標速度
	st_Info.f_last		= 0;																// 最終速度(未使用)

	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist - f_num * sliplengs;												// 移動距離[mm]
	st_Info.f_l1		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// 第1移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist;													// 第1+2移動距離[mm]
}


// *************************************************************************
//   機能		： 区画 前進、動作データ作成（加速＋等速（等速））
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PRIVATE void MOT_setData_MOT_ACC_CONST_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_trgt		= f_fin;															// 目標速度
	st_Info.f_last		= 0;																// 最終速度(未使用)

	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist - f_num * sliplengs;												// 移動距離[mm]

	/* 加速度 */
	st_Info.f_acc1 		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_dist - MOT_MOVE_ST_MIN ) * 2.0f );	// 加速度1[mm/s^2]（強制的に書き換え）
	st_Info.f_acc3 		= 0;																// 加速度3[mm/s^2](未使用)

	/* 距離 */
	st_Info.f_l1		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( st_Info.f_acc1 * 2 );			// 第1移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist;													// 第1+2移動距離[mm]
}


// *************************************************************************
//   機能		： 区画 前進、動作データ作成（等速＋減速）
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PRIVATE void MOT_setData_MOT_CONST_DEC( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 加速度 */
	st_Info.f_acc1 		= 0;																// 加速度1[mm/s^2](未使用)
	st_Info.f_acc3 		= MOT_getAcc3();													// 加速度3[mm/s^2]

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;													// 現在速度	
	st_Info.f_trgt		= f_MotNowSpeed;													// 目標速度
	st_Info.f_last		= f_fin;															// 最終速度(未使用)

	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist - f_num * sliplengs;												// 移動距離[mm]
	st_Info.f_l1		= 0;																// 第1移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist - ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_acc3 * -1 ) * 2 );			// 第1-2移動距離[mm]
}


// *************************************************************************
//   機能		： 区画 前進、動作データ作成（等速（等速）＋減速）
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PRIVATE void MOT_setData_MOT_CONST_DEC_CUSTOM( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT			f_1blockDist;				// 1区画の距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の直進
		f_1blockDist = BLOCK;
	}
	else{									// 斜めの直進
		f_1blockDist = BLOCK_SKEW;
	}

	/* 速度 */
	st_Info.f_now		= f_MotNowSpeed;									// 現在速度	
	st_Info.f_trgt		= f_MotNowSpeed;									// 目標速度
	st_Info.f_last		= f_fin;															// 最終速度

	/* 距離 */
	st_Info.f_dist		= f_num * f_1blockDist - f_num * sliplengs;									// 移動距離[mm]

	/* 加速度 */
	st_Info.f_acc1 		= 0;																// 加速度1[mm/s^2](未使用)
	st_Info.f_acc3 		= ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_dist - MOT_MOVE_ST_MIN ) * 2.0f ) * -1;	// 加速度3[mm/s^2]（強制的に書き換え）

	/* 距離 */
	st_Info.f_l1		= 0;																// 第1移動距離[mm]
	st_Info.f_l1_2		= st_Info.f_dist - ( f_fin * f_fin - f_MotNowSpeed * f_MotNowSpeed ) / ( ( st_Info.f_acc3 * -1 ) * 2 );			// 第1-2移動距離[mm]
}


// *************************************************************************
//   機能		： 前進のタイプを取得する
//   注意		： MOT_go_FinSpeedのみ実行可能
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。斜め走行と共通処理。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： 前進のタイプ
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PRIVATE enMOT_ST_TYPE MOT_getStType( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_type )
{
	FLOAT f_v1Div;
	FLOAT f_v3Div;
	FLOAT f_acc1;
	FLOAT f_acc3;
	FLOAT f_t1;
	FLOAT f_t3;
	FLOAT f_l1;							//加速距離
	FLOAT f_l3;							//減速距離
	FLOAT f_total;							// 移動距離[mm]
	
	/* 1区画の距離 */
	if( MOT_GO_ST_NORMAL == en_type ){		// 通常の直進
		f_total	= f_num * BLOCK;
	}
	else{									// 斜めの直進
		f_total	= f_num * BLOCK_SKEW;
	}
	

	/* ================ */
	/*  加速＋等速動作  */
	/* ================ */
	f_v1Div		= f_fin - f_MotNowSpeed;
	f_acc1		= MOT_getAcc1();				// 加速度1[mm/s^2]
	f_t1		= f_v1Div / f_acc1;

	f_l1 = ( f_MotNowSpeed + f_fin ) * 0.5f * f_t1;

	/* 加速＋等速動作 */
	if( f_total <= ( f_l1 + MOT_MOVE_ST_THRESHOLD ) ){

		/* 加速が最終速度に対して完了しない */
		if( f_total < ( f_l1 + MOT_MOVE_ST_MIN ) ){
//			printf("パターン4\n\r");
			return MOT_ACC_CONST_CUSTOM;		// パターン4（強制的に加速度を変更する）
		}
		else{
//			printf("パターン3\n\r");
			return MOT_ACC_CONST;				// パターン3（加速＋等速）
		}
	}

	/* ================ */
	/*  等速＋減速動作  */
	/* ================ */
	f_v3Div		= f_fin - f_MotNowSpeed;
	f_acc3		= MOT_getAcc3();				// 加速度3[mm/s^2]
	f_t3		= f_v3Div / ( f_acc3 * -1 );

	f_l3 = ( f_MotNowSpeed + f_fin ) * 0.5f * f_t3;

	/* 等速＋減速動作 */
	if( f_total <= ( f_l3 + MOT_MOVE_ST_THRESHOLD ) ){

		/* 減速が最終速度に対して完了しない */
		if( f_total < ( f_l3 + MOT_MOVE_ST_MIN ) ){
//			printf("パターン6\n\r");
			return MOT_CONST_DEC_CUSTOM;		// パターン6（強制的に加速度を変更する）
		}
		else{
//			printf("パターン5\n\r");
			return MOT_CONST_DEC;				// パターン5（等速＋減速）
		}
	}

	/* ========== */
	/*  台形動作  */
	/* ========== */
	f_v1Div		= f_MotTrgtSpeed - f_MotNowSpeed;					// 台形時の速度差
	f_t1		= f_v1Div / f_acc1;
	f_l1		= ( f_MotNowSpeed + f_MotTrgtSpeed ) * 0.5f * f_t1;

	f_v3Div		= f_fin - f_MotTrgtSpeed;							// 台形時の速度差
	f_acc3		= MOT_getAcc3();									// 加速度3[mm/s^2]
	f_t3		= -1.0f * f_v3Div / f_acc3;							// 減速時の所要時間
	f_l3		= ( f_MotTrgtSpeed + f_fin ) * 0.5f * f_t3;

	/* 通常の台形動作 */
	if( ( f_total - f_l1 - f_l3 - MOT_MOVE_ST_MIN) >= 0 ){
//		printf("パターン1\n\r");
		return MOT_ACC_CONST_DEC;				// パターン1（通常）
	}
	/* 等速値を変更する */
	else{
//		printf("パターン2\n\r");
		return MOT_ACC_CONST_DEC_CUSTOM;		// パターン2（目標速度を変更）
	}
}


// *************************************************************************
//   機能		： 区画前進
//   注意		： なし
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。
//   引数		： 区画数、最終速度、直進タイプ（通常or斜め）
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PRIVATE void MOT_go_FinSpeed( FLOAT f_num, FLOAT f_fin, enMOT_GO_ST_TYPE en_goStType )
{
	enMOT_ST_TYPE 		en_type 		= MOT_getStType( f_num, f_fin, en_goStType);			// 動作パターン取得

	/* 移動距離と指定値に応じで動作を変える */
	switch( en_type ){
	
		case MOT_ACC_CONST_DEC:				// [01] 台形加速
			MOT_setData_ACC_CONST_DEC( f_num, f_fin, en_goStType );					// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
	
		case MOT_ACC_CONST_DEC_CUSTOM:		// [02] 台形加速（等速）
			MOT_setData_MOT_ACC_CONST_DEC_CUSTOM( f_num, f_fin, en_goStType );		// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
	
		case MOT_ACC_CONST:				// [03] 加速＋等速
			MOT_setData_MOT_ACC_CONST( f_num, f_fin, en_goStType );					// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
	
		case MOT_ACC_CONST_CUSTOM:		// [04] 加速＋等速（等速）
			MOT_setData_MOT_ACC_CONST_CUSTOM( f_num, f_fin, en_goStType );			// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, MOT_GO_ST_NORMAL );			// 動作
			break;

		case MOT_CONST_DEC:				// [05] 等速＋減速
			MOT_setData_MOT_CONST_DEC( f_num, f_fin, en_goStType );					// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
	
		case MOT_CONST_DEC_CUSTOM:		// [06] 等速＋減速（減速値変更）
			MOT_setData_MOT_CONST_DEC_CUSTOM( f_num, f_fin, en_goStType );			// 動作データ作成
			MOT_goBlock_AccConstDec( f_fin, en_type, en_goStType );					// 動作
			break;
	
		default:
			break;
	}

#if 0
	printf(" \n\r [type]%d [acc1]%06.2f [acc3]%06.2f [dist]%06.2f [l1]%06.2f [l1_2]%06.2f [l_last]%06.2f [now]%06.2f [trgt]%06.2f \n\r", 
		en_type,
		st_Info.f_acc1,
		st_Info.f_acc3,
		st_Info.f_dist,
		st_Info.f_l1,
		st_Info.f_l1_2,
		st_Info.f_last,
		st_Info.f_now,
		st_Info.f_trgt
	);
#endif
}


// *************************************************************************
//   機能		： 区画前進（通常）
//   注意		： なし
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。☆
//   引数		： 区画数、最終速度
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void MOT_goBlock_FinSpeed( FLOAT f_num, FLOAT f_fin )
{
	MOT_go_FinSpeed( f_num, f_fin, MOT_GO_ST_NORMAL );		// 通常の直進
}

// *************************************************************************
//   機能		： 区画前進（斜め）
//   注意		： なし
//   メモ		： 区画数は1.5区画などの指定をしたい場合は、"1.5"を指定する。☆
//   引数		： 区画数、最終速度
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void MOT_goSkewBlock_FinSpeed( FLOAT f_num, FLOAT f_fin )
{
	MOT_go_FinSpeed( f_num, f_fin, MOT_GO_ST_SKEW );		// 通常の直進
}

// *************************************************************************
//   機能		： テスト走行用プログラム
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.3.28			翔			新規
// *************************************************************************/
PUBLIC void testrun(void)
{
	stCTRL_DATA test;
		test.en_type=CTRL_ACC;
		test.f_acc=MOT_getAcc1();
		test.f_trgt	= 0;
		test.f_now = 0;
		test.f_nowDist = 0;
		test.f_dist = 0;

		
		
		
	CTRL_clrData();
	CTRL_setData(&test);
	
	
}

// *************************************************************************
//   機能		： ジャイロテストプログラム
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.4.18			翔			新規
// *************************************************************************/
PUBLIC float GYRO_test(void)
{
	return s_GyroVal;
}


// *************************************************************************
//   機能		： 走行距離プログラム
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.6.1			翔			新規
// *************************************************************************/
PUBLIC float dist_check(void)
{
	return f_NowDist;

}

/****************************************************************************//*!
//   機能		： マウスの角加速度1を取得する
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
 *//**************************    履    歴    *******************************//*!
 *  @date      2012.09.05    外川      新規       
 *******************************************************************************/
PUBLIC FLOAT MOT_getAccAngle1( void )
{
//	return ( 1800 );
	return PARAM_getSpeed( PARAM_TRUN )->f_accAngle;
}


/****************************************************************************//*!
//   機能		： マウスの角加速度2を取得する
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
 *//**************************    履    歴    *******************************//*!
 *  @date       2012.09.05    外川      新規       
 *******************************************************************************/
PUBLIC FLOAT MOT_getAccAngle3( void )
{
//	return ( 1800 );
	return PARAM_getSpeed( PARAM_TRUN )->f_decAngle;
}



// *************************************************************************
//   機能		： 超信地旋回
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.8.14		翔	新規
//		v1.1		2017.09.19			第一距離を変更
// *************************************************************************/
PUBLIC void MOT_turn( enMOT_TURN_CMD en_type )
{
	stMOT_DATA	st_info;	//シーケンスデータ
	stCTRL_DATA	st_data;	//制御データ
//	FLOAT		f_angle2 = A2_MIN;	//最低第2移動角度[rad]
	FLOAT		f_angle1;	//第1移動角度[rad]
	FLOAT		f_angle3;	//第3移動角度[rad]
	FLOAT		us_trgtAngleS;	//目標角度[rad/s]
	
	us_trgtAngleS = 300;
	
	/* ---------------- */
	/*  動作データ計算  */
	/* ---------------- */
	/* 加速度 */
	st_info.f_accAngleS1= MOT_getAccAngle1();												// 角加速度1[rad/s^2]
	st_info.f_accAngleS3= MOT_getAccAngle3();												// 角加速度3[rad/s^2]

	/* 角速度 */
	st_info.f_nowAngleS	= 0;																// 現在角速度
	st_info.f_trgtAngleS= (FLOAT)us_trgtAngleS;												// 目標角速度
	st_info.f_lastAngleS= 0;																// 最終角速度

	/* 角度 */
	switch( en_type ){
		case MOT_R90:	st_info.f_angle =  -90 - ANGLE_OFFSET1_R;	break;					// 回転角度[rad]
		case MOT_L90:	st_info.f_angle =   90 + ANGLE_OFFSET1;		break;					// 回転角度[rad]
		case MOT_R180:	st_info.f_angle = -180 - ANGLE_OFFSET2_R;	break;					// 回転角度[rad]
		case MOT_L180:	st_info.f_angle =  180 + ANGLE_OFFSET2;		break;					// 回転角度[rad]
		case MOT_R360:	st_info.f_angle = -360 - ANGLE_OFFSET3;		break;					// 回転角度[rad]
		case MOT_L360:	st_info.f_angle =  360 + ANGLE_OFFSET3;		break;					// 回転角度[rad]
	}
	f_angle3 = ( st_info.f_trgtAngleS - st_info.f_lastAngleS ) / 2 * ( st_info.f_trgtAngleS - st_info.f_lastAngleS ) / st_info.f_accAngleS3;						// 第3移動角度[rad]
	f_angle1 = ( 0 - st_info.f_trgtAngleS) / 2 * ( 0 - st_info.f_trgtAngleS ) / st_info.f_accAngleS1;
	
	
	if( ( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 ) ){		// -方向
		st_info.f_trgtAngleS*= -1;															// 回転方向を逆にする
		f_angle1			*= -1;
//		f_angle2 			*= -1;															// 回転方向を逆にする
		f_angle3 			*= -1;															// 回転方向を逆にする
		st_info.f_angle1	= f_angle1;						// 第1移動角度[rad]
		st_info.f_angle1_2	= st_info.f_angle - f_angle3;									// 第1+2移動角度[rad]
		
		/* 最小移動距離を上書き */
		if( st_info.f_angle1 > ( A1_MIN * -1 ) ){
			st_info.f_angle1 = A1_MIN * -1;
		}
	}
	else{
		st_info.f_angle1	= f_angle1;						// 第1移動角度[rad]
		st_info.f_angle1_2	= st_info.f_angle - f_angle3;									// 第1+2移動角度[rad]
		
		/* 最小移動距離を上書き */
		if( st_info.f_angle1 < A1_MIN ){
			st_info.f_angle1 = A1_MIN;
		}
	}


	GYRO_staErrChkAngle();			// エラー検出開始
//	printf("目標角度 %f %f %f\r\n",st_info.f_angle,st_info.f_angle1,st_info.f_angle1_2);
	/* ================ */
	/*      実動作      */
	/* ================ */
	/* ------ */
	/*  加速  */
	/* ------ */
	st_data.en_type			= CTRL_ACC_TRUN;
	st_data.f_acc			= 0;						// 加速度指定
	st_data.f_now			= 0;						// 現在速度
	st_data.f_trgt			= 0;						// 目標速度
	st_data.f_nowDist		= 0;						// 進んでいない
	st_data.f_dist			= 0;						// 加速距離
	st_data.f_accAngleS		= st_info.f_accAngleS1;		// 角加速度
	st_data.f_nowAngleS		= 0;						// 現在角速度
	st_data.f_trgtAngleS		= st_info.f_trgtAngleS;		// 目標角度
	st_data.f_nowAngle		= 0;						// 現在角度
	st_data.f_angle			= st_info.f_angle1;			// 目標角度
	st_data.f_time 			= 0;						// 目標時間 [sec] ← 指定しない
	CTRL_clrData();										// マウスの現在位置/角度をクリア
	CTRL_setData( &st_data );							// データセット
	DCM_staMotAll();									// モータON
	
	if( ( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 ) ){		// -方向
		while( f_NowAngle > st_info.f_angle1 ){			// 指定角度到達待ち
//			DCMC_getAngleSpeedFB(&f_err);
//			printf("[NOW]%d [Trgt]%d [TrgtS]%d  \n\r", (LONG)f_NowAngle, (LONG)f_TrgtAngle, (LONG)f_TrgtAngleS );
//			TIME_wait(10);
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
		}
	}
	else{
		while( f_NowAngle < st_info.f_angle1 ){			// 指定角度到達待ち
//			DCMC_getAngleSpeedFB(&f_err);
//			printf("[NOW]%d [Trgt]%d [TrgtS]%d  \n\r", (LONG)f_NowAngle, (LONG)f_TrgtAngle, (LONG)f_TrgtAngleS);
//			TIME_wait(10);
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
		}
	}
//	printf("finish\n");
//	LED4 = 0x01;
//	log_in(100);
	/* ------ */
	/*  等速  */
	/* ------ */
	if( ( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 ) ){		// -方向
		f_angle3			= ( f_TrgtAngleS - st_info.f_lastAngleS ) / 2 * ( f_TrgtAngleS - st_info.f_lastAngleS ) / st_info.f_accAngleS3;		// 第3移動角度[rad]
		f_angle3			= -1 * f_angle3;
		if( f_angle3 > A3_MIN*-1 ) f_angle3 = A3_MIN * -1;																	// 減速最低角度に書き換え
		st_info.f_angle1_2		= st_info.f_angle - f_angle3;// 第1+2移動角度[rad]
		
	}
	else{
		f_angle3			= ( f_TrgtAngleS - st_info.f_lastAngleS ) / 2 * ( f_TrgtAngleS - st_info.f_lastAngleS ) / st_info.f_accAngleS3;		// 第3移動角度[rad]
		if( f_angle3 < A3_MIN ) f_angle3 = A3_MIN;																			// 減速最低角度に書き換え
		st_info.f_angle1_2		= st_info.f_angle - f_angle3;																// 第1+2移動角度[rad]
//		printf("   [f_angle3]%d [f_angle1_2]%d\n\r", (LONG)f_angle3, (LONG)	st_info.f_angle1_2 );
	}
//	printf("[f_TrgtAngleS] %5.2f,st_info.f_angle1_2%5.2f,f_angle2%5.2f\n\r",f_TrgtAngleS,st_info.f_angle1_2,f_angle3);
	st_data.en_type			= CTRL_CONST_TRUN;
	st_data.f_acc			= 0;						// 加速度指定
	st_data.f_now			= 0;						// 現在速度
	st_data.f_trgt			= 0;						// 目標速度
	st_data.f_nowDist		= 0;						// 進んでいない
	st_data.f_dist			= 0;						// 等速完了位置
	st_data.f_accAngleS		= 0;						// 角加速度
	st_data.f_nowAngleS		= f_TrgtAngleS;				// 現在角速度
	st_data.f_trgtAngleS		= f_TrgtAngleS;				// 目標角度
	st_data.f_nowAngle		= st_info.f_angle1;			// 現在角度
	st_data.f_angle			= st_info.f_angle1_2;			// 目標角度
	st_data.f_time 			= 0;						// 目標時間 [sec] ← 指定しない
	CTRL_setData( &st_data );							// データセット
	if( ( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 ) ){		// -方向
		while( f_NowAngle > st_info.f_angle1_2 ){			// 指定距離到達待ち
//			DCMC_getAngleSpeedFB(&f_err);
//			printf("[NOW]%d [Trgt]%d [TrgtS]%d \n\r", (LONG)f_NowAngle, (LONG)f_TrgtAngle, (LONG)f_TrgtAngleS);
//			TIME_wait(10);
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop();
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
//			log_in(f_TrgtAngle);
		}
	}
	else{
		while( f_NowAngle < st_info.f_angle1_2 ){			// 指定距離到達待ち
//			DCMC_getAngleSpeedFB(&f_err);
//			printf("[NOW]%d [Trgt]%d [TrgtS]%d  \n\r", (LONG)f_NowAngle, (LONG)f_TrgtAngle, (LONG)f_TrgtAngleS);
//			TIME_wait(10);
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
//			log_in(f_TrgtAngleS);
		}
	}
//	printf("finish2\n");
//	LED4 = 0x02;
//	log_in(100);
	/* ------ */
	/*  減速  */
	/* ------ */
	st_data.en_type			= CTRL_DEC_TRUN;
	st_data.f_acc			= 0;						// 減速
	st_data.f_now			= 0;						// 現在速度
	st_data.f_trgt			= 0;						// 最終速度
	st_data.f_nowDist		= 0;						// 等速完了位置
	st_data.f_dist			= 0;						// 全移動完了位置
	st_data.f_accAngleS		= st_info.f_accAngleS3;		// 角加速度
	st_data.f_nowAngleS		= f_TrgtAngleS;				// 現在角速度
	st_data.f_trgtAngleS		= 0;						// 目標角度
	st_data.f_nowAngle		= st_info.f_angle1_2;		// 現在角度
	st_data.f_angle			= st_info.f_angle;			// 目標角度
	st_data.f_time 			= 0;						// 目標時間 [sec] ← 指定しない
	CTRL_setData( &st_data );							// データセット
	if( ( en_type == MOT_R90 ) || ( en_type == MOT_R180 ) || ( en_type == MOT_R360 ) ){		// -方向
		while( f_NowAngle > ( st_info.f_angle + 1 ) ){		// 指定距離到達待ち
//			DCMC_getAngleSpeedFB(&f_err);
//			printf("[NOW]%d [Trgt]%d [TrgtS]%d  \n\r", (LONG)f_NowAngle, (LONG)f_TrgtAngle, (LONG)f_TrgtAngleS );
//			TIME_wait(10);
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
		}
	}
	else{
		while( f_NowAngle < ( st_info.f_angle - 1 ) ){		// 指定距離到達待ち
//			DCMC_getAngleSpeedFB(&f_err);
//			printf("[NOW]%d [Trgt]%d [TrgtS]%d  \n\r", (LONG)f_NowAngle, (LONG)f_TrgtAngle, (LONG)f_TrgtAngleS);
//			TIME_wait(10);
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
//			log_in(f_TrgtAngle);
		}
	}
//	printf("finish3\n");
//	LED4 = 0x04;
//	log_in(200);
	/* 停止 */
	TIME_wait(500);				// 安定待ち
	CTRL_stop();			// 制御停止
	DCM_brakeMot( DCM_R );		// ブレーキ
	DCM_brakeMot( DCM_L );		// ブレーキ
	GYRO_endErrChkAngle();					// エラー検出終了
	

	
}

// *************************************************************************
//   機能		： スラロームの開始速度を設定する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
//   その他     ：
// **************************    履    歴    *******************************
// 		v1.0		2018.09.04			sato			新規
// *************************************************************************/
PUBLIC void MOT_setSuraStaSpeed( FLOAT f_speed )
{
	f_MotSuraStaSpeed = f_speed;
	
}

// *************************************************************************
//   機能		： スラロームの開始速度を取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
//   その他     ：
// **************************    履    歴    *******************************
// 		v1.0		2018.09.04			sato			新規
// *************************************************************************/
PUBLIC FLOAT MOT_getSuraStaSpeed( void )
{
	return f_MotSuraStaSpeed;
}


// *************************************************************************
//   機能		： 目標速度を設定する。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
//   その他     ：
// **************************    履    歴    *******************************
// 		v1.0		2018.09.04			sato			新規
// *************************************************************************/
PUBLIC FLOAT MOT_setTrgtSpeed(FLOAT f_speed)
{
	f_MotTrgtSpeed = f_speed;
	return f_MotTrgtSpeed;
}

// *************************************************************************
//   機能		： 現在速度を設定する。
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
//   その他     ：
// **************************    履    歴    *******************************
// 		v1.0		2018.09.04			sato			新規
// *************************************************************************/
PUBLIC void MOT_setNowSpeed(FLOAT f_speed)
{
	f_MotNowSpeed = f_speed;
}

// *************************************************************************
//   機能		： 壁当て
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.10.12		翔	新規
// *************************************************************************/
PUBLIC void MOT_goHitBackWall(void)
{
	stMOT_DATA	st_info;	//シーケンスデータ
	stCTRL_DATA	st_data;	//制御データ

	/* ---------------- */
	/*  動作データ計算  */
	/* ---------------- */
	/* 加速度 */
	st_info.f_acc1= 1200;												// 角加速度1[rad/s^2]												// 角加速度3[rad/s^2]

	GYRO_staErrChkAngle();			// エラー検出開始
//	printf("");
	/* ================ */
	/*      実動作      */
	/* ================ */
	/* ------ */
	/*  加速  */
	/* ------ */
	st_data.en_type			= CTRL_HIT_WALL;
	st_data.f_acc			= st_info.f_acc1;						// 加速度指定
	st_data.f_now			= 0;						// 現在速度
	st_data.f_trgt			= 0;						// 目標速度
	st_data.f_nowDist		= 0;						// 進んでいない
	st_data.f_dist			= 0;						// 加速距離
	st_data.f_accAngleS		= 0;		// 角加速度
	st_data.f_nowAngleS		= 0;						// 現在角速度
	st_data.f_trgtAngleS		= 0;		// 目標角度
	st_data.f_nowAngle		= 0;						// 現在角度
	st_data.f_angle			= 0;			// 目標角度
	st_data.f_time 			= 0;						// 目標時間 [sec] ← 指定しない
	CTRL_clrData();										// マウスの現在位置/角度をクリア
	CTRL_setData( &st_data );							// データセット
	DCM_staMotAll();									// モータON
//	printf("目標速度 %f 目標位置 %f\r\n",st_data.f_trgt,st_data.f_dist);
//	LED4 =0x01;
	/*停止*/
	TIME_wait(700);				// 安定待ち
	CTRL_stop();			// 制御停止
	DCM_brakeMot( DCM_R );		// ブレーキ
	DCM_brakeMot( DCM_L );		// ブレーキ
	
	TIME_wait(100);
	
	f_MotNowSpeed = 0.0f;		//現在速度更新
//	LED4 = 0x08;
	GYRO_endErrChkAngle();					// エラー検出終了
	
}

// *************************************************************************
//   機能		： スラローム
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.11.11		翔	新規
// *************************************************************************/
PUBLIC void MOT_goSla( enMOT_SURA_CMD en_type, stSLA* p_sla )
{
	stMOT_DATA		st_info;					// シーケンスデータ
	stCTRL_DATA		st_data;					// 制御データ
//	FLOAT			f_err;
	FLOAT			f_entryLen;
	FLOAT			f_escapeLen;
	
	/* ---------------- */
	/*  動作データ計算  */
	/* ---------------- */
	/* 加速度 */
	st_info.f_acc1 		= 0;																// 加速度1[mm/s^2]
	st_info.f_acc3 		= 0;																// 加速度3[mm/s^2]

	/* 速度 */
	st_info.f_now		= p_sla->f_speed;													// 現在速度	
	st_info.f_trgt		= p_sla->f_speed;													// 目標速度
	st_info.f_last		= p_sla->f_speed;													// 最終速度
	
	/* 距離 */
	st_info.f_dist		= 0;																// 移動距離
	st_info.f_l1		= 0;																// 第1移動距離[mm]
	st_info.f_l1_2		= 0;																// 第1+2移動距離[mm]

	/* 角加速度 */
	st_info.f_accAngleS1= p_sla->f_angAcc;													// 角加速度1[deg/s^2]
	st_info.f_accAngleS3= p_sla->f_angAcc;													// 角加速度3[deg/s^2]

	/* 角速度 */
	st_info.f_nowAngleS	= 0;																// 現在角速度[deg/s]
	st_info.f_trgtAngleS= p_sla->f_angvel;													// 目標角速度
	st_info.f_lastAngleS= 0;																// 最終角速度

	/* 角度 */
	st_info.f_angle		= p_sla->f_ang_Total;												// 旋回角度[deg]
	st_info.f_angle1	= p_sla->f_ang_AccEnd;												// 第1移動角度[deg]
	st_info.f_angle1_2	= p_sla->f_ang_ConstEnd;											// 第1+2移動角度[deg]

	/* 方向に応じて符号を変更 */
	if( ( en_type == MOT_R90S ) || 
		( en_type == MOT_R45S_S2N ) || ( en_type == MOT_R45S_N2S ) || 
		( en_type == MOT_R90S_N ) || 
		( en_type == MOT_R135S_S2N ) || ( en_type == MOT_R135S_N2S )
	){
		st_info.f_accAngleS1 *= -1;
		st_info.f_trgtAngleS *= -1;
		st_info.f_angle      *= -1;
		st_info.f_angle1     *= -1;
		st_info.f_angle1_2   *= -1;
	}
	else{
		st_info.f_accAngleS3 *= -1;
	}
	
	/* 斜め走行のタイプに応じて、スラローム前の距離とスラローム後の退避距離を入れ替える */
	if( ( en_type == MOT_R45S_N2S ) || ( en_type == MOT_L45S_N2S ) || ( en_type == MOT_R135S_N2S ) || ( en_type == MOT_L135S_N2S ) ){ 		// 逆にするもの
		f_entryLen  = p_sla->f_escapeLen;
		f_escapeLen = p_sla->f_entryLen;
	}
	else{		// 通常
		f_entryLen  = p_sla->f_entryLen;
		f_escapeLen = p_sla->f_escapeLen;
	}

	GYRO_staErrChkAngle();			// エラー検出開始
	
//	LED_on(LED1);
	/* ================ */
	/*      実動作      */
	/* ================ */
	/* ------------------------ */
	/*  スラローム前の前進動作  */
	/* ------------------------ */
	st_data.en_type			= CTRL_ENTRY_SURA;
	st_data.f_acc			= 0;						// 加速度指定
	st_data.f_now			= st_info.f_now;			// 現在速度
	st_data.f_trgt			= st_info.f_now;			// 目標速度
	st_data.f_nowDist		= 0;						// 進んでいない
	st_data.f_dist			= f_entryLen;				// スラローム前の前進距離
	st_data.f_accAngleS		= 0;						// 角加速度
	st_data.f_nowAngleS		= 0;						// 現在角速度
	st_data.f_trgtAngleS	= 0;						// 目標角度
	st_data.f_nowAngle		= 0;						// 現在角度
	st_data.f_angle			= 0;						// 目標角度
	st_data.f_time 			= 0;						// 目標時間 [sec] ← 指定しない
	CTRL_clrData();										// マウスの現在位置/角度をクリア
	CTRL_setData( &st_data );							// データセット
	DCM_staMotAll();									// モータON

	while( f_NowDist < f_entryLen ){				// 指定距離到達待ち
		if( SYS_isOutOfCtrl() == TRUE ){
			CTRL_stop(); 
			DCM_brakeMot( DCM_R );		// ブレーキ
			DCM_brakeMot( DCM_L );		// ブレーキ
			break;
		}				// 途中で制御不能になった
	}
//	LED_off(LED1);
//	log_in(0);
	/* ------ */
	/*  加速  */
	/* ------ */
	st_data.en_type			= CTRL_ACC_SURA;
	st_data.f_acc			= 0;						// 加速度指定
	st_data.f_now			= st_info.f_now;			// 現在速度
	st_data.f_trgt			= st_info.f_now;			// 目標速度
	st_data.f_nowDist		= f_entryLen;				// 
	st_data.f_dist			= f_entryLen + st_info.f_now * p_sla->us_accAngvelTime * 0.001;		// 加速距離
	st_data.f_accAngleS		= st_info.f_accAngleS1;		// 角加速度
	st_data.f_nowAngleS		= 0;						// 現在角速度
	st_data.f_trgtAngleS		= st_info.f_trgtAngleS;		// 目標角速度
	st_data.f_nowAngle		= 0;						// 現在角度
	st_data.f_angle			= st_info.f_angle1;			// 目標角度
	st_data.f_time 			= p_sla->us_accAngvelTime * 0.001;			// [msec] → [sec]
	CTRL_setData( &st_data );							// データセット
//	printf("trgtangleS %5.2f\n\r",st_data.f_trgtAngleS);
	if( IS_R_SLA( en_type ) == TRUE ) {		// -方向
		while( ( f_NowAngle > st_info.f_angle1 ) || ( f_NowDist < st_data.f_dist ) ){			// 指定角度＋距離到達待ち
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
		}
	}
	else{
		while( ( f_NowAngle < st_info.f_angle1 ) || ( f_NowDist < st_data.f_dist ) ){			// 指定角度＋距離到達待ち
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
		}
	}

//	log_in(0);
//	log_in(f_NowAngle);
	/* ------ */
	/*  等速  */
	/* ------ */
	st_data.en_type			= CTRL_CONST_SURA;
	st_data.f_acc			= 0;						// 加速度指定
	st_data.f_now			= st_info.f_now;			// 現在速度
	st_data.f_trgt			= st_info.f_now;			// 目標速度
	st_data.f_nowDist		= f_entryLen + st_info.f_now * p_sla->us_accAngvelTime * 0.001;
	st_data.f_dist			= f_entryLen + st_info.f_now * ( p_sla->us_constAngvelTime + p_sla->us_accAngvelTime ) * 0.001;		// 等速距離
	st_data.f_accAngleS		= 0;						// 角加速度
	st_data.f_nowAngleS		= st_info.f_trgtAngleS;		// 現在角速度
	st_data.f_trgtAngleS	= st_info.f_trgtAngleS;		// 目標角速度
	st_data.f_nowAngle		= st_info.f_angle1;			// 現在角度
	st_data.f_angle			= st_info.f_angle1_2;		// 目標角度
	st_data.f_time 			= p_sla->us_constAngvelTime * 0.001;		// [msec] → [sec]
	CTRL_setData( &st_data );							// データセット

	if( IS_R_SLA( en_type ) == TRUE ) {		// -方向
		while( ( f_NowAngle > st_info.f_angle1_2 ) || ( f_NowDist < st_data.f_dist ) ){		// 指定角度＋距離到達待ち
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop();
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
		}
	}
	else{
		while( ( f_NowAngle < st_info.f_angle1_2 ) || ( f_NowDist < st_data.f_dist ) ){		// 指定角度＋距離到達待ち
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
		}
	}
//	log_in(0);
//	log_in(f_NowAngle);
	/* ------ */
	/*  減速  */
	/* ------ */
	st_data.en_type			= CTRL_DEC_SURA;
	st_data.f_acc			= 0;						// 加速度指定
	st_data.f_now			= st_info.f_now;			// 現在速度
	st_data.f_trgt			= st_info.f_now;			// 目標速度
	st_data.f_nowDist		= f_entryLen + st_info.f_now * ( p_sla->us_constAngvelTime + p_sla->us_accAngvelTime ) * 0.001;
	st_data.f_dist			= f_entryLen + st_info.f_now * ( p_sla->us_constAngvelTime + p_sla->us_accAngvelTime * 2 ) * 0.001;		// 減速距離
	st_data.f_accAngleS		= st_info.f_accAngleS3;		// 角加速度
	st_data.f_nowAngleS		= st_info.f_trgtAngleS;		// 現在角速度
	st_data.f_trgtAngleS		= 0;				// 目標角速度
	st_data.f_nowAngle		= st_info.f_angle1_2;		// 現在角度
	st_data.f_angle			= st_info.f_angle;			// 目標角度
	st_data.f_time			= p_sla->us_accAngvelTime * 0.001;			// [msec] → [sec]
	CTRL_setData( &st_data );							// データセット
//	log_in(st_data.f_angle);
	if( IS_R_SLA( en_type ) == TRUE ) {		// -方向
		while( ( f_NowAngle > st_info.f_angle + 0.2 ) || ( f_NowDist < st_data.f_dist ) ){			// 指定角度＋距離到達待ち
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
//		LED4 = LED4_ALL_ON;
		}
	}
	else{
		while( ( f_NowAngle < st_info.f_angle - 0.2) || ( f_NowDist < st_data.f_dist ) ){			// 指定角度＋距離到達待ち
			if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
//		LED4 = LED4_ALL_ON;
		}
	}
//	LED4 = LED4_ALL_OFF;
//	log_in(0);
//	LED_on(LED1);
	/* ------------------------ */
	/*  スラローム後の前進動作  */
	/* ------------------------ */
	st_data.en_type			= CTRL_EXIT_SURA;
	st_data.f_acc			= 0;						// 加速度指定
	st_data.f_now			= st_info.f_now;			// 現在速度
	st_data.f_trgt			= st_info.f_now;			// 目標速度
	st_data.f_nowDist		= f_entryLen + st_info.f_now * ( p_sla->us_constAngvelTime + p_sla->us_accAngvelTime * 2 ) * 0.001;
	st_data.f_dist			= f_escapeLen + f_entryLen + st_info.f_now * ( p_sla->us_constAngvelTime + p_sla->us_accAngvelTime * 2 ) * 0.001;	// スラローム後の前進距離
	st_data.f_accAngleS		= 0;						// 角加速度
	st_data.f_nowAngleS		= 0;						// 現在角速度
	st_data.f_trgtAngleS		= 0;						// 目標角度
	st_data.f_nowAngle		= 0;						// 現在角度
	st_data.f_angle			= 0;						// 目標角度
	st_data.f_time 			= 0;						// 目標時間 [sec] ← 指定しない
	CTRL_setData( &st_data );							// データセット
	
	while( f_NowDist < ( st_data.f_dist - 0.01 ) ){	// 指定距離到達待ち
		if( SYS_isOutOfCtrl() == TRUE ){
			CTRL_stop(); 
			DCM_brakeMot( DCM_R );		// ブレーキ
			DCM_brakeMot( DCM_L );		// ブレーキ
			break;
		}				// 途中で制御不能になった
	}
//	LED_off(LED1);
//	log_in(f_NowAngle);
	f_MotNowSpeed = st_info.f_now;			// 現在速度更新

	GYRO_endErrChkAngle();					// エラー検出終了

}

// *************************************************************************
//   機能		： ターンテーブル(宴会芸)
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.4.23		翔	新規
// *************************************************************************/
PUBLIC void turntable()
{
	stCTRL_DATA test;
		test.en_type = CTRL_CONST;
		test.f_acc			= 0;						// 加速度指定
		test.f_now			= 0;			// 現在速度
		test.f_trgt			= 0;			// 目標速度
		test.f_nowDist			= 0;
		test.f_dist			= 0;	
		test.f_accAngleS		= 0;						// 角加速度
		test.f_nowAngleS		= GYRO_getSpeedErr();						// 現在角速度
		test.f_trgtAngleS		= 0;						// 目標角度
		test.f_nowAngle			= GYRO_getNowAngle();						// 現在角度
		test.f_angle			= 0;						// 目標角度
		test.f_time 			= 0;						// 目標時間 [sec] ← 指定しない
		
	CTRL_clrData();
	CTRL_setData(&test);
	DCM_staMotAll();									// モータON
	while( 1 ){	// 指定距離到達待ち
		if ( SW_ON == SW_INC_PIN ){
			CTRL_stop();
			break;
		}
	}
}

// *************************************************************************
//   機能		： 等速区画前進
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.8.30		翔	新規
// *************************************************************************/
PUBLIC void MOT_goBlock_Const(FLOAT f_num)
{
	stCTRL_DATA		st_data;
	stMOT_DATA		st_info;
	
	GYRO_staErrChkAngle();
	
	/* ---------------- */
	/*  動作データ計算  */
	/* ---------------- */
	/* 距離 */
	st_info.f_dist		= f_num * BLOCK - f_num * sliplengs;													// 移動距離[mm]
	
	
	/* ------ */
	/*  等速  */
	/* ------ */
	st_data.en_type			= CTRL_CONST;
	st_data.f_acc			= 0;					// 加速度指定
	st_data.f_now			= f_MotNowSpeed;			// 現在速度
	st_data.f_trgt			= f_MotNowSpeed;			// 目標速度
	st_data.f_nowDist		= 0;				// 現在位置
	st_data.f_dist			= st_info.f_dist;			// 等速完了位置
	st_data.f_accAngleS		= 0;					// 角加速度
	st_data.f_nowAngleS		= 0;					// 現在角速度
	st_data.f_trgtAngleS		= 0;					// 目標角度
	st_data.f_nowAngle		= 0;					// 現在角度
	st_data.f_angle			= 0;					// 目標角度
	st_data.f_time 			= 0;					// 目標時間 [sec] ← 指定しない
	CTRL_clrData();										// 設定データをクリア
	CTRL_setData( &st_data );						// データセット
	f_TrgtSpeed		= f_MotNowSpeed;
//	printf("目標速度 %f 目標位置 %f \r\n",st_data.f_trgt,st_data.f_dist);
	while( f_NowDist < st_info.f_dist ){				// 指定距離到達待ち
		if( SYS_isOutOfCtrl() == TRUE ){
				CTRL_stop(); 
				DCM_brakeMot( DCM_R );		// ブレーキ
				DCM_brakeMot( DCM_L );		// ブレーキ
				break;
			}				// 途中で制御不能になった
	}
	
	GYRO_endErrChkAngle();
}

// *************************************************************************
//   機能		： 壁切れ補正のタイプを設定する
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.9.27		翔	新規
// *************************************************************************/
PUBLIC void MOT_setWallEdgeType( enMOT_WALL_EDGE_TYPE en_type )
{
	en_WallEdge = en_type;
	bl_IsWallEdge = FALSE;			// 非検知
	
}

// *************************************************************************
//   機能		： 壁切れ補正のタイプを取得する
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.9.27		翔	新規
// *************************************************************************/
PUBLIC enMOT_WALL_EDGE_TYPE MOT_getWallEdgeType( void )
{
	return en_WallEdge;
}

// *************************************************************************
//   機能		： 壁切れの検知を設定する
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.9.27		翔	新規
// *************************************************************************/
PUBLIC void MOT_setWallEdge( BOOL bl_val )
{
	bl_IsWallEdge = bl_val;
	
}

// *************************************************************************
//   機能		： 壁切れ距離
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.9.27		翔	新規
// *************************************************************************/
PRIVATE BOOL MOT_setWallEdgeDist( void )
{
	FLOAT f_addDist;
	
	/* 壁の切れ目を検知していない */
	if( ( bl_IsWallEdge == FALSE ) || ( en_WallEdge == MOT_WALL_EDGE_NONE ) ){		// 壁切れ設定されていないか、検出していない場合は処理を抜ける
	
		return FALSE;
	}
	
	f_addDist = f_NowDist + MOT_WALL_EDGE_DIST;		// 旋回開始位置
	
	/* 多く走る必要がある */
	if( f_addDist > st_Info.f_dist ){
		
		f_WallEdgeAddDist = f_addDist - st_Info.f_dist;
	}
	
	/* 壁の切れ目補正の変数を初期化 */
	en_WallEdge   = MOT_WALL_EDGE_NONE;		// 壁の切れ目タイプ
	bl_IsWallEdge = FALSE;					// 壁の切れ目検知
	
	return TRUE;
}
PRIVATE BOOL MOT_setWallEdgeDist_LoopWait( void )
{
	/* 壁の切れ目を検知していない */
	if( bl_IsWallEdge == FALSE ){		// 壁切れ設定されていないか、検出していない場合は処理を抜ける
	
		return FALSE;
	}
	
	f_WallEdgeAddDist = MOT_WALL_EDGE_DIST;		// 旋回開始位置
	
	return TRUE;
}

// *************************************************************************
//   機能		： フェイルセーフフラグオン
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.10.10		翔	新規
// *************************************************************************/
PUBLIC void Failsafe_flag(void)
{
	bl_failsafe = TRUE;
	LED4 = LED4_ALL_ON;
}

// *************************************************************************
//   機能		： フェイルセーフフラグオフ
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.10.10		翔	新規
// *************************************************************************/
PUBLIC void Failsafe_flag_off(void)
{
	bl_failsafe = FALSE;
}

// *************************************************************************
//   機能		： フェイルセーフフラグ
//   注意		： なし
//   メモ		： 
//   引数		： 
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.10.10		翔	新規
// *************************************************************************/
PUBLIC BOOL SYS_isOutOfCtrl( void )
{
	if( bl_failsafe == TRUE ){
		return TRUE;
	}
	else{
		return FALSE;
	}
}


// *************************************************************************
//   機能		： ログ取り関数(真)
//   注意		： なし
//   メモ		： ログは1～12 ログを取る際にlog_flagをTRUEに変更
//   引数		： ログを取るパラメータ
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.05.01		翔	新規
// *************************************************************************/
PRIVATE void log_in2( 	FLOAT log1,FLOAT log2,
			FLOAT log3,FLOAT log4,
			FLOAT log5,FLOAT log6,
			FLOAT log7,FLOAT log8,
			FLOAT log9,FLOAT log10,
			FLOAT log11,FLOAT log12)
{
	if((b_logflag == TRUE)&&(log_count < log_num)){
		Log_1[log_count] = log1;
		Log_2[log_count] = log2;
		Log_3[log_count] = log3;
		Log_4[log_count] = log4;
		Log_5[log_count] = log5;
		Log_6[log_count] = log6;
		Log_7[log_count] = log7;
		Log_8[log_count] = log8;
		Log_9[log_count] = log9;
		Log_10[log_count] = log10;
		Log_11[log_count] = log11;
		Log_12[log_count] = log12;
		
		log_count++;
	}
}

// *************************************************************************
//   機能		： ログ取り関数(割り込み用）
//   注意		： なし
//   メモ		： ログは1～12　バッテリーの割り込みに追加　ここで入力データを変更　intprg.hに直接書き込んで10msで割り込み
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.05.01		翔	新規
// *************************************************************************/
PUBLIC void log_interrupt ( void )
{
/*	log_in2(f_DistErrSum, f_NowSpeed,
		f_TrgtSpeed, f_NowDist,
		f_TrgtDist, f_AccAngleS,
		GYRO_getSpeedErr(), f_TrgtAngleS,)
		f_NowAngle, f_TrgtAngle,
		templog1,templog2);
*/

	log_in2(f_NowSpeed, f_TrgtSpeed,
		f_NowDist, f_TrgtDist,
		GYRO_getSpeedErr(), f_TrgtAngleS,
		f_NowAngle,f_TrgtAngle,
		f_AccAngleS,templog1,
		templog2,f_Duty_R);

/*	log_in2(DIST_getNowVal( DIST_SEN_R_FRONT ), DIST_getNowVal( DIST_SEN_L_FRONT ),
		DIST_getNowVal( DIST_SEN_R_SIDE ), DIST_getNowVal( DIST_SEN_L_SIDE ),
		GYRO_getSpeedErr(), f_TrgtAngleS,
		f_NowAngle,f_TrgtAngle,
		templog2,templog1,
		f_Duty_L,f_Duty_R);
*/
}

// *************************************************************************
//   機能		： ログフラグオン
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.05.01		翔	新規
// *************************************************************************/
PUBLIC void log_flag_on(void)
{
	b_logflag = TRUE;
}

// *************************************************************************
//   機能		： ログフラグオン
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.05.01		翔	新規
// *************************************************************************/
PUBLIC void log_flag_off(void)
{
	b_logflag = FALSE;
}

// *************************************************************************
//   機能		： ログ書き出し
//   注意		： なし
//   メモ		： ログは1～12　CSVで出力扱い
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.05.01		翔	新規
// *************************************************************************/
PUBLIC void log_read2(void)
{
	int i=0;
	while(i<log_num){
		printf("%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\n\r",
		Log_1[i],Log_2[i],Log_3[i],Log_4[i],Log_5[i],Log_6[i],Log_7[i],Log_8[i],Log_9[i],Log_10[i],Log_11[i],Log_12[i]);
		i++;
	}
}
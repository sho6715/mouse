//   ロボット名	： 電研ベーシックDCマウス、サンシャイン
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.28			sato			新規（ファイルのインクルード）
// *************************************************************************/
// 多重コンパイル抑止
#ifndef _HAL_H
#define _HAL_H

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <parameters.h>

//**************************************************
// 定義（define）
//**************************************************
/* 迷路距離 */
#define HALF_BLOCK			( 45.0f*2 )					// 半区間 [mm]
#define BLOCK				( 90.0f*2 )					// １区間 [mm]
#define HALF_BLOCK_SKEW			( 63.64f*2 )				// 斜め半区間 [mm]
#define BLOCK_SKEW			( 127.28f*2 )				// 斜め１区間 [mm]

#define IS_R_SLA(a)			( ( (a) % 2 == 0 ) ? (TRUE) : (FALSE))

//**************************************************
// 列挙体（enum）
//**************************************************
/* モータID */
typedef enum{
	DCM_R = 0,					// 右モータ
	DCM_L,						// 左モータ
	DCM_MAX
}enDCM_ID;

/* 旋回コマンドリスト */
typedef enum{
	MOT_R90 =0,					// 右 90度超信地旋回
	MOT_L90,					// 左 90度超信地旋回
	MOT_R180,					// 右180度超信地旋回
	MOT_L180,					// 左180度超信地旋回
	MOT_R360,					// 右360度超信地旋回
	MOT_L360,					// 左360度超信地旋回
	MOT_TURN_CMD_MAX
}enMOT_TURN_CMD;

/* スラロームコマンドリスト */
typedef enum{
	MOT_R90S =0,				// 右 90度超スラローム
	MOT_L90S,					// 左 90度超スラローム
	MOT_R45S_S2N,				// [斜め用] 右 45度超スラローム、ストレート ⇒ 斜め
	MOT_L45S_S2N,				// [斜め用] 左 45度超スラローム、ストレート ⇒ 斜め
	MOT_R45S_N2S,				// [斜め用] 右 45度超スラローム、斜め ⇒ ストレート
	MOT_L45S_N2S,				// [斜め用] 左 45度超スラローム、斜め ⇒ ストレート
	MOT_R90S_N,					// [斜め用] 右 90度超スラローム、斜め ⇒ 斜め
	MOT_L90S_N,					// [斜め用] 左 90度超スラローム、斜め ⇒ 斜め
	MOT_R135S_S2N,				// [斜め用] 右135度超スラローム、ストレート ⇒ 斜め
	MOT_L135S_S2N,				// [斜め用] 左135度超スラローム、ストレート ⇒ 斜め
	MOT_R135S_N2S,				// [斜め用] 右135度超スラローム、斜め ⇒ ストレート
	MOT_L135S_N2S,				// [斜め用] 左135度超スラローム、斜め ⇒ ストレート
	MOT_SURA_CMD_MAX,
}enMOT_SURA_CMD;

/* 壁切れ補正 */
typedef enum{
	MOT_WALL_EDGE_NONE =0,		// 壁のエッジ検出での補正なし
	MOT_WALL_EDGE_RIGHT,		// 右壁のエッジ検出での補正
	MOT_WALL_EDGE_LEFT,			// 左壁のエッジ検出での補正
	MOT_WALL_EDGE_MAX,
}enMOT_WALL_EDGE_TYPE;

//**************************************************
// 構造体（struct）
//**************************************************


//**************************************************
// グローバル変数
//**************************************************




//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
/*ログ取り*/
//PUBLIC void log_in(FLOAT log_input);
//PUBLIC void log_out(void);

PUBLIC void HAL_init( void );
/* シリアル通信 */
PUBLIC void SCI_putch(char data);
PUBLIC void SCI_puts(char *buffer);
PUBLIC void SCI_putsL(char *buffer, int len);
PUBLIC void charput(unsigned char data);
PUBLIC int SCI_chkRecv(void);
PUBLIC char SCI_getch(void);
PUBLIC unsigned char SCI_getch_uc(void);
PUBLIC int SCI_gets(char *buffer);
PUBLIC unsigned char charget(void);

/* 電圧監視 */
PUBLIC void BAT_Pol( void );
PUBLIC FLOAT BAT_getLv(void);

/*ジャイロ監視*/
PUBLIC void GYRO_Pol(void);
PUBLIC void GYRO_SetRef(void);
PUBLIC FLOAT GYRO_getSpeedErr(void);
PUBLIC FLOAT GYRO_getNowAngle(void);
PUBLIC FLOAT GYRO_getRef( void );

PUBLIC void GYRO_staErrChkAngle( void );
PUBLIC void GYRO_endErrChkAngle( void );

PUBLIC void ACCEL_SetRef( void );
PUBLIC FLOAT Accel_getSpeedErr( void );
PUBLIC void ACCEL_Pol( void );


/*SPI*/
PUBLIC USHORT recv_spi(USHORT spi_ad);
PUBLIC USHORT recv_spi_who(void);
PUBLIC void recv_spi_init(void);
PUBLIC USHORT recv_spi_gyro(void);
PUBLIC USHORT recv_spi_gyrooffset(void);
PUBLIC USHORT recv_spi_accel(void);

/* エンコーダ */
PUBLIC void ENC_Sta( void );
PUBLIC void ENC_Stop( void );
PUBLIC void ENC_Clr( void );
PUBLIC void ENC_GetDiv( LONG* p_r, LONG* p_l );

/* DCM */
PUBLIC void DCM_setDirCw( enDCM_ID en_id );
PUBLIC void DCM_setDirCcw( enDCM_ID en_id );
PUBLIC void DCM_stopMot( enDCM_ID en_id );
PUBLIC void DCM_brakeMot( enDCM_ID en_id );
PUBLIC void DCM_staMot( enDCM_ID en_id );
PUBLIC void DCM_staMotAll( void );
PUBLIC void DCM_setPwmDuty( enDCM_ID en_id, USHORT us_duty10 );

/* 動作 */
PUBLIC void MOT_goBlock_FinSpeed( FLOAT f_num, FLOAT f_fin );
PUBLIC void MOT_goSkewBlock_FinSpeed( FLOAT f_num, FLOAT f_fin );

/*テスト走行用*/
PUBLIC void testrun(void);

/*ジャイロテスト*/
PUBLIC float GYRO_test(void);

/*超信地旋回*/
PUBLIC void MOT_turn( enMOT_TURN_CMD en_type );

/*目的速度変更*/
PUBLIC FLOAT MOT_setTrgtSpeed(FLOAT f_speed);
PUBLIC void MOT_setNowSpeed(FLOAT f_speed);

PUBLIC void MOT_setSuraStaSpeed( FLOAT f_speed );
PUBLIC FLOAT MOT_getSuraStaSpeed( void );

/*壁当て*/
PUBLIC void MOT_goHitBackWall(void);

/*スラローム*/
PUBLIC void MOT_goSla( enMOT_SURA_CMD en_type, stSLA* p_sla );

/*等速区画前進*/
PUBLIC void MOT_goBlock_Const(FLOAT f_num);

/*壁切れ*/
PUBLIC void MOT_setWallEdgeType( enMOT_WALL_EDGE_TYPE en_type );
PUBLIC enMOT_WALL_EDGE_TYPE MOT_getWallEdgeType( void );
PUBLIC void MOT_setWallEdge( BOOL bl_val );

PRIVATE BOOL MOT_setWallEdgeDist( void );
PRIVATE BOOL MOT_setWallEdgeDist_LoopWait( void );

//フェイルセーフ
PUBLIC void Failsafe_flag(void);
PUBLIC void Failsafe_flag_off(void);
PUBLIC BOOL SYS_isOutOfCtrl( void );

//ログ関数
PUBLIC void log_interrupt ( void );
PUBLIC void log_flag_on(void);
PUBLIC void log_flag_off(void);
PUBLIC void log_read2(void);

#endif	// _HAL_H
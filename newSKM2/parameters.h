//   ロボット名	： 電研ベーシックDCマウス、サンシャイン
//   概要		： パラメータファイル
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.28			sato			新規（ファイルのインクルード）
// *************************************************************************/
// 多重コンパイル抑止
#ifndef _PARAMETER_H
#define _PARAMETER_H

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O

//**************************************************
// 定義（define）
//**************************************************
#define		SW_CHATTERING_WAIT		(200) 		//スイッチのチャタリング対策

#define	FF_BALANCE_R				( 1.00f )					// [FF] 右のバランス係数
#define	FF_BALANCE_L				( 1.00f )					// [FF] 左のバランス係数 
#define FF_HIT_BALANCE_R			(1.00f)						//バック時のバランス係数
#define FF_HIT_BALANCE_L			(1.00f)

#define SEARCH_SPEED				(500)

#define DEG_TO_RAD  (3.1416f/180.0f)
#define RAD_TO_DEG  (180.0f/3.1416f)

#define MOT_WALL_EDGE_DIST			( 45.0f )	// 壁切れセンサOFF～壁まで

//**************************************************
// 列挙体（enum）
//**************************************************
/* 制御方法 */
typedef enum{
	
	/* ========================================== */ 
	/*  パラメータを取得する際に使用するシンボル  */ 
	/* ========================================== */ 
	/* ---------- */
	/*  直進制御  */
	/* ---------- */
	PARAM_ST_TOP = 0,				// カウント用
	// ↓ 動作を追加する場合にはこの間に記載

		PARAM_ACC,					// 加速中(直進)
		PARAM_CONST,				// 等速中(直進)
		PARAM_DEC,					// 減速中(直進)
//		PARAM_BACK_ACC,				// 加速中(後進)
//		PARAM_BACK_CONST,			// 等速中(後進)
//		PARAM_BACK_DEC,				// 減速中(後進)
		PARAM_SKEW_ACC,				// 加速中(斜め)
		PARAM_SKEW_CONST,			// 等速中(斜め)
		PARAM_SKEW_DEC,				// 減速中(斜め)
		PARAM_HIT_WALL,				// 壁あて制御

	// ↑  動作を追加する場合にはこの間に記載
	PARAM_ST_BTM,					// カウント用
	
	/* -------- */
	/*  ターン  */
	/* -------- */
	PARAM_TRUN_TOP,					// カウント用
	// ↓  動作を追加する場合にはこの間に記載

		PARAM_ACC_TRUN,				// 加速中(超地信旋回)
		PARAM_CONST_TRUN,			// 等速中(超地信旋回)
		PARAM_DEC_TRUN,				// 減速中(超地信旋回)

	// ↑  動作を追加する場合にはこの間に記載
	PARAM_TRUN_BTM,					// カウント用
	
	/* ------------ */
	/*  スラローム  */
	/* ------------ */
	PARAM_SLA_TOP,					// カウント用
	// ↓  動作を追加する場合にはこの間に記載

		PARAM_ENTRY_SURA,			// スラローム前の前進動作(スラローム)
		PARAM_ACC_SURA,				// 加速中(スラローム)
		PARAM_CONST_SURA,			// 等速中(スラローム)
		PARAM_DEC_SURA,				// 減速中(スラローム)
		PARAM_EXIT_SURA,			// スラローム後の前進動作(スラローム)

	// ↑  動作を追加する場合にはこの間に記載
	PARAM_SLA_BTM,					// カウント用
	
	
	/* ===================================================================== */ 
	/*  PARAM_setGainType()にてモードを決める際に引数として使用するシンボル  */ 
	/* ===================================================================== */ 
	PARAM_ST,						// 直進制御
	PARAM_TRUN,						// 旋回制御
	PARAM_SLA,						// スラローム制御
	
	
	/* ====================================================== */ 
	/*  作成するデータ数をカウントするために使用するシンボル  */ 
	/* ====================================================== */ 
	PARAM_ST_MAX		= PARAM_ST_BTM   - PARAM_ST_TOP,		// 直進最大数
	PARAM_TRUN_MAX		= PARAM_TRUN_BTM - PARAM_TRUN_TOP,		// 旋回最大数
	PARAM_SURA_MAX		= PARAM_SLA_BTM  - PARAM_SLA_TOP,		// スラローム最大数
	
	
	PARAM_NC = 0xff,
	
}enPARAM_MODE;


/* 動作速度 */
typedef enum{
	
	PARAM_VERY_SLOW = 0,	// 超低速
	PARAM_SLOW,				// 低速
	PARAM_NORMAL,			// 通常
	PARAM_FAST,				// 高速
	PARAM_VERY_FAST,		// 超高速
	
	PARAM_MOVE_SPEED_MAX
	
}enPARAM_MOVE_SPEED;


//**************************************************
// 構造体（struct）
//**************************************************
/* 速度データ */
typedef struct{
	FLOAT			f_acc;					// 加速度（加速時）
	FLOAT			f_dec;					// 加速度（減速時）
	FLOAT			f_accAngle;				// 角加速度（加速時）
	FLOAT			f_decAngle;				// 角加速度（減速時）
}stSPEED;

/* ゲイン */
typedef struct{
	FLOAT			f_FF_speed_acc;				// フィードフォワード、加速度
	FLOAT			f_FF_speed;				// フィードフォワード、速度
	FLOAT			f_FF_angleS_acc;		// フィードフォワード、角加速度
	FLOAT			f_FF_angleS;			// フィードフォワード、角速度
	FLOAT 			f_FB_speed_kp;			// フィードバック、速度 比例制御
	FLOAT 			f_FB_speed_ki;			// フィードバック、速度 積分制御
	FLOAT 			f_FB_speed_kd;			// フィードバック、速度 微分制御
	FLOAT			f_FB_dist_kp;			// フィードバック、距離 比例制御
	FLOAT 			f_FB_dist_ki;			// フィードバック、距離 積分制御
	FLOAT			f_FB_angleS_kp;			// フィードバック、角速度 比例制御
	FLOAT			f_FB_angleS_ki;			// フィードバック、角速度 積分制御
	FLOAT			f_FB_angleS_kd;			// フィードバック、角速度 微分制御
	FLOAT			f_FB_angle_kp;			// フィードバック、角度 比例制御
	FLOAT			f_FB_angle_ki;			// フィードバック、角度 積分制御
	FLOAT			f_FB_wall_kp;			// フィードバック、壁 比例制御
	FLOAT			f_FB_wall_kd;			// フィードバック、壁 微分制御
}stGAIN;

/* スラロームデータ */
typedef struct{
	FLOAT	f_speed;
	FLOAT	f_angAcc;
	FLOAT	f_angvel;
	FLOAT	f_entryLen;
	FLOAT	f_escapeLen;
	USHORT	us_accAngvelTime;
	USHORT	us_constAngvelTime;
	FLOAT	f_ang_AccEnd;
	FLOAT	f_ang_ConstEnd;
	FLOAT	f_ang_Total;
}stSLA;

/* スラロームタイプ */
typedef enum{
	SLA_90,
	SLA_45,	
	SLA_135,
	SLA_N90,				// 斜め → 90°→ 斜め
	SLA_TYPE_MAX
}enSLA_TYPE;



//**************************************************
// グローバル変数
//**************************************************




//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
PUBLIC void PARAM_setSpeedType( enPARAM_MODE en_mode, enPARAM_MOVE_SPEED en_speed );
PUBLIC CONST stSPEED* PARAM_getSpeed( enPARAM_MODE en_mode );
PUBLIC CONST stGAIN* PARAM_getGain( enPARAM_MODE en_mode );

PUBLIC void PARAM_makeSra( float f_speed, float f_angAcc, float f_g , enSLA_TYPE en_mode);
PUBLIC stSLA* PARAM_getSra( enSLA_TYPE en_mode );
#endif	
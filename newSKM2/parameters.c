//   ロボット名	： 電研ベーシックDCマウス、サンシャイン
//   概要		： パラメータファイル
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.04			sato			新規（ファイルのインクルード）
// *************************************************************************/

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <stdio.h>
#include <math.h>
#include <parameters.h>
#include <hal.h>

//**************************************************
// 定義（define）
//**************************************************



//**************************************************
// 列挙体（enum）
//**************************************************


//**************************************************
// 構造体（struct）
//**************************************************


//**************************************************
// グローバル変数
//**************************************************
/* インデックス演算に使用 */
#define GET_INDEX_ST(i)			( i - PARAM_ST_TOP - 1 )		// 直進用のインデックスを取得
#define GET_INDEX_TRUN(i)		( i - PARAM_TRUN_TOP - 1 )		// 旋回用のインデックスを取得
#define GET_INDEX_SLA(i)		( i - PARAM_SLA_TOP - 1 )		// スラローム用のインデックスを取得

PRIVATE enPARAM_MOVE_SPEED		en_Speed_st	= PARAM_NORMAL;			// 直進時の移動速度タイプ
PRIVATE enPARAM_MOVE_SPEED		en_Speed_trun	= PARAM_NORMAL;				// 旋回時の移動速度タイプ
PRIVATE enPARAM_MOVE_SPEED		en_Speed_sla	= PARAM_NORMAL;				// スラローム時の移動速度タイプ
PRIVATE stSLA				st_Sla[SLA_TYPE_MAX];					// スラローム時の走行パラメータ

//********************************************************
/* モジュールグローバル変数（チューニングが必要なパラメータ）*/
//********************************************************

/* ============ */
/*  速度データ  */
/* ============ */

	/* 直進速度データ */
	PRIVATE CONST stSPEED f_StSpeedData[PARAM_MOVE_SPEED_MAX] = {
		
		//	加速度		減速度		角加速度		角減速度
		{ 800,			1000,		0,				0,				},		// 超低速(PARAM_VERY_SLOW)
		{ 1800,			1800,		0,				0,				},		// 低速(PARAM_SLOW)
		{ 1800,			1800,		0,				0,				},		// 通常(PARAM_NORMAL)
		{ 2500,			2500,		0,				0,				},		// 高速(PARAM_FAST)
		{ 4000,			4000,		0,				0,				},		// 超高速(PARAM_VERY_FAST)
	};

	/* 旋回速度データ */
	PRIVATE CONST stSPEED f_TurnSpeedData[PARAM_MOVE_SPEED_MAX] = {
		
		//	加速度		減速度		角加速度		角減速度
		{ 0,			0,			1800,			1800,			},		// 超低速(PARAM_VERY_SLOW)
		{ 0,			0,			1800,			1800,			},		// 低速(PARAM_SLOW)
		{ 0,			0,			1800,			1800,			},		// 通常(PARAM_NORMAL)
		{ 0,			0,			1800,			1800,			},		// 高速(PARAM_FAST)
		{ 0,			0,			1800,			1800,			},		// 超高速(PARAM_VERY_FAST)
	};

	/* スラローム速度データ */
	PRIVATE CONST stSPEED f_SlaSpeedData[PARAM_MOVE_SPEED_MAX] = {
		
		//	加速度		減速度		角加速度		角減速度
		{ 1800,			1800,		1800,			1800,			},		// 超低速(PARAM_VERY_SLOW)
		{ 1800,			1800,		1800,			1800,			},		// 低速(PARAM_SLOW)
		{ 1800,			1800,		1800,			1800,			},		// 通常(PARAM_NORMAL)
		{ 2500,			2500,		1800,			1800,			},		// 高速(PARAM_FAST)
		{ 4000,			4000,		1800,			1800,			},		// 超高速(PARAM_VERY_FAST)
	};


/* ============== */
/*  ゲインデータ  */
/* ============== */
	// 【アドバイス】 
	//    もしもゲインのパラメータ数を増やしたい場合は、stGAINのメンバと↓のデータを増やすだけでOKです。
	//    PARAM_getGain()でパラメータのアドレスを取得して、追加したメンバを参照して下さい。

	/* 直進ゲインデータ */
	PRIVATE CONST stGAIN f_StGainData[PARAM_MOVE_SPEED_MAX][PARAM_ST_MAX] = {
		
		/* 超低速(PARAM_VERY_SLOW) */
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp	角度ki	壁kp	壁kd
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_ACC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_CONST
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		6,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_DEC
//			{ 0.0242f*4,3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_ACC
//			{ 0,		3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_CONST
//			{ 0,		3,	0,	0,		0,	0.1,	0.5,		0,		0,		3,		0,		1,		0,	},	// PARAM_BACK_DEC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_ACC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_CONST
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		6,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_DEC
			{ 0.05,		0,		0,		0,		0,	0,		0,		0,		0,		0,			0,			0,		0,		0,		0,			0,		},	// PARAM_HIT_WALL
		},
		/* 低速(PARAM_SLOW) */
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp	角度ki	壁kp	壁kd
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_ACC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_CONST
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		6,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_DEC
//			{ 0.0242f*4,3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_ACC
//			{ 0,		3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_CONST
//			{ 0,		3,	0,	0,		0,	0.1,	0.5,		0,		0,		3,		0,		1,		0,	},	// PARAM_BACK_DEC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_ACC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_CONST
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		6,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_DEC
			{ 0.05,		0,		0,		0,		0,	0,		0,		0,		0,		0,			0,			0,		0,		0,		0,			0,		},	// PARAM_HIT_WALL
		},
		/* 通常(PARAM_NORMAL) */
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp	角度ki	壁kp	壁kd
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_ACC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_CONST
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		6,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_DEC
//			{ 0.0242f*4,3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_ACC
//			{ 0,		3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_CONST
//			{ 0,		3,	0,	0,		0,	0.1,	0.5,		0,		0,		3,		0,		1,		0,	},	// PARAM_BACK_DEC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_ACC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_CONST
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		6,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_DEC
			{ 0.05,		0,		0,		0,		0,	0,		0,		0,		0,		0,			0,			0,		0,		0,		0,			0,		},	// PARAM_HIT_WALL
		},
		/* 高速(PARAM_FAST) */
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp	角度ki	壁kp	壁kd
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_ACC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_CONST
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		6,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_DEC
//			{ 0.0242f*4,3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_ACC
//			{ 0,		3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_CONST
//			{ 0,		3,	0,	0,		0,	0.1,	0.5,		0,		0,		3,		0,		1,		0,	},	// PARAM_BACK_DEC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_ACC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_CONST
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		6,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_DEC
			{ 0.05,		0,		0,		0,		0,	0,		0,		0,		0,		0,			0,			0,		0,		0,		0,			0,		},	// PARAM_HIT_WALL
		},
		/* 超高速(PARAM_VERY_FAST) */
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp	角度ki	壁kp	壁kd
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_ACC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_CONST
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		6,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_DEC
//			{ 0.0242f*4,3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_ACC
//			{ 0,		3,	0,	0,		0,	0,	0.6,		0,		0,		3.5,		0,		1,		0,	},	// PARAM_BACK_CONST
//			{ 0,		3,	0,	0,		0,	0.1,	0.5,		0,		0,		3,		0,		1,		0,	},	// PARAM_BACK_DEC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_ACC
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		0,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_CONST
			{ 0.01,		0.2,	0,		0,		3,		0.03,		0,		6,		0,		0.15,		0.01,		0,		3.0,		0.1,		0.1,	0.03,	},	// PARAM_SKEW_DEC
			{ 0.05,		0,		0,		0,		0,	0,		0,		0,		0,		0,			0,			0,		0,		0,		0,			0,		},	// PARAM_HIT_WALL
		}
	};

	/* 旋回ゲインデータ */
	PRIVATE CONST stGAIN f_TurnGainData[PARAM_MOVE_SPEED_MAX][PARAM_TRUN_MAX] = {
		
		/* 超低速(PARAM_VERY_SLOW) */
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp	角度ki		壁kp	壁kd
			{ 0.0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			0,		0.1,			0,		0,	},	// PARAM_ACC_TRUN
			{ 0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			0,		0.1,			0,		0,	},	// PARAM_CONST_TRUN
			{ 0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			3,		0.5,			0,		0,	},	// PARAM_DEC_TRUN
		},
		/* 低速(PARAM_SLOW) */
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp	角度ki		壁kp	壁kd
			{ 0.0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			0,		0.1,			0,		0,	},	// PARAM_ACC_TRUN
			{ 0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			0,		0.1,			0,		0,	},	// PARAM_CONST_TRUN
			{ 0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			3,		0.5,			0,		0,	},	// PARAM_DEC_TRUN
		},
		/* 通常(PARAM_NORMAL) */
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp	角度ki		壁kp	壁kd
			{ 0.0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			0,		0.1,			0,		0,	},	// PARAM_ACC_TRUN
			{ 0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			0,		0.1,			0,		0,	},	// PARAM_CONST_TRUN
			{ 0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			3,		0.5,			0,		0,	},	// PARAM_DEC_TRUN
		},
		/* 高速(PARAM_FAST) */
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp	角度ki		壁kp	壁kd
			{ 0.0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			0,		0.1,			0,		0,	},	// PARAM_ACC_TRUN
			{ 0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			0,		0.1,			0,		0,	},	// PARAM_CONST_TRUN
			{ 0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			3,		0.5,			0,		0,	},	// PARAM_DEC_TRUN
		},
		/* 超高速(PARAM_VERY_FAST) */
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp	角度ki		壁kp	壁kd
			{ 0.0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			0,		0.1,			0,		0,	},	// PARAM_ACC_TRUN
			{ 0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			0,		0.1,			0,		0,	},	// PARAM_CONST_TRUN
			{ 0,		0,		0.005,	0.13,	0.1,	0,		0,		0.1,	0,		0.5,		0.0,		0,			3,		0.5,			0,		0,	},	// PARAM_DEC_TRUN
		}
	};

	/* スラロームゲインデータ */
	PRIVATE CONST stGAIN f_SlaGainData[PARAM_MOVE_SPEED_MAX][PARAM_SURA_MAX] = {

		/* 超低速(PARAM_VERY_SLOW) */
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp		角度ki		壁kp	壁kd
			{ 0.01,		0.2,	0,		0,		3,		0.03,	0,		0.1,	0,		0.1,		0.01,		0,			3,			0.1,		0.1,		0.03,	},	// PARAM_ENTRY_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		3,			0.1,		0,		0,	},	// PARAM_ACC_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		3,			0.1,		0,		0,	},	// PARAM_CONST_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		7,			0.5,	0,		0,	},	// PARAM_DEC_SURA
			{ 0.01,		0.2,	0,		0,		3,		0.03,	0,		0.1,	0,		0.1,		0.01,		0,			3,			0.1,		0.15,		0.04,	},	// PARAM_EXIT_SURA
		},
		/* 低速(PARAM_SLOW) */
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp		角度ki		壁kp	壁kd
			{ 0.01,		0.2,	0,		0,		3,		0.03,	0,		0.1,	0,		0.1,		0.01,		0,			3,			0.1,		0.1,		0.03,	},	// PARAM_ENTRY_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		3,			0.1,		0,		0,	},	// PARAM_ACC_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		3,			0.1,		0,		0,	},	// PARAM_CONST_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		7,			0.5,		0,		0,	},	// PARAM_DEC_SURA
			{ 0.01,		0.2,	0,		0,		3,		0.03,	0,		0.1,	0,		0.1,		0.01,		0,			3,			0.1,		0.15,		0.04,	},	// PARAM_EXIT_SURA
		},
		/* 通常(PARAM_NORMAL)500 *///角速度kp500 角度kp25000
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp		角度ki		壁kp	壁kd
			{ 0.01,		0.2,	0,		0,		3,		0.03,	0,		0.1,	0,		0.1,		0.01,		0,			3,			0.1,		0.1,		0.03,	},	// PARAM_ENTRY_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		3,			0.1,		0,		0,	},	// PARAM_ACC_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		3,			0.1,		0,		0,	},	// PARAM_CONST_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		7,			0.5,		0,		0,	},	// PARAM_DEC_SURA
			{ 0.01,		0.2,	0,		0,		3,		0.03,	0,		0.1,	0,		0.1,		0.01,		0,			3,			0.1,		0.15,		0.04,	},	// PARAM_EXIT_SURA
		},
		/* 高速(PARAM_FAST) 600*/
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp		角度ki		壁kp	壁kd
			{ 0.01,		0.2,	0,		0,		3,		0.03,	0,		0.1,	0,		0.1,		0.01,		0,			3,			0.1,		0.12,		0.035,	},	// PARAM_ENTRY_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		3,			0.1,		0,		0,	},	// PARAM_ACC_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		3,			0.1,		0,		0,	},	// PARAM_CONST_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		7,			0.5,		0,		0,	},	// PARAM_DEC_SURA
			{ 0.01,		0.2,	0,		0,		3,		0.03,	0,		0.1,	0,		0.1,		0.01,		0,			3,			0.1,		0.17,		0.045,	},	// PARAM_EXIT_SURA
		},
		/* 超高速(PARAM_VERY_FAST) */
		{	//	FFSA	FFS		FFAA	FFA		速度kp	速度ki	速度kd	位置kp	位置ki	角速度kp	角速度ki	角速度kd	角度kp		角度ki		壁kp	壁kd
			{ 0.01,		0.2,	0,		0,		3,		0.03,	0,		0.1,	0,		0.1,		0.01,		0,			3,			0.1,		0.14,		0.04,	},	// PARAM_ENTRY_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		3,			0.1,		0,		0,	},	// PARAM_ACC_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		3,			0.1,		0,		0,	},	// PARAM_CONST_SURA
			{ 0.01,		0.2,	0.005,	0.09,	3,		0.03,	0,		0.1,	0,		0.2,		0.015,		0.0,		7,			0.5,		0,		0,	},	// PARAM_DEC_SURA
			{ 0.01,		0.2,	0,		0,		3,		0.03,	0,		0.1,	0,		0.1,		0.01,		0,			3,			0.1,		0.2,		0.05,	},	// PARAM_EXIT_SURA
		}
	};
	

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************

// *************************************************************************
//   機能		： 制御方法に対応した動作速度を設定する
//   注意		： 動作前に予め設定しておく
//   メモ		： 速度値やゲイン値を取得する際に、どの動作速度のパラメータを取得するかを決定するために使用する
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.04			sato		新規
// *************************************************************************/
PUBLIC void PARAM_setSpeedType( enPARAM_MODE en_mode, enPARAM_MOVE_SPEED en_speed )
{
	switch( en_mode ){
		
		case PARAM_ST:
			en_Speed_st = en_speed;
			break;
		
		case PARAM_TRUN:
			en_Speed_trun = en_speed;
			break;
		
		case PARAM_SLA:
			en_Speed_sla = en_speed;
			break;
			
		default:
			printf("設定した速度のパラメータタイプがありません \n\r");
			break;
	}
}

// *************************************************************************
//   機能		： 加速度のポインタを取得する
//   注意		： 
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.04			sato		新規
// *************************************************************************/
PUBLIC CONST stSPEED* PARAM_getSpeed( enPARAM_MODE en_mode )
{
	CONST stSPEED* p_adr;
	
	switch( en_mode ){
		
		case PARAM_ST:													// 直進
		case PARAM_ACC:													// 加速中(直進)
		case PARAM_CONST:												// 等速中(直進)
		case PARAM_DEC:													// 減速中(直進)
//		case PARAM_BACK_ACC:											// 加速中(後進)
//		case PARAM_BACK_CONST:											// 等速中(後進)
//		case PARAM_BACK_DEC:											// 減速中(後進)
		case PARAM_SKEW_ACC:											// 加速中(斜め)
		case PARAM_SKEW_CONST:											// 等速中(斜め)
		case PARAM_SKEW_DEC:											// 減速中(斜め)
		case PARAM_HIT_WALL:											// 壁あて制御
			p_adr = &f_StSpeedData[en_Speed_st];
			break;
			
		case PARAM_TRUN:												// 旋回
		case PARAM_ACC_TRUN:											// 加速中(超地信旋回)
		case PARAM_CONST_TRUN:											// 等速中(超地信旋回)
		case PARAM_DEC_TRUN:											// 減速中(超地信旋回)
			p_adr = &f_TurnSpeedData[en_Speed_trun];
			break;
			
		case PARAM_SLA:													// スラローム
		case PARAM_ENTRY_SURA:											// スラローム前の前進動作(スラローム)
		case PARAM_ACC_SURA:											// 加速中(スラローム)
		case PARAM_CONST_SURA:											// 等速中(スラローム)
		case PARAM_DEC_SURA:											// 減速中(スラローム)
		case PARAM_EXIT_SURA:											// スラローム後の前進動作(スラローム)
			p_adr = &f_SlaSpeedData[en_Speed_sla];
			break;

		default:														// Err、とりあえず・・・（メモリ破壊を防ぐため）
			printf("設定した速度タイプがありません \n\r");
			p_adr = &f_SlaSpeedData[en_Speed_sla];
			break;
	}
	
	return p_adr;
}


// *************************************************************************
//   機能		： ゲインのポインタを取得する
//   注意		： 
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.04			sato		新規
// *************************************************************************/
PUBLIC CONST stGAIN* PARAM_getGain( enPARAM_MODE en_mode )
{
	CONST stGAIN* p_adr;
	
	switch( en_mode ){
		
		case PARAM_ACC:													// 加速中(直進)
		case PARAM_CONST:												// 等速中(直進)
		case PARAM_DEC:													// 減速中(直進)
//		case PARAM_BACK_ACC:											// 加速中(後進)
//		case PARAM_BACK_CONST:											// 等速中(後進)
//		case PARAM_BACK_DEC:											// 減速中(後進)
		case PARAM_SKEW_ACC:											// 加速中(斜め)
		case PARAM_SKEW_CONST:											// 等速中(斜め)
		case PARAM_SKEW_DEC:											// 減速中(斜め)
		case PARAM_HIT_WALL:											// 壁あて制御
			p_adr = &f_StGainData[en_Speed_st][GET_INDEX_ST( en_mode )];
			break;
			
		case PARAM_ACC_TRUN:											// 加速中(超地信旋回)
		case PARAM_CONST_TRUN:											// 等速中(超地信旋回)
		case PARAM_DEC_TRUN:											// 減速中(超地信旋回)
			p_adr = &f_TurnGainData[en_Speed_trun][GET_INDEX_TRUN( en_mode )];
			break;
			
		case PARAM_ENTRY_SURA:											// スラローム前の前進動作(スラローム)
		case PARAM_ACC_SURA:											// 加速中(スラローム)
		case PARAM_CONST_SURA:											// 等速中(スラローム)
		case PARAM_DEC_SURA:											// 減速中(スラローム)
		case PARAM_EXIT_SURA:											// スラローム後の前進動作(スラローム)
			p_adr = &f_SlaGainData[en_Speed_sla][GET_INDEX_SLA( en_mode )];
			break;
		
		default:														// Err、とりあえず・・・（メモリ破壊を防ぐため）
			printf("設定したゲインタイプがありません \n\r");
			p_adr = &f_SlaGainData[en_Speed_sla][GET_INDEX_SLA( en_mode )];
			break;
	}
	
	return p_adr;
}

// *************************************************************************
//   機能		： スラロームの走行パラメータを作成する
//   注意		： 
//   メモ		： f_speed：進入速度[mm/s]、f_angAcc：角加速度[deg/s^2]、f_g：横G[mm/s^2]、en_mode：スラロームタイプ
//   メモ		： 直進方向の速度は進入速度のまま、回転方向だけ加減速する。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.18			sato		新規
// *************************************************************************/
PUBLIC void PARAM_makeSra( float f_speed, float f_angAcc, float f_g , enSLA_TYPE en_mode)
{

	FLOAT	f_start_x;					// 開始x位置 [mm]
	FLOAT	f_start_y;					// 開始y位置 [mm]
	FLOAT	f_final_x;					// 最終x位置 [mm]
	FLOAT	f_final_y;					// 最終y位置 [mm]
	FLOAT	f_final_ang;				// 角減速時の最終角度 [rad]	
	FLOAT	f_maxAngleV		= 0;		// 最大角速度[rad/s]
	FLOAT	f_timeAcc		= 0;		// 加速時間[s]
	FLOAT	f_accAngle		= 0;		// 加速角度[rad]
	FLOAT	f_timeConst		= 0;		// 等速時間[s]
	FLOAT	f_constAngle	= 0;		// 等速角度[rad]
	FLOAT	f_ang			= 0;		// 演算用、角度 [rad]
	FLOAT	f_time			= 0;		// 演算用、時間 [s]
	FLOAT	f_x;						// 演算用x位置 [mm]
	FLOAT	f_y;						// 演算用y位置 [mm]
	USHORT	i = 0;						// ループ用
	stSLA* 	p_adr = &st_Sla[en_mode];		// 記録する走行データ

	/* スラロームに応じた設定値からスラロームに必要なパラメータを演算する */
	switch(en_mode){

		case SLA_90:
			f_start_x   = HALF_BLOCK;
			f_start_y   = 0.0f;
			f_final_x   = BLOCK;
			f_final_y   = HALF_BLOCK;
			f_final_ang = 90.0f * DEG_TO_RAD;
			break;

		case SLA_45:
			f_start_x   = HALF_BLOCK;
			f_start_y   = 0.0f;
			f_final_x   = BLOCK * 0.75f;
			f_final_y   = BLOCK * 0.75f;
			f_final_ang = 45.0f * DEG_TO_RAD;
			break;
			
		case SLA_N90:
			f_start_x   = HALF_BLOCK * 0.5f * 1.4142f;
			f_start_y   = 0.0f;
			f_final_x   = HALF_BLOCK * 1.4142f;
			f_final_y   = HALF_BLOCK * 0.5f * 1.4142f;
			f_final_ang = 90.0f * DEG_TO_RAD;
			break;
			
		case SLA_135:
			f_start_x   = HALF_BLOCK;
			f_start_y   = 0.0f;
			f_final_x   = BLOCK * 1.25f;
			f_final_y   = BLOCK * 0.25;
			f_final_ang = 135.0f * DEG_TO_RAD;
			break;
	}

	/* 加減速角度の算出 */
	f_maxAngleV		= f_g / f_speed;							// 最大角速度[rad/s] （ω[rad/s] = g[mm/s^2] / v[mm/s] ）
	f_timeAcc		= f_maxAngleV / f_angAcc;					// 最大の角速度になるまでの加速時間[s]
	f_accAngle		= 0.5f * f_angAcc * f_timeAcc * f_timeAcc;	// 加速をする区間の角度[rad] (θ[rad] = 1/2 * a[rad/s^2] * t[s]^2 )
	f_constAngle	= f_final_ang - f_accAngle * 2;				// 等角速度の区間の角度[rad] (θ[rad] = Total角度 - 加速角度 + 減速角度 )
	f_timeConst		= f_constAngle / f_maxAngleV;				// 最大の角速度で動作する時間[s]（ t[s] = θ[rad] / ω[rad/s] ）

	/* -------------------------------- */
	/*  スラローム完了時の位置を求める  */
	/* -------------------------------- */
	/* 座標開始位置 */
	f_x		= f_start_x;
	f_y		= f_start_y;

	/* 加速時の座標演算 */
	for( i=0; i<(USHORT)(f_timeAcc*1000); i++ ){				// [msec]
	
		f_time	=  0.001f * (FLOAT)i;								// 時間[s]
		f_ang	=  0.5f * f_angAcc * f_time * f_time;				// 角度[rad] (θ[rad] = 1/2 * a[rad/s^2] * t[s]^2 )
		f_x		+= f_speed * (FLOAT)sin( f_ang ) * 0.001f;			// X座標[mm] 
		f_y		+= f_speed * (FLOAT)cos( f_ang ) * 0.001f;			// Y座標[mm]
	}
	
	/* 等速時の座標演算 */
	for( i=0; i<(USHORT)(f_timeConst*1000); i++ ){				// [msec]
	
		f_time	 = 0.001f * (FLOAT)i;							// 時間[s]
		f_ang	 = f_accAngle + f_maxAngleV * f_time;			// 角度[rad] (θ[rad] = ω[rad/s] * t[s] )
		f_x		+= f_speed * (FLOAT)sin( f_ang ) * 0.001f;		// X座標[mm] 
		f_y		+= f_speed * (FLOAT)cos( f_ang ) * 0.001f;		// Y座標[mm]
	}

	/* 減速時の座標演算 */
	for( i=0; i<(USHORT)(f_timeAcc*1000); i++ ){				// [msec]
	
		f_time	 = 0.001f * (FLOAT)i;							// 時間[s]
		f_ang	 = f_accAngle + f_constAngle +0.5f * f_angAcc * f_time * f_time;	// 角度[rad] (θ[rad] = 1/2 * a[rad/s^2] * t[s]^2 )
		f_x		+= f_speed * (FLOAT)sin( f_ang ) * 0.001f;		// X座標[mm] 
		f_y		+= f_speed * (FLOAT)cos( f_ang ) * 0.001f;		// Y座標[mm]
	}

	/* ---------------------------- */
	/*  スラローム用パラメータ作成  */
	/* ---------------------------- */
	p_adr->f_speed				= f_speed;										// 進入速度[mm/s]
	printf("進入速度 %5.2f\n\r",f_speed);
	p_adr->f_angAcc				= f_angAcc * RAD_TO_DEG ;						// 角加速度[deg/s]
	printf("角加速度 %5.2f\n\r",f_angAcc * RAD_TO_DEG);
	p_adr->f_angvel				= f_maxAngleV * RAD_TO_DEG;						// 最大角速度を算出 最大角速度[deg/s]
	printf("最大角速度 %5.2f\n\r",f_maxAngleV * RAD_TO_DEG);
	p_adr->us_accAngvelTime		= (USHORT)( f_timeAcc * 1000.0f );				// 角加速時間[msec]
	printf("角加速時間 %5.2f\n\r",f_timeAcc * 1000.0f);
	p_adr->us_constAngvelTime	= (USHORT)( f_timeConst * 1000.0f );			// 等角速時間[msec]
	printf("等角速時間 %5.2f\n\r",f_timeConst * 1000.0f);
	p_adr->f_ang_AccEnd			= f_accAngle * RAD_TO_DEG;						// 角加速完了角度[deg]
	printf("角加速 %5.2f\n\r",f_accAngle * RAD_TO_DEG);
	p_adr->f_ang_ConstEnd		= ( f_accAngle + f_constAngle ) * RAD_TO_DEG;	// 等角速度完了角度[deg]
	printf("角等速 %5.2f\n\r",( f_accAngle + f_constAngle ) * RAD_TO_DEG);
	p_adr->f_ang_Total			= f_final_ang * RAD_TO_DEG;						// 全移動角度[deg]
	printf("全移動 %5.2f\n\r",f_final_ang * RAD_TO_DEG);
	
	/* 必要な進入と退出の距離を算出する */
	switch(en_mode){
		case SLA_90:
			p_adr->f_escapeLen = f_final_x - f_x ;//-4
			p_adr->f_entryLen  = f_final_y - f_y ;//+8
			break;

		case SLA_45:
			p_adr->f_escapeLen = 1.4142f * ( f_final_x - f_x );
			p_adr->f_entryLen  = f_final_y - f_y - ( f_final_x - f_x );
			break;

		case SLA_N90:
			p_adr->f_escapeLen = f_final_x - f_x;
			p_adr->f_entryLen  = f_final_y - f_y;
			break;

		case SLA_135:
			p_adr->f_escapeLen = 1.4142f * ( f_final_x - f_x );
			p_adr->f_entryLen  = f_final_y - f_y + ( f_final_x - f_x );
			break;
	}
	printf("entry %5.2f\n\r",f_final_x - f_x);
	printf("escape %5.2f\n\r",f_final_y - f_y);
}

// *************************************************************************
//   機能		： スラロームの走行パラメータの格納先アドレスを取得する
//   注意		： 
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.09.18			sato		新規
// *************************************************************************/
PUBLIC stSLA* PARAM_getSra( enSLA_TYPE en_mode )
{
	return &st_Sla[en_mode];
}
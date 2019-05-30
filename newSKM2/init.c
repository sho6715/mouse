// *************************************************************************
//   ロボット名	： 電研ベーシックDCマウス、サンシャイン
//   概要		： initファイル（レジスタ設定ファイル他）
//   注意		： なし
//   メモ		： WORD = BYTE＊2 LONG ＝BYTE＊４
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.30			sato			新規（ファイルのインクルード）
// *************************************************************************/

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <init.h>							//レジスタ


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




//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************

// *************************************************************************
//   機能		： クロックのレジスタ設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.30			sato			新規
// *************************************************************************/
PRIVATE void init_clock(void)
{


	SYSTEM.PRCR.WORD = 0xa50b;		//クロックソース選択の保護の解除

	SYSTEM.PLLCR.WORD = 0x0F00;		/* PLL 逓倍×16 入力1分周 (12.000MHz * 16 = 192MHz)*/
	SYSTEM.PLLCR2.BYTE = 0x00;		/* PLL ENABLE */
	
	SYSTEM.PLLWTCR.BYTE     = 0x0F;		/* 4194304cycle(Default) */

	
	// ICK   : 192/2 = 96MHz 		// システムクロック CPU DMAC DTC ROM RAM
	// PCLKA : 192/2 = 96MHz	 	// 周辺モジュールクロックA ETHERC、EDMAC、DEU
	// PCLKB : 192/4 = 48MHz 		// 周辺モジュールクロックB 上記以外 PCLKB=PCLK
/*	
	SYSTEM.SCKCR.BIT.FCK=0x02;		//FCLK MAX 50MHz  192/4
	SYSTEM.SCKCR.BIT.ICK=0x01;		//ICLK MAX 100MHz 192/2
	SYSTEM.SCKCR.BIT.PSTOP1=0x01;		//BCLK 出力停止
	SYSTEM.SCKCR.BIT.PSTOP0=0x01;		//SDCLK 出力停止
	SYSTEM.SCKCR.BIT.BCK=0x02;		//BCLK MAX 100MHz ICLK以下にする必要がある192/4
	SYSTEM.SCKCR.BIT.PCKA=0x01;		//PCLKA MAX 100MHz 192/2
	SYSTEM.SCKCR.BIT.PCKB=0x02;		//PCLKB MAX 50MHz 192/4
	//上記の設定では正しくclock設定ができないため下記のように一括で設定すること
*/
	SYSTEM.SCKCR.LONG = 0x21C21211;		//FCK1/4 ICK1/2 BCLK停止 SDCLK停止 BCK1/4 PCLKA1/2 PCLKB1/4
/*
	SYSTEM.SCKCR2.BIT.UCK=0x03;		//UCLK MAX 48MHz 192/4
	SYSTEM.SCKCR2.BIT.IEBCK=0x02;		//IECLK MAX 50MHz 192/4
*/
	SYSTEM.SCKCR2.WORD = 0x0032;		/* UCLK1/4 IEBCK1/4 */
	SYSTEM.BCKCR.BYTE = 0x01;		/* BCLK = 1/2 */
	
	SYSTEM.SCKCR3.WORD = 0x0400;		//PLL回路選択

}

// *************************************************************************
//   機能		： IOのレジスタ設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.30			sato			新規
// *************************************************************************/
PRIVATE void init_io(void)
{
	/* ================== */
	/*  GPIO(汎用入出力)  */
	/* ================== */
	/* 出力値 */
	PORTE.PODR.BIT.B0			= 0;		//ポートE-0の初期出力0[V]（デバッグ用LED）
	PORTE.PODR.BIT.B1			= 0;		//ポートE-1の初期出力0[V]（デバッグ用LED）
	PORTE.PODR.BIT.B2			= 0;		//ポートE-2の初期出力0[V]（デバッグ用LED）
	PORTE.PODR.BIT.B3			= 0;		//ポートE-3の初期出力0[V]（デバッグ用LED）
	
	PORTE.PODR.BIT.B4			= 0;		//ポートE-4の初期出力0[V]（電源用赤LED）
	PORTE.PODR.BIT.B5			= 0;		//ポートE-5の初期出力0[V]（電源用緑LED）
	/*センサ*/
	PORTA.PODR.BIT.B1			= 0;		//ポートA-1の初期出力0[V]（センサLED）
	PORTA.PODR.BIT.B3			= 0;		//ポートA-3の初期出力0[V]（センサLED）
	PORTA.PODR.BIT.B4			= 0;		//ポートA-4の初期出力0[V]（センサLED）
	PORTA.PODR.BIT.B6			= 0;		//ポートA-6の初期出力0[V]（センサLED）
	//DCM
	PORTA.PODR.BIT.B0			= 0;		//PWM(TPU0)
	PORTB.PODR.BIT.B0			= 0;		//PWM(TPU3)
	PORTB.PODR.BIT.B1			= 0;		//AIN1
	PORTB.PODR.BIT.B3			= 0;		//AIN2
	PORTB.PODR.BIT.B5			= 0;		//BIN1
	PORTB.PODR.BIT.B6			= 0;		//BIN2
	PORTB.PODR.BIT.B7			= 0;		//STBY
	
//	PORT3.PODR.BIT.B			= 0;		//ジャンプ配線の対処（ショートしてない？）入力させちゃって処理しなければいいだけでは
	
	/* 入出力設定 */
	PORTE.PDR.BIT.B0			= 1;		//ポートE-0を出力に設定
	PORTE.PDR.BIT.B1			= 1;		//ポートE-1を出力に設定
	PORTE.PDR.BIT.B2			= 1;		//ポートE-2を出力に設定
	PORTE.PDR.BIT.B3			= 1;		//ポートE-3を出力に設定
	
	PORTE.PDR.BIT.B4			= 1;		//ポートE-4を出力に設定
	PORTE.PDR.BIT.B5			= 1;		//ポートE-5を出力に設定
	/*センサ*/
	PORTA.PDR.BIT.B1			= 1;		//ポートA-1を出力に設定
	PORTA.PDR.BIT.B3			= 1;		//ポートA-3を出力に設定
	PORTA.PDR.BIT.B4			= 1;		//ポートA-4を出力に設定
	PORTA.PDR.BIT.B6			= 1;		//ポートA-6を出力に設定
	//DCM
//	PORTA.PDR.BIT.B0			= 1;		//PWM(TPU0)
//	PORTB.PDR.BIT.B0			= 1;		//PWM(TPU3)
	PORTB.PDR.BIT.B1			= 1;		//AIN1
	PORTB.PDR.BIT.B3			= 1;		//AIN2
	PORTB.PDR.BIT.B5			= 1;		//BIN1
	PORTB.PDR.BIT.B6			= 1;		//BIN2
	PORTB.PDR.BIT.B7			= 1;		//STBY
	
	/* 入力プルアップ設定 */
	PORTC.PCR.BIT.B2			= 1;		// ポートC-2はプルアップを使用	(プッシュスイッチ用)
	PORTC.PCR.BIT.B3			= 1;		// ポートC-3はプルアップを使用	(プッシュスイッチ用)
	
	/* ポート入力データレジスタ */
	PORTC.PIDR.BIT.B2			= 1;		// ポートC-6を入力ポートと接続する
	PORTC.PIDR.BIT.B3			= 1;		// ポートC-7を入力ポートと接続する
	
	
	/* ========= */
	/*  A/D変換  */
	/* ========= */
	SYSTEM.PRCR.WORD = 0xA502;
	SYSTEM.MSTPCRA.BIT.MSTPA17 	= 0;		// AD(12bit)スタンバイ解除
	SYSTEM.PRCR.WORD = 0xA500;
	
	S12AD.ADCSR.BIT.CKS 		= 3;		// PCLKで変換
	
	//TPUエンコーダ用
	MPC.PWPR.BIT.B0WI  = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P14PFS.BIT.PSEL		= 4;
	MPC.P15PFS.BIT.PSEL		= 4;
	MPC.P16PFS.BIT.PSEL		= 4;
	MPC.P17PFS.BIT.PSEL		= 4;
	MPC.PWPR.BIT.B0WI  = 1;
	MPC.PWPR.BIT.PFSWE = 0;
	PORT1.PMR.BIT.B4		= 1;
	PORT1.PMR.BIT.B5		= 1;
	PORT1.PMR.BIT.B6		= 1;
	PORT1.PMR.BIT.B7		= 1;
	
	//TPUPWM用
	MPC.PWPR.BIT.B0WI  = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.PA0PFS.BIT.PSEL		= 3;
	MPC.PB0PFS.BIT.PSEL		= 3;
	MPC.PWPR.BIT.B0WI  = 1;
	MPC.PWPR.BIT.PFSWE = 0;
	PORTA.PMR.BIT.B0		= 1;
	PORTB.PMR.BIT.B0		= 1;
	
	
}

// *************************************************************************
//   機能		： MTUのレジスタ設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.30			sato			新規
// *************************************************************************/
PRIVATE void init_mtu(void)
{
	SYSTEM.PRCR.WORD = 0xA502;
	MSTP(MTU0) 	= 0;						// 
	MSTP(MTU1) 	= 0;
	MSTP(MTU3) 	= 0;
	MSTP(MTU2)	= 0;
	SYSTEM.PRCR.WORD = 0xA500;
	
	MTU.TSTR.BYTE = 0;						//タイマ動作ストップ
	
	// -----------------------
	//  システム用(MTU0)
	// -----------------------
	/* タイマ割り込みの設定 */
	MTU0.TCR.BIT.CCLR 			= 1;		// TGRAのコンペアマッチでTCNTクリア
	MTU0.TCR.BIT.TPSC 			= 2;		// PCLK(48MHz)/16 で1カウント
	MTU0.TIER.BIT.TGIEA			= 1;		// TGRAとのコンペアマッチで割り込み許可
	MTU0.TGRA 					= 750 * 4;	// 1msec毎に割り込み
	MTU0.TCNT 					= 0;		// タイマクリア
	
	IEN(MTU0,TGIA0) = 1;	//割り込み要求を許可 
	IPR(MTU0,TGIA0) = 7;	//割り込み優先度を次点に設定
	IR(MTU0,TGIA0)	= 0;	//割り込みステータスフラグをクリア
	
	MTU.TSTR.BIT.CST0 = 0;	//タイマストップ
	
	// -----------------------
	//  バッテリー用(MTU1)
	// -----------------------
	/* タイマ割り込みの設定 */
	MTU1.TCR.BIT.CCLR 			= 1;		// TGRAのコンペアマッチでTCNTクリア
	MTU1.TCR.BIT.TPSC 			= 2;		// PCLK(48MHz)/16 で1カウント
	MTU1.TIER.BIT.TGIEA			= 1;		// TGRAとのコンペアマッチで割り込み許可
	MTU1.TGRA 					= 7500 * 4;	// 10msec毎に割り込み
	MTU1.TCNT 					= 0;		// タイマクリア
	
	IEN(MTU1,TGIA1) = 1;	//割り込み要求を許可 
	IPR(MTU1,TGIA1) = 6;	//割り込み優先度を次点に設定
	IR(MTU1,TGIA1)	= 0;	//割り込みステータスフラグをクリア
	
	MTU.TSTR.BIT.CST1 = 0;	//タイマストップ
	
	// -----------------------
	//  センサ用(MTU3)
	// -----------------------
	/* タイマ割り込みの設定 */
	MTU3.TCR.BIT.CCLR 			= 1;		// TGRAのコンペアマッチでTCNTクリア
	MTU3.TCR.BIT.TPSC 			= 2;		// PCLK(48MHz)/16 で1カウント
	MTU3.TIER.BIT.TGIEA			= 1;		// TGRAとのコンペアマッチで割り込み許可
	MTU3.TGRA 					= 750;	// 250usec毎に割り込み
	MTU3.TCNT 					= 0;		// タイマクリア
	
	IEN(MTU3,TGIA3) = 1;	//割り込み要求を許可 
	IPR(MTU3,TGIA3) = 8;	//割り込み優先度を次点に設定
	IR(MTU3,TGIA3)	= 0;	//割り込みステータスフラグをクリア
	
	MTU.TSTR.BIT.CST3 = 0;	//タイマストップ
	
	// -----------------------
	//  センサ用(MTU2)
	// -----------------------
	/* タイマ割り込みの設定 */
/*	MTU2.TCR.BIT.CCLR 			= 1;		// TGRAのコンペアマッチでTCNTクリア
	MTU2.TCR.BIT.TPSC 			= 2;		// PCLK(48MHz)/16 で1カウント
	MTU2.TIER.BIT.TGIEA			= 1;		// TGRAとのコンペアマッチで割り込み許可
	MTU2.TGRA 					= 15000;	// 5msec毎に割り込み
	MTU2.TCNT 					= 0;		// タイマクリア
	
	IEN(MTU2,TGIA2) = 1;	//割り込み要求を許可 
	IPR(MTU2,TGIA2) = 4;	//割り込み優先度を次点に設定
	IR(MTU2,TGIA2)	= 0;	//割り込みステータスフラグをクリア
	
	MTU.TSTR.BIT.CST2 = 0;	//タイマストップ
*/	
}

// *************************************************************************
//   機能		： TPUのレジスタ設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.07.03			sato			新規
// *************************************************************************/
PRIVATE void init_tpu(void)
{
	SYSTEM.PRCR.WORD 	= 0xA502;
	MSTP(TPU1) 			= 0;
	MSTP(TPU2) 			= 0;
	MSTP(TPU0) 			= 0;
	MSTP(TPU3) 			= 0;
	SYSTEM.PRCR.WORD 	= 0xA500;
	// -----------------------
	//  右モータ位相計測用(TPU1)
	// -----------------------
	/* タイマ割り込みの設定 */
	TPU1.TCR.BIT.CCLR 			= 0;		// コンペアマッチでTCNTクリア禁止
	TPU1.TIER.BIT.TGIEA			= 0;		// TGRAとのコンペアマッチで割り込み未発生
	TPU1.TMDR.BIT.MD			= 7;		// 位相計測モード4
	TPU1.TGRA 				= 0;		// 始めは未設定
	TPU1.TCNT 				= 32768;	// 初期化
	
	// -----------------------
	//  左モータ位相計測用(TPU2)
	// -----------------------
	/* タイマ割り込みの設定 */
	TPU2.TCR.BIT.CCLR 			= 0;		// コンペアマッチでTCNTクリア禁止
	TPU2.TIER.BIT.TGIEA			= 0;		// TGRAとのコンペアマッチで割り込み未発生
	TPU2.TMDR.BIT.MD			= 7;		// 位相計測モード4
	TPU2.TGRA 				= 0;		// 始めは未設定
	TPU2.TCNT 				= 32768;	// 初期化
	
	// -----------------------
	//  右モータPWM出力用(TPU0)
	// -----------------------
	TPU0.TCR.BIT.CCLR 			= 1;		// TGRAのコンペアマッチでTCNTクリア
	TPU0.TCR.BIT.CKEG			= 1;		// CKEG立ち上がりエッジでカウント
	TPU0.TCR.BIT.TPSC 			= 1;		// PCLK(48MHz)/4 で1カウント
	TPU0.TMDR.BIT.MD			= 2;		// PWM モード 1
	TPU0.TIORH.BIT.IOA			= 2;		// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 1 出力
	TPU0.TIORH.BIT.IOB			= 1;		// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	TPU0.TGRA 				= 600;		// 周期(50usec)
	TPU0.TGRB 				= 300;		// onDuty
	TPU0.TCNT 				= 0;		// タイマクリア
	
	// -----------------------
	//  左モータPWM出力用(TPU3)
	// -----------------------
	TPU3.TCR.BIT.CCLR 			= 1;		// TGRAのコンペアマッチでTCNTクリア
	TPU3.TCR.BIT.CKEG			= 1;		// CKEG立ち上がりエッジでカウント
	TPU3.TCR.BIT.TPSC 			= 1;		// PCLK(48MHz)/4 で1カウント
	TPU3.TMDR.BIT.MD			= 2;		// PWM モード 1
	TPU3.TIORH.BIT.IOA			= 2;		// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 1 出力
	TPU3.TIORH.BIT.IOB			= 1;		// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	TPU3.TGRA 				= 600;		// 周期(50usec)
	TPU3.TGRB 				= 300;		// onDuty
	TPU3.TCNT 				= 0;		// タイマクリア
	
}

// *************************************************************************
//   機能		： SCIのレジスタ設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.07.03			sato			新規
// *************************************************************************/
PRIVATE void init_sci(void)
{
/*	SYSTEM.PRCR.WORD 	= 0xA502;
	MSTP(SCI1) 			= 0;
	SYSTEM.PRCR.WORD 	= 0xA500;
	SCI1.SCR.BYTE		= 0x00;		// TE=0、RE=0、CKE1=0
	while (0x00 != (SCI1.SCR.BYTE & 0xF0));	//割り込み要求が禁止されるまで待つ
	PORT2.PODR.BIT.B6 = 1;			//TXDのDirctionの切り替え後の値をhigh
	PORT2.PDR.BIT.B6 = 1;			//出力に設定
	PORT3.PDR.BIT.B0 = 0;			//入力に設定
	PORT2.PMR.BIT.B6 = 0;			//汎用ポートに設定
	PORT3.PMR.BIT.B0 = 0;			//汎用ポートに設定
	MPC.PWPR.BIT.B0WI  = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P26PFS.BIT.PSEL = 0x0A;		//TXD1
	MPC.P30PFS.BIT.PSEL = 0x0A;		//RXD1
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI  = 1;
	PORT3.PMR.BIT.B0 = 1;			//周辺機能(RXD1)として使用
	SCI1.SCR.BIT.CKE = 0;
	SCI1.SMR.BYTE = 0x00;			//1stopbit parityなし 8bit 調歩同期
	SCI1.SCMR.BYTE = 0xF2;			//S=32clock
	SCI1.SEMR.BYTE = 0x00;
  	SCI1.BRR =38; 				//@48MHz 38400bps
  	SCI1.SCR.BYTE =0xF0;			//送信割り込み禁止
	PORT2.PMR.BIT.B6 = 1;			//周辺機能(TXD1)として使用
	SCI1.SCR.BIT.TE = 1;
	SCI1.SCR.BIT.RE = 1;
	
	IEN(SCI1,RXI1) 		= 1;
	IPR(SCI1,RXI1) 		= 2;
	IR(SCI1,RXI1) 		= 0;
*/
//	IR(SCI1,TXI1) 		= 0;
	
	SYSTEM.PRCR.WORD 	= 0xA502;
	MSTP(SCI1) 			= 0;
	SYSTEM.PRCR.WORD 	= 0xA500;
	SCI1.SCR.BYTE	= 0x00;
	while(0x00 != (SCI1.SCR.BYTE & 0xF0));
	SCI1.SMR.BYTE	= 0x00;
//	PORT2.PODR.BIT.B6 = 1;			//TXDのDirctionの切り替え後の値をhigh
//	PORT2.PDR.BIT.B6 = 1;			//出力に設定
//	PORT3.PDR.BIT.B0 = 0;			//入力に設定
//	PORT2.PMR.BIT.B6 = 0;			//汎用ポートに設定
//	PORT3.PMR.BIT.B0 = 0;			//汎用ポートに設定
	MPC.PWPR.BIT.B0WI  = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P26PFS.BIT.PSEL = 0x0A;		//TXD1
	MPC.P30PFS.BIT.PSEL = 0x0A;		//RXD1
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI  = 1;
	PORT3.PMR.BIT.B0 = 1;			//周辺機能(RXD1)として使用
	PORT2.PMR.BIT.B6 = 1;			//周辺機能(TXD1)として使用
	SCI1.BRR	= 78;
	SCI1.SEMR.BIT.ABCS = 1;
	SCI1.SCR.BYTE	=0x30;

	IEN(SCI1,RXI1) 		= 1;
	IEN(SCI1,TXI1) 		= 1;
	ICU.IPR[217].BIT.IPR	= 5;

}

// *************************************************************************
//   機能		： SPI設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.30			sato			新規
// *************************************************************************/
PUBLIC void init_spi(void)
{
	SYSTEM.PRCR.WORD 	= 0xA502;
	MSTP(RSPI0) 		= 0;
	SYSTEM.PRCR.WORD 	= 0xA500;
	
	RSPI0.SPCR.BYTE = 0x00;
	
//	PORTC.PMR.BIT.B4	= 1;
	PORTC.PMR.BIT.B5	= 1;
	PORTC.PMR.BIT.B6	= 1;
	PORTC.PMR.BIT.B7	= 1;
	MPC.PWPR.BIT.B0WI = 0;
	MPC.PWPR.BIT.PFSWE = 1;
//	MPC.PC4PFS.BYTE = 0x0D;
	MPC.PC5PFS.BYTE = 0x0D;  /* RSPI RSPCKA */       
	MPC.PC6PFS.BYTE = 0x0D;  /* RSPI MOSIA  */
	MPC.PC7PFS.BYTE = 0x0D;  /* RSPI MISOA  */
	MPC.PWPR.BIT.PFSWE = 0;
	MPC.PWPR.BIT.B0WI = 1;
	PORTC.PODR.BIT.B4 = 1;
	PORTC.PDR.BIT.B4 = 1;
	
	RSPI0.SPBR		= 2;
	RSPI0.SPCMD0.WORD	= 0x0783;	
	
	RSPI0.SPCR.BYTE = 0xF8;

}

// *************************************************************************
//   機能		： レジスタ設定
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.30			sato			新規
// *************************************************************************/
PUBLIC void CPU_init(void)
{
	init_clock();
	init_io();
	init_mtu();
	init_tpu();
	init_sci();
	init_spi();
}

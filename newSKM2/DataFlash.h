// *************************************************************************
//   ロボット名		： AEGIS
//   概要		： DataFlashのヘッダファイル
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.11.25			sato			新規（ファイルのインクルード）
// *************************************************************************/
// 多重コンパイル抑止
#ifndef _DATAFLASH_H
#define _DATAFLASH_H

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <init.h>
#include <parameters.h>

//**************************************************
// 定義（define）
//**************************************************
#define REF_SEN_R_ADD 0x00100000
#define REF_SEN_L_ADD 0x00100002
#define TH_SEN_R_ADD  0x00100004
#define TH_SEN_L_ADD  0x00100006
#define TH_SEN_FR_ADD 0x00100008
#define TH_SEN_FL_ADD 0x0010000a

#define MAP_ADD		  0x00100800

//**************************************************
// 列挙体（enum）
//**************************************************


//**************************************************
// 構造体（struct）
//**************************************************


//**************************************************
// グローバル変数
//**************************************************
void map_write(void);
void map_erase(void);
void hw_dflash_init(void);
void write_eeflash(ULONG addr, USHORT *data);
void erase(ULONG addr);
char blank_check(ULONG addr);
void map_copy(void);
static void transition_read(void);
static void transition_pe(void);
PUBLIC void flash_test(USHORT r);
PUBLIC void Flash_read(USHORT *add, USHORT *data);


//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************

#endif
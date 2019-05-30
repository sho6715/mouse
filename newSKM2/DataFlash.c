// *************************************************************************
//   ロボット名		： AEGIS
//   概要		： DataFlashファイル
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.11.25			sato			新規（ファイルのインクルード）
// *************************************************************************/

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <stdio.h>
#include <math.h>
#include <init.h>
#include <search2.h>
#include <parameters.h>
#include <DataFlash.h>

//**************************************************
// 定義（define）
//**************************************************
#define BASEADDR	0x00100000
#define FLASHSIZE	0x7fff		//32kB

#define BLOCKSIZE	32		//bytes
#define BLOCKCOUNT	1024		//count

#define write_byte(A,D)	*(UCHAR *)(A)=(D)
#define write_word(A,D)	*(USHORT *)(A)=(D)


//**************************************************
// 列挙体（enum）
//**************************************************


//**************************************************
// 構造体（struct）
//**************************************************


//**************************************************
// グローバル変数
//**************************************************
const UCHAR *BaseAddr = (const unsigned char *)BASEADDR;

static int isErr = 0;

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************

// *************************************************************************
//   機能		： mapストレージ書き込み
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.11.25			sato		新規
// *************************************************************************/
void map_write(void)
{
	short i;
	unsigned short *map_add;
	map_add = (unsigned short*)g_sysMap;
	
	//DataFlashイレース
	for(i=0;i<8;i++){
		erase((ULONG)(MAP_ADD+i*32));
	}
	//マップデータをDataFlashに書き込む
	for(i=0;i<128;i++){
		write_eeflash((ULONG)(MAP_ADD+i*2),map_add);
		map_add++;
	}
}

// *************************************************************************
//   機能		： マップデータをRAMにコピー
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.11.25			sato		新規
// *************************************************************************/
void map_copy(void)
{
	short i;
	unsigned short *map_add;
	map_add = (unsigned short*)&g_sysMap;

	//マップデータをRAMにコピー
	for(i=0;i<128;i++){
//		*map_add = *(unsigned short *)(MAP_ADD+i*2);
		Flash_read((unsigned short *)(MAP_ADD+i*2),map_add);
		map_add++;
//		printf("map_add %x\n\r",map_add);
	}
}

// *************************************************************************
//   機能		： mapイレーズ
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.11.28			sato		新規
// *************************************************************************/
void map_erase(void)
{
	short i;
	
	//DataFlashイレース
	for(i=0;i<8;i++){
		erase((ULONG)(MAP_ADD+i*32));
	}
}

// *************************************************************************
//   機能		： FCUをリセットする。エラー処理用
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.11.25			sato		新規
// *************************************************************************/
static void fcu_reset(void)
{
	FLASH.FRESETR.BIT.FRESET = 1;
	TIME_wait(2);
	FLASH.FRESETR.BIT.FRESET = 0;
}


//FCU処理待ち timeoutにタイムアウトカウント[mS]を設定
static void wait_fcuRdy(int timeout)
{
	int isTimeout = 0;
	
	TIME_wait(timeout);
	
	if(FLASH.FSTATR0.BIT.FRDY == 0)//P/E処理中
	{
		isTimeout = 1;
//		//SCI_printf("FCU time out\n\r");
	}
	
	//タイムアウトしていたらリセット
	if(isTimeout == 1)
	{
		fcu_reset();
	}
	
	return;
}


//エラーを確認し、エラーがあれば修正する
static void check_error(void)
{
	int iserr = 0;
	
	iserr |= FLASH.FSTATR0.BIT.ILGLERR;//FCUは不正なコマンドや不正やE2データフラッシュアクセスを検出
	iserr |= FLASH.FSTATR0.BIT.ERSERR;//イレース中にエラー発生
	iserr |= FLASH.FSTATR0.BIT.PRGERR;//プログラム中にエラー発生
	
	//printf("[%s]",tmp_source);
	if(iserr == 0)
	{
//		printf("No error\n\r");
		return;	//no error
	}
	
	isErr = 1;
//	_LED(0x07);
	LED4 = LED4_ALL_ON;
//	//SCI_printf("FCU Error\n\r");
//	printf("FCU Error\n\r");
	if(FLASH.FSTATR0.BIT.ILGLERR == 1)
	{
//		printf("FSTATR0:%02X\nFSTATR1:%02X\nFASTAT:%02X\n\n", FLASH.FSTATR0.BYTE, FLASH.FSTATR1.BYTE, FLASH.FASTAT.BYTE);
		
		if(FLASH.FASTAT.BYTE != 0x10)
		{
			FLASH.FASTAT.BYTE = 0x10;//フラグをクリア
		}
	}
	
	write_byte(BaseAddr, 0x50);	//status clear
}



//指定したブロックをイレースする
//P1888 図47.14 ROM/E2データフラッシュイレース方法 
//イレースは32バイト
void erase(ULONG addr)
{
	volatile UCHAR *a = (UCHAR *)addr;
	transition_pe();
		
		*a = 0x20;
		*a = 0xD0;
		wait_fcuRdy(5);
		check_error();
	
	transition_read();
}


//指定した領域に書き込む
//P1887 図47.13 ROM/E2データフラッシュへのプログラム方法 
void write_eeflash(ULONG addr, USHORT *data)
{
	volatile USHORT *a = (USHORT *)addr;
	volatile UCHAR *b = (UCHAR *)addr;
	
	transition_pe();
	{
		*b = 0xE8;
		*b = 0x01;
		*a = *data;
		*b = 0xD0;	
		wait_fcuRdy(3);
		check_error();
	}
	transition_read();
	
}

char blank_check(ULONG addr)
{
	volatile USHORT *a = (USHORT*)addr;
	
	transition_pe();
	
	FLASH.FMODR.BIT.FRDMD = 1;
	FLASH.DFLBCCNT.WORD = 0x0000;
	*a = 0x71;
	*a = 0xD0;
	wait_fcuRdy(2);
	check_error();
	transition_read();
	
	return FLASH.DFLBCSTAT.BIT.BCST;
	
}

//FCUを読み込みモードに遷移させる
static void transition_read(void)
{
	FLASH.FENTRYR.WORD = 0xAA00;	//E2Flash P/E mode

	while(FLASH.FENTRYR.WORD != 0x0000);
	
	/* Flash write/erase disabled */
	FLASH.FWEPROR.BYTE = 0x02;
}

//FCUをP/Eモードに遷移させる
static void transition_pe(void)
{
	
	FLASH.FENTRYR.WORD = 0xAA00;
	while(0x0000 != FLASH.FENTRYR.WORD);
	
	FLASH.FENTRYR.WORD = 0xAA80;	//E2Flash P/E mode
	check_error();
	
	FLASH.FWEPROR.BYTE = 0x01;	//フラッシュP/Eプロテクトレジスタ
					//P/E ロックビットのP/E ロックビットの読み出し、ブランクチェックの許可
}

//DataFlashを初期化する
void hw_dflash_init(void)
{
	unsigned int i;
	
	FLASH.DFLRE0.WORD = 0x2DFF;	// block read enable E2データフラッシュ読み出し許可レジスタ
	FLASH.DFLRE1.WORD = 0xD2FF;	// block read enable E2データフラッシュ読み出し許可レジスタ
	FLASH.DFLWE0.WORD = 0x1EFF;	// block P/E enable E2データフラッシュP/E許可レジスタ
	FLASH.DFLWE1.WORD = 0xE1FF;	// block P/E enable E2データフラッシュP/E許可レジスタ
//P1883 図47.10 P/E処理の概略フロー
//FCU RAMへのファームウェア転送
	if(FLASH.FENTRYR.WORD != 0x0000)
	{
		FLASH.FENTRYR.WORD = 0xAA00;//FCU停止
	}
	FLASH.FCURAME.WORD=0xC401;
	
	for(i=0;i<8192;i++)
	{
		*(unsigned char *)(0x007F8000+i) = *(unsigned char *)(0xFEFFE000+i);
	}
	
	transition_pe();
	FLASH.PCKAR.BIT.PCKA = 48;	//FlashIF clock is 48MHz
	
	//周辺クロック通知コマンド
	write_byte(BaseAddr, 0xE9);
	write_byte(BaseAddr, 0x03);
	write_word(BaseAddr, 0x0F0F);
	write_word(BaseAddr, 0x0F0F);
	write_word(BaseAddr, 0x0F0F);
	write_byte(BaseAddr, 0xD0);
	
	wait_fcuRdy(2);//63us tPCKAは63usでよい。タイマーとして1msがありでそれを使用しているため1msにしている
	transition_read();


}	

PUBLIC void flash_test(USHORT r)
{
	USHORT ram = r;
	USHORT *read;
	erase((ULONG)0x00101000);
	write_eeflash((ULONG)0x00101000, &ram);
	TIME_wait(1000);
			
	*read = *(USHORT *)0x00101000;
	printf("testdata %d\r\n",read);
	
}

// *************************************************************************
//   機能		： データフラッシュリード関数
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.11.26			sato		新規
// *************************************************************************/
PUBLIC void Flash_read(USHORT *add, USHORT *data)
{
	USHORT *read;
	if(FLASH.FENTRYR.WORD&0x00ff){
		FLASH.FENTRYR.WORD = 0xAA00;
	}
	FLASH.DFLRE0.WORD = 0x2DFF;
	FLASH.DFLRE1.WORD = 0xD2FF;
			
	*read = *(USHORT *)add;
	*data = *read;
//	printf("%d",read);
	
}

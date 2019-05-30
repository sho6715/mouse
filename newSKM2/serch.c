// *************************************************************************
//   ロボット名	： 電研ベーシックDCマウス、サンシャイン
//   概要		： サンシャインのsearchファイル
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
#include <search.h>
#include <parameters.h>
#include <init.h>

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
PUBLIC	UCHAR		map[MazeSize_x][MazeSize_y];
PUBLIC	t_wall		wall[MazeSize_x][MazeSize_y];
PRIVATE	stDIST_SEN	st_sen[DIST_SEN_NUM];
PUBLIC float		accel;
PUBLIC float		max_speed;
PUBLIC t_position	mypos;
PUBLIC int		counter = 0;
PUBLIC FLOAT		f_MoveBackDist = 0.0f;
PUBLIC FLOAT		MOVE_BACK_DIST = 0.3;
PUBLIC FLOAT		sura_speed;


//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************

// *************************************************************************
//   機能		： map初期化
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.08.25			sato		新規
// *************************************************************************/
PUBLIC void init_maze(void)
{
	int i,j;
	
	for(i = 0; i < MazeSize_x; i++)
	{
		for(j = 0; j < MazeSize_y; j++)
		{
			wall[i][j].north =wall[i][j].east = wall[i][j].south = wall[i][j].west = UNKNOWN;
		}
	}
	
	for(i = 0; i < MazeSize_x; i++)
	{
		wall[i][0].south = WALL;
		wall[i][MazeSize_y - 1].north = WALL;
	}
	for(j = 0; j < MazeSize_y; j++)
	{
		wall[0][j].west = WALL;
		wall[MazeSize_x - 1][j].east = WALL;
	}
	
	wall[0][0].east = wall[1][0].west = WALL;
	
}


// *************************************************************************
//   機能		： 歩数map初期化 
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.08.25			sato		新規
// *************************************************************************/
PUBLIC void clear_map(int x, int y)
{
	int i,j;
	
	for(i = 0;i < MazeSize_x; i++)
	{
		for(j = 0;j < MazeSize_y; j++)
		{
			map[ i ][ j ] = 255;
		}
	}
	map[ x ][ y ]  = 0;
}

// *************************************************************************
//   機能		： 歩数map作成
//   注意		： なし
//   メモ		： pico search static_parametersを参照中
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.08.25			sato		新規
// *************************************************************************/
PUBLIC void make_map(int x, int y, int mask)
{
	int i,j;
	UCHAR change_flag;
	
	clear_map(x,y);
	
	do
	{
		change_flag = FALSE;
		for(i = 0;i<MazeSize_x;i++)
		{
			for(j = 0; j < MazeSize_y; j++)
			{
				if(map[i][j] == 255)
				{
					continue;
				}
				if(j<MazeSize_y - 1)
				{
					if((wall[i][j].north & mask) == NOWALL)
					{
						if(map[i][j+1] == 255)
						{
							map[i][j+1] = map[i][j] + 1;
							change_flag = TRUE;
						}
					}
				}
			
				if(i<MazeSize_x - 1)
				{
					if((wall[i][j].east & mask) == NOWALL)
					{
						if(map[i+1][j] == 255)
						{
							map[i+1][j] = map[i][j] + 1;
							change_flag = TRUE;
						}
					}
				}
				
				if(j>0)
				{
					if((wall[i][j].south & mask) == NOWALL)
					{
						if(map[i][j-1] == 255)
						{
							map[i][j-1] = map[i][j] + 1;
							change_flag = TRUE;
						}
					}
				}
			
				if(i>0)
				{
					if((wall[i][j].west & mask) == NOWALL)
					{
						if(map[i-1][j] == 255)
						{
							map[i-1][j] = map[i][j] + 1;
							change_flag = TRUE;
						}
					}
				}
			}
		}
	}while(change_flag == TRUE);
	
}

// *************************************************************************
//   機能		： 壁情報を記録
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.08.27			sato		新規
// *************************************************************************/
PUBLIC void set_wall(int x,int y )
{
	int n_write, s_write, e_write, w_write;
/*	
	st_sen[DIST_SEN_R_FRONT].s_limit = 20;	//25
	st_sen[DIST_SEN_L_FRONT].s_limit = 15;	//15
	st_sen[DIST_SEN_R_SIDE].s_limit = 40;	//430
	st_sen[DIST_SEN_L_SIDE].s_limit = 30;	//800
*/	
	switch(mypos.dir){
		case north:
		
			n_write = CONV_SEN2WALL(DIST_getNowVal( DIST_SEN_R_FRONT ) > st_sen[DIST_SEN_R_FRONT].s_limit || DIST_getNowVal( DIST_SEN_L_FRONT ) > st_sen[DIST_SEN_L_FRONT].s_limit );	//センサが真（true)ならwall偽(false)ならNOWALL
			e_write = CONV_SEN2WALL(DIST_getNowVal( DIST_SEN_R_SIDE ) > st_sen[DIST_SEN_R_SIDE].s_limit );
			w_write = CONV_SEN2WALL(DIST_getNowVal( DIST_SEN_L_SIDE ) > st_sen[DIST_SEN_L_SIDE].s_limit );
			s_write = NOWALL;
			
			break;
		
		case east:
		
			e_write = CONV_SEN2WALL(DIST_getNowVal( DIST_SEN_R_FRONT ) > st_sen[DIST_SEN_R_FRONT].s_limit || DIST_getNowVal( DIST_SEN_L_FRONT ) > st_sen[DIST_SEN_L_FRONT].s_limit);
			s_write = CONV_SEN2WALL(DIST_getNowVal( DIST_SEN_R_SIDE ) > st_sen[DIST_SEN_R_SIDE].s_limit );
			n_write = CONV_SEN2WALL(DIST_getNowVal( DIST_SEN_L_SIDE ) > st_sen[DIST_SEN_L_SIDE].s_limit );
			w_write = NOWALL;
			
			break;
			
		case south:
		
			s_write = CONV_SEN2WALL(DIST_getNowVal( DIST_SEN_R_FRONT ) > st_sen[DIST_SEN_R_FRONT].s_limit || DIST_getNowVal( DIST_SEN_L_FRONT ) > st_sen[DIST_SEN_L_FRONT].s_limit);
			w_write = CONV_SEN2WALL(DIST_getNowVal( DIST_SEN_R_SIDE ) >  st_sen[DIST_SEN_R_SIDE].s_limit );
			e_write = CONV_SEN2WALL(DIST_getNowVal( DIST_SEN_L_SIDE ) > st_sen[DIST_SEN_L_SIDE].s_limit );
			n_write = NOWALL;
			
			break;
			
		case west:
		
			w_write = CONV_SEN2WALL(DIST_getNowVal( DIST_SEN_R_FRONT ) > st_sen[DIST_SEN_R_FRONT].s_limit || DIST_getNowVal( DIST_SEN_L_FRONT ) > st_sen[DIST_SEN_L_FRONT].s_limit);
			n_write = CONV_SEN2WALL(DIST_getNowVal( DIST_SEN_R_SIDE ) >  st_sen[DIST_SEN_R_SIDE].s_limit );
			s_write = CONV_SEN2WALL(DIST_getNowVal( DIST_SEN_L_SIDE ) > st_sen[DIST_SEN_L_SIDE].s_limit );
			e_write = NOWALL;
			
			break;
			
	}
	wall[x][y].north = n_write;
	wall[x][y].south = s_write;
	wall[x][y].east = e_write;
	wall[x][y].west = w_write;
	
	if(y < MazeSize_y - 1)
	{
		wall[x][y+1].south = n_write;
	}
	if(x < MazeSize_x - 1)
	{
		wall[x+1][y].west = e_write;
	}
	if(y > 0)
	{
		wall[x][y-1].north = s_write;
	}
	if(x > 0)
	{
		wall[x-1][y].east = w_write;
	}
/*	
	if(DIST_getNowVal( DIST_SEN_R_FRONT ) > st_sen[DIST_SEN_R_FRONT].s_limit && DIST_getNowVal(DIST_SEN_L_FRONT ) > st_sen[DIST_SEN_L_FRONT].s_limit && DIST_getNowVal(DIST_SEN_L_SIDE ) > st_sen[DIST_SEN_L_SIDE].s_limit && DIST_getNowVal(DIST_SEN_R_SIDE ) > st_sen[DIST_SEN_R_SIDE].s_limit)
	{
		LED4 = 0x05;
	}
	else if(DIST_getNowVal( DIST_SEN_R_FRONT ) > st_sen[DIST_SEN_R_FRONT].s_limit && DIST_getNowVal(DIST_SEN_L_FRONT ) > st_sen[DIST_SEN_L_FRONT].s_limit && DIST_getNowVal(DIST_SEN_L_SIDE ) > st_sen[DIST_SEN_L_SIDE].s_limit )
	{
		LED4 = 0x05;
	}
	else if(DIST_getNowVal( DIST_SEN_R_FRONT ) > st_sen[DIST_SEN_R_FRONT].s_limit && DIST_getNowVal(DIST_SEN_L_FRONT ) > st_sen[DIST_SEN_L_FRONT].s_limit && DIST_getNowVal(DIST_SEN_R_SIDE ) > st_sen[DIST_SEN_R_SIDE].s_limit )
	{
		LED4 = 0x04;
	}
	else if(DIST_getNowVal( DIST_SEN_R_FRONT ) > st_sen[DIST_SEN_R_FRONT].s_limit && DIST_getNowVal(DIST_SEN_R_SIDE ) > st_sen[DIST_SEN_R_SIDE].s_limit && DIST_getNowVal(DIST_SEN_L_SIDE ) > st_sen[DIST_SEN_L_SIDE].s_limit )
	{
		LED4 = 0x01;
	}
	else if(DIST_getNowVal( DIST_SEN_L_FRONT ) > st_sen[DIST_SEN_L_FRONT].s_limit && DIST_getNowVal(DIST_SEN_R_SIDE ) > st_sen[DIST_SEN_R_SIDE].s_limit && DIST_getNowVal(DIST_SEN_L_SIDE ) > st_sen[DIST_SEN_L_SIDE].s_limit )
	{
		LED4 = 0x05;
	}
	else if(DIST_getNowVal( DIST_SEN_R_FRONT ) > st_sen[DIST_SEN_R_FRONT].s_limit && DIST_getNowVal( DIST_SEN_L_FRONT ) > st_sen[DIST_SEN_L_FRONT].s_limit )
	{
		LED4 = 0x04;
	}
	else if(DIST_getNowVal( DIST_SEN_R_FRONT ) > st_sen[DIST_SEN_R_FRONT].s_limit && DIST_getNowVal( DIST_SEN_R_SIDE ) > st_sen[DIST_SEN_R_SIDE].s_limit )
	{
		LED4 = 0x00;
	}
	else if(DIST_getNowVal( DIST_SEN_L_FRONT ) > st_sen[DIST_SEN_L_FRONT].s_limit && DIST_getNowVal( DIST_SEN_L_SIDE ) > st_sen[DIST_SEN_L_SIDE].s_limit )
	{
		LED4 = 0x05;
	}
	else if(DIST_getNowVal( DIST_SEN_R_FRONT ) > st_sen[DIST_SEN_R_FRONT].s_limit && DIST_getNowVal( DIST_SEN_L_SIDE ) > st_sen[DIST_SEN_L_SIDE].s_limit )
	{
		LED4 = 0x01;
	}
	else if(DIST_getNowVal( DIST_SEN_L_FRONT ) > st_sen[DIST_SEN_L_FRONT].s_limit && DIST_getNowVal( DIST_SEN_R_SIDE ) > st_sen[DIST_SEN_R_SIDE].s_limit )
	{
		LED4 = 0x04;
	}
	else if(DIST_getNowVal( DIST_SEN_L_SIDE ) > st_sen[DIST_SEN_L_SIDE].s_limit && DIST_getNowVal( DIST_SEN_R_SIDE ) > st_sen[DIST_SEN_R_SIDE].s_limit )
	{
		LED4 = 0x01;
	}
	else if(DIST_getNowVal( DIST_SEN_L_SIDE ) > st_sen[DIST_SEN_L_SIDE].s_limit )
	{
		LED4 = 0x01;
	}
	else if(DIST_getNowVal( DIST_SEN_R_SIDE ) > st_sen[DIST_SEN_R_SIDE].s_limit )
	{
		LED4 = 0x00;
	}
	else if(DIST_getNowVal( DIST_SEN_L_FRONT ) > st_sen[DIST_SEN_L_FRONT].s_limit )
	{
		LED4 = 0x04;
	}
	else if(DIST_getNowVal( DIST_SEN_R_FRONT ) > st_sen[DIST_SEN_R_FRONT].s_limit )
	{
		LED4 = 0x00;
	}
*/	
	
	
	
/*	switch(mypos.dir){
		case north:
		
			if(wall[x][y].north == WALL){
				LED4 = 0x00;
			}
			else{
				LED4 = LED4_ALL_OFF;
			}
			
			break;
		
		case east:
		
			if(wall[x][y].east == WALL){
				LED4 = 0x08;
			}
			else{
				LED4 = LED4_ALL_OFF;
			}
			
			
			break;
			
		case south:
		
			if(wall[x][y].south == WALL){
				LED4 = 0x08;
			}
			else{
				LED4 = LED4_ALL_OFF;
			}
			
			
			break;
			
		case west:
		
			if(wall[x][y].west == WALL){
				LED4 = 0x08;
			}
			else{
				LED4 = LED4_ALL_OFF;
			}
			
			
			break;
			
	}
*/
/*	switch(mypos.dir){
		case north:
		
			if(wall[x][y].north == WALL){
				LED4 = 0x08;
			}
			else{
				LED4 = LED4_ALL_OFF;
			}
			
			break;
		
		case east:
		
			if(wall[x][y].east == WALL){
				LED4 = 0x08;
			}
			else{
				LED4 = LED4_ALL_OFF;
			}
			
			
			break;
			
		case south:
		
			if(wall[x][y].south == WALL){
				LED4 = 0x08;
			}
			else{
				LED4 = LED4_ALL_OFF;
			}
			
			
			break;
			
		case west:
		
			if(wall[x][y].west == WALL){
				LED4 = 0x08;
			}
			else{
				LED4 = LED4_ALL_OFF;
			}
			
			
			break;
			
	}
*/	
/*	if(wall[x][y].east == WALL && wall[x][y].west == WALL && wall[x][y].north == WALL && wall[x][y].south == WALL){
		LED4 = LED4_ALL_ON;
	}
	else if(wall[x][y].north == WALL && wall[x][y].west == WALL && wall[x][y].east == WALL){
		LED4 = 0x09;
	}
	else if(wall[x][y].north == WALL && wall[x][y].south == WALL && wall[x][y].east == WALL){
		LED4 = 0x0c;
	}
	else if(wall[x][y].south == WALL && wall[x][y].west == WALL && wall[x][y].east == WALL){
		LED4 = 0x05;
	}
	else if(wall[x][y].north == WALL && wall[x][y].west == WALL && wall[x][y].south == WALL){
		LED4 = 0x0d;
	}
	else if(wall[x][y].west == WALL && wall[x][y].east == WALL){
		LED4 = 0x01;
	}
	else if(wall[x][y].north == WALL && wall[x][y].south == WALL){
		LED4 = 0x0b;
	}
	else if(wall[x][y].north == WALL && wall[x][y].west == WALL ){
		LED4 = 0x09;
	}
	else if(wall[x][y].west == WALL && wall[x][y].south == WALL ){
		LED4 = 0x05;
	}
	else if(wall[x][y].south == WALL && wall[x][y].east == WALL ){
		LED4 = 0x04;
	}
	else if(wall[x][y].north == WALL && wall[x][y].east == WALL ){
		LED4 = 0x08;
	}
	else if(wall[x][y].north == WALL){
		LED4 = 0x08;
	}
	else if(wall[x][y].east == WALL){
		LED4 = 0x00;
	}
	else if(wall[x][y].south == WALL){
		LED4 = 0x04;
	}
	else if(wall[x][y].west == WALL){
		LED4 = 0x01;
	}
	else{
		LED4 = LED4_ALL_OFF;
	}
*/
	
	
		
	
}
	
// *************************************************************************
//   機能		： 指定区画が未探索か判断する
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.08.28			sato		新規
// *************************************************************************/
PUBLIC UCHAR is_unknown(int x, int y)
{
	if((wall[x][y].north == UNKNOWN)||(wall[x][y].east == UNKNOWN)||(wall[x][y].south == UNKNOWN)||(wall[x][y].west == UNKNOWN))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

// *************************************************************************
//   機能		： 探索の優先度の算出
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.08.28			sato		新規
// *************************************************************************/
PUBLIC int get_priority(int x, int y, en_direction dir )
{
	int priority;
	
	priority = 0;
	
	if(mypos.dir == dir)
	{
		priority = 2;
	}
	else if(((4 + mypos.dir - dir) % 4) == 2)
	{
		priority = 0;
	}
	else
	{
		priority = 1;
	}
	
	if(is_unknown(x,y) == TRUE)
	{
		priority += 4;
	}
	
	return priority;
	
}

// *************************************************************************
//   機能		： ゴール座標に向かう場合、今どちらに行くべきか判断
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.08.28			sato		新規
// *************************************************************************/
PUBLIC int get_nextdir(int x, int y, int mask, en_direction *dir)
{
	int little;
	int tmp_priority, priority;
	
	make_map(x,y,mask);
	little = 255;
	
	priority = 0;
	
	if((wall[mypos.x][mypos.y].north & mask) == NOWALL)
	{
//		LED4 = 0x08;
		tmp_priority = get_priority(mypos.x, mypos.y + 1, north);
		if(map[mypos.x][mypos.y + 1] < little)
		{
			little = map[mypos.x][mypos.y + 1];
			*dir = north;
			priority = tmp_priority;
		}
		else if(map[mypos.x][mypos.y] == little)
		{
			if(priority < tmp_priority)
			{
				*dir = north;
				priority = tmp_priority;
			}
		}
	}
	
	if((wall[mypos.x][mypos.y].east & mask) == NOWALL)
	{
//		LED4 = 0x00;
		tmp_priority = get_priority(mypos.x + 1, mypos.y, east);
		if(map[mypos.x + 1][mypos.y] < little)
		{
			little = map[mypos.x + 1][mypos.y];
			*dir = east;
			priority = tmp_priority;
		}
		else if(map[mypos.x + 1][mypos.y] == little)
		{
			if(priority < tmp_priority)
			{
				*dir = east;
				priority = tmp_priority;
			}
		}
	}
	
	if((wall[mypos.x][mypos.y].south & mask) == NOWALL)
	{
//		LED4 = 0x04;
		tmp_priority = get_priority(mypos.x, mypos.y - 1, south);
		if(map[mypos.x][mypos.y - 1] < little)
		{
			little = map[mypos.x][mypos.y - 1];
			*dir = south;
			priority = tmp_priority;
		}
		else if(map[mypos.x][mypos.y - 1] == little)
		{
			if(priority < tmp_priority)
			{
				*dir = south;
				priority = tmp_priority;
			}
		}
	}
	
	if((wall[mypos.x][mypos.y].west & mask) == NOWALL)
	{
//		LED4 = 0x01;
		tmp_priority = get_priority(mypos.x - 1, mypos.y, west);
		if(map[mypos.x - 1][mypos.y] < little)
		{
			little = map[mypos.x - 1][mypos.y];
			*dir = west;
			priority = tmp_priority;
		}
		else if(map[mypos.x - 1][mypos.y] == little)
		{
			if(priority < tmp_priority)
			{
				*dir = west;
				priority = tmp_priority;
			}
		}
	}
	
	return ((int)(4 + *dir - mypos.dir) % 4);
	
}

// *************************************************************************
//   機能		： 壁当て判定
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.10.19			sato		新規
// *************************************************************************/
/*PRIVATE UCHAR judge_wallhit(t_position mypos)
{
	switch(mypos.dir)
		{
			case front:
				return FALSE;
				break;
				
			case right:
				if((mypos.dir == north)&&(wall[mypos.x][mypos.y].west == WALL)||
					(mypos.dir == east)&&(wall[mypos.x][mypos.y].north == WALL)||
					(mypos.dir == south)&&(wall[mypos.x][mypos.y].east == WALL)||
					(mypos.dir == west)&&(wall[mypos.x][mypos.y].south == WALL))
				{
					return TRUE;
				}
				else{
					return FALSE;
				}
				break;
				
			case left:
				if((mypos.dir == north)&&(wall[mypos.x][mypos.y].east == WALL)||
					(mypos.dir == east)&&(wall[mypos.x][mypos.y].south == WALL)||
					(mypos.dir == south)&&(wall[mypos.x][mypos.y].west == WALL)||
					(mypos.dir == west)&&(wall[mypos.x][mypos.y].north == WALL))
				{
					return TRUE;
				}
				else{
					return FALSE;
				}
				break;
				
			case rear:
				if((mypos.dir == north)&&(wall[mypos.x][mypos.y].north == WALL)||
					(mypos.dir == east)&&(wall[mypos.x][mypos.y].east == WALL)||
					(mypos.dir == south)&&(wall[mypos.x][mypos.y].south == WALL)||
					(mypos.dir == west)&&(wall[mypos.x][mypos.y].west == WALL))
				{
					return TRUE;
				}
				else{
					return FALSE;
				}
				
				break;
				
		}
		return FALSE;
}
*/
// *************************************************************************
//   機能		： 自己座標加算
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.10.19			sato		新規
// *************************************************************************/
/*PRIVATE void myposition(t_position mypos)
{
	switch(mypos.dir)
		{
			case north:
				mypos.y++;
				break;
			
			case east:
				mypos.x++;
				break;
		
			case south:
				mypos.y--;
				break;
			
			case west:
				mypos.x--;
				break;
				
		}
}
*/

// *************************************************************************
//   機能		： 左手法
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.08.28			sato		新規
// *************************************************************************/
PUBLIC void search_lefthand(void)
{
	max_speed = f_MotSearchMaxSpeed;
	accel = MOT_getAcc1();
	st_sen[DIST_SEN_R_FRONT].s_limit = 25;	//25
	st_sen[DIST_SEN_L_FRONT].s_limit = 15;	//15
	st_sen[DIST_SEN_R_SIDE].s_limit = 200;	//430
	st_sen[DIST_SEN_L_SIDE].s_limit = 300;	//800
	
	MOT_goBlock_FinSpeed( 0.5f , max_speed );
	
	while(1)
	{
		
		if(DIST_getNowVal( DIST_SEN_L_SIDE ) < st_sen[DIST_SEN_L_SIDE].s_limit )
		{
			MOT_goBlock_FinSpeed( 0.5f , 0.0f );
			MOT_turn(MOT_L90);
			MOT_goBlock_FinSpeed( 0.5f , max_speed );
		}
		else if((DIST_getNowVal( DIST_SEN_L_FRONT ) < st_sen[DIST_SEN_L_FRONT].s_limit) && ( DIST_getNowVal( DIST_SEN_R_FRONT )< st_sen[DIST_SEN_R_FRONT].s_limit ))
		{
			MOT_goBlock_FinSpeed( 0.5 , max_speed );
		}
		else if( DIST_getNowVal( DIST_SEN_R_SIDE )< st_sen[DIST_SEN_R_SIDE].s_limit )
		{
			MOT_goBlock_FinSpeed( 0.5 , 0 );
			MOT_turn(MOT_R90);
			MOT_goBlock_FinSpeed( 0.5 , max_speed );
		}
		else
		{
			MOT_goBlock_FinSpeed( 0.5 , 0 );
			MOT_turn(MOT_R180);
			MOT_goBlock_FinSpeed( 0.5 , max_speed );
		}
	}
	
}

// *************************************************************************
//   機能		： 足立法
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.08.28			sato		新規
// *************************************************************************/
PUBLIC void search_adachi(int gx,int gy)
{
	en_direction glob_nextdir;		//次に向かう方向を記録する変数
	
	max_speed = 500;//f_MotSearchMaxSpeed;
//	accel = MOT_getAcc1();
//	init_maze();
//	make_map(gx,gy,MASK_SEARCH);
	
	switch(get_nextdir(gx,gy,MASK_SEARCH,&glob_nextdir))
	{
		case front:
			MOT_goBlock_FinSpeed( 0.5 , max_speed );
			break;
			
		case right:
			MOT_turn(MOT_R90);
			MOT_goBlock_FinSpeed( 0.5 , max_speed );
			break;
			
		case left:
			MOT_turn(MOT_L90);
			MOT_goBlock_FinSpeed( 0.5 , max_speed );
			break;
			
		case rear:
			MOT_turn(MOT_R180);
			MOT_goBlock_FinSpeed( 0.5 , max_speed );
			break;
	}
	
	mypos.dir = glob_nextdir;
	
	switch(mypos.dir)
	{
		case north:
			mypos.y++;
			break;
			
		case east:
			mypos.x++;
			break;
			
		case south:
			mypos.y--;
			break;
			
		case west:
			mypos.x--;
			break;
			
	}
	
//	mypos.dir = glob_nextdir;
	
	while((mypos.x != gx) || (mypos.y != gy)){
		
		set_wall(mypos.x,mypos.y);
		
		switch(get_nextdir(gx,gy,MASK_SEARCH,&glob_nextdir))
		{
			case front:
				MOT_goBlock_FinSpeed( 1.0 , max_speed );
				break;
				
			case right:
				MOT_goBlock_FinSpeed( 0.5 , 0.0 );
//				TIME_wait(100);
				MOT_turn(MOT_R90);
//				TIME_wait(100);
				MOT_goBlock_FinSpeed( 0.5 , max_speed );
				break;
				
			case left:
				MOT_goBlock_FinSpeed( 0.5 , 0.0 );
//				TIME_wait(100);
				MOT_turn(MOT_L90);
//				TIME_wait(100);
				MOT_goBlock_FinSpeed( 0.5 , max_speed );
				break;
				
			case rear:
				MOT_goBlock_FinSpeed( 0.5 , 0.0 );
//				TIME_wait(100);
				MOT_turn(MOT_R180);
//				TIME_wait(100);
				MOT_goBlock_FinSpeed( 0.5 , max_speed );
				break;
				
		}
	
		mypos.dir = glob_nextdir;
/*		if(mypos.dir == north){
			LED4 = 0x01;
		}
		else if(mypos.dir == east){
			LED4 = 0x02;
		}
		else if(mypos.dir == south){
			LED4 = 0x04;
		}
		else if(mypos.dir == west){
			LED4 = 0x08;
		}*/
		
		switch(mypos.dir)
		{
			case north:
				mypos.y++;
				break;
			
			case east:
				mypos.x++;
				break;
			
			case south:
				mypos.y--;
				break;
			
			case west:
				mypos.x--;
				break;
				
		}
//		mypos.dir = glob_nextdir;
	}
	set_wall(mypos.x,mypos.y);
	MOT_goBlock_FinSpeed( 0.5 , 0.0 );
		
}

// *************************************************************************
//   機能		： ゴールアピール
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.09.09			sato		新規
// *************************************************************************/
PUBLIC void goal_appeal(void)
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
	LED4 = LED4_ALL_ON;
}
		
// *************************************************************************
//   機能		： 最短走行
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.09.09			sato		新規
// *************************************************************************/
PUBLIC void fast_run(int x, int y,enSearch_Mode en_Mode)
{
	en_direction glob_nextdir;
	int straight_count = 0;
	MOVE_BACK_DIST = 0.24;
	max_speed = MOT_getTrgtSpeed(en_Mode);
	
	switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))
	{
		case front:
			straight_count++;
			break;
			
		case right:
			MOT_turn(MOT_R90);
			straight_count = 1;
			break;
			
		case left:
			MOT_turn(MOT_L90);
			straight_count =1;
			break;
			
		case rear:
			MOT_turn(MOT_R180);
			straight_count = 1;
			break;
	}
	
	mypos.dir = glob_nextdir;
//	myposition(mypos);
	switch(mypos.dir)
	{
		case north:
			mypos.y++;
			break;
			
		case east:
			mypos.x++;
			break;
			
		case south:
			mypos.y--;
			break;
			
		case west:
			mypos.x--;
			break;
	}
	
	while((mypos.x != x)||(mypos.y != y))
	{
		switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))
		{
			case front:
				straight_count++;
				break;
				
			case right:
				MOT_goBlock_FinSpeed(straight_count + MOVE_BACK_DIST,0.0 );
//				TIME_wait(100);
				MOT_turn(MOT_R90);
//				TIME_wait(100);
				straight_count = 1;
				MOVE_BACK_DIST = 0;
				break;
				
			case left:
				MOT_goBlock_FinSpeed(straight_count + MOVE_BACK_DIST,0.0 );
//				TIME_wait(100);
				MOT_turn(MOT_L90);
//				TIME_wait(100);
				straight_count = 1;
				MOVE_BACK_DIST = 0;
				break;
				
			case rear:
				MOT_goBlock_FinSpeed(straight_count + MOVE_BACK_DIST,0.0 );
//				TIME_wait(100);
				MOT_turn(MOT_R180);
//				TIME_wait(100);
				straight_count = 1;
				MOVE_BACK_DIST = 0;
				break;
		}
		mypos.dir = glob_nextdir;
//		myposition(mypos);
		switch(mypos.dir)
		{
			case north:
				mypos.y++;
				break;
				
			case east:
				mypos.x++;
				break;
				
			case south:
				mypos.y--;
				break;
				
			case west:
				mypos.x--;
				break;
		}

		if ( SW_ON == SW_INC_PIN ){
			break;
		}
	}
	if( SW_ON == SW_INC_PIN ){
		LED4 = LED4_ALL_ON;
	}
	else{
		MOT_goBlock_FinSpeed(straight_count + MOVE_BACK_DIST,0.0);
		MOVE_BACK_DIST = 0;
		MOT_turn(MOT_R180);
		TIME_wait(500);
		goal_appeal();
	}
}
			
// *************************************************************************
//   機能		： 足立法壁当てバージョン
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.08.28			sato		新規
// *************************************************************************/
PUBLIC void search_adachi_hitwall(int gx,int gy)
{
	en_direction glob_nextdir;		//次に向かう方向を記録する変数
	
	max_speed = 500;//f_MotSearchMaxSpeed;
	counter = 0;
	f_MoveBackDist = 0.0f;
	MOVE_BACK_DIST = 0.24;
	if(gx == Goal_x && gy == Goal_y){
		f_MoveBackDist = 0.25;
	}
	
	switch(get_nextdir(gx,gy,MASK_SEARCH,&glob_nextdir))
	{
		case front:
			MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, max_speed );
			break;
			
		case right:
			MOT_turn(MOT_R90);
			MOT_goBlock_FinSpeed( 0.5 , max_speed );
			break;
			
		case left:
			MOT_turn(MOT_L90);
			MOT_goBlock_FinSpeed( 0.5 , max_speed );
			break;
			
		case rear:
			MOT_turn(MOT_R180);
			MOT_goBlock_FinSpeed( 0.5 , max_speed );
			break;
	}
	f_MoveBackDist = 0;
	mypos.dir = glob_nextdir;
//	myposition(mypos);
	switch(mypos.dir)
	{
		case north:
			mypos.y++;
			break;
			
		case east:
			mypos.x++;
			break;
			
		case south:
			mypos.y--;
			break;
			
		case west:
			mypos.x--;
			break;
			
	}
	
//	mypos.dir = glob_nextdir;
	
	while((mypos.x != gx) || (mypos.y != gy)){
		
		set_wall(mypos.x,mypos.y);
		
		switch(get_nextdir(gx,gy,MASK_SEARCH,&glob_nextdir))
		{
			case front:
				MOT_goBlock_FinSpeed( 1.0 , max_speed );
				break;
				
			case right:
				if(counter < 2){
					MOT_goBlock_FinSpeed( 0.5 , 0.0 );
//					TIME_wait(100);
					MOT_turn(MOT_R90);
//					TIME_wait(100);
					counter++;
				}
				else{
					MOT_goBlock_FinSpeed( 0.5 , 0.0 );
//					TIME_wait(100);
					MOT_turn(MOT_R90);
//					TIME_wait(100);
					f_MoveBackDist = 0;
					if((mypos.dir == north)&&(wall[mypos.x][mypos.y].west == WALL)||
						(mypos.dir == east)&&(wall[mypos.x][mypos.y].north == WALL)||
						(mypos.dir == south)&&(wall[mypos.x][mypos.y].east == WALL)||
						(mypos.dir == west)&&(wall[mypos.x][mypos.y].south == WALL)){
						MOT_goHitBackWall();
						f_MoveBackDist = MOVE_BACK_DIST;
//						TIME_wait(200);
						counter = 0;
					}
				}
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, max_speed );
				f_MoveBackDist = 0;
				break;
				
			case left:
				if(counter <2){
					MOT_goBlock_FinSpeed( 0.5 , 0.0 );
//					TIME_wait(100);
					MOT_turn(MOT_L90);
//					TIME_wait(100);
					counter++;
				}
				else{
					MOT_goBlock_FinSpeed( 0.5 , 0.0 );
//					TIME_wait(100);
					MOT_turn(MOT_L90);
//					TIME_wait(100);
					f_MoveBackDist = 0;
					if((mypos.dir == north)&&(wall[mypos.x][mypos.y].east == WALL)||
						(mypos.dir == east)&&(wall[mypos.x][mypos.y].south == WALL)||
						(mypos.dir == south)&&(wall[mypos.x][mypos.y].west == WALL)||
						(mypos.dir == west)&&(wall[mypos.x][mypos.y].north == WALL)){
						MOT_goHitBackWall();
						f_MoveBackDist = MOVE_BACK_DIST;
//						TIME_wait(200);
						counter =0;
					}
				}
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, max_speed );
				f_MoveBackDist = 0;
				break;
				
			case rear:
				MOT_goBlock_FinSpeed( 0.5 , 0.0 );
//				TIME_wait(100);
				MOT_turn(MOT_R180);
//				TIME_wait(100);
				if((mypos.dir == north)&&(wall[mypos.x][mypos.y].north == WALL)||
					(mypos.dir == east)&&(wall[mypos.x][mypos.y].east == WALL)||
					(mypos.dir == south)&&(wall[mypos.x][mypos.y].south == WALL)||
					(mypos.dir == west)&&(wall[mypos.x][mypos.y].west == WALL)){
						MOT_goHitBackWall();
						f_MoveBackDist = MOVE_BACK_DIST;
//						TIME_wait(200);
						}
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, max_speed );
				f_MoveBackDist =0;
				counter =0;
				
				break;
				
		}
	
		mypos.dir = glob_nextdir;
/*		if(mypos.dir == north){
			LED4 = 0x01;
		}
		else if(mypos.dir == east){
			LED4 = 0x02;
		}
		else if(mypos.dir == south){
			LED4 = 0x04;
		}
		else if(mypos.dir == west){
			LED4 = 0x08;
		}*/
//		myposition(mypos);
		switch(mypos.dir)
		{
			case north:
				mypos.y++;
				break;
			
			case east:
				mypos.x++;
				break;
			
			case south:
				mypos.y--;
				break;
			
			case west:
				mypos.x--;
				break;
				
		}
		if ( SW_ON == SW_INC_PIN ){
			break;
		}
//		mypos.dir = glob_nextdir;
	}
	if( SW_ON == SW_INC_PIN ){
		LED4 = LED4_ALL_ON;
		CTRL_stop();
	}
	else{
		set_wall(mypos.x,mypos.y);
		MOT_goBlock_FinSpeed( 0.5 , 0.0 );
		MOT_turn(MOT_R180);
		TIME_wait(500);
		goal_appeal();
	}
		
}

// *************************************************************************
//   機能		： 最短走行スラローム追加
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.09.09			sato		新規
// *************************************************************************/
PUBLIC void fast_runsura(int x, int y,enSearch_Mode en_Mode)
{
	en_direction glob_nextdir;
	int straight_count = 0;
	MOVE_BACK_DIST = 0.25;
	max_speed = MOT_getTrgtSpeed(en_Mode);
	sura_speed = 500;
	
	switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))
	{
		case front:
			straight_count++;
			break;
			
		case right:
			MOT_turn(MOT_R90);
			straight_count = 1;
			break;
			
		case left:
			MOT_turn(MOT_L90);
			straight_count =1;
			break;
			
		case rear:
			MOT_turn(MOT_R180);
			straight_count = 1;
			break;
	}
	
	mypos.dir = glob_nextdir;
//	myposition(mypos);
	switch(mypos.dir)
	{
		case north:
			mypos.y++;
			break;
			
		case east:
			mypos.x++;
			break;
			
		case south:
			mypos.y--;
			break;
			
		case west:
			mypos.x--;
			break;
	}
	
	while((mypos.x != x)||(mypos.y != y))
	{
		switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))
		{
			case front:
				straight_count++;
				break;
				
			case right:
				MOT_goBlock_FinSpeed(straight_count + MOVE_BACK_DIST,sura_speed );
//				TIME_wait(100);
				MOT_goSla(MOT_R90);
//				TIME_wait(100);
				straight_count = 1;
				MOVE_BACK_DIST = 0;
				break;
				
			case left:
				MOT_goBlock_FinSpeed(straight_count + MOVE_BACK_DIST,sura_speed );
//				TIME_wait(100);
				MOT_goSla(MOT_L90);
//				TIME_wait(100);
				straight_count = 1;
				MOVE_BACK_DIST = 0;
				break;
				
			case rear:
				MOT_goBlock_FinSpeed(straight_count + MOVE_BACK_DIST,0.0 );
//				TIME_wait(100);
				MOT_turn(MOT_R180);
//				TIME_wait(100);
				straight_count = 1;
				MOVE_BACK_DIST = 0;
				break;
		}
		mypos.dir = glob_nextdir;
//		myposition(mypos);
		switch(mypos.dir)
		{
			case north:
				mypos.y++;
				break;
				
			case east:
				mypos.x++;
				break;
				
			case south:
				mypos.y--;
				break;
				
			case west:
				mypos.x--;
				break;
		}

		if ( SW_ON == SW_INC_PIN ){
			break;
		}
	}
	if( SW_ON == SW_INC_PIN ){
		LED4 = LED4_ALL_ON;
	}
	else{
		MOT_goBlock_FinSpeed(straight_count + MOVE_BACK_DIST,0.0);
		MOVE_BACK_DIST = 0;
		MOT_turn(MOT_R180);
		TIME_wait(500);
		goal_appeal();
	}
}
			
// *************************************************************************
//   機能		： 足立法壁当てバージョンスラローム追加
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.08.28			sato		新規
// *************************************************************************/
PUBLIC void search_adachi_hitwallsura(int gx,int gy)
{
	en_direction glob_nextdir;		//次に向かう方向を記録する変数
	
	max_speed = 500;//f_MotSearchMaxSpeed;
	counter = 0;
	f_MoveBackDist = 0.0f;
	MOVE_BACK_DIST = 0.25;
	if(gx == Goal_x && gy == Goal_y){
		f_MoveBackDist = 0.27;
	}
	
	switch(get_nextdir(gx,gy,MASK_SEARCH,&glob_nextdir))
	{
		case front:
			MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, max_speed );
			break;
			
		case right:
			MOT_turn(MOT_R90);
			MOT_goBlock_FinSpeed( 0.5 , max_speed );
			break;
			
		case left:
			MOT_turn(MOT_L90);
			MOT_goBlock_FinSpeed( 0.5 , max_speed );
			break;
			
		case rear:
			MOT_turn(MOT_R180);
			MOT_goBlock_FinSpeed( 0.5 , max_speed );
			break;
	}
	f_MoveBackDist = 0;
	mypos.dir = glob_nextdir;
//	myposition(mypos);
	switch(mypos.dir)
	{
		case north:
			mypos.y++;
			break;
			
		case east:
			mypos.x++;
			break;
			
		case south:
			mypos.y--;
			break;
			
		case west:
			mypos.x--;
			break;
			
	}
	
//	mypos.dir = glob_nextdir;
	
	while((mypos.x != gx) || (mypos.y != gy)){
		
		set_wall(mypos.x,mypos.y);
		
		switch(get_nextdir(gx,gy,MASK_SEARCH,&glob_nextdir))
		{
			case front:
				MOT_goBlock_FinSpeed( 1.0 , max_speed );
				break;
				
			case right:
				if(counter < 2){
					MOT_goBlock_FinSpeed( 0.5 , max_speed );
//					TIME_wait(100);
					MOT_goSla(MOT_R90);
//					TIME_wait(100);
					counter++;
				}
				else{
					MOT_goBlock_FinSpeed( 0.5 , 0.0 );
//					TIME_wait(100);
					MOT_turn(MOT_R90);
//					TIME_wait(100);
					f_MoveBackDist = 0;
					if((mypos.dir == north)&&(wall[mypos.x][mypos.y].west == WALL)||
						(mypos.dir == east)&&(wall[mypos.x][mypos.y].north == WALL)||
						(mypos.dir == south)&&(wall[mypos.x][mypos.y].east == WALL)||
						(mypos.dir == west)&&(wall[mypos.x][mypos.y].south == WALL)){
						MOT_goHitBackWall();
						f_MoveBackDist = MOVE_BACK_DIST;
//						TIME_wait(200);
						counter = 0;
						MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, max_speed );
					}
				}
				f_MoveBackDist = 0;
				break;
				
			case left:
				if(counter <2){
					MOT_goBlock_FinSpeed( 0.5 , max_speed );
//					TIME_wait(100);
					MOT_goSla(MOT_L90);
//					TIME_wait(100);
					counter++;
				}
				else{
					MOT_goBlock_FinSpeed( 0.5 , 0.0 );
//					TIME_wait(100);
					MOT_turn(MOT_L90);
//					TIME_wait(100);
					f_MoveBackDist = 0;
					if((mypos.dir == north)&&(wall[mypos.x][mypos.y].east == WALL)||
						(mypos.dir == east)&&(wall[mypos.x][mypos.y].south == WALL)||
						(mypos.dir == south)&&(wall[mypos.x][mypos.y].west == WALL)||
						(mypos.dir == west)&&(wall[mypos.x][mypos.y].north == WALL)){
						MOT_goHitBackWall();
						f_MoveBackDist = MOVE_BACK_DIST;
//						TIME_wait(200);
						counter =0;
						MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, max_speed );
					}
				}
				f_MoveBackDist = 0;
				break;
				
			case rear:
				MOT_goBlock_FinSpeed( 0.5 , 0.0 );
//				TIME_wait(100);
				MOT_turn(MOT_R180);
//				TIME_wait(100);
				if((mypos.dir == north)&&(wall[mypos.x][mypos.y].north == WALL)||
					(mypos.dir == east)&&(wall[mypos.x][mypos.y].east == WALL)||
					(mypos.dir == south)&&(wall[mypos.x][mypos.y].south == WALL)||
					(mypos.dir == west)&&(wall[mypos.x][mypos.y].west == WALL)){
						MOT_goHitBackWall();
						f_MoveBackDist = MOVE_BACK_DIST;
//						TIME_wait(200);
						}
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, max_speed );
				f_MoveBackDist =0;
				counter =0;
				
				break;
				
		}
	
		mypos.dir = glob_nextdir;
/*		if(mypos.dir == north){
			LED4 = 0x01;
		}
		else if(mypos.dir == east){
			LED4 = 0x02;
		}
		else if(mypos.dir == south){
			LED4 = 0x04;
		}
		else if(mypos.dir == west){
			LED4 = 0x08;
		}*/
//		myposition(mypos);
		switch(mypos.dir)
		{
			case north:
				mypos.y++;
				break;
			
			case east:
				mypos.x++;
				break;
			
			case south:
				mypos.y--;
				break;
			
			case west:
				mypos.x--;
				break;
				
		}
		if ( SW_ON == SW_INC_PIN ){
			break;
		}
//		mypos.dir = glob_nextdir;
	}
	if( SW_ON == SW_INC_PIN ){
		LED4 = LED4_ALL_ON;
	}
	else{
		set_wall(mypos.x,mypos.y);
		MOT_goBlock_FinSpeed( 0.5 , 0.0 );
		MOT_turn(MOT_R180);
		TIME_wait(500);
		goal_appeal();
	}
		
}
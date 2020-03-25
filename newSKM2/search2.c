// *************************************************************************
//   ���{�b�g��	�F �d���x�[�V�b�NDC�}�E�X�A�T���V���C��
//   �T�v		�F �T���V���C����search2�t�@�C��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2017.09.27			sato		�V�K�i�t�@�C���̃C���N���[�h�j
// *************************************************************************/


//**************************************************
// �C���N���[�h�t�@�C���iinclude�j
//**************************************************
#include <typedefine.h>						// ��`
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
// ��`�idefine�j
//**************************************************
#define MAP_SMAP_MAX_VAL	( MAP_X_SIZE * MAP_Y_SIZE ) 			///< ������map�̍ő�l
#define MAP_SMAP_MAX_PRI_VAL	( MAP_SMAP_MAX_VAL * 4 )				///< ������map�̗D��x�ő�l

#define MOVE_BACK_DIST		(0.3f)

//**************************************************
// �񋓑́ienum�j
//**************************************************

//**************************************************
// �\���́istruct�j
//**************************************************
#define MAP_SMAP_MAX_VAL		( MAP_X_SIZE * MAP_Y_SIZE ) 			///< ������map�̍ő�l
#define MAP_SMAP_MAX_PRI_VAL	( MAP_SMAP_MAX_VAL * 4 )				///< ������map�̗D��x�ő�l


//**************************************************
// �O���[�o���ϐ�
//**************************************************
PRIVATE enMAP_HEAD_DIR	en_Head;										///< �}�E�X�̐i�s���� 0:N 1:E 2:S 3:W
PRIVATE UCHAR		my;												///< �}�E�X�̂w���W
PRIVATE UCHAR		mx;												///< �}�E�X�̂x���W
PUBLIC USHORT		us_cmap[MAP_Y_SIZE][MAP_X_SIZE];				///< ������ �f�[�^
PUBLIC UCHAR		g_sysMap[ MAP_Y_SIZE ][ MAP_X_SIZE ];			///< ���H���
PRIVATE FLOAT		f_MoveBackDist;									///< �Ǔ��ē���Ō�ނ�������[���]
PRIVATE UCHAR		uc_SlaCnt = 0;									// �X�����[���A����
PRIVATE UCHAR		uc_back[ MAP_Y_SIZE ][ MAP_X_SIZE ];			// ���H�f�[�^

PUBLIC UCHAR		GOAL_MAP_X;					//�S�[�����W�ύX�v���O�����p��
PUBLIC UCHAR		GOAL_MAP_Y;					//�S�[�����W�ύX�v���O�����p��

PUBLIC BOOL			search_flag;

PUBLIC UCHAR		GOAL_SIZE;
//�������}�b�v���X�V���~�߂邽�߂̈ړ����K��ϐ�
PRIVATE UCHAR		uc_max_x = GOAL_MAP_X_def;
PRIVATE UCHAR		uc_max_y = GOAL_MAP_Y_def;

//TKR
/* ���m��ԉ��� */
typedef struct
{
	UCHAR	uc_StrCnt;
	BOOL	bl_Known;
}stMAP_KNOWN;

PRIVATE stMAP_KNOWN		st_known = { 0,FALSE };

//**************************************************
// �v���g�^�C�v�錾�i�t�@�C�����ŕK�v�Ȃ��̂����L�q�j
//**************************************************
extern PUBLIC	UCHAR		dcom[];					// ���n�M����p
extern PUBLIC	UCHAR		scom[];					// �X�����[���p
extern PUBLIC	UCHAR		tcom[];					// �΂ߑ��s�p


// *************************************************************************
//   �@�\		�F ���H���W���[��������������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PUBLIC void MAP_init( void )
{
	UCHAR uc_dummy[ MAP_Y_SIZE ][ MAP_X_SIZE ];			// ���H�f�[�^

	/* ���W�A�����A���H���������� */
	en_Head		= NORTH;
	mx		= 0;
	my		= 0;
	MAP_clearMap();
	
	/* ���s�p�̃p�����[�^ */
	f_MoveBackDist = 0;
	uc_SlaCnt = 0;
	
//	Storage_Load( (const void*)uc_dummy, sizeof(uc_dummy), ADR_MAP );			// �f�[�^���[�h(dummy) ���ꂪ�Ȃ��Ǝ��̂P���ڂ�save�����܂������Ȃ�

	/* �o�b�N�A�b�v���H�̕��A */
/*	Storage_Load( (const void*)uc_back, sizeof(uc_back), ADR_MAP );				// �o�b�N�A�b�v�f�[�^�𕜋A����
	if( ( uc_back[0][0] & 0xf0 ) == 0xf0  ){									// �f�[�^����������
		
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�_�@���@�^");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�@�^�P�_�@�@�^�P�P�P�P�P�P�P�P�P");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@���i� �� ߁j���@���H�f�[�^��������");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�@�_�Q�^�@�@�_�Q�Q�Q�Q�Q�Q�Q�Q�Q");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�^�@���@�_");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@ �� �ȁ@�ȁ� �^�P�P�P�P�P�P�P�P�P�P");
		printf("\n\r�@�P�P�P�P�P�P�P�P�_ ���� �ȁ� �_�i ߁�߁j���@���A�����A�����A���I");
		printf("\n\r�@�@���A���`�`�`�I ���i߁�߁j/ �@�b�@�@�@/�@�_�Q�Q�Q�Q�Q�Q�Q�Q�Q�Q");
		printf("\n\r�@�Q�Q�Q�Q�Q�Q�Q�Q�^ �b�@�@�q�@�@�b�@�@ �b");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@ /�@�^�__�v�@/�@�^�_�v");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@ �P�@�@�@�@ / �^");
		printf("\n\r�@�@�@�@�@�@�@�@�@�@�@�@�@�@�@  �P");
		Storage_Load( (const void*)g_sysMap, sizeof(g_sysMap), ADR_MAP );		// �o�b�N�A�b�v���H�f�[�^�Ō��݂̖��H�����㏑��
	}

*/
}

// *************************************************************************
//   �@�\		�F �S�[�����W������������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.08.05			sato		�V�K
// *************************************************************************/
PUBLIC void MAP_Goal_init( void )
{
	GOAL_MAP_X = GOAL_MAP_X_def;
	GOAL_MAP_Y = GOAL_MAP_Y_def;
}

// *************************************************************************
//   �@�\		�F X�S�[�����W�ύX�v���O����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.08.05			sato		�V�K
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
			TIME_wait(SW_CHATTERING_WAIT);			// SW���������܂ő҂�
			printf("GOAL_x %d\r\n",GOAL_MAP_X);
		} 
		else if (( SW_ON == SW_EXE_PIN )||(TRUE == MODE_CheckExe())){
			LED4 = LED4_ALL_OFF;
			break;
		}
	}
}

// *************************************************************************
//   �@�\		�F y�S�[�����W�ύX�v���O����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.08.05			sato		�V�K
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
			TIME_wait(SW_CHATTERING_WAIT);			// SW���������܂ő҂�
			printf("GOAL_y %d\r\n",GOAL_MAP_Y);
		} 
		else if (( SW_ON == SW_EXE_PIN )||(TRUE == MODE_CheckExe())){
			LED4 = LED4_ALL_OFF;
			break;
		}
	}
}

// *************************************************************************
//   �@�\		�F ���H�����N���A����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PUBLIC void MAP_ClearMapData( void )
{
	/* ���W�A�����A���H���������� */
	en_Head		= NORTH;
	mx			= 0;
	my			= 0;
	MAP_clearMap();
	
	/* ���s�p�̃p�����[�^ */
	f_MoveBackDist = 0;
	uc_SlaCnt = 0;

//	Storage_Clear( sizeof(g_sysMap), ADR_MAP );			// �f�[�^�Z�[�u

}

// *************************************************************************
//   �@�\		�F ���W�ʒu��ݒ肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PUBLIC void MAP_setPos( UCHAR uc_x, UCHAR uc_y, enMAP_HEAD_DIR en_dir )
{
	mx		= uc_x;
	my		= uc_y;
	en_Head		= en_dir;
	
	MAP_setCmdPos( uc_x, uc_y, en_dir );

}

// *************************************************************************
//   �@�\		�F ���H�f�[�^��\������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PUBLIC void MAP_showLog( void )
{
	SHORT	x,y;
	CHAR	c_data;
	
	/* ---------- */
	/*  �ʏ���H  */
	/* ---------- */
	printf("\n\r  /* ---------- */   ");
	printf("\n\r  /*  �ʏ���H  */   ");
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
//   �@�\		�F ���H�f�[�^���N���A����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PUBLIC void MAP_clearMap( void )
{
	USHORT	x, y;
	UCHAR	uc_data;

	/* ���ׂẴ}�b�v�f�[�^�𖢒T����Ԃɂ��� */
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
//   �@�\		�F �Z���T��񂩂�Ǐ����擾����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PRIVATE UCHAR MAP_getWallData( void )
{
	UCHAR	 uc_wall;

//	LED_offAll();			// debug

	// �Z���T��񂩂�Ǐ��쐬
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

	// �}�E�X�̐i�s�����ɂ��킹�ăZ���T�f�[�^���ړ����ǃf�[�^�Ƃ���
	if		( en_Head == EAST ){
		uc_wall = uc_wall >> 3;
	}
	else if ( en_Head == SOUTH ){
		uc_wall = uc_wall >> 2;
	}
	else if ( en_Head == WEST ){
		uc_wall = uc_wall >> 1;
	}

	//	�T���ς݃t���O�𗧂Ă�
	return ( uc_wall | 0xf0 );
}

// *************************************************************************
//   �@�\		�F �Ǐ����쐬����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PRIVATE void MAP_makeMapData( void )
{
	UCHAR uc_wall;

	//	���s���̕Ǐ�����H���ɏ���
	if ( ( mx == 0 ) && ( my == 0 ) ){
		uc_wall = 0xfe;
	}
	else{
		uc_wall = MAP_getWallData();
	}
	g_sysMap[my][mx] = uc_wall;

	//	�ׂ̋�Ԃ̂l�`�o�f�[�^���X�V����
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
//   �@�\		�F �������}�b�v���쐬����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PUBLIC void  MAP_makeContourMap( 
	UCHAR uc_goalX, 			///< [in] �S�[��X���W
	UCHAR uc_goalY, 			///< [in] �S�[��Y���W
	enMAP_ACT_MODE	en_type		///< [in] �v�Z���@�i�܂����g�p�j
){
	USHORT		x, y, i;		// ���[�v�ϐ�
	UCHAR		uc_dase;		// ��l
	UCHAR		uc_new;			// �V�l
	UCHAR		uc_level;		// ������
	UCHAR		uc_wallData;	// �Ǐ��

	en_type = en_type;		// �R���p�C�����[�j���O����i������폜�j

	/* �������}�b�v������������ */
	for ( i = 0; i < MAP_SMAP_MAX_VAL; i++ ){
		us_cmap[ i / MAP_Y_SIZE][ i & (MAP_X_SIZE-1) ] = MAP_SMAP_MAX_VAL - 1;
	}
	/* �ڕW�n�_�̓�������0�ɐݒ� */
	us_cmap[uc_goalY][uc_goalX] = 0;

	/* �������}�b�v���쐬 */
	uc_dase = 0;
	do{
		uc_level = 0;
		uc_new = uc_dase + 1;
		for ( y = 0; y < MAP_Y_SIZE; y++ ){
			for ( x = 0; x < MAP_X_SIZE; x++ ){
				if ( us_cmap[y][x] == uc_dase ){
					uc_wallData = g_sysMap[y][x];
					
					/* �T�����s */
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
					/* �ŒZ���s */
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
//   �@�\		�F �}�E�X�̐i�s���������肷��
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PRIVATE void MAP_calcMouseDir( 
	enMAP_SEARCH_TYPE	en_calcType,	///< [in] �v�Z���@
	enMAP_HEAD_DIR* 	p_head			///< [out] �i�s�����i�߂�l�j
){
	UCHAR		uc_wall;				// �Ǐ��
	USHORT		us_base;				// �������D��x����l
	USHORT		us_new;
	enMAP_HEAD_DIR	en_tmpHead;

	/* �����v�Z */
	// ������MAP�@
	if( CONTOUR_SYSTEM == en_calcType ){
		// ���ӂ�4���ň�ԖړI�n�ɋ߂��ړ��������Z�o����B
		// �������A�ړ��ł����ԋ߂���Ԃ���������ꍇ�ɂ́A���̏��őI������B
		// �@���T�����,���i �A���T�����,���� �B���T�����,���i �C���T�����,����
		uc_wall = g_sysMap[my][mx];
		us_base = MAP_SMAP_MAX_PRI_VAL;					// 16[���]�~16[���]�~4[����]

		/* 4�������r */
		//	�k�����̋��̊m�F
		if ( ( uc_wall & 1 ) == 0 ){
			us_new = us_cmap[my+1][mx] * 4 + 4;
			if ( ( g_sysMap[my+1][mx] & 0xf0 ) != 0xf0 ) us_new = us_new - 2;
			if ( en_Head == NORTH ) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = NORTH;
			}
		}
		//	�������̋��̊m�F
		if ( ( uc_wall & 2 ) == 0 ){
			us_new = us_cmap[my][mx+1] * 4 + 4;
			if ( ( g_sysMap[my][mx+1] & 0xf0 ) != 0xf0 ) us_new = us_new - 2;
			if ( en_Head == EAST) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = EAST;
			}
		}
		//	������̋��̊m�F
		if ( ( uc_wall & 4 ) == 0 ){
			us_new = us_cmap[my-1][mx] * 4 + 4;
			if ( ( g_sysMap[my-1][mx] & 0xf0 ) != 0xf0) us_new = us_new - 2;
			if ( en_Head == SOUTH ) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = SOUTH;
			}
		}
		//	�������̋��̊m�F
		if ( ( uc_wall & 8 ) == 0 ){
			us_new = us_cmap[my][mx-1] * 4 + 4;
			if ( ( g_sysMap[my][mx-1] & 0xf0 ) != 0xf0 ) us_new = us_new - 2;
			if ( en_Head == WEST ) us_new = us_new - 1;
			if ( us_new < us_base ){
				us_base = us_new;
				en_tmpHead = WEST;
			}
		}
		
		*p_head = (enMAP_HEAD_DIR)( (en_tmpHead - en_Head) & 3 );		// �ړ�����
		
	}
	// ������@�w��Ȃ�
	else{
		*p_head = (enMAP_HEAD_DIR)0;
	}

}

// *************************************************************************
//   �@�\		�F �}�E�X�̍��W�ʒu���X�V����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PRIVATE void MAP_refMousePos( 
	enMAP_HEAD_DIR 			en_head			///< [in] �i�s����
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
//   �@�\		�F ���̋��Ɉړ�����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PRIVATE void MAP_moveNextBlock( 
	enMAP_HEAD_DIR 	en_head,		///< [in] ���ΐi�s�����i�}�E�X�i�s������k�Ƃ��Ă���j
	BOOL*			p_type			///< [in] FALSE: �P��ԑO�i��ԁATURE:����ԑO�i���
){
	*p_type = TRUE;
	f_MoveBackDist = 0;				// �ړ����������Z�l�N���A
	
	/* ���� */
	switch( en_head ){

		/* ���̂܂ܑO�i */
		case NORTH:
			*p_type = FALSE;
			MOT_goBlock_Const( 1 );				// 1���O�i
			break;
		// �E�ɐ��񂷂�
		case EAST:
			MOT_goBlock_FinSpeed( 0.5, 0 );		// �����O�i
//			TIME_wait( MAP_TURN_WAIT );
			MOT_turn(MOT_R90);									// �E90�x����
//			TIME_wait( MAP_TURN_WAIT );
			break;
		// ���ɐ��񂷂�
		case WEST:
			MOT_goBlock_FinSpeed( 0.5, 0 );		// �����O�i
//			TIME_wait( MAP_TURN_WAIT );
			MOT_turn(MOT_L90);									// �E90�x����
//			TIME_wait( MAP_TURN_WAIT );
			break;
		// ���]���Ė߂�
		case SOUTH:
			MOT_goBlock_FinSpeed( 0.5, 0 );		// �����O�i
//			TIME_wait( MAP_TURN_WAIT );
			MOT_turn(MOT_R180);									// �E180�x����
//			TIME_wait( MAP_TURN_WAIT );
			
			/* �Ǔ��Ďp������i���ɕǂ���������o�b�N�{�ړ����������Z����j */
			if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) )  ||		// �k�������Ă��Ėk�ɕǂ�����
				( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// ���������Ă��ē��ɕǂ�����
				( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) )  ||		// ��������Ă��ē�ɕǂ�����
				( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) ) 			// ���������Ă��Đ��ɕǂ�����
			){
				MOT_goHitBackWall();					// �o�b�N����
				f_MoveBackDist = MOVE_BACK_DIST;		// �o�b�N�������̈ړ�����[���]�����Z
//				TIME_wait( MAP_TURN_WAIT );				// ���ԑ҂�
			}
			break;
		default:
			break;
	}
	
	/* �O�i���Ƀp���[�����[�X�@�\�������ă��W���[�����Ȃ���΂Ȃ�Ȃ� */
/*	if( ( TRUE == DCMC_isPowerRelease() ) && ( en_head == NORTH ) ){
		
		MOT_goBack_Const( MOT_BACK_POLE );					// �P�O�̒��܂Ō��
		MAP_makeMapData();									// �ǃf�[�^������H�f�[�^���쐬			�� �����Ńf�[�^�쐬���~�X���Ă���
		MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			// ������MAP�@�Ői�s�������Z�o			�� �����MAP���쐬
		MAP_moveNextBlock(en_head, p_type);					// �����P�x�Ăяo���i���̋��ֈړ��j
	}
	else{*/
		/* �i�s�����X�V */
		en_Head = (enMAP_HEAD_DIR)( (en_Head + en_head) & (MAP_HEAD_DIR_MAX-1) );
//	}

}

// *************************************************************************
//   �@�\		�F �X�����[���ɂĎ��̋��Ɉړ�����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PRIVATE void MAP_moveNextBlock_Sura( 
	enMAP_HEAD_DIR 	en_head,		///< [in] ���ΐi�s�����i�}�E�X�i�s������k�Ƃ��Ă���j
	BOOL*			p_type,			///< [in] FALSE: �P��ԑO�i��ԁATURE:����ԑO�i���
	BOOL			bl_resume		///< [in] FALSE: ���W���[������ł͂Ȃ��ATURE:���W���[������
){
	*p_type = FALSE;
	f_MoveBackDist = 0;				// �ړ����������Z�l�N���A
	
	/* ���� */
	switch( en_head ){

		// ���̂܂ܑO�i
		case NORTH:
			
			/* ���W���[������ł͂Ȃ� */
			if( bl_resume == FALSE ){
		
				MOT_goBlock_Const( 1 );					// 1���O�i
				uc_SlaCnt = 0;							// �X�����[�����Ă��Ȃ�
			}
			/* ���W���[������ */
			else{
				MOT_goBlock_FinSpeed( 1.0f, SEARCH_SPEED );		// �����O�i(�o�b�N�̈ړ��ʂ��܂�)
				uc_SlaCnt = 0;										// �X�����[�����Ă��Ȃ�
			}
			break;

		// �E�ɃX�����[������
		case EAST:
			if( uc_SlaCnt < 7 ){
				MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// �E�X�����[��
				uc_SlaCnt++;
			}
			else{
				/* �Ǔ��Ďp������i���ɕǂ���������o�b�N�{�ړ����������Z����j */
				if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) )  ||		// �k�������Ă��Đ��ɕǂ�����
					( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) )  ||		// ���������Ă��Ėk�ɕǂ�����
					( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// ��������Ă��ē��ɕǂ�����
					( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) ) 			// ���������Ă��ē�ɕǂ�����
				){
					MOT_goBlock_FinSpeed( 0.5, 0 );			// �����O�i
//					TIME_wait( MAP_TURN_WAIT );
					MOT_turn(MOT_R90);						// �E90�x����
//					TIME_wait( MAP_TURN_WAIT );
					uc_SlaCnt = 0;
					MOT_goHitBackWall();					// �o�b�N����
					f_MoveBackDist = MOVE_BACK_DIST;		// �o�b�N�������̈ړ�����[���]�����Z
//					TIME_wait( MAP_SLA_WAIT );				// ���ԑ҂�
					*p_type = TRUE;							// ���͔���ԁi�{�o�b�N�j���i�߂�
				}
				else{
					MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// �E�X�����[��
					uc_SlaCnt++;
				}
			}
			break;

		// ���ɃX�����[������
		case WEST:
			if( uc_SlaCnt < 7 ){
				MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );	// ���X�����[��
				uc_SlaCnt++;
			}
			else{
				/* �Ǔ��Ďp������i���ɕǂ���������o�b�N�{�ړ����������Z����j */
				if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// �k�������Ă��ē��ɕǂ�����
					( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) )  ||		// ���������Ă��ē�ɕǂ�����
					( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) )  ||		// ��������Ă��Đ��ɕǂ�����
					( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) ) 			// ���������Ă��Ėk�ɕǂ�����
				){
					MOT_goBlock_FinSpeed( 0.5, 0 );		// �����O�i
//					TIME_wait( MAP_TURN_WAIT );
					MOT_turn(MOT_L90);					// �E90�x����
//					TIME_wait( MAP_TURN_WAIT );
					uc_SlaCnt = 0;
					MOT_goHitBackWall();					// �o�b�N����
					f_MoveBackDist = MOVE_BACK_DIST;		// �o�b�N�������̈ړ�����[���]�����Z
//					TIME_wait( MAP_SLA_WAIT );				// ���ԑ҂�
					*p_type = TRUE;							// ���͔���ԁi�{�o�b�N�j���i�߂�
				}
				else{
					MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );	// ���X�����[��
					uc_SlaCnt++;
				}
			}
			break;

		// ���]���Ė߂�
		case SOUTH:
			MOT_goBlock_FinSpeed( 0.5, 0 );			// �����O�i
//			TIME_wait( MAP_SLA_WAIT );
			MOT_turn(MOT_R180);									// �E180�x����
//			TIME_wait( MAP_SLA_WAIT );
			uc_SlaCnt = 0;
			
			/* �Ǔ��Ďp������i���ɕǂ���������o�b�N�{�ړ����������Z����j */
			if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) )  ||		// �k�������Ă��Ėk�ɕǂ�����
				( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// ���������Ă��ē��ɕǂ�����
				( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) )  ||		// ��������Ă��ē�ɕǂ�����
				( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) ) 			// ���������Ă��Đ��ɕǂ�����
			){
				MOT_goHitBackWall();					// �o�b�N����
				f_MoveBackDist = MOVE_BACK_DIST;		// �o�b�N�������̈ړ�����[���]�����Z
//				TIME_wait( MAP_SLA_WAIT );				// ���ԑ҂�
			}
			*p_type = TRUE;								// ���͔���ԁ{�o�b�N���i�߂�
			break;
			
		default:
			break;
	}
	
	/* �O�i���Ƀp���[�����[�X�@�\�������ă��W���[�����Ȃ���΂Ȃ�Ȃ� */
/*	if( ( TRUE == DCMC_isPowerRelease() ) && ( en_head == NORTH ) ){
		
		TIME_wait(1000);
		MOT_goBack_Const( MOT_BACK_POLE );					// �P�O�̒��܂Ō��
		MAP_makeMapData();									// �ǃf�[�^������H�f�[�^���쐬			�� �����Ńf�[�^�쐬���~�X���Ă���
		MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			// ������MAP�@�Ői�s�������Z�o			�� �����MAP���쐬
		MAP_moveNextBlock_Sura(en_head, p_type, TRUE );		// �����P�x�Ăяo���i���̋��ֈړ��j
	}
	else{*/
		/* �i�s�����X�V */
		en_Head = (enMAP_HEAD_DIR)( (en_Head + en_head) & (MAP_HEAD_DIR_MAX-1) );
//	}

}

// *************************************************************************
//   �@�\		�F �S�[�����̓���
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PRIVATE void MAP_actGoal( void )
{	
	MOT_goBlock_FinSpeed( 0.5, 0 );			// �����O�i
	TIME_wait(500);
	MOT_turn(MOT_R180);										// �E180�x����
	TIME_wait(500);
	
//	MAP_SaveMapData();						// ���H���̃o�b�N�A�b�v
	log_flag_off();
	MAP_actGoalLED();
	
	en_Head = (enMAP_HEAD_DIR)( (en_Head + 2) & (MAP_HEAD_DIR_MAX-1) );			//	�i�s�����X�V

}

// *************************************************************************
//   �@�\		�F �S�[������LED����
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
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
//   �@�\		�F �S�[�����W�T�C�Y�ύX
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.11.23			sato		�V�K
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
//   �@�\		�F �A�ҒT���p�������}�b�v�쐬
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.11.23			sato		�V�K
// *************************************************************************/
PUBLIC void  MAP_makeReturnContourMap(UCHAR uc_staX,UCHAR uc_staY) 
{
	USHORT		x, y, i;		// ���[�v�ϐ�
	UCHAR		uc_dase;		// ��l
	UCHAR		uc_new;			// �V�l
	UCHAR		uc_level;		// ������
	UCHAR		uc_wallData;	// �Ǐ��

	/* �������}�b�v������������ */
	for (i = 0; i < MAP_SMAP_MAX_VAL; i++) {
		us_cmap[i / MAP_Y_SIZE][i & (MAP_X_SIZE - 1)] = MAP_SMAP_MAX_VAL - 1;
	}
	/* �ڕW�n�_�̓�������0�ɐݒ� */
	us_cmap[0][0] = 0;

	/* �������}�b�v���쐬 */
	uc_dase = 0;
	do {
		uc_level = 0;
		uc_new = uc_dase + 1;
		for (y = 0; y < MAP_Y_SIZE; y++) {
			for (x = 0; x < MAP_X_SIZE; x++) {
				if ((us_cmap[uc_staY][uc_staX] != MAP_SMAP_MAX_VAL - 1) && (us_cmap[uc_staY][uc_staX] + 2 < uc_new))break;
				if (us_cmap[y][x] == uc_dase) {
					uc_wallData = g_sysMap[y][x];
					/* �T�����s */
	
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
//   �@�\		�F �i�ދ��������T���ς݂����T�����𔻒�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F TRUE:�T���ς�	FALSE:���T��
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.29			TKR			�V�K
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
//   �@�\		�F ���̋��Ɉړ�����i���m��ԉ����j
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F ���ΐi�s�����i�}�E�X�i�s������k�Ƃ��Ă���j�A�O�i��ԁiFALSE: �P��ԑO�i��ԁATURE:����ԑO�i��ԁj
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2014.09.30			�O��			�V�K
// *************************************************************************/
PRIVATE void MAP_moveNextBlock_acc(enMAP_HEAD_DIR en_head, BOOL* p_type)
{
	*p_type = FALSE;
	f_MoveBackDist = 0;

	/* ���� */
	switch (en_head) {

		/* ���̂܂ܑO�i */
	case NORTH:
//		*p_type = FALSE;
//		LED = LED6;
		if (MAP_KnownAcc() == FALSE) {					// ���ɐi�ދ�悪���T���̂Ƃ�
			if (st_known.bl_Known == TRUE){
				if (st_known.uc_StrCnt < 2) {
					MOT_goBlock_Const(1);					// 1���̏ꍇ�͓����̂܂�
				}
				else {
					MOT_setTrgtSpeed(MAP_KNOWN_ACC_SPEED);									// ���m��ԉ�������Ƃ��̖ڕW���x	
					MOT_goBlock_FinSpeed((FLOAT)(st_known.uc_StrCnt), SEARCH_SPEED);				// n���O�i
					MOT_setTrgtSpeed(SEARCH_SPEED);										// �ڕW���x���f�t�H���g�l�ɖ߂�
				}
			}
			MOT_goBlock_Const(1);	////////////////////
			st_known.uc_StrCnt = 0;
			st_known.bl_Known = FALSE;

		}
		else {

			st_known.uc_StrCnt++;			// �ړ����̉��Z
			st_known.bl_Known = TRUE;
		}

		break;

		/* �E�ɐ��񂷂� */
	case EAST:
//		LED = LED8;
		if (st_known.bl_Known == TRUE) {		// ������������
			if (st_known.uc_StrCnt < 2) {
				MOT_goBlock_Const(1);					// 1���̏ꍇ�͓����̂܂�
			}
			else {
//				LED = LED_ALL_ON;
				MOT_setTrgtSpeed(MAP_KNOWN_ACC_SPEED);									// ���m��ԉ�������Ƃ��̖ڕW���x	
				MOT_goBlock_FinSpeed((FLOAT)(st_known.uc_StrCnt), SEARCH_SPEED);				// n���O�i
				MOT_setTrgtSpeed(SEARCH_SPEED);										// �ڕW���x���f�t�H���g�l�ɖ߂�
//				LED = LED_ALL_OFF;
			}
			st_known.uc_StrCnt = 0;		/////////////////////////////////////////
			st_known.bl_Known = FALSE;
		}

		if( uc_SlaCnt < 7 ){
				MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// �E�X�����[��
				uc_SlaCnt++;
			}
			else{
				f_MoveBackDist = 0;
				/* �Ǔ��Ďp������i���ɕǂ���������o�b�N�{�ړ����������Z����j */
				if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) )  ||		// �k�������Ă��Đ��ɕǂ�����
					( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) )  ||		// ���������Ă��Ėk�ɕǂ�����
					( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// ��������Ă��ē��ɕǂ�����
					( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) ) 			// ���������Ă��ē�ɕǂ�����
				){
					MOT_goBlock_FinSpeed( 0.5, 0 );			// �����O�i
//					TIME_wait( MAP_TURN_WAIT );
					MOT_turn(MOT_R90);						// �E90�x����
//					TIME_wait( MAP_TURN_WAIT );
					uc_SlaCnt = 0;
					MOT_goHitBackWall();					// �o�b�N����
					f_MoveBackDist = MOVE_BACK_DIST;		// �o�b�N�������̈ړ�����[���]�����Z
//					TIME_wait( MAP_SLA_WAIT );				// ���ԑ҂�
					*p_type = TRUE;							// ���͔���ԁi�{�o�b�N�j���i�߂�
				}
				else{
					MOT_goSla( MOT_R90S, PARAM_getSra( SLA_90 ) );	// �E�X�����[��
					uc_SlaCnt++;
				}
			}
		break;

		/* ���ɐ��񂷂� */
	case WEST:
//		LED = LED1;
		if (st_known.bl_Known == TRUE) {		// ������������
			if (st_known.uc_StrCnt < 2) {
				MOT_goBlock_Const(1);					// 1���̏ꍇ�͓����̂܂�
			}
			else {
//				LED = LED_ALL_ON;
				MOT_setTrgtSpeed(MAP_KNOWN_ACC_SPEED);									// ���m��ԉ�������Ƃ��̖ڕW���x	
				MOT_goBlock_FinSpeed((FLOAT)(st_known.uc_StrCnt), SEARCH_SPEED);				// n���O�i
				MOT_setTrgtSpeed(SEARCH_SPEED);										// �ڕW���x���f�t�H���g�l�ɖ߂�
//				LED = LED_ALL_OFF;
			}
			st_known.uc_StrCnt = 0;			//////////////////////////////////////
			st_known.bl_Known = FALSE;
		}

		if( uc_SlaCnt < 7 ){
				MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );	// ���X�����[��
				uc_SlaCnt++;
			}
			else{
				f_MoveBackDist = 0;
				/* �Ǔ��Ďp������i���ɕǂ���������o�b�N�{�ړ����������Z����j */
				if( ( ( en_Head == NORTH ) && ( ( g_sysMap[my][mx] & 0x02 ) != 0 ) )  ||		// �k�������Ă��ē��ɕǂ�����
					( ( en_Head == EAST  ) && ( ( g_sysMap[my][mx] & 0x04 ) != 0 ) )  ||		// ���������Ă��ē�ɕǂ�����
					( ( en_Head == SOUTH ) && ( ( g_sysMap[my][mx] & 0x08 ) != 0 ) )  ||		// ��������Ă��Đ��ɕǂ�����
					( ( en_Head == WEST  ) && ( ( g_sysMap[my][mx] & 0x01 ) != 0 ) ) 			// ���������Ă��Ėk�ɕǂ�����
				){
					MOT_goBlock_FinSpeed( 0.5, 0 );		// �����O�i
//					TIME_wait( MAP_TURN_WAIT );
					MOT_turn(MOT_L90);					// �E90�x����
//					TIME_wait( MAP_TURN_WAIT );
					uc_SlaCnt = 0;
					MOT_goHitBackWall();					// �o�b�N����
					f_MoveBackDist = MOVE_BACK_DIST;		// �o�b�N�������̈ړ�����[���]�����Z
//					TIME_wait( MAP_SLA_WAIT );				// ���ԑ҂�
					*p_type = TRUE;							// ���͔���ԁi�{�o�b�N�j���i�߂�
				}
				else{
					MOT_goSla( MOT_L90S, PARAM_getSra( SLA_90 ) );	// ���X�����[��
					uc_SlaCnt++;
				}
			}
		break;

		/* ���]���Ė߂� */
	case SOUTH:
//		LED = LED_ALL_ON;
		MOT_goBlock_FinSpeed(0.5, 0);			// �����O�i
//		TIME_wait(MAP_SLA_WAIT);
		MOT_turn(MOT_R180);									// �E180�x����
//		TIME_wait(MAP_SLA_WAIT);
		uc_SlaCnt = 0;

		/* �Ǔ��Ďp������i���ɕǂ���������o�b�N�{�ړ����������Z����j */
		if (((en_Head == NORTH) && ((g_sysMap[my][mx] & 0x01) != 0)) ||		// �k�������Ă��Ėk�ɕǂ�����
			((en_Head == EAST) && ((g_sysMap[my][mx] & 0x02) != 0)) ||		// ���������Ă��ē��ɕǂ�����
			((en_Head == SOUTH) && ((g_sysMap[my][mx] & 0x04) != 0)) ||		// ��������Ă��ē�ɕǂ�����
			((en_Head == WEST) && ((g_sysMap[my][mx] & 0x08) != 0)) 			// ���������Ă��Đ��ɕǂ�����
			) {
			MOT_goHitBackWall();					// �o�b�N����
			f_MoveBackDist = MOVE_BACK_DIST;	// �o�b�N�������̈ړ�����[���]�����Z
//			TIME_wait(MAP_SLA_WAIT);				// ���ԑ҂�
		}
		*p_type = TRUE;								// ���͔���ԁ{�o�b�N���i�߂�
		break;

	default:
		break;
	}

#ifndef POWER_RELESASE
	/* �i�s�����X�V */
//	en_Head = (enMAP_HEAD_DIR)( (en_Head + en_head) & (MAP_HEAD_DIR_MAX-1) );
	en_Head = (enMAP_HEAD_DIR)(((UCHAR)en_Head + (UCHAR)en_head) & (MAP_HEAD_DIR_MAX - 1));
#else
	/* �O�i���Ƀp���[�����[�X�@�\�������ă��W���[�����Ȃ���΂Ȃ�Ȃ� */
	if ((TRUE == DCMC_isPowerRelease()) && (en_head == NORTH)) {

		MOT_goBack_Const(MOT_BACK_POLE);					// �P�O�̒��܂Ō��
		MAP_makeMapData();									// �ǃf�[�^������H�f�[�^���쐬			�� �����Ńf�[�^�쐬���~�X���Ă���
		MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			// ������MAP�@�Ői�s�������Z�o			�� �����MAP���쐬
		MAP_moveNextBlock(en_head, p_type);					// �����P�x�Ăяo���i���̋��ֈړ��j
	}
	else {
		/* �i�s�����X�V */
		en_Head = (enMAP_HEAD_DIR)((en_Head + en_head) & (MAP_HEAD_DIR_MAX - 1));
	}
#endif
}


// *************************************************************************
//   �@�\		�F �S�[���̒T������
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2018.09.27			sato		�V�K
// *************************************************************************/
PUBLIC void MAP_searchGoal(
	UCHAR 			uc_trgX, 		///< [in] �ڕWx���W
	UCHAR 			uc_trgY, 		///< [in] �ڕWy���W 
	enMAP_ACT_MODE 	en_type, 		///< [in] �T�����@
	enSEARCH_MODE	en_search 		///< [in] �T�����@
){
	enMAP_HEAD_DIR	en_head = NORTH;
	BOOL		bl_type = TRUE;			// ���݈ʒu�AFALSE: �P��ԑO�i��ԁATURE:����ԑO�i���
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

//	SYS_setDisable( SYS_MODE );				// ���[�h�ύX�֎~

	MOT_setTrgtSpeed(SEARCH_SPEED);		// �ڕW���x
	MOT_setNowSpeed( 0.0f );
	f_MoveBackDist = 0;
	uc_SlaCnt = 0;
	if(uc_trgX == GOAL_MAP_X && uc_trgY == GOAL_MAP_Y){
		f_MoveBackDist = MOVE_BACK_DIST;
	}
	
	log_flag_on();	//���O�֐��X�^�[�g�i���폜�j
	
	/* ���H�T�� */
	while(1){
		MAP_refMousePos( en_Head );								// ���W�X�V
//		MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// �������}�b�v�����
		
		/* ���M�n����T�� */
		if( SEARCH_TURN == en_search ){
			MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// �������}�b�v�����
			if( TRUE == bl_type ){
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, SEARCH_SPEED );		// �����O�i(�o�b�N�̈ړ��ʂ��܂�)
			}
			MAP_makeMapData();												// �ǃf�[�^������H�f�[�^���쐬			�� �����Ńf�[�^�쐬���~�X���Ă���
			MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);						// ������MAP�@�Ői�s�������Z�o			�� �����MAP���쐬
			
			/* ���̋��ֈړ� */
			if(( mx == uc_trgX ) && ( my == uc_trgY )){
				MAP_actGoal();										// �S�[�����̓���
				break;
			}
			else{
				MAP_moveNextBlock(en_head, &bl_type);				// ���̋��ֈړ�								�� �����ŉ��߂ă����[�X�`�F�b�N�{�Ǎēx�쐬�{�������{���M�n���񓮍�
			}
		}
		/* �X�����[���T�� */
		else if( SEARCH_SURA == en_search ){
			MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// �������}�b�v�����
			if( TRUE == bl_type ){
				
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, SEARCH_SPEED );		// �����O�i(�o�b�N�̈ړ��ʂ��܂�)
			}
			MAP_makeMapData();		// �ǃf�[�^������H�f�[�^���쐬
			
			MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);				// ������MAP�@�Ői�s�������Z�o			�� �����MAP���쐬
			
			/* ���̋��ֈړ� */
			if(( mx == uc_trgX ) && ( my == uc_trgY )){
				MAP_actGoal();										// �S�[�����̓���
				break;
			}
			else{
				MAP_moveNextBlock_Sura(en_head, &bl_type, FALSE );	// ���̋��ֈړ�						�� �����ŉ��߂ă����[�X�`�F�b�N�{�Ǎēx�쐬�{�������{���M�n���񓮍�
//				MAP_moveNextBlock_acc(en_head, &bl_type);
			}
		}
		/* �A�ҒT�� */
		else if (SEARCH_RETURN == en_search) {
			
			if( TRUE == bl_type ){
				
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, SEARCH_SPEED );		// �����O�i(�o�b�N�̈ړ��ʂ��܂�)
			}
			MAP_makeMapData();		// �ǃf�[�^������H�f�[�^���쐬
			MAP_makeReturnContourMap(uc_staX,uc_staY);
			MAP_searchCmdList(uc_staX, uc_staY, en_Head, uc_goalX, uc_goalX, &en_endDir);
			uc_trgX = Return_X;
			uc_trgY = Return_Y;
			MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// �������}�b�v�����
			MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			
			
			/* ���̋��ֈړ� */
//			if ((us_cmap[my][mx] == 0)||((g_sysMap[uc_trgY][uc_trgX]&0xf0) == 0xf0)) {
			if ((mx == 0)&&(my == 0)){
				MAP_actGoal();
				break;
			}
//			}
			else {
				MAP_moveNextBlock_Sura(en_head, &bl_type, FALSE);	// ���̋��ֈړ�						�� �����ŉ��߂ă����[�X�`�F�b�N�{�Ǎēx�쐬�{�������{���M�n���񓮍�
			}
//			LED_count(uc_trgY);
		}

		
		/* �r���Ő���s�\�ɂȂ��� */
		if( SYS_isOutOfCtrl() == TRUE ){
			CTRL_stop();
			DCM_brakeMot( DCM_R );		// �u���[�L
			DCM_brakeMot( DCM_L );		// �u���[�L
			
			/* ���H�֘A�������� */
			en_Head		= NORTH;
			mx			= 0;
			my			= 0;
			f_MoveBackDist = 0;
			
			// DCMC�͉��ʃ��W���[���Ŋ��ɃN���A�Ƌً}��~���s���Ă���B
			break;
		}
	}
	search_flag = FALSE;
	TIME_wait(1000);
//	SYS_setEnable( SYS_MODE );				// ���[�h�ύX�L��

}

// *************************************************************************
//   �@�\		�F �T���i���m��ԉ����j
//   ����		�F �Ȃ�
//   ����		�F �Ȃ�
//   ����		�F �ڕWx���W�A�ڕWy���W�A�T�����@
//   �Ԃ�l		�F �Ȃ�
// **************************    ��    ��    *******************************
// 		v1.0		2019.11.3			TKR			searchGoal�֐�����ڐA
// *************************************************************************/
PUBLIC void MAP_searchGoalKnown(
	UCHAR 			uc_trgX, 		///< [in] �ڕWx���W
	UCHAR 			uc_trgY, 		///< [in] �ڕWy���W 
	enMAP_ACT_MODE 	en_type, 		///< [in] �T�����@
	enSEARCH_MODE	en_search 		///< [in] �T�����@
){
	enMAP_HEAD_DIR	en_head = NORTH;
	BOOL		bl_type = TRUE;			// ���݈ʒu�AFALSE: �P��ԑO�i��ԁATURE:����ԑO�i���
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

//	SYS_setDisable( SYS_MODE );				// ���[�h�ύX�֎~

	MOT_setTrgtSpeed(SEARCH_SPEED);		// �ڕW���x
	MOT_setNowSpeed( 0.0f );
	f_MoveBackDist = 0;
	uc_SlaCnt = 0;
	if(uc_trgX == GOAL_MAP_X && uc_trgY == GOAL_MAP_Y){
		f_MoveBackDist = MOVE_BACK_DIST;
	}
	
	log_flag_on();	//���O�֐��X�^�[�g�i���폜�j
	
	/* ���H�T�� */
	while(1){
		MAP_refMousePos( en_Head );								// ���W�X�V
//		MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// �������}�b�v�����
		
		/* ���M�n����T�� */
		if( SEARCH_TURN == en_search ){
			MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// �������}�b�v�����
			if( TRUE == bl_type ){
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, SEARCH_SPEED );		// �����O�i(�o�b�N�̈ړ��ʂ��܂�)
			}
			MAP_makeMapData();												// �ǃf�[�^������H�f�[�^���쐬			�� �����Ńf�[�^�쐬���~�X���Ă���
			MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);						// ������MAP�@�Ői�s�������Z�o			�� �����MAP���쐬
			
			/* ���̋��ֈړ� */
			if(( mx == uc_trgX ) && ( my == uc_trgY )){
				MAP_actGoal();										// �S�[�����̓���
				break;
			}
			else{
				MAP_moveNextBlock(en_head, &bl_type);				// ���̋��ֈړ�								�� �����ŉ��߂ă����[�X�`�F�b�N�{�Ǎēx�쐬�{�������{���M�n���񓮍�
			}
		}
		/* �X�����[���T�� */
		else if( SEARCH_SURA == en_search ){
			MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// �������}�b�v�����
			if( TRUE == bl_type ){
				
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, SEARCH_SPEED );		// �����O�i(�o�b�N�̈ړ��ʂ��܂�)
			}
			if (st_known.bl_Known != TRUE) {
				MAP_makeMapData();		// �ǃf�[�^������H�f�[�^���쐬
			}
			MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);				// ������MAP�@�Ői�s�������Z�o			�� �����MAP���쐬
			
			/* ���̋��ֈړ� */
			if(( mx == uc_trgX ) && ( my == uc_trgY )){
				MAP_actGoal();										// �S�[�����̓���
				break;
			}
			else{
//				MAP_moveNextBlock_Sura(en_head, &bl_type, FALSE );	// ���̋��ֈړ�						�� �����ŉ��߂ă����[�X�`�F�b�N�{�Ǎēx�쐬�{�������{���M�n���񓮍�
				MAP_moveNextBlock_acc(en_head, &bl_type);
			}
		}
		/* �A�ҒT�� */
		else if (SEARCH_RETURN == en_search) {
			
			if( TRUE == bl_type ){
				
				MOT_goBlock_FinSpeed( 0.5 + f_MoveBackDist, SEARCH_SPEED );		// �����O�i(�o�b�N�̈ړ��ʂ��܂�)
			}
			if (st_known.bl_Known != TRUE) {
				MAP_makeMapData();		// �ǃf�[�^������H�f�[�^���쐬
			}
			MAP_makeReturnContourMap(uc_staX,uc_staY);
			MAP_searchCmdList(uc_staX, uc_staY, en_Head, uc_goalX, uc_goalX, &en_endDir);
			uc_trgX = Return_X;
			uc_trgY = Return_Y;
			MAP_makeContourMap( uc_trgX, uc_trgY, en_type );		// �������}�b�v�����
			MAP_calcMouseDir(CONTOUR_SYSTEM, &en_head);			
			
			/* ���̋��ֈړ� */
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

		
		/* �r���Ő���s�\�ɂȂ��� */
		if( SYS_isOutOfCtrl() == TRUE ){
			CTRL_stop();
			DCM_brakeMot( DCM_R );		// �u���[�L
			DCM_brakeMot( DCM_L );		// �u���[�L
			
			/* ���H�֘A�������� */
			en_Head		= NORTH;
			mx			= 0;
			my			= 0;
			f_MoveBackDist = 0;
			
			// DCMC�͉��ʃ��W���[���Ŋ��ɃN���A�Ƌً}��~���s���Ă���B
			break;
		}
	}
	search_flag = FALSE;
	TIME_wait(1000);
//	SYS_setEnable( SYS_MODE );				// ���[�h�ύX�L��

}
#ifndef DEFINES_H
#define DEFINES_H
#include "stdint.h"


typedef struct   
{
	uint8_t ms_2_flag : 1;
	uint8_t ten_ms_flag : 1;
	uint8_t fifty_ms_flag : 1;
	uint8_t hundred_ms_flag : 1;	
	uint8_t one_sec : 1;
}flags;

typedef struct
{
	uint8_t	key_pressed : 1;
	uint8_t proceed : 1;
	uint8_t Temp_key_pressed : 1;
	uint8_t Prev_Up_ky :1;
	uint8_t Prev_Down_ky :1;
	uint8_t Prev_Hs_ky :1;
	uint8_t Prev_Ack_ky :1;
	uint8_t Up_ky :1;
	uint8_t Down_ky :1;
	uint8_t Hs_ky :1;
	uint8_t Ack_ky :1;
	uint8_t up : 1;
	uint8_t down :1;
	uint8_t shift :1;
	uint8_t ack :1;
	uint8_t ACKON :1;
}keys;
typedef struct   
{
	uint8_t door1 : 1;
	uint8_t door2 : 1;
	uint8_t lightflag1 : 1;
	uint8_t lightflag2 : 1;
}doorflgs;
/*
struct GeneralFlag
{
    uint8_t generalg :1;
    uint8_t inputg   :1;
    uint8_t batteryg :1;
    uint8_t outputg  :1;
    uint8_t bypassg  :1;
};

union
{
    struct GeneralFlag Gen_Flag;
    uint8_t group_flag_byte;
}Gen_Flag_uni;
*/
typedef struct
{
	uint8_t IPAB : 1;
	uint8_t OC   : 1;
  uint8_t DCUV : 1;
  uint8_t OPUV : 1;
  uint8_t BLOW : 1;
  uint8_t OPOV : 1;
  uint8_t OT   : 1;
	uint8_t mains_ok_flag : 1;
	uint8_t mains_abnormal : 1;
	uint8_t fast_mains_fail : 1;
	uint8_t Mains_Fail1 : 1;
	uint8_t Mains_Fail : 1;
	uint8_t Main_Fail_ip_Temp_Flag : 1;
	uint8_t boost_flag : 1;
	uint8_t Normal_flag : 1;
	uint8_t Buck_flag : 1;
  uint8_t Normal_1_flag : 1;
}mains_status;

typedef union //WR_Arry
{
   uint16_t DATA_Words[8];
   uint8_t DATA_Bytes[16];
}uarry;

typedef union
{
		uint16_t UV_Words[1];
		uint8_t UV_Bytes[2];
}UVarry;



#define key_pressed 				key.key_pressed
#define proceed  						key.proceed
#define Temp_key_pressed 		key.Temp_key_pressed
#define Prev_Up_ky 					key.Prev_Up_ky
#define Prev_Down_ky 				key.Prev_Down_ky
#define Prev_Hs_ky 					key.Prev_Hs_ky
#define Prev_Ack_ky 				key.Prev_Ack_ky
#define Up_ky 							key.Up_ky
#define Down_ky 						key.Down_ky
#define Hs_ky 							key.Hs_ky
#define Ack_ky 							key.Ack_ky
#define up 									key.up
#define down 								key.down
#define shift 							key.shift
#define ack 								key.ack
#define ACKON               key.ACKON

#define door1               drflg.door1
#define door2               drflg.door2
#define lightflag1          drflg.lightflag1
#define lightflag2          drflg.lightflag2

#define ms_2_flag						flg.ms_2_flag	
#define ten_ms_flag         flg.ten_ms_flag
#define fifty_ms_flag       flg.fifty_ms_flag
#define hundred_ms_flag     flg.hundred_ms_flag
#define one_sec             flg.one_sec
/*
#define generalg         Gen_Flag_uni.Gen_Flag.generalg
#define inputg           Gen_Flag_uni.Gen_Flag.inputg
#define batteryg         Gen_Flag_uni.Gen_Flag.batteryg
#define outputg          Gen_Flag_uni.Gen_Flag.outputg
#define bypassg          Gen_Flag_uni.Gen_Flag.bypassg
#define group_flag_byte  Gen_Flag_uni.group_flag_byte
*/

#define IPAB    mainsstatus.IPAB
#define OC      mainsstatus.OC 
#define DCUV    mainsstatus.DCUV
#define OPUV    mainsstatus.OPUV
#define BLOW    mainsstatus.BLOW
#define OPOV    mainsstatus.OPOV
#define OT      mainsstatus.OT
#define mains_ok_flag    mainsstatus.mains_ok_flag
#define mains_abnormal   mainsstatus.mains_abnormal
#define fast_mains_fail  mainsstatus.fast_mains_fail
#define Mains_Fail1      mainsstatus.Mains_Fail1
#define Mains_Fail       mainsstatus.Mains_Fail
#define Main_Fail_ip_Temp_Flag  mainsstatus.Main_Fail_ip_Temp_Flag
#define boost_flag       mainsstatus.boost_flag
#define	Normal_flag      mainsstatus.Normal_flag 
#define	Buck_flag        mainsstatus.Buck_flag 
#define Normal_1_flag    mainsstatus.Normal_1_flag

#endif

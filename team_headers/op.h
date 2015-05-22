/*
 * op.h
 *
 *  Created on: Oct 30, 2013
 *      Author: Nathan
 */

#ifndef OP_H_
#define OP_H_

//todo USER: define operation struct
//State defines
#define STATE_INIT 			0
#define STATE_SENSOR_COV 	1
#define STATE_BOOT			2
#define STATE_POWER_DOWN	3

#define OPS_ID_STATE 			0
#define OPS_ID_STOPWATCHERROR 	1

typedef struct CHANGE_OPS
{
	char State : 1;
	char Flags : 1;
}change_struct;

typedef struct MIRROR_STATS
{
	unsigned char Buffer_level		: 8;
	unsigned char Buffer_Overflows	: 8;
	unsigned char Write_Timeouts	: 8;
	unsigned char can_error			: 1;
	unsigned char flags				: 7;

}mirrorstat_struct;

typedef struct OPERATIONS
{
  unsigned long State;
  union Flag
  {
	  Uint32					all;
	  mirrorstat_struct 		fields;
  } can2toA, canAto2;

  union CHANGE
  {
  	long 			all;
  	change_struct 	bit;
  }					Change;
} ops_struct;


#endif /* OP_H_ */

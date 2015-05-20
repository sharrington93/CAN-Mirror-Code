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

//todo User: stopwatch errors
typedef struct FLAGS
{
	unsigned char Read_Overflow	 : 8;
	unsigned char Write_Timeout	 : 7;
	char Status : 1;
}flags_struct;

typedef struct COUNTERS
{
	unsigned char Buf2toA	 : 8;
	unsigned char BufAto2	 : 8;
}count_struct;

typedef struct OPERATIONS
{
  unsigned long State;
  union Flag
  {
	  long						all;
	  flags_struct 				fields;
  } canA, can2;

  union count
  {
	  long						all;
	  count_struct				fields;
  } Counters;
  union CHANGE
  {
  	long 			all;
  	change_struct 	bit;
  }					Change;
} ops_struct;


#endif /* OP_H_ */

/*
 * common.h
 *
 *  Created on: Nov 6, 2013
 *      Author: Nathan
 */

#ifndef COMMON_H_
#define COMMON_H_


void* myMalloc(int size);
void myFree(void*);

//CAN Mirror defines
#define CANQUEUEDEPTH 25

typedef struct
{
	unsigned int buf[CANQUEUEDEPTH][13];
	int in;
	int out;
	int full;
	int empty;
	int count;
}buffer_struct;

#endif /* COMMON_H_ */

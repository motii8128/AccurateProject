/*
 * string_utils.h
 *
 *  Created on: Feb 17, 2025
 *      Author: motii
 */

#ifndef INC_STRING_UTILS_H_
#define INC_STRING_UTILS_H_

#include <stdio.h>
#include <stdlib.h>

typedef struct Recv
{
	int arm_1;
	int arm_2;
	int arm_3;
	int pwm_1;
	int pwm_2;
	int pwm_3;
}Recv;

Recv parseSerialBuffer(uint8_t *buffer)
{
	Recv result;
	char *token = strtok((char *)buffer, ",");
	result.arm_1 = atoi(token) - 500;

	token = strtok(NULL, ",");
	result.arm_2 = atoi(token) - 500;

	token = strtok(NULL, ",");
	result.arm_3 = atoi(token) - 500;

	token = strtok(NULL, ",");
	result.pwm_1 = atoi(token)- 300;

	token = strtok(NULL, ",");
	result.pwm_2 = atoi(token) - 300;

	token = strtok(NULL, ",");
	result.pwm_3 = atoi(token) - 300;

	return result;
}





#endif /* INC_STRING_UTILS_H_ */

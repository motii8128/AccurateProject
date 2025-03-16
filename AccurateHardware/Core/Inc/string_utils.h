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

Recv parseSerialBuffer(uint8_t *buffer, Recv prev)
{
	Recv result;
	char *token = strtok((char *)buffer, ",");
	int convert = atoi(token);
	if(convert == 0)
	{
		result.arm_1 = 0;
		result.arm_2 = 0;
		result.arm_3 = 0;
		result.pwm_1 = 0;
		result.pwm_1 = 0;
		result.pwm_1 = 0;

		return result;
	}
	result.arm_1 = convert - 50;
	result.arm_1 = result.arm_1 * 10;
	if(result.arm_1 > 500)
	{
		result.arm_1 = prev.arm_1;
	}
	else if(result.arm_1 < -500)
	{
		result.arm_1 = prev.arm_1;
	}

	token = strtok(NULL, ",");
	convert = atoi(token);
	if(convert == 0)
	{
		result.arm_1 = 0;
		result.arm_2 = 0;
		result.arm_3 = 0;
		result.pwm_1 = 0;
		result.pwm_1 = 0;
		result.pwm_1 = 0;

		return result;
	}
	result.arm_2 = convert - 50;
	result.arm_2 = result.arm_2 * 10;

	if(result.arm_2 > 500)
	{
		result.arm_2 = prev.arm_2;
	}
	else if(result.arm_2 < -500)
	{
		result.arm_2 = prev.arm_2;
	}

	token = strtok(NULL, ",");
	convert = atoi(token);
	if(convert == 0)
	{
		result.arm_1 = 0;
		result.arm_2 = 0;
		result.arm_3 = 0;
		result.pwm_1 = 0;
		result.pwm_1 = 0;
		result.pwm_1 = 0;

		return result;
	}
	result.arm_3 = convert - 50;
	result.arm_3 = result.arm_3 * 10;
	if(result.arm_3 > 500)
	{
		result.arm_3 = prev.arm_3;
	}
	else if(result.arm_3 < -500)
	{
		result.arm_3 = prev.arm_3;
	}
	if(result.arm_3 > 0)
	{
		result.arm_3 = result.arm_3 / 4.0;
	}


	token = strtok(NULL, ",");
	convert = atoi(token);
	if(convert == 0)
	{
		result.arm_1 = 0;
		result.arm_2 = 0;
		result.arm_3 = 0;
		result.pwm_1 = 0;
		result.pwm_1 = 0;
		result.pwm_1 = 0;

		return result;
	}
	result.pwm_1 = convert- 30;
	result.pwm_1 = result.pwm_1 * -100;
	if(result.pwm_1 > 1000)
	{
		result.pwm_1 = 0;
	}
	else if(result.pwm_1 < -1000)
	{
		result.pwm_1 = 0;
	}

	token = strtok(NULL, ",");
	convert = atoi(token);
	if(convert == 0)
	{
		result.arm_1 = 0;
		result.arm_2 = 0;
		result.arm_3 = 0;
		result.pwm_1 = 0;
		result.pwm_1 = 0;
		result.pwm_1 = 0;

		return result;
	}
	result.pwm_2 = convert - 30;
	result.pwm_2 = result.pwm_2 * 100;
	if(result.pwm_2 > 1000)
	{
		result.pwm_2 = 0;
	}
	else if(result.pwm_2 < -1000)
	{
		result.pwm_2 = 0;
	}

	token = strtok(NULL, ",");
	convert = atoi(token);
	if(convert == 0)
	{
		result.arm_1 = 0;
		result.arm_2 = 0;
		result.arm_3 = 0;
		result.pwm_1 = 0;
		result.pwm_1 = 0;
		result.pwm_1 = 0;

		return result;
	}
	result.pwm_3 = convert - 30;
	result.pwm_3 = result.pwm_3 * -100;
	if(result.pwm_3 > 1000)
	{
		result.pwm_3 = 0;
	}
	else if(result.pwm_3 < -1000)
	{
		result.pwm_3 = 0;
	}

	return result;
}





#endif /* INC_STRING_UTILS_H_ */

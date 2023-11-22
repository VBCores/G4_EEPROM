/*
 * joint_config.h
 *
 *  Created on: Nov 22, 2023
 *      Author: VR
 */

#ifndef INC_JOINT_CONFIG_H_
#define INC_JOINT_CONFIG_H_

struct joint_config
{
	float number;
	float motor_type;
	float upper_limit;
	float lower_limit;
	float zero;
};

typedef struct joint_config joint_config;

#endif /* INC_JOINT_CONFIG_H_ */

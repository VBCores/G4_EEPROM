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
	int32_t number;
	int32_t motor_type;
	int32_t upper_limit;
	int32_t lower_limit;
	int32_t zero;
};

typedef struct joint_config joint_config;

#endif /* INC_JOINT_CONFIG_H_ */

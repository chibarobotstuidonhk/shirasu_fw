/*
 * conf.hpp
 *
 *  Created on: Oct 25, 2020
 *      Author: ryuni
 */

#ifndef INC_CONF_HPP_
#define INC_CONF_HPP_

struct ConfStruct
{
        uint16_t can_id;
        uint8_t default_mode;
        double cpr;
        double Kp;
        double Ki;
        double Kv;
        double HomVel;
};

extern ConfStruct confStruct;

void readConf(void);
void writeConf(void);

inline ConfStruct * getConf(void)
{
    return &confStruct;
}

#endif /* INC_CONF_HPP_ */

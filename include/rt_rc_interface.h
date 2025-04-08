/*
 * @Author: linzhuyue && lzyue@mae.cuhk.edu.hk
 * @Date: 2024-03-04 09:52:53
 * @LastEditors: linzhuyue 
 * @LastEditTime: 2024-03-04 10:02:26
 * @FilePath: /little_human_sdk/include/rt_rc_interface.h
 * @Description: Writed In ERB 106 CUHK. Non-Commercial Used.
 * 
 * Copyright (c) 2024 by LinzhuYue, All Rights Reserved. 
 */

/**
 * @file rt_rc_interface.h
 *
 */
#ifndef _RT_RC_INTERFACE
#define _RT_RC_INTERFACE

class rc_control_settings {
  public:
    double     mode;
    double     p_des[2]; // (x, y) -1 ~ 1
    double     height_variation; // -1 ~ 1
    double     v_des[3]; // -1 ~ 1 * (scale 0.5 ~ 1.5)
    double     rpy_des[3]; // -1 ~ 1
    double     omega_des[3]; // -1 ~ 1
    double     variable[3];
    double     step_height;
};


namespace RC_mode{
  constexpr int OFF = 0;
  constexpr int STAND_UP =1;
  constexpr int LOCOMOTION=2;
  constexpr int WBC_STAND = 3; //this is balanced stand
  constexpr int DAMP=4;
  constexpr int LOWLEVELSDK=5;
  constexpr int TESTMODE=100;
};



void get_rc_control_settings(void* settings);
void* v_memcpy(void* dest, volatile void* src, size_t n);
// use for xbox
void js_complete(int port);
int init_xbox_js();
#endif

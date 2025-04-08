#include <pthread.h>
#include "rt_rc_interface.h"

#include <string.h> // memcpy
#include <stdio.h>
#include <unistd.h>  
#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h>  
#include <errno.h>  
#include <linux/joystick.h>  
#include <iostream>
#include "gamepad_lcmt.hpp"
#include <lcm/lcm-cpp.hpp>

static pthread_mutex_t lcm_get_set_mutex =
PTHREAD_MUTEX_INITIALIZER; /**< mutex to protect gui settings coming over
                             LCM */

// Controller Settings
rc_control_settings rc_control;

/* ------------------------- HANDLERS ------------------------- */

// Controller Settings
void get_rc_control_settings(void *settings) {
  pthread_mutex_lock(&lcm_get_set_mutex);
  v_memcpy(settings, &rc_control, sizeof(rc_control_settings));
  pthread_mutex_unlock(&lcm_get_set_mutex);
}

#define XBOX_TYPE_BUTTON    0x01  
#define XBOX_TYPE_AXIS      0x02  

#define XBOX_BUTTON_A       0x00  
#define XBOX_BUTTON_B       0x01  
#define XBOX_BUTTON_X       0x02  
#define XBOX_BUTTON_Y       0x03  
#define XBOX_BUTTON_LB      0x04  
#define XBOX_BUTTON_RB      0x05  
// MOCUTE
// #define XBOX_BUTTON_START   0x06  
// #define XBOX_BUTTON_BACK    0x07  
#define XBOX_BUTTON_START   0x07  
#define XBOX_BUTTON_SELECT  0x06

// #define XBOX_BUTTON_HOME    0x08

#define XBOX_BUTTON_LO      0x08    // 左侧控制下压
#define XBOX_BUTTON_RO      0x09    //右侧下压

#define XBOX_BUTTON_ON      0x01  
#define XBOX_BUTTON_OFF     0x00  
//      /\ y
// x    |
// <-----
// 
#define XBOX_AXIS_LX        0x00    /* 左摇杆X轴 */  
#define XBOX_AXIS_LY        0x01    /* 左摇杆Y轴 */  
#define XBOX_AXIS_RX        0x03    /* 右摇杆X轴 */  
#define XBOX_AXIS_RY        0x04    /* 右摇杆Y轴 */  
#define XBOX_AXIS_LT        0x02  
#define XBOX_AXIS_RT        0x05  
#define XBOX_AXIS_XX        0x06    /* 方向键X轴 */  
#define XBOX_AXIS_YY        0x07    /* 方向键Y轴 */  

#define XBOX_AXIS_VAL_UP        -32767  
#define XBOX_AXIS_VAL_DOWN      32767  
#define XBOX_AXIS_VAL_LEFT      -32767  
#define XBOX_AXIS_VAL_RIGHT     32767  

#define XBOX_AXIS_VAL_MIN       -32767  
#define XBOX_AXIS_VAL_MAX       32767  
#define XBOX_AXIS_VAL_MID       0x00  

typedef struct xbox_map  
{  
    int     time;  
    int     a;  
    int     b;  
    int     x;  
    int     y;  
    int     lb;  
    int     rb;  
    int     start;  
    int     back;  
    int     select;
    int     home;  
    int     lo;  
    int     ro;  

    int     lx;  
    int     ly;  
    int     rx;  
    int     ry;  
    int     lt;  
    int     rt;  
    int     xx;  
    int     yy;  

}xbox_map_t;  


int xbox_open(const char *file_name)  
{  
    int xbox_fd;  

    xbox_fd = open(file_name, O_RDONLY);  
    if (xbox_fd < 0)  
    {  
        perror("open");  
        return -1;  
    }  

    return xbox_fd;  
}  

int xbox_map_read(int xbox_fd, xbox_map_t *map)  
{  
    int len, type, number, value;  
    struct js_event js;  

    len = read(xbox_fd, &js, sizeof(struct js_event));  
    if (len < 0)  
    {  
        perror("read");  
        return -1;  
    }  

    type = js.type;  
    number = js.number;  
    value = js.value;  

    map->time = js.time;  

    if (type == JS_EVENT_BUTTON)  
    {  
        switch (number)  
        {  
            case XBOX_BUTTON_A:  
                map->a = value;  
                break;  

            case XBOX_BUTTON_B:  
                map->b = value;  
                break;  

            case XBOX_BUTTON_X:  
                map->x = value;  
                break;  

            case XBOX_BUTTON_Y:  
                map->y = value;  
                break;  

            case XBOX_BUTTON_LB:  
                map->lb = value;  
                break;  

            case XBOX_BUTTON_RB:  
                map->rb = value;  
                break;  

            case XBOX_BUTTON_START:  
                map->start = value;  
                break;  

            case XBOX_BUTTON_SELECT:  
                map->select = value;  
                break;  

            // case XBOX_BUTTON_HOME:  //no
            //     map->home = value;  
            //     break;  

            case XBOX_BUTTON_LO:  
                map->lo = value;  
                break;  

            case XBOX_BUTTON_RO:  
                map->ro = value;  
                break;  

            default:  
                break;  
        }  
    }  
    else if (type == JS_EVENT_AXIS)  
    {  
        switch(number)  
        {  
            case XBOX_AXIS_LX:  
                map->lx = value;  
                break;  

            case XBOX_AXIS_LY:  
                map->ly = value;  
                break;  

            case XBOX_AXIS_RX:  
                map->rx = value;  
                break;  

            case XBOX_AXIS_RY:  
                map->ry = value;  
                break;  

            case XBOX_AXIS_LT:  
                map->lt = value;  
                break;  

            case XBOX_AXIS_RT:  
                map->rt = value;  
                break;  

            case XBOX_AXIS_XX:  //方向键
                map->xx = value;  
                break;  

            case XBOX_AXIS_YY:  
                map->yy = value;  
                break;  

            default:  
                break;  
        }  
    }  
    else  
    {  
        /* Init do nothing */  
    }  

    return len;  
}  

void xbox_close(int xbox_fd)  
{  
    close(xbox_fd);  
    return;  
}  

xbox_map_t map;  
int js_gait=9;
lcm::LCM gamepad2controller("udpm://239.255.76.67:7667?ttl=255");
gamepad_lcmt gamepad_data{};
void js_complete(int port){
    try
    {
        int len = xbox_map_read(port, &map);  
        if (len < 0) return;
        // printf("               rc_control.mode =%f \n",rc_control.mode);
        if (map.rt>30000 && map.lt>30000)
            rc_control.mode = RC_mode::LOWLEVELSDK;
        if (map.rt>30000 && map.b)
            rc_control.mode = RC_mode::OFF;
        if (map.lt>30000 && map.rb)
            rc_control.mode = RC_mode::TESTMODE;
        if (map.rt>30000 && map.rb)
            rc_control.mode = RC_mode::DAMP;

        
        gamepad_data.a = map.a;
        gamepad_data.b = map.b;
        gamepad_data.x = map.x;
        gamepad_data.y = map.y;
        gamepad_data.back     =map.select;                        
        gamepad_data.start    =map.start;      
        gamepad_data.leftBumper         = map.lb; 
        gamepad_data.rightBumper        = map.rb;   
        if (map.yy>30000)gamepad_data.leftStickButton = 1;
        if (map.yy<-30000)gamepad_data.rightStickButton=1;
        if (map.xx>30000)gamepad_data.leftTriggerButton = 1;
        if (map.xx<-30000)gamepad_data.rightTriggerButton=1;
        // float
        gamepad_data.leftTriggerAnalog   = (float)map.lt/32768;
        gamepad_data.rightTriggerAnalog  = (float)map.rt/32768; 
        gamepad_data.leftStickAnalog[0]  = (float)map.lx/32768;//lx
        gamepad_data.leftStickAnalog[1]  = -(float)map.ly/32768; //ly
        gamepad_data.rightStickAnalog[0] = (float)map.rx/32768;//
        gamepad_data.rightStickAnalog[1] = -(float)map.ry/32768;  
        gamepad2controller.publish("gamepad2controller",&gamepad_data);

    }
    catch(const std::exception& e)
    {
        std::cerr<<"Something error! in Xbox controller " << e.what() << '\n';
    }
    
}

int init_xbox_js(){
    int fd = xbox_open("/dev/input/js0");
      
    if (fd>0) {
        memset(&map, 0, sizeof(xbox_map_t));
        printf("xbox joystik open successfull!\n\r");
    }else{
        printf("xbox joystik open failed!\n\r");
    } 
    rc_control.step_height=0.8;
    rc_control.height_variation = 0;
    js_gait=9;
    return fd; 

}

void *v_memcpy(void *dest, volatile void *src, size_t n) {
  void *src_2 = (void *)src;
  return memcpy(dest, src_2, n);
}


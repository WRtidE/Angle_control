#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "Can_user.h"
#include "Friction_task.h"
#include "Exchange_task.h"
#include "struct_typedef.h"
#include <stdlib.h> 
#include <math.h>
//发射相关
fp32 target_angle1;
uint8_t r_model= 0; //判断弹仓盖模式 0静止 1打开 2关闭

//拨盘相关
uint8_t friction_flag = 0; 
uint16_t press_time;//判断按键按下时长
int16_t trigger_target_angle;
uint16_t pos = 8191/8;
int8_t shoot_continue;
uint8_t err1  = 0;
uint8_t turns = 0; //转动圈数
int16_t trigger_angle = 0; //拨爪实际角度
int16_t angle_dif = 0;

fp32 count = 0;
fp32 heat_warning = 0;
fp32 shoot_speed_set;
fp32 speed_choice;
int16_t test9;

uint8_t flg = 1;
uint16_t target_angle2;
pid_struct_t trigger_pid[7];//拨盘pid

//读取鼠标状态
uint8_t pre_mouse_l=0;
uint8_t mouse_state_check=0; //如果为0则状态不变，为1则状态改变
//读取鼠标状态
int8_t pre_key_l=0;
int8_t key_state_check; //如果为0则状态不变，为1则状态改变

//PID初始化
static void Friction_init();

//模式选择
static void model_choice();


//弹仓
static void magazine_task();
static void magazine_init();

//枪口热量限制
static void heat_limit();

//can发送
static void shoot_data_send();

//红外激光使能
static void laser_init();

static void friction_enable();
static void friction_disable();

static void friction_flag_change();

//拨盘
static void trigger_init();
static void trigger_task();
static void motor_2006_calc(uint16_t i);

//根据裁判系统改变弹速
static void shoot_speed_choice();

static void trigger_mode_choose();

void Friction_task(void const * argument)
{
  Friction_init();
  magazine_init();
	trigger_init();
  //laser_init();
		
  for(;;)
  {	  
	 motor_2006_calc(2);
	 shoot_speed_choice();
	 model_choice();
	//magazine_task();
	//heat_limit();       
    shoot_data_send();	  
  }
  osDelay(1);
}

static void Friction_init()
{
  pid_init(&motor_pid[0], 30,   0, 0, 16384, 16384); 
	pid_init(&motor_pid[1], 30,   0, 0, 16384, 16384);
	pid_init(&motor_pid[2], 20 ,  0, 0, 16384, 16384);
	pid_init(&trigger_pid[2], 2,  0, 1, 16384, 16384);		
}

static void laser_init()
{
	HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_SET);
}
static void trigger_init()
{
		pid_init(&trigger_pid[2], 2,  0, 1, 16384, 16384);	
	
		motor_info[2].last_angle = motor_info[2].rotor_angle;
		
}
//===========================================枪口弹速选择===========================================
static void shoot_speed_choice()
{
	
	if(infantry.speed_limit == 15 ) //爆发优先
	{
		shoot_speed_set = 3950;
	}
	else if(infantry.speed_limit == 18 ) //冷却优先
	{
		shoot_speed_set = 4625;  
	}
	else if(infantry.speed_limit == 30) //弹速优先
	{
		shoot_speed_set = 10000;
	}
	else
	{
		shoot_speed_set = 8700;
	}
		
}

//===========================================控制摩擦轮============================================================
static void model_choice()
{
	friction_flag_change(); //读取按键是否按下
	//if(friction_flag||rc_ctrl.rc.s[0] == 1)
	if(rc_ctrl.rc.s[0] == 1)
	{
		friction_enable();
	}
	else //其余情况电机转速置0
	{
		friction_disable();
	}
  trigger_mode_choose();
	trigger_task();
}

static void friction_enable()
{

		target_speed[0] = -7900;//-8700
    target_speed[1] =  7900;// 8700
	
		motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
    motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
	
}

static void friction_disable()
{
	
	  target_speed[0] = 0;
    target_speed[1] = 0;
	  
		
	  motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
    motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
	
	
}

static void friction_flag_change()
{
	if(q_flag)
	{
		friction_flag = friction_flag + 1;
	}
	else if(e_flag)
	{
		friction_flag = 0;
	}
}
//=============================================拨盘电机==========================================================
static void trigger_mode_choose()
{
	key_state_check = pre_key_l -  rc_ctrl.rc.s[1];
	pre_key_l = rc_ctrl.rc.s[1];
	
	if(press_time > 0 && press_time < 1000) //短按
	{
				if(key_state_check == -2)//鼠标松开
				{
							press_time = 0;
							trigger_target_angle = trigger_angle - pos;
              if(trigger_target_angle<0)
               { 
                   trigger_target_angle = trigger_target_angle+8191;
               }	
              if(trigger_target_angle>8192)	
              {
                   err1++;
              } 
						shoot_continue = 0;
				}			
	}
	if(press_time > 1000)//长按
	{
				shoot_continue = 1;
	}
	
	
	if(rc_ctrl.mouse.press_l || rc_ctrl.rc.s[1] == 1)
	{
		press_time += 1;
			
	}
	else if(rc_ctrl.rc.s[1]!=1)
	{ 
		if(press_time > 1000  || abs(trigger_target_angle-trigger_angle) <100) //为了防止一直转停不下来
		 {
       press_time = 0;
       shoot_continue = 2;
     }
	
	}
}

static void trigger_task()
{
	if(shoot_continue==0)      //单发
	{
		target_speed[2] =  pid_trigger_calc(&trigger_pid[2],trigger_target_angle,trigger_angle);
	}
	if(shoot_continue==1)      //连射
	{
		target_speed[2] = -1500;
	}
	else if(shoot_continue==2) //停转
	{
		target_speed[2] = 0;
	}
	motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
}

//2006电机处理函数，将减速比消除
static void motor_2006_calc(uint16_t i)
{
	angle_dif = motor_info[i].rotor_angle - motor_info[i].last_angle;  //2006转过的角度差
	motor_info[i].last_angle = motor_info[i].rotor_angle;       
	
	if(angle_dif>8191/2)
	{
		angle_dif -= 8191;
	}
	else if(angle_dif<- 8191/2)
	{
		angle_dif +=8191;
	}
	
	angle_dif/=36;               //减速比  把2006的角度差转换为拨爪的角度差
	
	trigger_angle += angle_dif;  //角度差累加,得到拨爪的绝对角度
	
	if(trigger_angle>8191)
	{
		trigger_angle -= 8191;
	}
	else if(trigger_angle<0)
	{
		trigger_angle +=8191;
	}
																
}
//=============================================弹仓盖==========================================================
static void magazine_task()
	
{

	if(r_flag)
	{
		target_speed[3] = -800;
	}
	else if(f_flag)//关闭弹仓盖
	{
		target_speed[3] =  800;
	}
	else //弹仓盖静止
	{
		target_speed[3] = 0;
	}	
	motor_info[3].set_voltage = pid_calc(&motor_pid[3], target_speed[3], motor_info[3].rotor_speed);
}

static void magazine_init()
{
	pid_init(&motor_pid[3], 10 , 0.1, 0, 10000, 10000);
}

static void shoot_data_send()
{
	
    //set_motor_voltage(0, 0, 0,motor_info[2].set_voltage,0);
	  set_motor_voltage(0, motor_info[0].set_voltage, motor_info[1].set_voltage,motor_info[2].set_voltage,motor_info[3].set_voltage);

    osDelay(1);		
}


//=============================================热量上限=============================================
static void heat_limit()
{
  if(infantry.heat_limit )
  {
	  if(infantry.shooter_heat > infantry.heat_limit )
	  {
		 	motor_info[2].set_voltage   = motor_info[2].set_voltage * 0;
		     
	  }
	  if(infantry.shooter_heat > infantry.heat_limit * 0.9)
	  {
			
		  	motor_info[2].set_voltage = motor_info[2].set_voltage * 0.5;
	  }
	  else
	  {
		//do nothing
	  }	  
  }
  else
  {
	  //do nothing
  }
}





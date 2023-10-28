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
//�������
fp32 target_angle1;
uint8_t r_model= 0; //�жϵ��ָ�ģʽ 0��ֹ 1�� 2�ر�

//�������
uint8_t friction_flag = 0; 
uint16_t press_time;//�жϰ�������ʱ��
int16_t trigger_target_angle;
uint16_t pos = 8191/8;
int8_t shoot_continue;
uint8_t err1  = 0;
uint8_t turns = 0; //ת��Ȧ��
int16_t trigger_angle = 0; //��צʵ�ʽǶ�
int16_t angle_dif = 0;

fp32 count = 0;
fp32 heat_warning = 0;
fp32 shoot_speed_set;
fp32 speed_choice;
int16_t test9;

uint8_t flg = 1;
uint16_t target_angle2;
pid_struct_t trigger_pid[7];//����pid

//��ȡ���״̬
uint8_t pre_mouse_l=0;
uint8_t mouse_state_check=0; //���Ϊ0��״̬���䣬Ϊ1��״̬�ı�
//��ȡ���״̬
int8_t pre_key_l=0;
int8_t key_state_check; //���Ϊ0��״̬���䣬Ϊ1��״̬�ı�

//PID��ʼ��
static void Friction_init();

//ģʽѡ��
static void model_choice();


//����
static void magazine_task();
static void magazine_init();

//ǹ����������
static void heat_limit();

//can����
static void shoot_data_send();

//���⼤��ʹ��
static void laser_init();

static void friction_enable();
static void friction_disable();

static void friction_flag_change();

//����
static void trigger_init();
static void trigger_task();
static void motor_2006_calc(uint16_t i);

//���ݲ���ϵͳ�ı䵯��
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
//===========================================ǹ�ڵ���ѡ��===========================================
static void shoot_speed_choice()
{
	
	if(infantry.speed_limit == 15 ) //��������
	{
		shoot_speed_set = 3950;
	}
	else if(infantry.speed_limit == 18 ) //��ȴ����
	{
		shoot_speed_set = 4625;  
	}
	else if(infantry.speed_limit == 30) //��������
	{
		shoot_speed_set = 10000;
	}
	else
	{
		shoot_speed_set = 8700;
	}
		
}

//===========================================����Ħ����============================================================
static void model_choice()
{
	friction_flag_change(); //��ȡ�����Ƿ���
	//if(friction_flag||rc_ctrl.rc.s[0] == 1)
	if(rc_ctrl.rc.s[0] == 1)
	{
		friction_enable();
	}
	else //����������ת����0
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
//=============================================���̵��==========================================================
static void trigger_mode_choose()
{
	key_state_check = pre_key_l -  rc_ctrl.rc.s[1];
	pre_key_l = rc_ctrl.rc.s[1];
	
	if(press_time > 0 && press_time < 1000) //�̰�
	{
				if(key_state_check == -2)//����ɿ�
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
	if(press_time > 1000)//����
	{
				shoot_continue = 1;
	}
	
	
	if(rc_ctrl.mouse.press_l || rc_ctrl.rc.s[1] == 1)
	{
		press_time += 1;
			
	}
	else if(rc_ctrl.rc.s[1]!=1)
	{ 
		if(press_time > 1000  || abs(trigger_target_angle-trigger_angle) <100) //Ϊ�˷�ֹһֱתͣ������
		 {
       press_time = 0;
       shoot_continue = 2;
     }
	
	}
}

static void trigger_task()
{
	if(shoot_continue==0)      //����
	{
		target_speed[2] =  pid_trigger_calc(&trigger_pid[2],trigger_target_angle,trigger_angle);
	}
	if(shoot_continue==1)      //����
	{
		target_speed[2] = -1500;
	}
	else if(shoot_continue==2) //ͣת
	{
		target_speed[2] = 0;
	}
	motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
}

//2006����������������ٱ�����
static void motor_2006_calc(uint16_t i)
{
	angle_dif = motor_info[i].rotor_angle - motor_info[i].last_angle;  //2006ת���ĽǶȲ�
	motor_info[i].last_angle = motor_info[i].rotor_angle;       
	
	if(angle_dif>8191/2)
	{
		angle_dif -= 8191;
	}
	else if(angle_dif<- 8191/2)
	{
		angle_dif +=8191;
	}
	
	angle_dif/=36;               //���ٱ�  ��2006�ĽǶȲ�ת��Ϊ��צ�ĽǶȲ�
	
	trigger_angle += angle_dif;  //�ǶȲ��ۼ�,�õ���צ�ľ��ԽǶ�
	
	if(trigger_angle>8191)
	{
		trigger_angle -= 8191;
	}
	else if(trigger_angle<0)
	{
		trigger_angle +=8191;
	}
																
}
//=============================================���ָ�==========================================================
static void magazine_task()
	
{

	if(r_flag)
	{
		target_speed[3] = -800;
	}
	else if(f_flag)//�رյ��ָ�
	{
		target_speed[3] =  800;
	}
	else //���ָǾ�ֹ
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


//=============================================��������=============================================
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





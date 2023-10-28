# 步兵单发实现流程

判断按键短按或者长按，选择连射或者单发

```C
/*
	key_state_check 鼠标/按键状态检测
	pre_key_l       之前的鼠标状态
	rc_ctrl.rc.s[1] 当前的鼠标状态
	press_time      按键计时
	shoot_continue  发射机构状态判断 0单发模式 1连发模式 2停止射击
*/
static void trigger_init()
{
		pid_init(&trigger_pid[2], 2,  0, 1, 16384, 16384);	
	
		motor_info[2].last_angle = motor_info[2].rotor_angle; //初始化，令当前角度=之前角度
		
}

static void trigger_mode_choose()
{
    //记录鼠标、遥控器状态是否发生变化（即是否由按下变为松开）
	key_state_check = pre_key_l -  rc_ctrl.rc.s[1];  
  	pre_key_l = rc_ctrl.rc.s[1];
	
    //判断按压时长
	if(press_time > 0 && press_time < 1000) //短按
	{
		if(key_state_check == -2)//鼠标松开
		{
			  //计数清0
              press_time = 0;
            
			  //计算目标角度
              trigger_target_angle = trigger_angle - pos;
              
              //越界处理
           	  if(trigger_target_angle<0)
               { 
                   trigger_target_angle = trigger_target_angle+8191;
               }	
              if(trigger_target_angle>8192)	
              {
                   err1++;
              } 
             //设置发射状态为单发
			 shoot_continue = 0;
		}			
	}
	if(press_time > 1000)//长按
	{		
        	//设置发射状态为连发
			shoot_continue = 1;
	}
    
    
    //当按下鼠标，press_time开始记录按压时长
    if(rc_ctrl.mouse.press_l || rc_ctrl.rc.s[1] == 1)
	{
		press_time += 1;
			
	}
	else if(rc_ctrl.rc.s[1]!=1)//松开鼠标
	{ 
		if(press_time > 1000  || abs(trigger_target_angle-trigger_angle) <100) //为了防止一直转停不下来
		 {
             //计数清0
      		 press_time = 0;
             //停转
      		 shoot_continue = 2;
         }
    }
}
	
```
# 减速比角度换算
```C
tatic void trigger_init()
{
		pid_init(&trigger_pid[2], 2,  0, 1, 16384, 16384);	
	
		motor_info[2].last_angle = motor_info[2].rotor_angle; //初始化，令之前角度==当前角度
		
}

//2006电机处理函数，将减速比消除
static void motor_2006_calc(uint16_t i)
{
	angle_dif = motor_info[i].rotor_angle - motor_info[i].last_angle;  //2006转过的角度差
	motor_info[i].last_angle = motor_info[i].rotor_angle;              //记录上次的角度
	
	//越界处理
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
	
        //越界处理
	if(trigger_angle>8191)
	{
		trigger_angle -= 8191;
	}
	else if(trigger_angle<0)
	{
		trigger_angle +=8191;
	}
																
}
```


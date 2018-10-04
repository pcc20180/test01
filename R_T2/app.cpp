#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "Pid.h"
#include "Kyori.h"
#include <stdio.h>

using namespace ev3api;

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/* Bluetooth */
static int32_t   bt_cmd = 0;      /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt = NULL;      /* Bluetoothファイルハンドル */
static FILE     *fp_log = NULL;

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
#define GYRO_OFFSET           4  /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */	//3
#define DANSA_GYRO_OFFSET     2
#define LIGHT_WHITE          33  /* 白色の光センサ値 */
#define LIGHT_BLACK           0  /* 黒色の光センサ値 */
#define SONAR_ALERT_DISTANCE 30  /* 超音波センサによる障害物検知距離[cm] */
#define TAIL_ANGLE_STAND_UP  88  /* 完全停止時の角度[度] */	//88->86
#define TAIL_ANGLE_DRIVE      0  /* バランス走行時の角度[度] */
#define TAIL_ANGLE_DASH      95  /* Voltage MAX時 95~97*/
#define P_GAIN             2.5F  /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60  /* 完全停止用モータ制御PWM絶対最大値 */

//#define DEVICE_NAME     "ET0"  /* Bluetooth名 hrp2/target/ev3.h BLUETOOTH_LOCAL_NAMEで設定 */
//#define PASS_KEY        "1234" /* パスキー    hrp2/target/ev3.h BLUETOOTH_PIN_CODEで設定 */
#define CMD_START         '1'    /* リモートスタートコマンド */


/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

#define DELTA_T 0.004
#define KP 2.03				//元の数値1.58
#define KI 0.3
#define KD 0.018

#define T_KP 0.8
#define T_KI 0.2
#define T_KD 0.01

/*#define S_KP	0.36
#define S_KI	0.18
#define S_KD	0.027*/

/*spidの各ゲインの値*/                             //直線用な感じ
#define S_KP	0.36
#define S_KI	0.18
#define S_KD	0.027

#define ST_KP	0.8
#define ST_KI	0.2
#define ST_KD	0//0.01

#define R_KP	1.6
#define R_KI	0.25
#define R_KD	0.018

/*customize_pid*/
#define CUS_KP	0.9
#define CUS_KI	0.2
#define CUS_KD	0.04

/*out*/
#define OUT_KP	0.6
#define OUT_KI	0.14
#define OUT_KD	0.03

/*in*/
#define IN_KP	1.15
#define IN_KI	0.15
#define IN_KD	0.05

#define TURNMAX	100
#define DANSA_DETECTION_VALUE (130)	//80->130
#define SECOND1 (250)
#define TURN_90 (375)

/* 関数プロトタイプ宣言 */
static void tail_control(int32_t angle);
static void start_tail_control(int32_t angle);

/* 共通変数 */
static int black,white,target;
static char str[100];
static int count;
static int i,k;
static int m_target;
static char mode=1;
static char dansa=1;
static int balance_run_flag = 1;
static int t_angle=82;
static bool mark_flag = 1;
static int32_t linetrace_edge = 1;
static int32_t old_pwm_L;
static int32_t old_pwm_R;
static bool end_flag = 0;
static bool start_dash_flag = 1;
static int32_t step_down_target = 0;
static int d_target;
static int s_target;
static bool switch_flag = 1;
static int f = 0;
static int a = 0;
static int b = 0;

/* 関数プロトタイプ宣言 */
static void normal_run(void);
static void dansa_kenti(void);

static int32_t motor_ang_l, motor_ang_r;
static int32_t gyro, volt;
static int8_t forward;      /* 前後進命令 */
static int8_t turn;         /* 旋回命令 */
static int8_t pwm_L, pwm_R; /* 左右モータPWM出力 */

#define OFFSET_LINE			 10
#define OFFSET_LINE_STAIR    11
#define OFFSET_LINE_GARAGE   5         //10->2->5
#define KYORI_NUM	8	//9->7->8
#define NANSHO_OFFSET_LINE (10)          //8->4->0->-3->12->10->6

static int32_t i_cnt = 0;

static int16_t my_distance[KYORI_NUM] = 
{
 // 1    2     3     4     5     6     7     8      9      10     11    12
	//800,2580,8760,11320,12270,12470,12770,15000//,15000	//12240
//	800,2580,8760,11450,12250,12550,12800,13200
	300,2100,8180,10950,12150,12400,12650,13500
//	300,2050,8180,10900,12200,12370,12670,13300
//    800, 2950, 5080, 6360, 7030, 8295, 9267, 10977, 11800, 14010, 14400, 16200 //大会で使用
 // 800, 3012, 5080, 6100, 7350, 8295, 9267, 10577, 11760, 14150, 16400          //距離の区切り（付箋にありまする～）     //idx:3 6195->6100, idx:4 7259->7350, idx:10 16550->16400
//3012, 5080, 6100, 7350, 8295, 9267, 10577, 11760, 14150, 16400
//500
};

static int16_t ary_speed[KYORI_NUM] =
{
	70,100,60,100,20,120,10,1//,1	//35->40->30
//	70,120,70,100,20,120,10,1
//	70,90,60,90, 10,100,20,1
};

static int16_t pid_variety[KYORI_NUM] =                     //0:pid, 1:rpid, 2:spid ,3:customize_pid, 4:out_pid , 5: in_pid
{
	2,2,4,2,3,2,2,2//,2
//    2, 2, 5, 5, 2, 4, 2, 5, 3, 2, 1, 1
    //1
};

static int16_t pid_offset[KYORI_NUM] =
{
	0,0,0,0,0,0,0,0//,0
  //1, 2, 3,  4, 5, 6, 7, 8,  9, 10, 11  
//    0, 0, -2, 5, 5, 8, 4, 5, -2, 5, 15, 0                               //オフセット値     //idx:3  11->15->20 , idx:4 9->15->20 , idx:10 5->10->5
    //0
};

/* オブジェクトへのポインタ定義 */
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          tailMotor;
Clock*          clock;
Pid*			pid;
Pid*			spid;
Pid*			rpid;
Pid*			tpid;
Pid*            stpid;
Pid*            customize_pid;
Pid*            out_pid;
Pid*            in_pid;
Pid*            nansyo_pid;
Kyori*			kyori;

/* メインタスク */
void main_task(intptr_t unused)
{
    /* 各オブジェクトを生成・初期化する */
    touchSensor = new TouchSensor(PORT_1);
    colorSensor = new ColorSensor(PORT_3);
    sonarSensor = new SonarSensor(PORT_2);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_A);
    clock       = new Clock();
	pid			= new Pid(KP,KI,KD);
	spid		= new Pid(S_KP,S_KI,S_KD);
	rpid		= new Pid(R_KP,R_KI,R_KD);
	tpid		= new Pid(T_KP,T_KI,T_KD);
    stpid       = new Pid(ST_KP, ST_KI, ST_KD);
    customize_pid   = new Pid(CUS_KP, CUS_KI, CUS_KD);
    out_pid     = new Pid(OUT_KP, OUT_KI, OUT_KD);
    in_pid      = new Pid(IN_KP, IN_KI, IN_KD);
    nansyo_pid  = new Pid(0.2, 0, 0.06);
	kyori		= new Kyori(*leftMotor,*rightMotor);


    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("TEAM-PCC2018", 0, CALIB_FONT_HEIGHT*1);
    
    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);
    fp_log = fopen("ev3_log.csv", "w");

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);

	ev3_lcd_draw_string("Get Black Value !!",0,CALIB_FONT_HEIGHT*2 );
	while(1)
	{
		sprintf(str,"black = %02d",(int)colorSensor->getBrightness());
		ev3_lcd_draw_string(str,0,CALIB_FONT_HEIGHT*3);
		
		if(touchSensor->isPressed())
		{
			while(touchSensor->isPressed());
			black = colorSensor->getBrightness();
			ev3_lcd_draw_string("Black Value Comfirmed !!",0,CALIB_FONT_HEIGHT*3);
			for(i = 0;i < 3 ; i++ )
			{
				ev3_speaker_play_tone(400,50);
			}
			break;
		}
	}

	clock->sleep(100);

	ev3_lcd_draw_string("Get White Value !!",0,CALIB_FONT_HEIGHT*4);
	while(1)
	{
		sprintf(str,"white = %02d",(int)colorSensor->getBrightness());
		ev3_lcd_draw_string(str,0,CALIB_FONT_HEIGHT*5);

		if(touchSensor->isPressed())
		{
			while(touchSensor->isPressed());
			white = colorSensor->getBrightness();
			ev3_lcd_draw_string("White Value Comfirmed !!",0,CALIB_FONT_HEIGHT*5);
			for(i = 0;i < 3 ; i++ )
			{
				ev3_speaker_play_tone(400,50);
			}
			break;
		}
	}



	clock->sleep(100);

	ev3_lcd_draw_string("Calibration has finished !!",0,CALIB_FONT_HEIGHT*6);
	target = (black+white) / 2;    
    if(fp_log == NULL)
    {
        ;
    }
    else
    {
        fprintf(fp_log,"open !!\r\target:%04d\r\n",(int)target);
    }
	while(1)
	{
		if(touchSensor->isPressed())
		{
			while(touchSensor->isPressed());
			break;
		}
	}
        /* 尻尾モーターのリセット */
    tailMotor->reset();

	fprintf(bt,"black\t%03d\twhite\t%03d\r\n",black,white);
	clock->sleep(100);
    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    /* スタート待機 */
    while(1)
    {
        start_tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */
		if(colorSensor->getBrightness() >= target + 2){
			count++;
			if(count>100){
				ev3_speaker_play_tone(600,10);
				count=0;
			}
		}else if(colorSensor->getBrightness() <= target - 2){
			count++;
			if(count > 100){
				ev3_speaker_play_tone(800,10);
				count=0;
			}
        }
        
        sprintf(str,"voltage = %02d",(int)ev3_battery_voltage_mV());
        ev3_lcd_draw_string(str,0,CALIB_FONT_HEIGHT*8);
        
        if (bt_cmd == 1)
        {
            break; /* リモートスタート */
        }
	
        if (touchSensor->isPressed())
        {
            break; /* タッチセンサが押された */
        }
	
        clock->sleep(10);
    }
    
    /* 走行モーターエンコーダーリセット */
    leftMotor->reset();
    rightMotor->reset();
    
    /* ジャイロセンサーリセット */
    gyroSensor->reset();
    
    /* 倒立振子API初期化 */
    balance_init(); 
    
    /* スタート通知 */
    ev3_led_set_color(LED_GREEN);
    
#ifdef SUZUKI_FUNC_TEST
#endif
    /**
    * Main loop for the self-balance control algorithm
    */
    while(1)
    {
        
	
        if (ev3_button_is_pressed(BACK_BUTTON)) break;
		if (bt_cmd == 2) break;
        if(end_flag) break;
	
		switch(mode)
		{
		
			case 1:
                if(start_dash_flag){
                    if(tailMotor->getCount() >= TAIL_ANGLE_DASH){
                        start_dash_flag = 0;
                    }
                }
                normal_run();
				break;
		
			case 2:
                if(mark_flag){
                    mark_flag = 0;
                    fprintf(bt,"/*************mode 2 start !!****************/\r\n");
		
                }
				dansa_kenti();
                if(mark_flag){
                    mark_flag = 0;
                    fprintf(bt,"/*************mode 2 end !!****************/\r\n");
                    mark_flag = 1;
                }
				break;
            default:
		break;
            }
		
		
		if(k++ >= 25)
		{
            fprintf(bt,"switch_flag:%d,i_cnt:%d,a:%d,b:%d\r\nforward:%d,turn:%d,gyro:%d,Brightness:%d\r\npwm_R:%d,pwm_L:%d,m_target:%d,s_target:%d\r\nstep_down:%d,kyori:%d,volt:%d,tailAngle:%d\r\nt_angle:%d,dansa_mode:%d,mode:%d,R:%d,L:%d\r\n\n",
            (int)switch_flag,(int)i_cnt,(int)a,(int)b,(int)forward,(int)turn,(int)gyro,(int)colorSensor->getBrightness(),(int)pwm_R,(int)pwm_L,(int)m_target,(int)s_target,(int)step_down_target,(int)kyori->Count(),(int)volt,(int)tailMotor->getCount(),(int)t_angle,(int)dansa,(int)mode,(int)rightMotor->getCount(),(int)leftMotor->getCount());
            fprintf(fp_log,"forward:%d,turn:%d,gyro:%d,Brightness:%d,kyori:%d,volt:%d,R:%d,L:%d\r\n",(int)forward,(int)turn,(int)gyro,(int)colorSensor->getBrightness(),(int)kyori->Count(), (int)volt,(int)rightMotor->getCount(),(int)leftMotor->getCount());
	    k=0;
		}
	clock->sleep(4);
	}
    
    leftMotor->reset();
    rightMotor->reset();
    tailMotor->reset();
    
    ter_tsk(BT_TASK);
    fclose(bt);
    fclose(fp_log);
    
    ext_tsk();
}

//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************

static void tail_control(int32_t angle)
{
  //  float pwm = (float)(angle - tailMotor->getCount()) * 0.8F; //比例制御
	float pwm = tpid->Caluculation(angle,tailMotor->getCount());

    //PWM出力飽和処理
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    tailMotor->setPWM(pwm);
}

//*****************************************************************************
// 関数名 : start_tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
static void start_tail_control(int32_t angle)
{
    //float pwm = (float)(angle - tailMotor->getCount()) * P_GAIN; /* ????? */
	float pwm = stpid->Caluculation(angle,tailMotor->getCount());


   
    /* PWM?o??O?a???? */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    tailMotor->setPWM(pwm);
}

//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************

void bt_task(intptr_t unused)
{
    while(1)
    {
        uint8_t c = fgetc(bt); // 受信
        switch(c)
        {
        case '1':
            bt_cmd = 1;
            break;
		case '2':
			bt_cmd = 2;
			break;
        default:
            break;
        }
        fputc(c, bt); // エコーバック
    }
}

//*****************************************************************************
// 関数名 : normal_run
// 引数 : なし
// 返り値 : なし
// 概要 : スタートから難所手前までのライントレース
//*****************************************************************************
static void normal_run(void)
{
    /*if (i_cnt >= 7) {
	if(gyro > DANSA_DETECTION_VALUE || gyro < -DANSA_DETECTION_VALUE) {	//段差を検知したら
		b++;
		ev3_speaker_play_tone(1000, 100);
	}
    }*/
	//if (!b) {
		tail_control(TAIL_ANGLE_DRIVE);
		
		forward = ary_speed[i_cnt];
		
		switch (pid_variety[i_cnt]) {
			case 0:
				turn = pid->Caluculation(colorSensor->getBrightness(),target + pid_offset[i_cnt]) * linetrace_edge;
				break;
			case 1:
				turn = rpid->Caluculation(colorSensor->getBrightness(),target + pid_offset[i_cnt]) * linetrace_edge;
				break;
			case 2:
				turn = spid->Caluculation(colorSensor->getBrightness(),target + pid_offset[i_cnt]) * linetrace_edge;
				break;
			case 3:
				turn = customize_pid->Caluculation(colorSensor->getBrightness(),target + pid_offset[i_cnt]);
				break;
			case 4:
				turn = out_pid->Caluculation(colorSensor->getBrightness(),target + pid_offset[i_cnt]);
				break;
			case 5:
				turn = in_pid->Caluculation(colorSensor->getBrightness(),target + pid_offset[i_cnt]);
				break;
			default:
				break;
		}
/*		
		if(i_cnt == 4){
			if(kyori->Count() <= 11500 && kyori->Count() >= 12100){
				pwm_R = 20;
				pwm_L = 0;
			}
		}
*/
		if (kyori->Count() >= my_distance[i_cnt]) {
			i_cnt++;
			ev3_speaker_play_tone(800, 50);
		}else{
			;
		}
		
		if (pwm_L <= -10 || pwm_R <= -10) {
			a++;
		}else{
			a = 0;
    		}
		
		if (a >= 40 && i_cnt) {	//40
			mode = 2;
		}
	/*}else{
		forward = -100;
		turn = 0;
		tail_control(t_angle - 10);
		balance_run_flag = 0;
		pwm_L = 10;
		pwm_R = 10;
	}*/
	old_pwm_L = pwm_L;
	old_pwm_R = pwm_R;
	
	/* 倒立振子制御API に渡すパラメータを取得する */
	motor_ang_l = leftMotor->getCount();
	motor_ang_r = rightMotor->getCount();
	gyro = gyroSensor->getAnglerVelocity();
	volt = ev3_battery_voltage_mV();
	
	/* 倒立振子制御APIを呼び出し、倒立走行するための */
	/* 左右モータ出力値を得る */
	if(balance_run_flag){
	    balance_control(
		    (float)forward,
		    (float)turn,
		    (float)gyro,
		    (float)GYRO_OFFSET,
		    (float)motor_ang_l,
		    (float)motor_ang_r,
		    (float)volt,
		    (int8_t *)&pwm_L,
		    (int8_t *)&pwm_R);
	}
	
	pwm_L = (old_pwm_L + pwm_L) / 2;
	pwm_R = (old_pwm_R + pwm_R) / 2;

    if( i_cnt >= KYORI_NUM )
    {
        mode=2;                                                       //段差上るプログラムに
//        detection_mark_flag = 1;
//        d_target_get_flag = 1;
        d_target = kyori->Count();                                    //値を取得(誤作動を防ぐため)
        d_target += 500;                                               //本来ならいらない。難所のみで作ると＋aとして必要
//	kyori_num = kyori->Count();
	ev3_speaker_play_tone(800, 50);
    }
    
    leftMotor->setPWM(pwm_L);
    rightMotor->setPWM(pwm_R);
}

//*****************************************************************************
// 関数名 : dansa_kenti
// 引数 : なし
// 返り値 : なし
// 概要 :
//*****************************************************************************
/*mode = 2*/
static void dansa_kenti(void)
{
    tail_control(t_angle - 20);	//10->20
    balance_run_flag = 0;
    pwm_L = -10;
    pwm_R = -10;
    
    if (f < 300) {	//100->200->300
	f++;
    }else{
		end_flag = 1;
		pwm_L = 0;
 	   	pwm_R = 0;
    }

    
    /* 倒立振子制御API に渡すパラメータを取得する */
    motor_ang_l = leftMotor->getCount();
    motor_ang_r = rightMotor->getCount();
    gyro = gyroSensor->getAnglerVelocity();
    volt = ev3_battery_voltage_mV();
    
    /* 倒立振子制御APIを呼び出し、倒立走行するための */
    /* 左右モータ出力値を得る */
    if(balance_run_flag){
        balance_control(
            (float)forward,
            (float)turn,
            (float)gyro,
            (float)GYRO_OFFSET,
            (float)motor_ang_l,
            (float)motor_ang_r,
            (float)volt,
            (int8_t *)&pwm_L,
            (int8_t *)&pwm_R);
    }
    
    leftMotor->setPWM(pwm_L);
    rightMotor->setPWM(pwm_R);
}

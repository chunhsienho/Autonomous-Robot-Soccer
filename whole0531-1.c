//include desire header file//

#include <pwm.h>
#include <p30f4011.h>
#include <adc10.h>
#include <timer.h>
#include <uart.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>



////substitue the stdbool.h
typedef int bool;
#define false 0
#define true 1


//set the osillator/
_FOSC(CSW_FSCM_OFF & XT_PLL8)
	_FWDT(WDT_OFF);
_FBORPOR(PBOR_OFF & MCLR_EN);
_FGS(CODE_PROT_OFF);

#define Fcy 20000000
#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))
#define Pi 3.141593
#define iFcy 100  //intp timer

//define any pick
#define LED1 	LATDbits.LATD0
#define LED2 	LATDbits.LATD2
#define PWMAEN	LATEbits.LATE4
#define PWMBEN	LATEbits.LATE5
#define A1      PORTBbits.RB0
#define B1      PORTBbits.RB1
#define A2      PORTBbits.RB4
#define B2      PORTBbits.RB5
#define UART_TX_PIN_NORMAL      0xF7FF  /* UART TX pin operates normally */
#define UART_TX_PIN_LOW         0xFFFF  /* UART TX pin driven low */
#define PWM3 	LATBbits.LATB8



//define every parameter
//***************************
#define SIGN(A)  ( (A) >= 0 ? 1: -1 )
#define ACC			1000  //tire acceleration 1000mm/s^2
#define CEN_ACC		50  //Centripetal Acceleration for turning 100mm/s^2
#define cR			75    //wheelbase 80mm
#define cr			15    //flange width 15mm
#define ball_R		35
#define V_MAX		300  //max speed for motor 300mm/s

#define D_ANG_GAIN  0.5///////////////use for arc  set angle * D_ANG_GAIN = (rotative speed difference /2 )

#define STRAIGHT     1
#define TURNNING	3
#define ROTATE		5

#define PRE_SET 1
#define TURN	2
#define ARC		3	
#define LINE	4
#define WAIT	5
#define CATCH_BALL 6



// control:
//#define Motor1_Igain 20
#define Motor1_Igain 24
#define Motor1_Pgain 0.5
#define Motor2_Igain 24
#define Motor2_Pgain 0.5
//***************************

//Define every parameter
signed int systemflag=0;
signed int systemflag_1=0;
signed int systemflag_2=0;
signed int CNflag=0;


signed int count=0;

signed int count2_new,count2_old,state2_old,state2_new;
signed int ref1=1000,ref2=1000;
signed int pwm_3_open=0, pwm_3_duty=60, pwm_3_period=0;
float ad_read_0, ad_read_1, ad_read_2, ad_read_3;
int renewDataPls =0;

char data;	//input by keyboard
signed int modenum;

//bluetooth transmitting parameter
unsigned char RX_data[14];
int RX_order;
int RX_flag;



int first_input=0;
int data_signed_bit = 1; 

//caculate the motor 1 rotate speed and parameter for PI-loop
int RPM_motor1_signed_bit;
signed int count1_new,count1_old,state1_old,state1_new,c1;
double RPM_motor1, RPM_wheel1, RPM_wheel1_cmd, RPM_wheel1_cmd_tmp, RPM_wheel1_out, error1, ierror1, error_tmp;
int feedforward1;

//Motor 2
int RPM_motor2_signed_bit;
signed int count2_new,count2_old,state2_old,state2_new,c2;
double RPM_motor2, RPM_wheel2, RPM_wheel2_cmd, RPM_wheel2_cmd_tmp, RPM_wheel2_out, error2, ierror2;
int feedforward2;




//set the parameter
void init_TMR1(void);			//Timer1 function
void init_TMR2(void);			//Timer2 function
void _ISR _T1Interrupt(void);		//stop the function
void _ISR _T2Interrupt(void);
void init_ad(void);			//ADC init function
void read_ad(void);			//ADC read function
void init_PWM(void);			//PWM init function
void SetDCMPWM(void);			//PWM driver function
void init_CN(void);			//CN init function
void _ISR _CNInterrupt(void);		//CN stop function
void init_uart(void);
void init_MTR(void); // encoder init function
void _ISR _U2RXInterrupt(void);
void mode(int);
void sitoa(int, unsigned char *TXdata);



typedef struct 
{
	
	
	int		state;			//intp statement
	double	dist;			//the remain distance
	double  dDist;			//unit time per distance
	double	ang;			//remain angle(have negative and positive)
	double	restAng;		//remain rotate angle
	double	dAng;			//unit time per rotation angle
	int		newCommend;		//receive new signal
	int		catchBall;		//signal for catching ball
	double	vNow[3];		//0 for left tire speed , 1 for right tire speed ,2 for tangential speed
	double  vDes[3];		// desire speed


}MOTION_DATA;

MOTION_DATA		mData;
MOTION_DATA*	mDataPtr;




//self-define function
//interpolator~~~
void initMotion(MOTION_DATA* mDataPtr);
void getTar(MOTION_DATA* mDataPtr);
void intp(MOTION_DATA* mDataPtr);
void arcExe(MOTION_DATA* mDataPtr );
void turnExe(MOTION_DATA* mDataPtr);
void lineExe(MOTION_DATA* mDataPtr );
//~~~~~~~~~~~~
void controlSpeed(MOTION_DATA* mDataPtr );   // closed loop control: PI & feedforward
void renewEncoder(void);
void assign_xyEnd(MOTION_DATA* mDataPtr, double d, double ang);  // 找從絕對目標點到相對目標點


void claimBall(void); //夾球                                                                          ///////欠缺**
void renewData(MOTION_DATA* mDataPtr); //renew the bluetooth receive data       ///////欠缺**


//main function
int main (void)
{
	mDataPtr =&mData;
	//	define the I/O
	ADPCFG=0xffff;
	TRISBbits.TRISB0=1;	
	TRISBbits.TRISB1=1;	
	TRISBbits.TRISB4=1;	
	TRISBbits.TRISB5=1;	
	TRISEbits.TRISE4=0;
	TRISEbits.TRISE5=0;
	TRISBbits.TRISB8=0;
	TRISDbits.TRISD0=0;
	TRISDbits.TRISD2=0;

	//init

	init_TMR1();
	init_TMR2();
	//	init_ad();
	init_PWM();
	init_CN();
	init_uart();
	init_MTR();
	initMotion( mDataPtr);


	/////////////////////////////////fake signal/////////////////////////////
/*	mDataPtr->dist = 500;
	mDataPtr->ang = -Pi/3;
	mDataPtr->vNow[2] = 150;
	mDataPtr->vDes[2] = 250;
	getTar(mDataPtr);
	mDataPtr->vDes[0] = 300; // test the bluetooth
	mDataPtr->vDes[1] = 50;*/

	//////////////////////////////////////////////////////////////////

	while(1)
	{
		//Timer1 stop(1ms/per)
		if (systemflag_1==1)
		{
			c1++; 
			if(c1==10) //stop each 10ms
			{ 
				c1=0;
				if(renewDataPls == 1)  //global variable
				{
					renewData(mDataPtr);
					getTar(mDataPtr);
					first_input =1;
					renewDataPls=0;
				}

				intp( mDataPtr);

				controlSpeed( mDataPtr);

				// int t1 = -mDataPtr->vDes[0]*3+1000;
				// int t2 = mDataPtr->vDes[1]*3+1000;
				//ref1 = -mDataPtr->vDes[0]*3+1000;
				//ref2 = mDataPtr->vDes[1]*3+1000;
				//SetDCMCPWM(1,1800,0);
				//SetDCMCPWM(2,200,0);
			if(ref1>2000)
			{
				ref1=2000;
			}
			if(ref1<0)
			{
				ref1=0;
			}
			SetDCMCPWM(1,ref1,0);

			//second motor
			if(ref2>2000)
			{
				ref2=2000;
			}
			if(ref2<0)
			{
				ref2=0;
			}
			SetDCMCPWM(2,ref2,0);
			}

			

			systemflag_1=0;
		}

		//Timer2 stop(each 0.01ms)  ~~ for servo motor
		if (systemflag_2==1)
		{

			systemflag_2=0;		
		}      //~~ for servo motor

		//CN stop
		if(CNflag==1)
		{		
			CNflag=0;
			renewEncoder();

		}
	}
}

//change mode(mode1 motor mode mode2 servor motor mode
void mode(signed int modenum)
{
	if(modenum==1)   //T1,CN stop
	{
		IEC0bits.T1IE=1;
		IEC0bits.T2IE=0;
		IEC0bits.CNIE=1;
		pwm_3_open=0;
	}

	if(modenum==2)   //T2 stop
	{
		IEC0bits.T1IE=0;
		IEC0bits.T2IE=1;
		IEC0bits.CNIE=0;

	}

}

//stop function
void _ISR _T1Interrupt(void)
{
	systemflag_1=1;
	IFS0bits.T1IF = 0;

}
void _ISR _T2Interrupt(void)
{
	systemflag_2=1;
	IFS0bits.T2IF = 0;

}
//CN stop function
void _ISR _CNInterrupt(void)
{
	CNflag=1;
	IFS0bits.CNIF=0;
}

//timer1 set function
void init_TMR1(void)
{
	ConfigIntTimer1( T1_INT_PRIOR_4 & T1_INT_ON );
	OpenTimer1( T1_ON & T1_IDLE_STOP & T1_GATE_OFF & T1_PS_1_1 
		& T1_SYNC_EXT_OFF & T1_SOURCE_INT , (20000));
}

//timer2 set function
void init_TMR2(void)
{
	ConfigIntTimer2( T2_INT_PRIOR_4 & T2_INT_ON );
	OpenTimer2( T2_ON & T2_IDLE_STOP & T2_GATE_OFF & T2_PS_1_1 
		& T2_32BIT_MODE_OFF & T2_SOURCE_INT , (200));
}
//ADC set function
void init_ad(void)
{
	unsigned int Channel,PinConfig,ADCON1REG,ADCON2REG,ADCON3REG,SCANSELECT;
	ADCON1bits.ADON = 0;


	ConfigIntADC10(ADC_INT_DISABLE);

	// config parameters for OpenADC 
	PinConfig = ENABLE_AN0_ANA &
		ENABLE_AN1_ANA &
		ENABLE_AN2_ANA &
		ENABLE_AN3_ANA;

	ADCON1REG	=	ADC_MODULE_ON &
		ADC_FORMAT_INTG &
		ADC_CLK_MANUAL &
		ADC_AUTO_SAMPLING_OFF &
		ADC_SAMPLE_SIMULTANEOUS &
		ADC_SAMP_OFF ;
	ADCON2REG	=	ADC_VREF_AVDD_AVSS &
		ADC_SCAN_OFF &
		ADC_SAMPLES_PER_INT_1 &
		ADC_CONVERT_CH_0ABC &
		ADC_ALT_BUF_OFF &
		ADC_ALT_INPUT_OFF ;
	ADCON3REG	=	ADC_SAMPLE_TIME_16 &
		ADC_CONV_CLK_SYSTEM &
		ADC_CONV_CLK_4Tcy;

	SCANSELECT	=	SCAN_NONE;

	OpenADC10(ADCON1REG,ADCON2REG,ADCON3REG,PinConfig,SCANSELECT);

	// config channel
	Channel = ADC_CH0_POS_SAMPLEA_AN3 &
		ADC_CH0_NEG_SAMPLEA_NVREF &
		ADC_CHX_POS_SAMPLEA_AN0AN1AN2 &
		ADC_CHX_NEG_SAMPLEA_NVREF;

	SetChanADC10(Channel);
}

void init_MTR(void)
{

	//init parameter
	SetDCMCPWM(1,1000,0);
	SetDCMCPWM(2,1000,0);

	count1_old=0;
	count1_new=0;
	c1=0;

	count2_old=0;
	count2_new=0;
	PWMAEN=1;
	PWMBEN=1;

	LED1=1;
	LED2=1;


	RX_order=0;
	RX_flag=0;


	RPM_motor1=0;
	RPM_wheel1=0;
	RPM_wheel1_cmd=0;
	RPM_wheel1_cmd_tmp=0;
	RPM_wheel1_out=0;

	RPM_motor2=0;
	RPM_wheel2=0;
	RPM_wheel2_cmd=0;
	RPM_wheel2_cmd_tmp=0;
	RPM_wheel2_out=0;

	error1=0;
	ierror1=0;
	error2=0;
	ierror2=0;
	feedforward1=1000;
	feedforward2=1000;
	RPM_motor1_signed_bit=1;
	RPM_motor2_signed_bit=1;

	if (A1==0 && B1==0)   
	{
		state1_old=0;
	}
	if(A1==1 && B1==0)
	{
		state1_old=1;
	}
	if(A1==1 && B1==1)
	{
		state1_old=2;
	}
	if(A1==0 && B1==1)
	{
		state1_old=3;
	}
	if (A2==0 && B2==0)
	{
		state2_old=0;
	}
	if(A2==1 && B2==0)
	{
		state2_old=1;
	}
	if(A2==1 && B2==1)
	{
		state2_old=2;
	}
	if(A2==0 && B2==1)
	{
		state2_old=3;
	}

}


void initMotion(MOTION_DATA* mDataPtr)
{
	int i; // C99
	for(i=0;i<2;i++)
	{
		mDataPtr->vNow[i]=0;
		mDataPtr->vDes[i]=0;
	}
	mDataPtr->state = PRE_SET;
	mDataPtr->restAng = 0;
	mDataPtr->ang =0;
	mDataPtr->dAng =0;
	mDataPtr->dist = 0;
	mDataPtr->dDist = 0;	
	mDataPtr->newCommend =false;
	mDataPtr->catchBall = false;

}

//ADC read
void read_ad(void)
{
	int LOOP;
	ad_read_0=0;
	ad_read_1=0;
	ad_read_2=0;
	ad_read_3=0;


	ADCON1bits.SAMP=1;
	for (LOOP=0;LOOP<100;LOOP++);
	ADCON1bits.SAMP=0;
	while(BusyADC10());
	ad_read_0=ReadADC10(1);
	ad_read_1=ReadADC10(2);
	ad_read_2=ReadADC10(3);
	ad_read_3=ReadADC10(0);
	IFS0bits.ADIF=0;
}

//PWM set function
void init_PWM(void)
{
	unsigned int config;
	unsigned int period;
	unsigned int sptime;
	unsigned int config1;
	unsigned int config2;
	unsigned int config3;
	unsigned int dutycyclereg; 
	unsigned int dutycycle;
	unsigned char updatedisable;

	config=(PWM_INT_DIS & PWM_FLTA_DIS_INT & PWM_INT_PR1 & PWM_FLTA_INT_PR0);
	ConfigIntMCPWM(config);

	dutycyclereg=1;	
	dutycycle=1000;
	updatedisable=0;

	//	SetDCMCPWM(dutycyclereg,dutycycle,updatedisable);

	period= 1000;
	sptime= 0x0;	

	config1=(PWM_EN & PWM_IDLE_STOP & PWM_OP_SCALE1 & PWM_IPCLK_SCALE1 & PWM_MOD_FREE);
	config2=(PWM_MOD1_COMP &PWM_MOD2_COMP &PWM_MOD3_COMP&
		PWM_PDIS3H & PWM_PEN2H & PWM_PEN1H &
		PWM_PDIS3L & PWM_PEN2L & PWM_PEN1L	);
	config3=(PWM_SEVOPS10 & PWM_OSYNC_PWM & PWM_UEN);
	OpenMCPWM(period,sptime,config1,config2,config3);

}

//CN set function
void init_CN(void)
{
	CNEN1bits.CN2IE=1;
	CNEN1bits.CN3IE=1;
	CNEN1bits.CN6IE=1;
	CNEN1bits.CN7IE=1;	
	IPC3|=0x1000;
	IEC0bits.CNIE=1;
}

//UART set funciton
void init_uart(void)
{
	unsigned int BRG;
	unsigned int U2MODEvalue;
	unsigned int U2STAvalue;

	CloseUART2();

	ConfigIntUART2(UART_RX_INT_EN & UART_RX_INT_PR5 & 
		UART_TX_INT_DIS & UART_TX_INT_PR2);

	BRG = 64;		//UxBRG = ((FCY/Desired Baud Rate)/16) - 1
	//UxBRG = (20000000/19200/16-2) = 64

	U2MODEvalue = UART_EN & UART_IDLE_CON &
		UART_DIS_WAKE & UART_DIS_LOOPBACK &
		UART_DIS_ABAUD & UART_NO_PAR_8BIT &
		UART_1STOPBIT;

	U2STAvalue = UART_INT_TX_BUF_EMPTY & UART_TX_PIN_NORMAL &
		UART_TX_ENABLE & UART_INT_RX_CHAR &
		UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;

	OpenUART2( U2MODEvalue, U2STAvalue, BRG);

}


//sitoa number change to character
void sitoa( int The_Number, unsigned char *buff) 
{
	unsigned int Temp_Char_1000, Temp_Char_100, Temp_Char_10, Temp_Char ;
	unsigned char *temp = (unsigned char *) buff ;

	Temp_Char_1000=The_Number/1000;
	temp[0]=Temp_Char_1000+'0';
	The_Number=The_Number-Temp_Char_1000*1000;

	Temp_Char_100=The_Number/100;
	temp[1]=Temp_Char_100+'0';
	The_Number=The_Number-Temp_Char_100*100;

	Temp_Char_10=The_Number/10;
	temp[2]=Temp_Char_10+'0';

	Temp_Char=The_Number-Temp_Char_10*10;
	temp[3]=Temp_Char+'0';	
}



/////////////////////////////
void renewEncoder(void)
{

	int t;
	//4 state machine
	//	state1_new =1+( A1| B1<<1);//1+ 1^A1+2^B1 // wrong
	//  change to state1_new =  A1+3*B1-2*A1*B1
	//((a<<1|b)>>1)^(a<<1|b)
	if(A1==0&&B1==0)
	{
		state1_new=1;
	}
	if(A1==1&&B1==0)
	{
		state1_new=2;
	}
	if(A1==1&&B1==1)
	{
		state1_new=3;
	}
	if(A1==0&&B1==1)
	{
		state1_new=4;
	}
	/*t = ((A1<<1)|B1);
	state1_new = (t>>1)^t;
	state1_new ++;*/
	////////////////////
	//	state2_new =1+( A2| B2<<1);//1+ 1^A2+2^B2    // wrong
	//  change to state2_new =  A2+3*B2-2*A2*B2
	if(A2==0&&B2==0)
	{
		state2_new=1;
	}
	if(A2==1&&B2==0)
	{
		state2_new=2;
	}
	if(A2==1&&B2==1)
	{
		state2_new=3;
	}
	if(A2==0&&B2==1)
	{
		state2_new=4;
	}
	/*t = ((A2<<1)|B2);
	state2_new = (t>>1)^t;
	state2_new++;*/
	//正逆轉
	//t = ((state1_new-state1_old+6)%4-2)%2; //正轉為1，逆轉為-1
	//count1_new +=t;
	//	state1_old = state1_new;
	if(state1_new-state1_old==-3||state1_new-state1_old==1)
	{
		//motor 1 run clockwise
		count1_new=count1_new+1;
		state1_old=state1_new;
	}
	if(state1_new-state1_old==3||state1_new-state1_old==-1)
	{
		//motor 1 run  counter-clockwise
		count1_new=count1_new-1;
		state1_old=state1_new;
	}



	/////////////////////////////////////////////////////////////////注意下，兩邊馬達對於正轉的定義可能不同
	if(state2_new-state2_old==-3||state2_new-state2_old==1)
	{
		//motor 2 run clockwise
		count2_new=count2_new+1;
		state2_old=state2_new;
	}
	if(state2_new-state2_old==3||state2_new-state2_old==-1)
	{
		//motor 2 run  counter-clockwise
		count2_new=count2_new-1;
		state2_old=state2_new;
	}


	//t = ((state2_new-state2_old+6)%4-2)%2; 
	//count2_new +=t; 
	//	state2_old = state2_new;
	/////////////////
}


void controlSpeed(MOTION_DATA* mDataPtr )
{

	//caculate the speed of the motor
	RPM_motor1=(count1_new-count1_old)*500;    //60/12*100rps
	//RPM_wheel1=RPM_motor1/29;//motor reduction rate  29
	mDataPtr->vNow[0] = -RPM_motor1/29*cr/60*2*Pi;
	count1_old=count1_new;

	RPM_motor2= (count2_new-count2_old)*500;
	mDataPtr->vNow[1] =  RPM_motor2/29*cr/60*2*Pi;
	count2_old=count2_new;

	//caculate the motor speed
	//RPM_motor1=(count1_new-count1_old)*500;//60/12*100
	//RPM_wheel1=RPM_motor1/29;//馬達減速比 29
	//mDataPtr->vNow[0] = RPM_motor1/29*cr;
	//count1_old=count1_new;

	//	RPM_motor2=(count2_new-count2_old)*500;
	//	mDataPtr->vNow[1] = RPM_motor2/29*cr;
	//	count2_old=count2_new;

	//caculate the space speed
	//	if(mDataPtr->vNow[0]>0 && mDataPtr->vNow[1]>0)
	//		mDataPtr->vNow[2]= max(mDataPtr->vNow[0],mDataPtr->vNow[1]);
	//	else if(mDataPtr->vNow[0]<0 && mDataPtr->vNow[1]<0)
	//		mDataPtr->vNow[2]= min(mDataPtr->vNow[0],mDataPtr->vNow[1]);
	//	else
	//	{
	//		mDataPtr->vNow[2]=(mDataPtr->vNow[0]+mDataPtr->vNow[1]);
	//	}
	mDataPtr->vNow[2]=(mDataPtr->vNow[0] + mDataPtr->vNow[1])/2;

	//Measure the RPM
	//mDataPtr->vDes[1]  minus->counter-clockwise
	/*	feedforward1 = 1000 + 3*mDataPtr->vDes[0];
	if(feedforward1 >2000)
	{	
	feedforward1 = 2000;
	}
	if(feedforward1 < 0 ) 
	{
	feedforward1=0;
	}*/
	if(mDataPtr->vDes[0]>-50&&mDataPtr->vDes[0]<50)
	{
		feedforward1 = 1000;
	}
	else if(mDataPtr->vDes[0]>=50&&mDataPtr->vDes[0]<100)
	{
		feedforward1 = 700;
	}
	else if(mDataPtr->vDes[0]>=100&&mDataPtr->vDes[0]<200)
	{
		feedforward1 = 500;
	}
	else if(mDataPtr->vDes[0]>=200&&mDataPtr->vDes[0]<300)
	{
		feedforward1 = 300;
	}
	else if(mDataPtr->vDes[0]>=300)
	{
		feedforward1 = 0;
	}
	else if(mDataPtr->vDes[0]<=-50&&mDataPtr->vDes[0]>-100)
	{
		feedforward1 = 1300;
	}
	else if(mDataPtr->vDes[0]<=-100&&mDataPtr->vDes[0]>-200)
	{
		feedforward1 = 1500;
	}
	else if(mDataPtr->vDes[0]<=-200&&mDataPtr->vDes[0]>-300)
	{
		feedforward1 = 1700;
	}
	else if(mDataPtr->vDes[0]<=-300)
	{
		feedforward1 = 2000;
	}

	error1 = mDataPtr->vDes[0]-mDataPtr->vNow[0];
	ierror1=ierror1+error1*0.01;
	ref1 = -(Motor1_Pgain*error1+Motor1_Igain*ierror1)+feedforward1;         //RPM_wheel1_out




	/*feedforward2 = 1000 - 3*mDataPtr->vDes[1];
	if(feedforward2 >2000)
	{ 
	feedforward2 = 2000;
	}
	if(feedforward2 < 0 )
	{
	feedforward2=0;
	}*/

	if(mDataPtr->vDes[1]>-50&&mDataPtr->vDes[1]<50)
	{
		feedforward2 = 1000;
	}
	else if(mDataPtr->vDes[1]>=50&&mDataPtr->vDes[1]<100)
	{
		feedforward2 = 1300;
	}
	else if(mDataPtr->vDes[1]>=100&&mDataPtr->vDes[1]<200)
	{
		feedforward2 = 1500;
	}
	else if(mDataPtr->vDes[1]>=200&&mDataPtr->vDes[1]<300)
	{
		feedforward2 = 1700;
	}
	else if(mDataPtr->vDes[1]>=300)
	{
		feedforward2 = 2000;
	}
	else if(mDataPtr->vDes[1]<=-50&&mDataPtr->vDes[1]>-100)
	{
		feedforward2 = 800;
	}
	else if(mDataPtr->vDes[1]<=-100&&mDataPtr->vDes[1]>-200)
	{
		feedforward2 = 500;
	}
	else if(mDataPtr->vDes[1]<=-200&&mDataPtr->vDes[1]>-300)
	{
		feedforward2 = 300;
	}
	else if(mDataPtr->vDes[1]<=-300)
	{
		feedforward2 = 0;
	}

	error2 = mDataPtr->vDes[1] - mDataPtr->vNow[1];  //positive and negative is different
	ierror2=ierror2+error2*0.01;
	ref2 = (Motor2_Pgain*error2+Motor2_Igain*ierror2)+feedforward2;         //RPM_wheel2_out

}



void getTar(MOTION_DATA* mDataPtr)
{
	
	int sign1 = SIGN(mDataPtr->ang) ;
	double tAng = sign1*mDataPtr->ang;
	
	//dist
		if(tAng <0.785 && tAng>0.0872  && mDataPtr->vNow[2] > mDataPtr->vDes[2]/3)//誤差小於45度，目前速度大於目標速度/3
		{
			mDataPtr->restAng = (Pi/2+tAng)/2;

			//physical model
			if(sign1 >0)//turn left
			{
				mDataPtr->vDes[1] = mDataPtr->vDes[2]*0.8;
				mDataPtr->vDes[0] = mDataPtr->vDes[1]*(mDataPtr->dist-2*cR)/mDataPtr->dist;
			}
			else
			{
				mDataPtr->vDes[0] = mDataPtr->vDes[2]*0.8;
				mDataPtr->vDes[1] = mDataPtr->vDes[0]*(mDataPtr->dist-2*cR)/mDataPtr->dist;				
			}
			mDataPtr->dAng =mDataPtr->vDes[2]/mDataPtr->dist/iFcy;
			
			
			//Use PI to control the ver
			/*mDataPtr->vDes[0] =   mDataPtr->vDes[2]*(1-mDataPtr->ang*D_ANG_GAIN);
			mDataPtr->vDes[1] =   mDataPtr->vDes[2]*(1+mDataPtr->ang*D_ANG_GAIN);
			double R = cR/(1-mDataPtr->ang*D_ANG_GAIN);
			mDataPtr->dAng = mDataPtr->vDes[2]/R/iFcy;*/
			mDataPtr->dDist = mDataPtr->vDes[2]/iFcy;
			mDataPtr->state = ARC;
		}
		else if(tAng <0.0872)//small angle
		{
			int i;
			for( i=0;i<2;i++)
			{
				mDataPtr->vDes[i] = mDataPtr->vDes[2];
			}
			mDataPtr->restAng =sign1* mDataPtr->ang;
			mDataPtr->dAng = 0;
			mDataPtr->dDist = mDataPtr->vDes[2]*0.8/iFcy;
			mDataPtr->state = LINE;
		}
		else// big turning
		{
			mDataPtr->restAng = mDataPtr->ang*sign1;			
			mDataPtr->vDes[1] =  sign1*V_MAX*0.5;
			mDataPtr->vDes[0] =  -mDataPtr->vDes[1];
			mDataPtr->dAng = 0.5*V_MAX/cR/iFcy;
			mDataPtr->dDist = V_MAX*0.8/iFcy;
			mDataPtr->state = TURN;

		}

	mDataPtr->newCommend = false;

}

void intp(MOTION_DATA* mDataPtr)
{
	//if(mDataPtr->catchBall == true)
	//{
	//	mDataPtr->state = CATCH_BALL;
	//}

	switch(mDataPtr->state)
	{
		//	case PRE_SET:

		//	getTar(mDataPtr);			
		//	break;
	case ARC:    //small angle to turn
		LED1 = 1;
		LED2 = 0;
		arcExe(mDataPtr);
		break;
	case TURN:  //big turning
		LED1 = 0;
		LED2 = 1;
		turnExe(mDataPtr);
		break;
	case LINE:  //run straight
		LED1 = 0;
		LED2 = 0;
		lineExe(mDataPtr);
		break;
	case WAIT:

		mDataPtr->vDes[0] = 0;
		mDataPtr->vDes[1] = 0;
		mDataPtr->vDes[2] = 0;
		LED1 = 1;
		LED2 = 1;

		if(mDataPtr->newCommend == true)
		{
			mDataPtr->state = PRE_SET;
		}
		break;

	case CATCH_BALL:
		break;
	}

}
void arcExe(MOTION_DATA* mDataPtr)
{
	mDataPtr->restAng -= mDataPtr->dAng;
	mDataPtr->dist -=mDataPtr->dDist;
	if( mDataPtr->restAng<0.1  )//meet the turning requirement
	{
		mDataPtr->state = LINE;
		int i;
		for( i=0;i<2;i++)
		{
			mDataPtr->vDes[i] = mDataPtr->vDes[2];
		}
	}
}
void turnExe(MOTION_DATA* mDataPtr)
{
	mDataPtr->restAng -= mDataPtr->dAng;

	if(mDataPtr->restAng < 0.1 )
	{
		int i;
		for( i=0;i<2;i++)
		{
			mDataPtr->vDes[i] = mDataPtr->vDes[2];
		}
		mDataPtr->state = LINE;
	}
}

void lineExe(MOTION_DATA* mDataPtr)
{
	mDataPtr->dist -= mDataPtr->dDist; 
	if( mDataPtr->dist  <= 0 )/////////////////////////////////////////
	{
		mDataPtr->state = WAIT;
	}
}


void assign_xyEnd(MOTION_DATA* mDataPtr, double d, double ang)
{

//	mDataPtr->xyEnd[1] = d * sin(ang); //y
//	mDataPtr->xyEnd[0] = d * cos(ang);  //x
}

/************************************************************************************************************************************************************************************************************************************/



/************************************************************************************************************************************************************************************************************************************/


//UART stop function
void _ISR _U2RXInterrupt(void)
{
	data = ReadUART2();
	if(RX_flag==0)
	{
		if(data == '$')
		{
			RX_flag = 1;
			RX_order = 0;

		}
		else if(data == 's')
		{
			mDataPtr->vDes[2]=0;
		}
	}
	//adjust the motor rotating speed
	else if(RX_flag == 1)
	{		
		if(RX_order<13)
		{
			RX_data[RX_order] = data;
			RX_order++;
		}
		else
		{
			RX_data[RX_order] = data;
			RX_order = 0;
			RX_flag = 0;
			renewDataPls = 1;  //global variable




			/*****************************************************************************************************************************************************************************************************************     please edit here*/   

		}
	}

	IFS1bits.U2RXIF=0;
}

void claimBall(void) //get ball
{
	// do the get ball commend

}

void renewData(  MOTION_DATA* mDataPtr) // renew the data after transfer through bluetooth
{


	if(RX_data[0] == '1')//get the absolute coordinates
	{
		//get x
		//X = (RX_data[4]-'0')*1000+(RX_data[5]-'0')*100+(RX_data[6]-'0')*10+(RX_data[7]-'0')*1;
		mDataPtr->vDes[2] = (RX_data[8]-'0')*1000+(RX_data[9]-'0')*100+(RX_data[10]-'0')*10+(RX_data[11]-'0')*1; 
	}
	else if(RX_data[0] == '2')//get the angle distance
	{ 
		//get the distance
		mDataPtr->dist = (RX_data[1]-'0')*1000+(RX_data[2]-'0')*100+(RX_data[3]-'0')*10+(RX_data[4]-'0')*1; 

		//get the angle
		if(RX_data[5] == '+'|| RX_data[5] == '0')
		{
			data_signed_bit = 1; 
		}
		else if(RX_data[5] == '-')
		{
			data_signed_bit = -1; 
		} 
		//double RessssAng = ((RX_data[6]-'0')*100+(RX_data[7]-'0')*10+(RX_data[8]-'0')*1)*-1*data_signed_bit *Pi/180;  //angle
		mDataPtr->ang = (((RX_data[6]-'0')*100+(RX_data[7]-'0')*10+(RX_data[8]-'0')*1)*-1*data_signed_bit) *Pi/180;  //change rad
		//measure the motor speed, v0 = 角度		
//mDataPtr->vDes[0] = ((RX_data[6]-'0')*100+(RX_data[7]-'0')*10+(RX_data[8]-'0')*1)*data_signed_bit; 

		//get the speed
		if(RX_data[9] == '+'|| RX_data[9] == '0')
		{
			data_signed_bit = 1; 
		}
		else if(RX_data[9] == '-')
		{
			data_signed_bit = -1; 
		} 
		mDataPtr->vDes[2] = ((RX_data[10]-'0')*100+(RX_data[11]-'0')*10+(RX_data[12]-'0')*1)*data_signed_bit; 

//measure the motor speed, v1 = vDes[2]
//mDataPtr->vDes[1] = mDataPtr->vDes[2];

		//assign_xyEnd(mDataPtr, mDataPtr->dist, RessssAng );




		//renew the data
		//vDes[2]
		//xyEnd angle and distance
		//getTar(mDataPtr);
	}




}


// 丟的坐標,需要經過兩次換算
// Ang的資料也可以直接給予

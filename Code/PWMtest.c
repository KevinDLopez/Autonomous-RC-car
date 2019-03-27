

/* 
////Kevin Lopez, Cristian Lopez 
/// Robot car wiht programable speed and sensors
/// Project 3 
////Dec 12, 2018
 */
#include <stdint.h>
#include "PLL.h"
#include "PWM.h"
#include "tm4c123gh6pm.h"
#include "Nokia5110.h"
#include "ADCSWTrigger.c" 
#include <math.h>
#include "ADCSWTrigger1.c"

#define START_TURNING 53
#define WALL 20 
#define SIDE_WALL 20
#define OUT_OF_RANGE 50

int update, i;
/*
		WE USE PINS 
			
			PC 6  PC.7   for weels 
			PD 2  PD 3   for weels
 			PB.6  PB.7   for weels speed 
			
			PE.1, 4, 5 	 for ADC Sensors and potentiometer 
			PE 2         for Sensor 
*/

void delay(int x){
	for (int k = 0; k < x; k++){
		
	}
}

unsigned int PotPecentage(unsigned int ADC){
	int percentage = (double)ADC * 0.02442 ;//it will return a percentage depending on ADC
	if (percentage > 98){ percentage = 98; }
	return percentage;
}

unsigned int ir3Distance(unsigned long ADC){
	return  (double)(34524.53/ADC)  - 2.71;
}
unsigned int ir2Distance(unsigned long ADC){
	return  (double)(31900.87/ADC)  - 3.802;
}
unsigned int ir1Distance(unsigned long ADC){
	return  (double)(26064/ADC)  - 0.5426;
}

void Systick(void){ // Systick interrupt for adc
	NVIC_ST_CTRL_R = 0;								// Disables SysTick timer in order to change reload value
	NVIC_ST_RELOAD_R = 80000-1;		// reload reg with about .5 seconds 
	NVIC_ST_CTRL_R = 7;								// enable SysTick timer, and interrupt, system clock source

}
void Switch_Init(void){  unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x00000024; // (a) activate clock for port F,A
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x11;         // allow changes to PF4,0
	  GPIO_PORTF_DIR_R |= 0x0E;    // (c) ENABLE LIGHTS
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4,0 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x11;  //     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4,0
  GPIO_PORTF_PCTL_R &= ~0x000F000F; //  configure PF4,0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x11;  //     disable analog functionality on PF4,0
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; // (g) priority 2
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
	
	GPIO_PORTC_DEN_R |= 0xC0;     //     enable digital I/O on PC4,5
	GPIO_PORTC_DIR_R |= 0xC0;     // digital output for PA3,4
	GPIO_PORTC_DEN_R |= 0x30;     //     enable digital I/O on PC6,7
	GPIO_PORTC_DIR_R |= 0x30;     // digital output for PA6,7
}
int main(void){
		unsigned long s1, s2, s3, ir3, ADCpot;
		unsigned int ir1, ir2, pPercent, leftWheel;
		ADC_Init298();
		ADC0_InitSWTriggerSeq3_Ch1();
		PLL_Init();                      // bus clock at 80 MHz
		Switch_Init();
		Nokia5110_Init();	
		PWM0A_Init(40000, 1);         // initialize PWM0, 1000 Hz, 0% duty
		PWM0B_Init(40000, 1);         // initialize PWM0, 1000 Hz, 0% duty
		GPIO_PORTC_DATA_R = 0xC0; //for well rotation
	
	
		while(1){	
				GPIO_PORTC_DATA_R = 0xC0; //forward
				//GPIO_PORTC_DATA_R = 0x30; // backwards 

				ADC_In298( &s1, &s2, &ADCpot);
				s3 = ADC0_InSeq3();
				Nokia5110_Clear();
				Nokia5110_OutString("S1  S2  PWM");
				ir1= ir1Distance(s1);//this output distance 
				ir2 = ir2Distance(s2);// this wil outout distance 
				ir3 =  ir3Distance(s3);// this wil outout distance
			
					pPercent = PotPecentage(ADCpot) ;	// this will return a percentage 
					
					unsigned int PWM_Val = (pPercent * 400);
					//unsigned int PWM_Val = 18000;
					leftWheel = PWM_Val * 0.725; 
					PWM0_0_CMPA_R = leftWheel ;   // set Potentiometer as duty for PWM
					PWM0_0_CMPB_R = PWM_Val;	 // set Potentiometer as duty for PWM
				
				Nokia5110_OutUDec(ir1);
				Nokia5110_OutUDec(ir2);
				Nokia5110_OutUDec(ir3);
				
				

					
					
					/**************************** GETTING TO THE MIDDLE ************************/ 
					while((ir1 < SIDE_WALL) || (ir2 < SIDE_WALL ) ){
						Nokia5110_Clear();
						Nokia5110_OutString("S1  S2  PWM");						
						
							if (ir1 < 11) { ////////// EXTREAMLY CLOSE TO THE WALL
								PWM0_0_CMPA_R = leftWheel * 1.1;
								PWM0_0_CMPB_R = PWM_Val * 0.95;
							} 
							else if ( ir2 < 11) {////////// EXTREAMLY CLOSE TO THE WALL
								PWM0_0_CMPA_R =leftWheel * 0.96;
								PWM0_0_CMPB_R = PWM_Val  * 1.07;	
							}
							else if (ir1 < ir2) {
								PWM0_0_CMPA_R = leftWheel * 1.14;
								PWM0_0_CMPB_R = PWM_Val * 0.9;
							} 
							else if ( ir2 < ir1) {
								PWM0_0_CMPA_R =leftWheel * 0.94;
								PWM0_0_CMPB_R = PWM_Val  * 1.0;	
							}
							
							
							
						ADC_In298( &s1, &s2, &ADCpot);
						s3 = ADC0_InSeq3();
						ir1= ir1Distance(s1);//  this output distance 
						ir2 = ir2Distance(s2);// this wil outout distance 
						ir3 = ir3Distance(s3);
						pPercent = PotPecentage(ADCpot) ;	// this will return a percentage 
						PWM_Val = (pPercent * 400);
										
						Nokia5110_OutUDec(ir1);
						Nokia5110_OutUDec(ir2);
						Nokia5110_OutUDec(ir3);
						
						if ( ir3 < START_TURNING + 4) {			
							PWM0_0_CMPA_R = leftWheel * 0.5 ;   
							PWM0_0_CMPB_R = PWM_Val * 0.5 ;	 
							break;
						}
					
					}//while
					
			//PWM0_0_CMPA_R = leftWheel * 0.7 ;   
			//PWM0_0_CMPB_R = PWM_Val * 0.7 ;	 

					
				/******************************* TURNING ***************************/ 
			while(ir3 < START_TURNING  ) {
				if ( ir1 > ir2) { // turn right 
							GPIO_PORTC_DATA_R = 0xC0; //forward
							GPIO_PORTC_DATA_R &= ~0x30; // backwards						
							PWM0_0_CMPA_R = 10000; 
							PWM0_0_CMPB_R = 18000 ;	
					}
					else if (ir1 < ir2){ // turn left 
							PWM0_0_CMPA_R = 18000;
							PWM0_0_CMPB_R = 10000;	 
							GPIO_PORTC_DATA_R = 0xC0; //forward
							GPIO_PORTC_DATA_R &= ~0x30; // backwards
							
					}

									////	GET NEW VALUES 	/////.
				ADC_In298( &s1, &s2, &ADCpot);
				s3 = ADC0_InSeq3();
				ir1= ir1Distance(s1);//  this output distance 
				ir2 = ir2Distance(s2);// this wil outout distance 
				ir3 = ir3Distance(s3);
				
				//// DISPLAY NEW VAL ////.
				Nokia5110_Clear();
				Nokia5110_OutString("S1  S2  PWM");
				Nokia5110_OutUDec(ir1);
				Nokia5110_OutUDec(ir2);
				Nokia5110_OutUDec(ir3);

					if (ir3 < WALL ) {break;}; 
					if(ir3 > 55) {break;}
			}			
			
			////// CONTINUE RUNING STRAIGHT //////////////
			PWM0_0_CMPA_R = leftWheel * 0.6 ;   
			PWM0_0_CMPB_R = PWM_Val * 0.6 ;	 
			
			/******************************* WALL ***************************/ 
			while( ir3 < WALL){ 					
					if( ir3 < 15){
						
						if( ir1 > ir2){ // turn right
							PWM0_0_CMPA_R =  11000;
							PWM0_0_CMPB_R = 0;	
							GPIO_PORTC_DATA_R &= ~0xC0; //forward
							GPIO_PORTC_DATA_R = 0x30; // backwards
						}
						else if (ir1 < ir2){ // turn left 
							GPIO_PORTC_DATA_R &= ~0xC0; //forward
							GPIO_PORTC_DATA_R = 0x30; // backwards
							PWM0_0_CMPA_R = 0;
							PWM0_0_CMPB_R = 11500;	
						} 
						
					}
					else { 
								PWM0_0_CMPA_R = 1100;
								PWM0_0_CMPB_R = 1100;	
								GPIO_PORTC_DATA_R = 0xC0; //forward
								GPIO_PORTC_DATA_R &= ~0x30; // backwards
				}
										
					////	GET NEW VALUES 	/////.
					ADC_In298( &s1, &s2, &ADCpot);
					s3 = ADC0_InSeq3();
					ir1= ir1Distance(s1);//  this output distance 
					ir2 = ir2Distance(s2);// this wil outout distance 
					ir3 = ir3Distance(s3);
					//// DISPLAY NEW VAL ////.
					Nokia5110_OutUDec(ir1);
					Nokia5110_OutUDec(ir2);
					Nokia5110_OutUDec(ir3);
				}//while
			
			////// CONTINUE RUNING STRAIGHT //////////////
			PWM0_0_CMPA_R = leftWheel ;   
			PWM0_0_CMPB_R = PWM_Val;	 
			
			
			while(ir1 > 50 && ir2 > 50 && ir3 > 50) {
						PWM0_0_CMPA_R = 0 ;   
						PWM0_0_CMPB_R = 0;	 
						
							////	GET NEW VALUES 	/////.
							ADC_In298( &s1, &s2, &ADCpot);
							s3 = ADC0_InSeq3();
							ir1= ir1Distance(s1);//  this output distance 
							ir2 = ir2Distance(s2);// this wil outout distance 
							ir3 = ir3Distance(s3);
							//// DISPLAY NEW VAL ////.
							Nokia5110_OutUDec(ir1);
							Nokia5110_OutUDec(ir2);
							Nokia5110_OutUDec(ir3);
				
			} 
			
			PWM0_0_CMPA_R = leftWheel ;   
			PWM0_0_CMPB_R = PWM_Val;	 

			
	
			}//	while
}// main



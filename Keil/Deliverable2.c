// 2DX3 Deliverable 2

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
#include <math.h>

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define MAXRETRIES              5           // number of receive attempts before giving up

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

	GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
	GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

	GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//	GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3
                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
	I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
	I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//  I2C0_MTPR_R = 0x3B;                                       						// 8) configure for 100 kbps clock   
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
	//Use PortG0
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
	GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

	return;
}

void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // Activate the clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTM_DIR_R = 0b00000000;                                                            // Make PM0 and PM1 inputs 
	GPIO_PORTM_DEN_R = 0b00000011;
	return;
}

void PortN_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                 // Activate the clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};
	GPIO_PORTN_DIR_R=0b00000001;															// Enable PN0 as digital output													
	GPIO_PORTN_DEN_R=0b00000001;
	return;
}

//Enable D3, D4. Remember D3 is connected to PF4 and D4 is connected to PF0
void PortF_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;                 // Activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};
	GPIO_PORTF_DIR_R=0b00000001;                                                                                // Enable PF0 as digital output
	GPIO_PORTF_DEN_R=0b00000001;
	return;
}

void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                    // activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};     // allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0xFF;
	GPIO_PORTH_DEN_R |= 0xFF;
	return;
}

void spin(){
	int time = 100000;

	GPIO_PORTH_DATA_R = 0b00001001;
	SysTick_Wait(time);
	GPIO_PORTH_DATA_R = 0b00000011;
	SysTick_Wait(time);
	GPIO_PORTH_DATA_R = 0b00000110;
	SysTick_Wait(time);
	GPIO_PORTH_DATA_R = 0b00001100;
	SysTick_Wait(time);
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
	GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
	GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
	FlashAllLEDs();
	SysTick_Wait10ms(10);
	GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
}

void start(){
	uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	
	uint16_t dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
	int status = 0;

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
	
	int spinbool = -1;
	int steps_tracker = 0;
	int step_angle = 16;
	int data = -1;
	int input = 0;
	int rotations = 0; // number of full 360 degree rotations
	int done = 0;
      
	while(1) {
		// wait for the right transmition initiation code
		while(1){
			input = UART_InChar();
			if (input == 's') {
				break;
			}
		}
		
		status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
		
		while(1){// Keep checking if a button is pressed 
			int flag = 0; // sets flag to ensure motor state is only complemented once per press

			// Checks if button is pressed
			while((GPIO_PORTM_DATA_R&0b00000001)==0){
				if (flag == 0){
					spinbool *= -1; // start/stop motor
					flag++;
					steps_tracker = 0;
				}
			}
			
			while((GPIO_PORTM_DATA_R&0b00000010)==0) {
				if (flag == 0){
					if (data == 1) {
						UART_printf("mapping...\r\n");
						done = 1; // break out of loop
					}
					data *= -1; // start/stop data acquisition process
					GPIO_PORTN_DATA_R ^= 0b00000001;
					rotations = 0;
					flag++;
				}
			}
			
			if (spinbool == 1){ // spin motor
				steps_tracker++;
				spin();
			}
			
			if (steps_tracker % step_angle == 0 && spinbool == 1){ // flash LED and take measurements every 45 degrees
				//GPIO_PORTF_DATA_R = 0b00000001;
				FlashLED4(1);
				
				if(data == 1) {
					//wait until the ToF sensor's data is ready
					while (dataReady == 0){
						status = VL53L1X_CheckForDataReady(dev, &dataReady);
						//FlashLED3(1);
						VL53L1_WaitMs(dev, 5);
					}
					dataReady = 0;
					
					//read the data values from ToF sensor
					status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
					status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
					status = VL53L1X_GetSignalRate(dev, &SignalRate);
					status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
					status = VL53L1X_GetSpadNb(dev, &SpadNum);
					
					//FlashLED4(1);

					status = VL53L1X_ClearInterrupt(dev); //clear interrupt has to be called to enable next interrupt
					
					// Only print if range status is 0
					if (RangeStatus == 0){
						// Using trigonomentry to find x and y coordinates
						float x = Distance * cos(steps_tracker*4*(360.0/2048.0)*(3.141592/180));
						float y = Distance * sin(steps_tracker*4*(360.0/2048.0)*(3.141592/180));

						// print the resulting x and y coordinates to UART
						sprintf(printf_buffer,"%f,%f,%i\r\n", x, y, rotations*500);
						
						UART_printf(printf_buffer);
					}
					else{
						// print the resulting x and y coordinates to UART
						sprintf(printf_buffer,"Out of Range\r\n");

						UART_printf(printf_buffer);
					}
				}
			}
			
			if (steps_tracker >= 512) {
				spinbool = -1; // stop motor after 360 degrees
				steps_tracker = 0;
				rotations++; // increase number of full rotations
			}
			
			if (done == 1) {
				done = 0;
				break;
			}
		}
	}
}

int main(void){
	PLL_Init();                                                                                                             // Set System Clock to 20MHz
	SysTick_Init();																																																					// Initialize SysTick configuration
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortM_Init();
	PortF_Init();
	PortH_Init();

	GPIO_PORTF_DATA_R = 0b00000000;
	
	start();
}

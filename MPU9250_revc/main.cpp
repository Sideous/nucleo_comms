/**********************************************************************************
* @file    main.cpp
* @author  Marta Krepelkova
* @version V0.1
* @date    22-March-2015
* @brief   Simply UART communication for STM32F0 Discovery kit.
*          Received character is sent back to your computer.
***********************************************************************************/

/* Includes ----------------------------------------------------------------------*/
#include "mbed.h"
#include "MPU9250.h" //imu 8/16/15
/* Defines -----------------------------------------------------------------------*/

/* Function prototypes -----------------------------------------------------------*/
unsigned long convertToDecimal(char hex[]);
void enable_fifo(void);
int  pull_data_from_fifo( void);

/* Variables ---------------------------------------------------------------------*/
char buffer[45];	//9/6/15 [255];               // for receiving more characters from the computer
int received=0;                 // how many characters were received from computer
int sent=0;                     // how many characters were sent to computer

uint32_t sumCount = 0; //imu 8/16/15
//9/6/15 float sum = 0; //imu 8/16/15
MPU9250 mpu9250; //imu 8/16/15
int32_t az_max=0, az_min=30000; //imu 8/30/15

struct inertial_device {
	int16_t acc_x, acc_y, acc_z;
	int16_t gyr_x, gyr_y, gyr_z;
  } sample[30];

int data_flag=0;

    struct data_passed { // float = 4 bytes, so data_passed is 10*4=40 bytes
        float ax, ay, az;
        float gx, gy, gz;
        float mx, my, mz, temp;
      } data_from_imu;
// mbed - initialization of peripherals

int16_t avg_of_median( inertial_device sample_set[], int16_t samples, int16_t start);//( int16_t sample_set[], int16_t samples);


Serial pc(PA_2, PA_3);         // initialize SERIAL_TX=PA_9, SERIAL_RX=PA_10
Serial bb(PA_11, PA_12); // tx, rx jvm added 8/15/15

Timer t; //imu 8/16/15


/* Functions----------------------------------------------------------------------*/

// Set up interupt service routine to read the MPU9250
Ticker imu_isr_ticker; ///8/16/15
void imu_isr(); //8/16/15

/*******************************************************************************
* Function Name  : serialRx.
* Description    : Saves all received characters to the buffer.
* Input          : None
* Output         : None.
* Return         : None
*******************************************************************************/

void serialRx()
{
    while(pc.readable()) {              // while there is a character to read from the serial port.
        char c=pc.getc();               // receive the charracter
	if(received < 54)
        	buffer[received++]=c;           // save the charracter to the next place in buffer, increments number of receive charactbers
	else
		received=0;		//9/6/15
	pc.putc(c);			//echo
    }
}

/***********************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
***********************************************************************************/
int main()
{
//--------------------------------------
// Hyperterminal configuration is default
// 9600 bauds, 8-bit data, no parity
//--------------------------------------
// Communication settings:
// pc.format(bits,parity,stop_bits)
//      bits: 5-8
//      parity: SerialBase::None, SerialBase::Odd, SerialBase::Even, SerialBase::Forced1, SerialBase::Forced0
//      stop_bits: 1 or 2
// pc.baudrate(baud)
//      baud: The desired baudrate (speed)
//--------------------------------------
// Example for default settings:
// pc.format(8,SerialBase::None,1);
//pc.baud(115200); //9600, 19200, 57600, 115200 testing git again
pc.baud(9600); //9600, 19200, 57600, 115200 testing git again 8/23
//--------------------------------------
bb.baud(9600); 
// bb.baud(115200); 
    int i = 1;

    char *ptr;
        ptr=(char *)&data_from_imu;
           
    //Set up I2C
    i2c.frequency(400000);  // use fast (400 kHz) I2C  
    pc.printf("CPU SystemCoreClock is %d Hz\r\n", SystemCoreClock);   

    t.start();        

    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
    wait(2);
    mpu9250.MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    wait(2);
    mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
    wait(2);
    mpu9250.initMPU9250(); 
    wait(2);
    mpu9250.initAK8963(magCalibration);
    mpu9250.getAres(); // Get accelerometer sensitivity
    mpu9250.getGres(); // Get gyro sensitivity
    mpu9250.getMres(); // Get magnetometer sensitivity
    pc.printf("Accel sensitivity is %f LSB/g \n\r", 1.0f/aRes);
    pc.printf("Gyrosensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
    pc.printf("Magnetom sensitivity is %f LSB/G \n\r", 1.0f/mRes);
    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
    imu_isr_ticker.attach(&imu_isr, .025); // 8/29 changed from .1//for now only call 10hz.005);   

    pc.attach(&serialRx,Serial::RxIrq);  // Attach a function serialRx to be called whenever a serial interrupt generated
    enable_fifo();//jvm 8/29	
    char * hptr, * qptr; //ptr for terminal input
    char hex[9]={"00000000"};
    int address, temp;

    while(1) {
        if(received >0) {
	    switch (buffer[0]) {
	        case    'R':    //pc.printf("Received char: %c (%d). Success!\r\n", buffer[sent],(int)buffer[sent]);   
				// send the character and the character number

pc.printf("ax=%f, ay=%f, az=%f,", data_from_imu.ax, data_from_imu.ay, data_from_imu.az);
pc.printf(" azmax=%f, azmin=%f, %f \r\n", (float)(az_max*aRes - accelBias[2]), (float)(az_min*aRes - accelBias[2]), data_from_imu.temp);
  
/*keep I want this later
	            for( int k=0; k<39; k++)
	                pc.putc(*(ptr+k));
	                //bb.putc(*(ptr+k));
	            
	            pc.putc('\n');    
	            //bb.putc('\n');    
	            //bb.printf("Success!\r\n");
	            received=0;
end keep I want this later*/
			data_flag=0;
	            break;
		case	'D': //push all samples every interup
			pc.printf("Push all samples\r\n");
			data_flag=1;
			for(received=54; received >-1 ;received--)
				buffer[received]=0;
			received=0;

		break;
	        default :
			//pc.printf("%c,\r\n", buffer[received-1]);
	            break;
	    } //for switch


	hptr=strchr(buffer,':');
	if (*hptr) {
		if(strstr(buffer,"read")) {
			//read $$ reads register dec$$, read 0x##: reads register hex##
			data_flag=0;
			if(*(hptr-3)=='x') {	//its in hex 
				hex[8]='\0';
				hex[7]=*(hptr-1);
				hex[6]=*(hptr-2);
				address=(int) convertToDecimal(hex);
			}
			else	
				address=atoi((hptr-3));
			pc.printf("\r\n");

			buffer[54]=mpu9250.readByte(MPU9250_ADDRESS, (char) address);
			pc.printf("value read 0x%x\r\n", buffer[54]);
			qptr=strchr(buffer,'Q');
			if(*qptr || (strstr(buffer,"loop") == NULL)) { //should enter only if loop is not input
				for(received=54; received >-1 ;received--)
					buffer[received]=0;
				received=0;
			} 
		}

		else if(strstr(buffer,"reset")) {
			//read $$ reads register dec$$, read 0x##: reads register hex##
			data_flag=0;
			pc.printf("reset\r\n", buffer[54]);
			az_max=0;
			az_min=30000;
			for(received=54; received >-1 ;received--)	
				buffer[received]=0;
			received=0;
		}	
	}
     } //for while

        wait(1);
        i++;                                                                       // wait 1 second
        myled = !myled;
    }
}

/*******************************************************************************
* Function Name  : imu_isr.
* Description    : polls mpu9250 8/16/15.
* Input          : None
* Output         : None.
* Return         : None
*******************************************************************************/

 void imu_isr()  {

   pull_data_from_fifo();
}

/**
* Title               : Convert Hexadecimal to Decimal(Hexadecimal to Decimal.c)
* Program Description : Write a C Program to Convert
* Hexadecimal Number to Decimal Number.
* Author              : robustprogramming.com
* Interface           : Console
* IDE                 : Code::Blocks 13.12
* Operating System    : Windows 8.1
*/
 

unsigned long convertToDecimal(char hex[])
{
    char *hexString;
    int length = 0;
    const int base = 16; // Base of Hexadecimal Number
    unsigned long decimalNumber = 0;
    int i;
    // Find length of Hexadecimal Number
    for (hexString = hex; *hexString != '\0'; hexString++)
    {
        length++;
    }
    // Find Hexadecimal Number
    hexString = hex;
    for (i = 0; *hexString != '\0' && i < length; i++, hexString++)
    {
        // Compare *hexString with ASCII values
        if (*hexString >= 48 && *hexString <= 57)   // is *hexString Between 0-9
        {
            decimalNumber += (((int)(*hexString)) - 48) * pow(base, length - i - 1);
        }
        else if ((*hexString >= 65 && *hexString <= 70))   // is *hexString Between A-F
        {
            decimalNumber += (((int)(*hexString)) - 55) * pow(base, length - i - 1);
        }
        else if (*hexString >= 97 && *hexString <= 102)   // is *hexString Between a-f
        {
            decimalNumber += (((int)(*hexString)) - 87) * pow(base, length - i - 1);
        }
        else
        {
            printf(" Invalid Hex Number \n");
 
            printf(" Press enter to continue... \n");
            fflush(stdin);
            getchar();
            return 0;
            exit(0);
        }
    }
    return decimalNumber;
}

void enable_fifo(void)	{
  

  

// Configure MPU9250 gyro and accelerometer for bias calculation
  mpu9250.writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  mpu9250.writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  mpu9250.writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  mpu9250.writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
//  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
//  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  mpu9250.writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
  mpu9250.writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
}
int  pull_data_from_fifo( void)	{

  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_avg[3] = {0, 0, 0}, accel_avg[3] = {0, 0, 0};
int status;
//#if 0
//jvm 9/6

  int32_t	lowest, highest, sum;

  int idx;
  mpu9250.readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
 
//sample array is only 35 elements
  if( packet_count > 29)
	packet_count=29;
  
  for (idx=0; idx < packet_count; idx++) {
	mpu9250.readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
	if(data_flag)	{
	  	pc.putc(data[4]);
		pc.putc(data[5]);
		pc.putc(',');
		pc.putc('\n');
	}
	else	{
		sample[idx].acc_x=(int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each
		sample[idx].acc_y= (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		sample[idx].acc_z =(int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    	
		sample[idx].gyr_x  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		sample[idx].gyr_y  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		sample[idx].gyr_z  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
	}	
  }

	

  //find the lowest and the highest sample of each imu element
  lowest=100000, highest=0;
  for( idx=0; idx< packet_count; idx++)	{
	if (sample[idx].acc_x < lowest)
		lowest=sample[idx].acc_x;
	if (sample[idx].acc_x > highest)
		highest=sample[idx].acc_x;
  }

  //eliminate the lowest and the highest sample avg the rest
  sum=0;
  for (idx=0; idx < packet_count; idx++)	{
	if (sample[idx].acc_x == lowest || sample[idx].acc_x == highest )
		;
	else  
		sum += sample[idx].acc_x;
  }

  if (packet_count == 6)
	sum >>= 2;
  else 
	sum /= (packet_count-2);

  data_from_imu.ax = (float)(sum*aRes - accelBias[0]);  // get actual g value, this depends on scale being

  lowest=100000, highest=0;
  for( idx=0; idx< packet_count; idx++)	{
	if (sample[idx].acc_y < lowest)
		lowest=sample[idx].acc_y;
	if (sample[idx].acc_y > highest)
		highest=sample[idx].acc_y;
  }

  //eliminate the lowest and the highest sample avg the rest
  sum=0;
  for (idx=0; idx < packet_count; idx++)	{
	if (sample[idx].acc_y == lowest || sample[idx].acc_y == highest )
		;
	else  
		sum += sample[idx].acc_y;
  }

  if (packet_count == 6)
	sum >>= 2;
  else 
	sum /= (packet_count-2);

  data_from_imu.ay = (float)(sum*aRes - accelBias[1]);  // get actual g value, this depends on scale being
/*9/8/9
  lowest=100000, highest=0;
  for( idx=0; idx< packet_count; idx++)	{
	if (sample[idx].acc_z < lowest)
		lowest=sample[idx].acc_z;
	if (sample[idx].acc_z > highest)
		highest=sample[idx].acc_z;
  }

  //eliminate the lowest and the highest sample avg the rest
  sum=0;
  for (idx=0; idx < packet_count; idx++)	{
	if (sample[idx].acc_z == lowest || sample[idx].acc_z == highest )
		;
	else  
		sum += sample[idx].acc_z;
  }

  if (packet_count == 6)
	sum >>= 2;
  else 
	sum /= (packet_count-2);
9/8/15*/
sum = avg_of_median( sample, 6, 0);

  if (az_max < sum)
	az_max=sum;
  if (az_min > sum)
	az_min= sum; 

  data_from_imu.az = (float)(sum*aRes - accelBias[2]);  // get actual g value, this depends on scale being
data_from_imu.temp= (float)packet_count;

	return(status);
  }
int16_t avg_of_median( inertial_device sample_set[], int16_t samples, int16_t start)	{
/**************************************************************************************************
function avg_of_median by jvm 9/7/15
	Function eliminates the lowest and highest value of the sample set passed and returns the 
	average of the remaining samples.
**************************************************************************************************/
	int16_t avg=0, idx, smallest=32767, largest=-32766;
	int32_t sum=0;
for (idx=0; idx < samples; idx++)	{
		if (sample_set[idx].acc_z < smallest)
			smallest=sample_set[idx].acc_z;
		if (sample_set[idx].acc_z > largest)
			largest=sample_set[idx].acc_z;

  	}
	for (idx=0; idx < samples; idx++)	{
		if (sample_set[idx].acc_z == smallest || sample_set[idx].acc_z == largest )
			;
		else  
			sum += sample_set[idx].acc_z;
	}
	if(samples <2)
		return(-32766);
	else
 		avg= sum / (samples-2);
/*  	for (idx=0; idx < samples; idx++)	{
		if (sample_set[idx] < smallest)
			smallest=sample_set[idx];
		if (sample_set[idx] > largest)
			largest=sample_set[idx];
  	}

	for (idx=0; idx < samples; idx++)	{
		if (sample_set[idx] == smallest || sample_set[idx] == largest )
			;
		else  
			sum += sample_set[idx];
	}
	if(samples <2)
		return(-32766);
	else
 		avg= sum / (samples-2);
*/
	return(avg);
}


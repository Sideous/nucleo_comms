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
char buffer[255];               // for receiving more characters from the computer
int received=0;                 // how many characters were received from computer
int sent=0;                     // how many characters were sent to computer

uint32_t sumCount = 0; //imu 8/16/15
float sum = 0; //imu 8/16/15
MPU9250 mpu9250; //imu 8/16/15
int32_t az_max=0, az_min=30000; //imu 8/30/15
int azint[100];
    struct data_passed { // float = 4 bytes, so data_passed is 10*4=40 bytes
        float ax, ay, az;
        float gx, gy, gz;
        float mx, my, mz, temp;
      } data_from_imu;
// mbed - initialization of peripherals

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
        buffer[received++]=c;           // save the charracter to the next place in buffer, increments number of receive charactbers
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
    pc.printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/aRes);
    pc.printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
    pc.printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mRes);
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

//pc.printf("ax=%f, ay=%f, az=%f \r\n", data_from_imu.ax, data_from_imu.ay, data_from_imu.az);
pc.printf("ax=%f, ay=%f, az=%f,", data_from_imu.ax, data_from_imu.ay, data_from_imu.az);
pc.printf(" azmax=%f, azmin=%f \r\n", (float)(az_max*aRes - accelBias[2]), (float)(az_min*aRes - accelBias[2]));
  
/*keep I want this later
	            for( int k=0; k<39; k++)
	                pc.putc(*(ptr+k));
	                //bb.putc(*(ptr+k));
	            
	            pc.putc('\n');    
	            //bb.putc('\n');    
	            //bb.printf("Success!\r\n");
	            received=0;
end keep I want this later*/
	            break;
	        default :
			//pc.printf("%c,\r\n", buffer[received-1]);
	            break;
	    } //for switch


	hptr=strchr(buffer,':');
	if (*hptr) {
		if(strstr(buffer,"read")) {
			//read $$ reads register dec$$, read 0x##: reads register hex##

			if(*(hptr-3)=='x') {	//its in hex 
				hex[8]='\0';
				hex[7]=*(hptr-1);
				hex[6]=*(hptr-2);
				address=(int) convertToDecimal(hex);
			}
			else	
				address=atoi((hptr-3));
			pc.printf("\r\n");

			buffer[254]=mpu9250.readByte(MPU9250_ADDRESS, (char) address);
			pc.printf("value read 0x%x\r\n", buffer[254]);
			qptr=strchr(buffer,'Q');
			if(*qptr || (strstr(buffer,"loop") == NULL)) { //should enter only if loop is not input
				for(received=254; received >-1 ;received--)
					buffer[received]=0;
				received=0;
			} 
		}

		else if(strstr(buffer,"reset")) {
			//read $$ reads register dec$$, read 0x##: reads register hex##

			pc.printf("reset\r\n", buffer[254]);
			az_max=0;
			az_min=30000;
			for(received=254; received >-1 ;received--)	
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
            printf(" Invalid Hexadecimal Number \n");
 
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
#if 0
//jvm 9/6
  struct inertial_device {
  int16_t acc_x, acc_y, acc_z;
  int16_t gyr_x, gyr_y, gyr_z;
  } ;
  struct element {
  int32_t	reading;
  
  } median_filter[6];

  int32_t	lowest, highest, sum;

  int idx;
  mpu9250.readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  struct inertial_device sample[packet_count];
  
  for (idx=0; idx < packet_count; idx++) {
	mpu9250.readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
	sample[idx].acc_x=(int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each
	sample[idx].acc_y= (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
	sample[idx].acc_z =(int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
	sample[idx].gyr_x  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
	sample[idx].gyr_y  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
	sample[idx].gyr_z  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;	
  }
  //find the lowest and the highest sample of each imu element
  lowest=100000, highest=0;
  for( idx=0; idx< packet_count; idx++)	{
	//median_filter[idx].reading= sample[idx].acc_x;
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
	sum /= (packet_count);

  data_from_imu.ax = (float)(sum*aRes - accelBias[0]);  // get actual g value, this depends on scale being

//jvm 9/6
#endif
  mpu9250.readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
	    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
	    mpu9250.readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
	    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
	    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
	    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
azint[ii]=accel_temp[2];
if (az_max < accel_temp[2])
	az_max=accel_temp[2];
if (az_min > accel_temp[2])
	az_min= accel_temp[2];    
	    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
	    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
	    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
	    
	    accel_avg[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	    accel_avg[1] += (int32_t) accel_temp[1];
	    accel_avg[2] += (int32_t) accel_temp[2];
	    gyro_avg[0]  += (int32_t) gyro_temp[0];
	    gyro_avg[1]  += (int32_t) gyro_temp[1];
	    gyro_avg[2]  += (int32_t) gyro_temp[2];
		    
    }
    accel_avg[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_avg[1] /= (int32_t) packet_count;
    accel_avg[2] /= (int32_t) packet_count;
    gyro_avg[0]  /= (int32_t) packet_count;
    gyro_avg[1]  /= (int32_t) packet_count;
    gyro_avg[2]  /= (int32_t) packet_count;
    
//8/29
 
  //  mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values   
    // Now we'll calculate the accleration value into actual g's
    data_from_imu.ax =ax = (float)(accel_avg[0]*aRes - accelBias[0]);  // get actual g value, this depends on scale being set
    data_from_imu.ay =ay = (float)(accel_avg[1]*aRes - accelBias[1]);   //jvm 8/16/15 added () to entire eq
    data_from_imu.az =az = (float)(accel_avg[2]*aRes - accelBias[2]);  
   
    //mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    data_from_imu.gx =gx = (float)(gyro_avg[0]*gRes - gyroBias[0]);  // get actual gyro value, this depends on scale being set
    data_from_imu.gy =gy = (float)(gyro_avg[1]*gRes - gyroBias[1]);  
    data_from_imu.gz =gz = (float)(gyro_avg[2]*gRes - gyroBias[2]);   
 /*8/29 
    mpu9250.readMagData(magCount);  // Read the x/y/z adc values   
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    data_from_imu.mx =mx = (float)(magCount[0]*mRes*magCalibration[0] - magbias[0]);  // get actual magnetometer value, this depends on scale being set
    data_from_imu.my =my = (float)(magCount[1]*mRes*magCalibration[1] - magbias[1]);  
    data_from_imu.mz =mz = (float)(magCount[2]*mRes*magCalibration[2] - magbias[2]); 
8/29*/  
//8/29
/* 
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
*/
/// Push gyro biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
*/
/*
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
*/
	return(status);
  }

// leftovers
/*        pc.printf("mpu9250 address 0x%x\n\r", MPU9250_ADDRESS); 
        pc.printf("I AM 0x%x\n\r", whoami); pc.printf("I SHOULD BE 0x71\n\r");
    

      if (whoami == 0x71) // WHO_AM_I should always be 0x68
      {  
        pc.printf("MPU9250 WHO_AM_I is 0x%x\n\r", whoami);
        pc.printf("MPU9250 is online...\n\r");
        sprintf(buffer, "0x%x", whoami);
        wait(1);
        mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
        mpu9250.MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
        pc.printf("x-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[0]);  
        pc.printf("y-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[1]);  
        pc.printf("z-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[2]);  
        pc.printf("x-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[3]);  
        pc.printf("y-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[4]);  
        pc.printf("z-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[5]);  
        mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
        pc.printf("x gyro bias = %f\n\r", gyroBias[0]);
        pc.printf("y gyro bias = %f\n\r", gyroBias[1]);
        pc.printf("z gyro bias = %f\n\r", gyroBias[2]);
        pc.printf("x accel bias = %f\n\r", accelBias[0]);
        pc.printf("y accel bias = %f\n\r", accelBias[1]);
        pc.printf("z accel bias = %f\n\r", accelBias[2]);
        wait(2);
        mpu9250.initMPU9250(); 
        pc.printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        mpu9250.initAK8963(magCalibration);
        pc.printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
        pc.printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
        pc.printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));
        if(Mscale == 0) pc.printf("Magnetometer resolution = 14  bits\n\r");
        if(Mscale == 1) pc.printf("Magnetometer resolution = 16  bits\n\r");
        if(Mmode == 2) pc.printf("Magnetometer ODR = 8 Hz\n\r");
        if(Mmode == 6) pc.printf("Magnetometer ODR = 100 Hz\n\r");
        wait(1);
    }
   else  {
        pc.printf("Could not connect to MPU9250: \n\r");
        pc.printf("%#x \n",  whoami);
        while(1) ; // Loop forever if communication doesn't happen
    }
    mpu9250.getAres(); // Get accelerometer sensitivity
    mpu9250.getGres(); // Get gyro sensitivity
    mpu9250.getMres(); // Get magnetometer sensitivity
    pc.printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/aRes);
    pc.printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
    pc.printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mRes);
    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
    imu_isr_ticker.attach(&imu_isr, .1); //for now only call 10hz.005);                                                                  
      
    pc.printf("Program started !\r\n");                                                 // text displayed on a computer

    
//    while(1) {
            wait(1);
            sum += deltat;
            sumCount++;
            // Serial print and/or display at 0.5 s rate independent of data rates
            delt_t = t.read_ms() - count;
            if (delt_t > 500) { // update LCD once per half-second independent of read rate
                //jvm 8/2/15
                pc.printf(" az max= %f, az_min= %f mg\n\r", 1000*az_max, 1000*az_min);
                pc.printf("ax = %f", 1000*ax); 
                pc.printf(" ay = %f", 1000*ay); 
                pc.printf(" az = %f  mg\n\r", 1000*az); 
        
                pc.printf("gx = %f", gx); 
                pc.printf(" gy = %f", gy); 
                pc.printf(" gz = %f  deg/s\n\r", gz); 
                
                pc.printf("gx = %f", mx); 
                pc.printf(" gy = %f", my); 
                pc.printf(" gz = %f  mG\n\r", mz); 
                
                tempCount = mpu9250.readTempData();  // Read the adc values
                data_from_imu.temp =temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade
                pc.printf(" temperature = %f  C\n\r", temperature); 
                
                pc.printf("q0 = %f\n\r", q[0]);

                pc.printf("q1 = %f\n\r", q[1]);
                pc.printf("q2 = %f\n\r", q[2]);
                pc.printf("q3 = %f  this took %f usec TO READ\n\r", q[3], deltat);      
                //bb.printf("hellow world\n\r"); //jvm 8/4/15
         
          // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.

          // In this coordinate system, the positive z-axis is down toward Earth. 
          // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
          // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
          // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
          // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
          // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
          // applied in the correct order which for this configuration is yaw, pitch, and then roll.
          // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
                yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
                pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
                roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
                pitch *= 180.0f / PI;
                yaw   *= 180.0f / PI; 
                yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
                roll  *= 180.0f / PI;
            
                pc.printf("Yaw, Pitch, Roll: %f %f %f\n\r", yaw, pitch, roll);
                pc.printf("average rate = %f\n\r", (float) sumCount/sum);
              
                myled= !myled;


                count = t.read_ms(); 
            
                if(count > 1<<21) {
                    t.start(); // start the timer over again if ~30 minutes has passed
                    count = 0;
                    deltat= 0;
                    lastUpdate = t.read_us();
                }
                sum = 0;
                sumCount = 0; 
            } //// if > .5s

     //****   
        data_from_imu.ax=1.01;
        data_from_imu.ay=2.02;
        data_from_imu.az=3.03;
        data_from_imu.gx=4.04;
        data_from_imu.gy=5.05;
        data_from_imu.gz=6.06;
        data_from_imu.mx=7.07;
        data_from_imu.my=8.08;
        data_from_imu.mz=9.09;
        data_from_imu.temp=10.1;   
*/ 
//    }//while loop

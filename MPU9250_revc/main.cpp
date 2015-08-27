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
 
//#include<stdio.h>
//#include<stdlib.h>
//#include<math.h>
 

/* Function prototypes -----------------------------------------------------------*/
unsigned long convertToDecimal(char hex[]);
/* Variables ---------------------------------------------------------------------*/
char buffer[255];               // for receiving more characters from the computer
int received=0;                 // how many characters were received from computer
int sent=0;                     // how many characters were sent to computer

uint32_t sumCount = 0; //imu 8/16/15
float sum = 0; //imu 8/16/15
MPU9250 mpu9250; //imu 8/16/15
float az_max=0, az_min=10000; //imu 8/16/15

    struct data_passed { // float = 4 bytes, so data_passed is 10*4=40 bytes
        float ax, ay, az;
        float gx, gy, gz;
        float mx, my, mz, temp;
      } data_from_imu;


// mbed - initialization of peripherals

Serial pc(PA_2, PA_3);         // initialize SERIAL_TX=PA_9, SERIAL_RX=PA_10
Serial bb(PA_11, PA_12); // tx, rx jvm added 8/15/15
// temp switch so I can test host code
//Serial bb(PA_2, PA_3);         // initialize SERIAL_TX=PA_9, SERIAL_RX=PA_10
//Serial pc(PA_11, PA_12); // tx, rx jvm added 8/15/15

//?? 8/16/15 DigitalOut myled(LED1);

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
    imu_isr_ticker.attach(&imu_isr, .1); //for now only call 10hz.005);   

    pc.attach(&serialRx,Serial::RxIrq);  // Attach a function serialRx to be called whenever a serial interrupt is generated
	
	char * hptr, * qptr; //ptr for terminal input
	char hex[9]={"00000000"};
	int address, temp;
    while(1) {
        if(received >0) {
	    switch (buffer[0]) {
	        case    'R':    //pc.printf("Received char: %c (%d). Success!\r\n", buffer[sent],(int)buffer[sent]);   // send the character and the character number
	            for( int k=0; k<39; k++)
	                pc.putc(*(ptr+k));
	                //bb.putc(*(ptr+k));
	            
	            pc.putc('\n');    
	            //bb.putc('\n');    
	            //bb.printf("Success!\r\n");
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
//			pc.printf("read REG address is ");

			if(*(hptr-3)=='x') {	//its in hex 
				hex[8]='\0';
				hex[7]=*(hptr-1);
				hex[6]=*(hptr-2);
				address=(int) convertToDecimal(hex);
			}
			else	
				address=atoi((hptr-3));
			pc.printf("\r\n");
//			pc.printf("%d \r\n", address);

			buffer[254]=mpu9250.readByte(MPU9250_ADDRESS, (char) address);
			pc.printf("value read 0x%x\r\n", buffer[254]);
			qptr=strchr(buffer,'Q');
			if(*qptr || (strstr(buffer,"loop") == NULL)) { //should enter only if loop is not input
				for(received=254; received >-1 ;received--)
					buffer[received]=0;
				received=0;
			} //end else if

		}	


	}

     } //for while
//8/23        received=0;                                                             // number of received charracters is 0
        wait(1);
        i++;                                                                       // wait 1 second
        //pc.printf("This program runs since %d seconds.\r\n", i++);                      // sends how long is the program running
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
   
  // If intPin goes high, all data registers have new data
  if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt

    mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values   
    // Now we'll calculate the accleration value into actual g's
    data_from_imu.ax =ax = (float)(accelCount[0]*aRes - accelBias[0]);  // get actual g value, this depends on scale being set
    data_from_imu.ay =ay = (float)(accelCount[1]*aRes - accelBias[1]);   //jvm 8/16/15 added () to entire eq
    data_from_imu.az =az = (float)(accelCount[2]*aRes - accelBias[2]);  
   
    mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    data_from_imu.gx =gx = (float)(gyroCount[0]*gRes - gyroBias[0]);  // get actual gyro value, this depends on scale being set
    data_from_imu.gy =gy = (float)(gyroCount[1]*gRes - gyroBias[1]);  
    data_from_imu.gz =gz = (float)(gyroCount[2]*gRes - gyroBias[2]);   
  
    mpu9250.readMagData(magCount);  // Read the x/y/z adc values   
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    data_from_imu.mx =mx = (float)(magCount[0]*mRes*magCalibration[0] - magbias[0]);  // get actual magnetometer value, this depends on scale being set
    data_from_imu.my =my = (float)(magCount[1]*mRes*magCalibration[1] - magbias[1]);  
    data_from_imu.mz =mz = (float)(magCount[2]*mRes*magCalibration[2] - magbias[2]);   

#if 0
    //remove for debug**
    mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values    
    data_from_imu.ax =(float)accelCount[0];//*aRes;// - accelBias[0]);  // get actual g value, this depends on scale being set
    data_from_imu.ay =(float)accelCount[1];//*aRes;// - accelBias[1]);   //jvm 8/16/15 added () to entire eq
    data_from_imu.az =(float)accelCount[2];//*aRes;// - accelBias[2]);  
mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    data_from_imu.gx =(float)gyroCount[0];  // get actual gyro value, this depends on scale being set
    data_from_imu.gy =(float)gyroCount[1];  
    data_from_imu.gz =(float)gyroCount[2];   
 mpu9250.readMagData(magCount);  // Read the x/y/z adc values  
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    data_from_imu.mx =(float)magCount[0];  // get actual magnetometer value, this depends on scale being set
    data_from_imu.my =(float)magCount[1];  
    data_from_imu.mz =(float)magCount[2];  
  //*****end remove

#endif 
  }
    
   // Pass gyro rate as rad/s
if (az > az_max)
    az_max=az;
if (az < az_min)
    az_min=az;
    
  //mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
    
    Now = t.read_us();
    deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
    lastUpdate = Now;

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
*/ 
//    }//while loop

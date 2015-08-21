//============================================================================
// Name        : talk2nucleo.cpp
// Author      : jvm
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
/*
#include <iostream>
using namespace std;

int main() {
	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
	return 0;
}*/

/* Simple send and receive C example for communicating with the
*  Arduino echo program using UART4.
*
* Written by Derek Molloy for the book "Exploring BeagleBone: Tools and
* Techniques for Building with Embedded Linux" by John Wiley & Sons, 2014
* ISBN 9781118935125. Please see the file README.md in the repository root
* directory for copyright and GNU GPLv3 license information.            */

// Beginning
//added by jvm 8/10/15
#include <iostream>
#include <fstream>
using namespace std;

//**
#include<stdio.h>
#include<fcntl.h>
#include<unistd.h>
#include<termios.h>
#include<string.h>



int main(int argc, char *argv[]){
   int file, count;


   ofstream myfile;


   if(argc!=2){
       printf("Invalid number of arguments, exiting!\n");
       return -2;
   }

   for(count=0; count<150; count++) { //jvm 8/14/15
	   if ((file = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY))>=0){
	        break;
	      }
	   usleep(1000000);
   }
   if (file < 0) {
         perror("UART: Failed to open the file.\n");
         return -1;
      }

 /*  if ((file = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY))<0){
      perror("UART: Failed to open the file.\n");
      return -1;
   }*/

   struct termios options;
   tcgetattr(file, &options);
  options.c_cflag = B115200 | CS8 | CREAD | CLOCAL;//jvm 8/11
   //options.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
   options.c_iflag = IGNPAR | ICRNL;
   tcflush(file, TCIFLUSH);
   tcsetattr(file, TCSANOW, &options);

   // send the string plus the null character
   if ((count = write(file, argv[1], strlen(argv[1])+1))<0){
      perror("Failed to write to the output\n");
      return -1;
   }
   usleep(100000);

   unsigned char receive[100];
   int short * iptr;

//8/14/15
  struct data_passed { // float = 4 bytes, so data_passed is 9*4=36 bytes
	float ax, ay, az;
	float gx, gy, gz;
	float mx, my, mz, temp;
  };

  //8/18/15
  struct imu {
       unsigned short ax_reg, ay_reg, az_reg;
       unsigned short gx_reg, gy_reg, gz_reg;
       unsigned short mx_reg, my_reg, mz_reg;
       };



  //8/18/15
 data_passed * imuptr;
//8/19/15imu * imuptr;
   imuptr=(data_passed *) &receive; //mapping received buffer to values passed
//18/19/15  imuptr=(imu *) &receive; //mapping received buffer to values passed
   //if ((count = read(file, (void*)receive, 40))<0){
   if ((count = read(file, (void *)receive, 3))<0){
      perror("Failed to read from the input\n");
      return -1;
   }
   if (count==0) printf("There was no data available to read!\n");
   else {

      for(int k=0;k< 100;k++) {
    	  write(file, "R\n", 1);
    	  usleep(1000000);
     	  count = read(file, (void *)receive, 50);
    	  if (count!=0) {
    	//8/19/15	  printf("%d, ax=%d, ay=%d, az%d\n",k,imuptr->ax_reg,imuptr->ay_reg,imuptr->az_reg);

    		  printf("%d, ax=%9.7f, ay=%9.7f, az=%9.7f,",k,(float)imuptr->ax,(float)imuptr->ay,(float)imuptr->az);
    		  printf("gx=%9f, gy=%9f, gz=%9f,",(float)imuptr->gx,(float)imuptr->gy,(float)imuptr->gz);
    		  printf("mx=%9f, my=%9f, mz=%9f\n",(float)imuptr->mx,(float)imuptr->my,(float)imuptr->mz);
    		  myfile.open("mydata.txt", ios::out | ios::app); //jvm 8/10/15
    		  if(myfile.is_open()){	//can write to file
    			 // myfile << std::hex << k << ',' <<imuptr->ax << ',' << imuptr->ay << ','  << imuptr->az << ',';
    			  myfile << k << ', ' <<imuptr->ax << ',' << imuptr->ay << ','  << imuptr->az << ',';
    			  myfile <<  imuptr->gx << ',' << imuptr->gy << ',' << imuptr->gz << ',';
    			  myfile <<  imuptr->mx << ',' << imuptr->my << ',' << imuptr->mz << '\n';
    			 myfile.close();
    		  }
    	  }

      }}
   close(file);
   return 0;
}

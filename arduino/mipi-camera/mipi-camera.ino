#include <Arduino.h>
#include "FPGA_Controller.h"


void setup() {
  // put your setup code here, to run once:
  // initialize serial communication at 9600 bits per second:
  // This serial is between the PC and the SAMD21
  Serial.begin(115200); // Serial.begin() should be called before uploadBitstream

  // Upload the bitstream to the FPGA
  uploadBitstream();
  while(!Serial);
  //Serial.println("Welcome!");

  initJTAG();

  // UART between FPGA and SAMD21
  Serial1.begin(115200);
  while(!Serial1);
  
}

uint32_t pixel_count = 0;
uint32_t counter = 0;
void loop() {
  // put your main code here, to run repeatedly:

  // UART WRITE
  
  Serial.println("Transmitting Data to FPGA begins...");
  while (counter <= 255) {   // 640 * 2 is because sdram 1 address can store 2 counter's value. Making the sdram address == counter/2
    Serial1.write(counter);
    counter++;
    //delay(5);

    
    //uint32_t debug0 = readJTAG(0);
    //Serial.print("DEBUG0: ");
    //Serial.println(debug0, HEX);
    //uint32_t debug1 = readJTAG(1);
    //Serial.print("DEBUG1: ");
    //Serial.println(debug1, HEX);
    //uint32_t debug2 = readJTAG(2);
    //Serial.print("DEBUG2: ");
    //Serial.println(debug2, HEX);
    
  }
  Serial.println("Transmission Done...");
  

  // UART READ
  Serial.println("Receiving Data from FPGA begins...");
  while (true) {
  while (Serial1.available()) {
    char rcv = Serial1.read();
    if (pixel_count > 0 && pixel_count <= 640*480) {
      pixel_count++;
    } else {
      pixel_count = 0;
      //Serial.println("FrameStart");
      pixel_count++;
    }
    Serial.print("RX: ");
    Serial.println(rcv,DEC); 
    
    
    //uint32_t fifo_full = readJTAG(0);
    //if (fifo_full != 0xFFFFFFFF) {
    //  Serial.print("FIFO Indicator: ");
    //  Serial.println(fifo_full, HEX);
    //}  
  }
  }
  //Serial.println("UART is FREE");


  //uint32_t line_count = readJTAG(1);
  //Serial.print("Line COUNT: ");
  //Serial.println(line_count);
  //delay(10000); // delay
}

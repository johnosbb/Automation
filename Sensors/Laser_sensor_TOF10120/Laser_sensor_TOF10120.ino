#include <Wire.h>
 

// I2C device found at address 0x29 (Decimal: 41)

#define I2C_ADDRESS 41 //(0x29)
unsigned char ok_flag;
unsigned char fail_flag;
 
unsigned short lenth_val = 0;
unsigned char i2c_rx_buf[16];
unsigned char dirsend_flag=0;
 
 
void setup() {
Wire.begin();
Serial.begin(9600);
delay(2000); 
}
 
void loop() {
 
 
int x=ReadDistance();
Serial.print(x);
Serial.println(" mm");
 
}
 
int serial_putc( char c, struct __file * )
{
Serial.write( c );
return c;
}
 
void printf_begin(void)
{
fdevopen( &serial_putc, 0 );
}
 
 
 
 
void SensorRead(unsigned char addr,unsigned char* datbuf,unsigned char cnt)
{
unsigned short result=0;
// step 1: instruct sensor to read echoes
Wire.beginTransmission(I2C_ADDRESS); // transmit to device I2C_ADDRESS
// the address specified in the datasheet is 164 (0xa4)
// but i2c adressing uses the high 7 bits so it's I2C_ADDRESS
Wire.write(byte(addr)); // sets distance data address (addr)
Wire.endTransmission(); // stop transmitting
// step 2: wait for readings to happen
delay(5); // datasheet suggests at least 30uS
// step 3: request reading from sensor
Wire.requestFrom(I2C_ADDRESS, cnt); // request cnt bytes from slave I2C_ADDRESS
Serial.print("Received ");
Serial.print(cnt);
Serial.println(" bytes");
// step 5: receive reading from sensor
if (cnt <= Wire.available()) { // if two bytes were received

unsigned char highByte = Wire.read();
unsigned char lowByte = Wire.read();
*datbuf++ = highByte; // receive high byte (overwrites previous reading)
*datbuf++ = lowByte; // receive low byte as lower 8 bits
Serial.print("Lo: ");
Serial.print(lowByte);
Serial.print("   High: ");
Serial.println(highByte);
}
}
 
int ReadDistance(){
SensorRead(0x00,i2c_rx_buf,2);
lenth_val=i2c_rx_buf[0];
lenth_val=lenth_val<<8;
lenth_val|=i2c_rx_buf[1];
delay(300);
return lenth_val;
}
/*
Arduino Line Follower Program for 5 Light Sensor confirguration.
Uses PID (Proportional Integral Derivative) Algorithm.
Owner: Shaurya Jain.
*/
#include <SoftwareSerial.h>
#include <EEPROM.h>
#define lmf 6
#define rmf 11
#define lmb 9
#define rmb 10

float kp=0.00f;//1
float ki=0.00f;//2
float kd=0.00f;//3
float sp=0.00f;//4
float kr=0.00f;//5
int maxspeed=0;
int upper_read,lower_read;
const int full_rev_delay=2000;
int error, preverror;
float p;
float i;
float d;
float correction_term;
int Speed=0, turn=0, manual_control=0;
enum{LINE,NOLINE,RLINE,LLINE,BLACK,MANUAL}mode;
void motors_write(int right, int left);
void readval();
void correction();
void read_const();
void write_const();
void bluetooth_receive();
void remote_control();
void manual_flush();
void calibrate();
bool todigital(int a);
void send_settings();
void(* reset) (void) = 0;
SoftwareSerial BTSerial(2,3);
void motor()
{
  if(error>0)
  {
    motors_write(0,maxspeed);
    
  }
  else if(error<0)
  {
    motors_write(maxspeed,0);
  }
  else
  {
    motors_write(maxspeed,maxspeed);
  }
}

void setup()
{
  Serial.begin(9600);
	preverror=0;
	i=0;
	read_const();//use when using BTSerial
	calibrate();
	BTSerial.begin(9600);//for bluetooth
	send_settings();//can be improved


}

void loop()//add modes if required
{
  /*
	BTSerial.print(p);
	BTSerial.print(" ");
	BTSerial.print(i);
	BTSerial.print(",");
	BTSerial.print(d);
	BTSerial.print("\t");
	BTSerial.println(error);
  */
	readval();
	correction();
  //motor();
	bluetooth_receive();//or just BTSerial receive
  //delay(10);
}
void correction()
{
	//error=sp-error;
  if(preverror<0 && error>0)
  {
    error=-7;
    motors_write(0,maxspeed);delay(100);
  }
    if(preverror>0 && error<0)
  {
    error=7;
    motors_write(maxspeed,0);delay(100);
  }
	p=error;
	i+=error/100;
	d=error-preverror;
	correction_term=p*kp+i*ki+d*kd;
	preverror=error;
  	int lspeed=maxspeed+correction_term;
  	int rspeed=maxspeed-correction_term;
	if(correction_term<0)
	{
    	lspeed=maxspeed+correction_term;
    	rspeed=maxspeed-(kr*correction_term);
    	if(lspeed>255)
    	  lspeed=255;
    	else if(lspeed<-255)
    	  lspeed=-255;
    	if(rspeed>255)
    	  rspeed=255;
    	else if(rspeed<-255)
    	  rspeed=-255;
		motors_write(rspeed,lspeed);
	}
	else
	{
		lspeed=maxspeed+(kr*correction_term);
    	rspeed=maxspeed-correction_term;
    	if(lspeed>255)
    	  lspeed=255;
    	else if(lspeed<-255)
    	  lspeed=-255;
    	if(rspeed>255)
    	  rspeed=255;
    	else if(rspeed<-255)
    	  rspeed=-255;
    	motors_write(rspeed,lspeed);
	}
}
void motors_write(int right=0, int left=0)
{
	if(right>0)
	{
		analogWrite(rmb,0);
		analogWrite(rmf,right);
	}
	else
	{
		analogWrite(rmf, 0);
		analogWrite(rmb, abs(right));
	}
	if(left>0)
	{
		analogWrite(lmb,0);
		analogWrite(lmf,left);
	}
	else
	{
		analogWrite(lmf, 0);
		analogWrite(lmb, abs(left));
	}

}
void readval()
{
/*
10000 -4
11000 -3
01000 -2
01100 -1
00100 0
00110 1
00010 2
00011 3
00001 4
00000 


*/
int sensor[5];
for(int i=0;i<5;i++)
{
  //Serial.print(analogRead(i));
  //Serial.print(", ");
	sensor[i]=todigital(analogRead(i));
}
//Serial.println();
if(sensor[0]==1&&sensor[1]==0&&sensor[2]==0&&sensor[3]==0&&sensor[4]==0)
{
	error= -4;
}
else if(sensor[0]==1&&sensor[1]==1&&sensor[2]==0&&sensor[3]==0&&sensor[4]==0)
{
	error= -3;
}
else if(sensor[0]==0&&sensor[1]==1&&sensor[2]==0&&sensor[3]==0&&sensor[4]==0)
{
	error= -2;
}
else if(sensor[0]==0&&sensor[1]==1&&sensor[2]==1&&sensor[3]==0&&sensor[4]==0)
{
	error= -1;
}
else if(sensor[0]==0&&sensor[1]==0&&sensor[2]==1&&sensor[3]==0&&sensor[4]==0)
{
	error= 0;
}
else if(sensor[0]==0&&sensor[1]==0&&sensor[2]==1&&sensor[3]==1&&sensor[4]==0)
{
	error= 1;
}
else if(sensor[0]==0&&sensor[1]==0&&sensor[2]==0&&sensor[3]==1&&sensor[4]==0)
{
	error= 2;
}
else if(sensor[0]==0&&sensor[1]==0&&sensor[2]==0&&sensor[3]==1&&sensor[4]==1)
{
	error= 3;
}
else if(sensor[0]==0&&sensor[1]==0&&sensor[2]==0&&sensor[3]==0&&sensor[4]==1)
{
	error= 4;
}
else if(sensor[0]==1&&sensor[1]==1&&sensor[2]==1&&sensor[3]==1&&sensor[4]==1)
{
  error= 0;
}
else if(sensor[0]==1&&sensor[1]==1&&sensor[2]==1&&sensor[3]==0&&sensor[4]==0)
{
  error= -6;//aggressive turning
}
else if(sensor[0]==0&&sensor[1]==0&&sensor[2]==1&&sensor[3]==1&&sensor[4]==1)
{
  error= 6;//aggressive turning
}
else if(sensor[0]==1&&sensor[1]==0&&sensor[2]==1&&sensor[3]==0&&sensor[4]==0)
{
  error= -6;
  motors_write(-maxspeed/2,maxspeed);delay(100);//aggressive turning//exp
}
else if(sensor[0]==0&&sensor[1]==0&&sensor[2]==1&&sensor[3]==0&&sensor[4]==1)
{
  error= 6;
  motors_write(maxspeed,-maxspeed/2);delay(100);//aggressive turning//exp
}
else if(sensor[0]==0&&sensor[1]==0&&sensor[2]==0&&sensor[3]==0&&sensor[4]==0)
{
  if(preverror<0)
  {
    error=-6;//experimental
  }
  else if(preverror>0)
  {
    error=6;
  }
}
else 
{
  error=0;
  i=0;
}
//BTSerial.print(error);

}

bool todigital(int a)
{
	int middle=(upper_read+lower_read)/2;
	if(a>middle)
		return 1;
	else
		return 0;
}
void calibrate()
{
	long upper_sum=0,lower_sum=0;
	for(int i=0;i<50;i++)
	{
		upper_sum+=analogRead(2);//middle sensor on black
		lower_sum+=(analogRead(0)+analogRead(4))/2;//side sensors on black
	}
	upper_read=upper_sum/50;
	lower_read=lower_sum/50;
	Serial.print(upper_read);
	Serial.print(", ");
	Serial.println(lower_read);
}
void read_const()
{
  int add=0;
  //BTSerial.println("read");
  EEPROM.get(add,kp);
  add+=sizeof(float);
  EEPROM.get(add,ki);
  add+=sizeof(float);
  EEPROM.get(add,kd);
  add+=sizeof(float);
  EEPROM.get(add,sp);
  add+=sizeof(float);
  EEPROM.get(add,kr);
  add+=sizeof(float);
  EEPROM.get(add,maxspeed);
  add+=sizeof(int);
}
void write_const()
{
  //BTSerial.println("written");
  int add=0;
  EEPROM.put(add,kp);
  add+=sizeof(float);
  EEPROM.put(add,ki);
  add+=sizeof(float);
  EEPROM.put(add,kd);
  add+=sizeof(float);
  EEPROM.put(add,sp);
  add+=sizeof(float);
  EEPROM.put(add,kr);
  add+=sizeof(float);
  EEPROM.put(add,maxspeed);
  add+=sizeof(int);
}
void bluetooth_receive()
{
	if(BTSerial.available()>0)
	{
		char test=BTSerial.read();
		switch(test)
		{
			case 'r':
					manual_control=1;
					remote_control();
					break;
			case 'a':
					manual_control=0;
					break;
			case 'm':
            //BTSerial.println("motor var received!");
						Speed=BTSerial.readStringUntil(',').toInt()-255;
						turn=BTSerial.readStringUntil(',').toInt()-255;
					  break;
			case 's':
						kp=BTSerial.readStringUntil(',').toFloat();
						ki=BTSerial.readStringUntil(',').toFloat();
						kd=BTSerial.readStringUntil(',').toFloat();
						kr=BTSerial.readStringUntil(',').toFloat();
						sp=BTSerial.readStringUntil(',').toFloat()-4;
						maxspeed=BTSerial.readStringUntil(',').toInt();
					write_const();
					break;
			case 'c':
						calibrate();
						break;
			case 'x':
						reset();
						break;
			case 'i':
						i=0;
						break;
			default:
					manual_flush();
		}
	}
}
void remote_control()
{
  int lspeed,rspeed;
	while(manual_control)
	{
    lspeed=Speed-turn;
    rspeed=Speed+turn;
    if(lspeed>255)
      lspeed=255;
    else if(lspeed<-255)
      lspeed=-255;
    if(rspeed>255)
      rspeed=255;
    else if(rspeed<-255)
      rspeed=-255;
  
		motors_write(lspeed,rspeed);
		bluetooth_receive();
	}

}
void manual_flush()
{
	while(BTSerial.available()>0)
	{
		char a=BTSerial.read();
	}
}

void send_settings()
{
	BTSerial.print("Kp= ");
	BTSerial.print(kp);
	BTSerial.print("Ki= ");
	BTSerial.print(ki);
	BTSerial.print("Kd= ");
	BTSerial.print(kd);
	BTSerial.print("Kr= ");
	BTSerial.print(kr);
	BTSerial.print("SP= ");
	BTSerial.print(sp);
	BTSerial.print("Maxspeed= ");
	BTSerial.println(maxspeed);

}

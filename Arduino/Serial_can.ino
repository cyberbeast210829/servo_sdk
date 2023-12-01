// demo: CAN-BUS Shield, Serial and CAN  LSY
#include <mcp_can.h>
#include <motctrl_prot.h>
#include <SPI.h>

unsigned char Flag_Recv = 0;
unsigned char len = 8, chose = 0, index = 0, i=0, j=0;
unsigned char buf[8];
unsigned char tmp[8];
bool stringComplete = false;  // whether the string is complete
uint8_t type;
float data;
int32_t duration;

char outData[80];
char position_str[10]; // current position returned, in RAD for output shaft
char speed_str[10]; // current speed returned, in RPM for output shaft
char torque_str[10]; // current torque returned, in Amper, to convert which to N.m for output shaft, multiply by TORQUE_CONSTANT*GEAR_RATIO 

int8_t temp; // current temperature returned, in Centigrade
float position; // current position returned, in RAD for output shaft
float speed; // current speed returned, in RPM for output shaft
float torque; // current torque returned, in Amper, to convert which to N.m for output shaft, multiply by TORQUE_CONSTANT*GEAR_RATIO

String inputString = "";      // a String to hold incoming data
String outputString = "";     // a String to hold the output data

void setup()
{
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  // init can bus, baudrate: 500k
  if(CAN.begin(CAN_500KBPS) == CAN_OK) Serial.print("can init ok!!\r\n");
  else Serial.print("Can init fail!!\r\n");
  attachInterrupt(0, MCP2515_ISR, FALLING);     // start interrup
}

void MCP2515_ISR()
{
    Flag_Recv = 1;
}

void loop()
{
  if(Flag_Recv)                           // check if get data
  {
    Flag_Recv = 0;                        // clear flag
    CAN.readMsgBuf(&len, buf);            // read data,  len: data length, buf: data buf
    switch(buf[0])                        //Here are a few examples of how to process data
    {
      case 0x91:Serial.println("Start Motor");break;
      
      case 0x92:Serial.println("Stop Motor");break;

      case 0x93:MCResTorqueControl(buf, &temp, &position, &speed, &torque);chose = 1;break;
	
			case 0x94:MCResSpeedControl(buf, &temp, &position, &speed, &torque);chose = 1;break;
													
			case 0x95:MCResPositionControl(buf, &temp, &position, &speed, &torque);chose = 1;break;

      default: break;
    }
    switch(chose)
    {
      case 1:   chose = 0;
                dtostrf(position,1,2,position_str);
                dtostrf(speed,1,2,speed_str);
                dtostrf(torque,1,2,torque_str);
                sprintf(outData ,"temp: %d position: %s speed: %s torque: %s",temp , position_str, speed_str, torque_str);
                outputString = String(outData);
                Serial.println(outputString);
                break;
            
      default: break;
    }
  }
  // print the string when a newline arrives:
  if (stringComplete) {
    switch(type)
		{
			case 5:	MCReqPositionControl(tmp, data, duration);break;
											
			case 4: MCReqSpeedControl(tmp, data, duration);break;
														
			case 3: MCReqTorqueControl(tmp, data, duration);break;
								
			case 2: MCReqStopMotor(tmp);break;
								
			case 1: MCReqStartMotor(tmp);break;
								
			default: j=0;break;
		}
    // send data:  id , standrad flame, data len = 8, stmp: data buf
    CAN.sendMsgBuf(0x01, 0, 8, tmp);
    delay(30);                       // send data per 100ms
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if (inChar == ' ') {
        j++;
        switch(j)
        {
          case 1: type = inputString.toInt();break; 
          case 2: data = inputString.toFloat();break;                    
          default: break;
        }
        inputString = "";
    }
    inputString += inChar;
    if (inChar == '\n') {
      duration = inputString.toInt();
      stringComplete = true;
      j = 0;
    }
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
  }
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
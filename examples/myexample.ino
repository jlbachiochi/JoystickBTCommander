//rForward 6,x or 7,x (6=moves robot forward, 7=backward)
//  where  x=number of pixels to move (0-255)
//  return rBumper, rFeel, rSense, 0, 0
//rSense linesensor (bit0=right, bit1=middle, bit2=left)
//rBumper switch (bit0=back, bit1=right, bit2=front, bit3=left)
//rFeel infrared (bit0=90right, bit1=45right, bit2=front, bit3=45left, bit4=90right)
//rLocate 3,x initiailizes RROS
//  where x=horizontal position (not used)
//rTurn 12,x or 13,x (12=turns robot right, 13=left)
//  where  x=number of degrees to move (0-180)
//  return rBumper, rFeel, rSense, 0, 0
#include <SoftwareSerial.h>
 
#define STX 0x02
#define ETX 0x03
#define left 1
#define right 0
#define ETX 0x03
#define rEncoder_Pin 3
#define lEncoder_Pin 2
#define dTX_Pin 4
#define dRX_Pin 5
#define    SLOW         750             // Datafields refresh rate (ms)
#define    FAST         250             // Datafields refresh rate (ms)
SoftwareSerial DebugPort(dRX_Pin, dTX_Pin); // RX, TX
byte cmd[8];                            // bytes received
byte buttonStatus;                      // new button status of Android device
byte lastButtonStatus = 0;              // last button status of Android device
long previousMillis = 0;                // will store last time Buttons status was updated
long sendInterval = SLOW;
String displayStatus = "xxxx";
int joyX;
int lastJoyX = -1;
int joyY;
int lastJoyY = -1;
volatile byte newButtonValue;
volatile byte newJoyValue[8];
//
const boolean DEBUG = true;             // LCD
//                .------------ use Bluetooth Commander
//                |.-----------
//                ||.----------
//                |||.--------- Display Odometer, Target, Speed
//                ||||.-------- Display PWM,      Ramp,   Speed
//                |||||.------- Display Percent,  Ramp,   Motor
//                ||||||.------ Display Robot Basic Command
//                |||||||.----- Bluetooth Commander Comm
byte myDebug =  0b10000001;
//
const int MIN_MOTOR_VALUE = 10;
const int MAX_MOTOR_VALUE = 245;
const int ZERO_MOTOR_VALUE = 128;

String myString = "                    ";
int Motor_PWM[] = { 6, 11 };            // rENA,lENB
int Motor_DIRA[] = { 7, 10  };          // rIN1,lIN4
int Motor_DIRB[] = { 8, 9 };            // rIN2,lIN3
int Motor[] = { 0, 0 };
int Percent[] = { 0, 0 };
const int SERIAL_READ_BUFFER_SIZE = 256;
char* serialReadBuffer;
int serialReadBufferIndex = 0;
int status;
int laststatus;
unsigned long previousTime = millis();
unsigned long presentTime;
volatile long Odometer[] = {0,0};
int Ramp[] = {0,0};
int PotentialSpeed[] = {0,0}; 
int Target[] = {0,0};
int StepFactorPerPixel = 10;
int StepFactorPerDegree = 10;
int TurnDegree = 0;
//int index;
volatile boolean newButton;
volatile boolean newJoy;
  
byte Command;
byte CommandValue;
byte myBits;
byte row;
byte column;
int ResponseValue1=0; // rBumper switch (bit0=back, bit1=right, bit2=front, bit3=left)
int ResponseValue2=0; // rFeel infrared (bit0=90right, bit1=45right, bit2=front, bit3=45left, bit4=90right)
int ResponseValue3=2; // rSense linesensor (bit0=right, bit1=left, bit2=midddle)
int ResponseValue4=0;
int ResponseValue5=0;

#define BTPort Serial
/*--------------------------------------
// start - setup()
//------------------------------------*/
void setup()
{
  BTPort.begin(19200);                  // maximum for UNO
  while (!BTPort);
  BTPort.println("SR15 RROS for Arduino");
  sendBlueToothData();

  //
  DebugPort.begin(9600);
  while (!DebugPort);
  DebugPort.print("?f");              // cls
  DebugPort.print("?a");              // home
  DebugPort.print("SR15 Comm Test");
  pinMode( LED_BUILTIN, OUTPUT );
  pinMode( Motor_PWM[0], OUTPUT );
  pinMode( Motor_DIRA[0], OUTPUT );
  pinMode( Motor_DIRB[0], OUTPUT );
  pinMode( Motor_PWM[1], OUTPUT );
  pinMode( Motor_DIRA[1], OUTPUT );
  pinMode( Motor_DIRB[1], OUTPUT );
  digitalWrite( Motor_PWM[0], LOW );
  digitalWrite( Motor_DIRA[0], LOW );
  digitalWrite( Motor_DIRB[0], LOW );
  digitalWrite( Motor_PWM[1], LOW );
  digitalWrite( Motor_DIRA[1], LOW );
  digitalWrite( Motor_DIRB[1], LOW );
  Motor[0] = 0;
  Motor[1] = 0;  
  attachInterrupt(digitalPinToInterrupt(rEncoder_Pin), rEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(lEncoder_Pin), lEncoderInterrupt, CHANGE);
  delay(1000);
  while(BTPort.available())
  {
    BTPort.read();
  }
}
/*--------------------------------------
// end - setup()
//------------------------------------*/
/*--------------------------------------
// start - rEncoderInterrupt()
//------------------------------------*/
void rEncoderInterrupt()
{
  if(Motor_DIRB[right])                 // forward
  {
    Odometer[right]++; 
  }
  else                                  // backward
  {
    Odometer[right]--;    
  }
}
/*--------------------------------------
// end - rEncoderInterrupt()
//------------------------------------*/
/*--------------------------------------
// start - lEncoderInterrupt()
//------------------------------------*/
void lEncoderInterrupt()
{
  if(Motor_DIRB[left])                 // forward
  {
    Odometer[left]++; 
  }
  else                                  // backward
  {
    Odometer[left]--;    
  }
}
/*--------------------------------------
// end - rEncoderInterrupt()
//------------------------------------*/
/*--------------------------------------
// start - serialEvent()
//------------------------------------*/
void serialEvent() 
{
// 
  while(BTPort.available())
  {   
    if(BTPort.available()>2)
    {
      cmd[0] = BTPort.read(); 
      if(cmd[0] = STX)
      {
        cmd[1] = BTPort.read();
        cmd[2] = BTPort.read();
        if(cmd[2] == ETX)
        {
          if(cmd[1] > 64)  // if data > 64 ('A' to ...)
          {
            newButtonValue = cmd[1];        // save Button data
            newButton = true;
            digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
          }
        }
        else
        {
          while(BTPort.available()<5);
          cmd[3] = BTPort.read(); 
          cmd[4] = BTPort.read();
          cmd[5] = BTPort.read();
          cmd[6] = BTPort.read(); 
          cmd[7] = BTPort.read();
          if(cmd[7] == ETX)
          {
            newJoyValue[1] = cmd[1];        // save Joy data
            newJoyValue[2] = cmd[2];
            newJoyValue[3] = cmd[3];
            newJoyValue[4] = cmd[4];
            newJoyValue[5] = cmd[5];
            newJoyValue[6] = cmd[6];
            newJoy = true;
            digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
          }
        }
      }
    }
  }
  while(BTPort.available())
  {
    BTPort.read();
  }
}
//
/*--------------------------------------
// start - serialEvent()
//------------------------------------*/
/*--------------------------------------
// start - loop()
//------------------------------------*/
void loop()
{
  if(get_command())                     // true
  {
    if(myDebug & 128)
    {
      parse_BTCommander();
    }
    else
    {
      parse_command();         
    }
  }
  //
  presentTime = millis();
  if (abs(presentTime - previousTime )>= 100) 
  {
    previousTime = presentTime;
    if(myDebug & 128)                 // joystick Commander
    {
      update_Motors();
      robotBasicStatus();
    }
    else
    {
      update_Speed(); 
      update_Odometers();
      update_Motors();
      robotBasicStatus();
    }
  }
  //
}
/*--------------------------------------
// end - loop()
//------------------------------------*/
/*--------------------------------------
// start - get_command()
//------------------------------------*/
boolean get_command()
{
  if(myDebug & 128)
  {
    return bluetoothCommander();
  }
  else
  {
    return robotBasic();
  }
}
/*--------------------------------------
// end - get_command()
//------------------------------------*/
/*--------------------------------------
// start - robotBasic()
//------------------------------------*/
boolean robotBasic()
{  
  if(myDebug & 1)
  {
    //
  }
  if (BTPort.available()>1)
  {
    Command = BTPort.read();
    CommandValue = BTPort.read();
    while (BTPort.available()>0)
    {
      Command = CommandValue; 
      CommandValue = BTPort.read();
    } 
    return true;
  }
  else
  {
    return false;
  }
}
/*--------------------------------------
// end - robotBasic()
//------------------------------------*/
/*--------------------------------------
// start - bluetoothCommander()
//------------------------------------*/
boolean bluetoothCommander()
{
  boolean OK = false;
  if(myDebug & 1)
  {
    myString = "Bluetooth Commander ";
    row = 0;
    column = 0;
    positionCursor();    
    DebugPort.print(myString);
  }
  if(newJoy)
  {
    newJoy = false;
    if(getJoystickState())     // 8 Bytes  ex: < STX "200" "180" ETX >
    {
      if(lastJoyX != joyX)
      {
        lastJoyX = joyX;
        OK = true;
      }
      if(lastJoyY != joyY)
      {
        lastJoyY = joyX;
        OK = true;
      }
      if(myDebug & 1)
      {
        if(OK)
        {
          myString = "x = ";
          myString = myString + String(joyX);
          addBlankSpaces(10);
          myString = myString + "y = ";
          myString = myString + String(joyY);
          addBlankSpaces(20);
          row = 1;
          column = 0;
          positionCursor();    
          DebugPort.print(myString);
        }
      }
    }
  }
  if(newButton)
  {
    newButton = false;
    getButtonState();    // 3 Bytes  ex: < STX "C" ETX >
    if(buttonStatus != lastButtonStatus)
    {
      lastButtonStatus = buttonStatus;
      OK = true;
      if((myDebug & 1) && OK)
      {
        myString = "Buttons = 0b";
        for(int i=5; i>=0; i--)
        {
          if(buttonStatus & 1<<i)
          {
            myString = myString + "1";
          }
          else
          {
            myString = myString + "0";        
          }
        }
        addBlankSpaces(20);
        row = 2;
        column = 0;
        positionCursor();    
        DebugPort.print(myString);
      }
    }
  }
  sendBlueToothData();
  if(OK)
  { 
    return true;
  }
}
/*--------------------------------------
// end - bluetoothCommander()
//------------------------------------*/
/*--------------------------------------
// start - getJoystickState()
//------------------------------------*/
boolean getJoystickState()    
{
  joyX = (newJoyValue[1]-48)*100 + (newJoyValue[2]-48)*10 + (newJoyValue[3]-48); // obtain the Int from the ASCII representation
  joyY = (newJoyValue[4]-48)*100 + (newJoyValue[5]-48)*10 + (newJoyValue[6]-48);
  joyX = joyX - 200;                    // Offset to avoid
  joyY = joyY - 200;                    // transmitting negative numbers
  //
  if(joyX<-100 || joyX>100 || joyY<-100 || joyY>100)     
  {
    return false;                       // commmunication error
  }
  //
  return true;
}
/*--------------------------------------
// end - getJoystickState()
//------------------------------------*/
/*--------------------------------------
// start - getButtonState()
//------------------------------------*/
void getButtonState()  
{
  boolean OK = true;
  switch (newButtonValue) 
  {
    case 'A':                           //  BUTTON #1 
      buttonStatus = buttonStatus | B000001;  // ON
      break;
    case 'B':
      buttonStatus = buttonStatus & B111110;  // OFF
      break;
    case 'C':                            // BUTTON #2
      buttonStatus |= buttonStatus | B000010;  // ON
      break;
    case 'D':
      buttonStatus = buttonStatus & B111101; // OFF
      break;
    case 'E':                             // BUTTON #3
      buttonStatus = buttonStatus | B000100;  // ON
      break;
    case 'F':
      buttonStatus = buttonStatus & B111011;  // OFF
      break;
    case 'G':                             // BUTTON #4
      buttonStatus = buttonStatus | B001000;  // ON
      break;
    case 'H':
      buttonStatus = buttonStatus & B110111;  // OFF
      break;
    case 'I':                             // BUTTON #5
      buttonStatus = buttonStatus | B010000;  // ON
      break;
    case 'J':
      buttonStatus = buttonStatus & B101111; // OFF
      break;
    case 'K':                             // BUTTON #6
      buttonStatus = buttonStatus | B100000;  // ON
      break;
    case 'L':
      buttonStatus = buttonStatus & B011111; // OFF
      break;
    default:
      break;
  }
}
/*--------------------------------------
// end - getButtonState()
//------------------------------------*/
/*--------------------------------------
// start - sendBlueToothData()
//------------------------------------*/
void sendBlueToothData()  
{
  static long previousMillis = 0;                            
  long currentMillis = millis();
  if(currentMillis - previousMillis > sendInterval) 
  {   // send data back to smartphone
    previousMillis = currentMillis;
    // Data frame transmitted back from Arduino to Android device:
    // < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >  
    // < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example
    BTPort.print((char)STX);                                             // Start of Transmission
    BTPort.print(getButtonStatusString());  
    BTPort.print((char)0x1);   // buttons status feedback
    BTPort.print(GetdataInt1());            
    BTPort.print((char)0x4);   // datafield #1
    BTPort.print(GetdataFloat2());          
    BTPort.print((char)0x5);   // datafield #2
    BTPort.print(displayStatus);                                         // datafield #3
    BTPort.print((char)ETX); 
  }  
}
/*--------------------------------------
// end - sendBlueToothData()
//------------------------------------*/
/*--------------------------------------
// start - getButtonStatusString()
//------------------------------------*/
String getButtonStatusString()  
{
  String bStatus = "";
  for(int i=0; i<6; i++)  
  {
   if(buttonStatus & (B100000 >>i))      
   {
      bStatus = bStatus + "1";
   }
   else                                  
   {
      bStatus = bStatus + "0";
   }
  }
  return bStatus;
}
/*--------------------------------------
// end - getButtonStatusString()
//------------------------------------*/
/*--------------------------------------
// start - GetdataInt1()
//------------------------------------*/
int GetdataInt1()  
{              // Data dummy values sent to Android device for demo purpose
  static int i = -30;              // Replace with your own code
  i = i + 1;
  if(i > 0)    
  {
    i = -30;
  }
  return i;  
}
/*--------------------------------------
// end - GetdataInt1()
//------------------------------------*/
/*--------------------------------------
// start - GetdataFloat2()
//------------------------------------*/
float GetdataFloat2()  
{           // Data dummy values sent to Android device for demo purpose
  static float i=50;               // Replace with your own code
  i = i - .5;
  if(i < -50)    
  {
    i = 50;
  }
  return i;  
}
/*--------------------------------------
// end - GetdataFloat2()
//------------------------------------*/
/*--------------------------------------
// start - parse_command()
//------------------------------------*/
void parse_command()
{
  switch(Command) 
  {
  case 0: // myDebug
    myDebug = CommandValue;
    break;
  case 3: // rLocate
    if(myDebug & 2)
    {
      myString = "Loc:";
      myString = myString + String(CommandValue);
      addBlankSpaces(10);
    }
    Odometer[0] = 0;
    Odometer[1] = 0;
    Target[0] = 0;
    Target[1] = 0;    
    break;
  case 6: // rForward
    if(myDebug & 2)
    {
      myString = "Fwd:";
      myString = myString + String(CommandValue);
      addBlankSpaces(10);
    }  
    Target[0] = Target[0] + (CommandValue * StepFactorPerPixel);  // Pixel = how far to move
    Target[1] = Target[1] + (CommandValue * StepFactorPerPixel);  // Pixel = how far to move
    PotentialSpeed[0]=100; // 100%
    PotentialSpeed[1]=100; // 100%
    break;
  case 7: // rForward (reverse)
    if(myDebug & 2)
    {
      myString = "Rev:";
      myString = myString + String(CommandValue);
      addBlankSpaces(10);
    }  
    Target[0] = Target[0] + (CommandValue * -StepFactorPerPixel);  // Pixel = how far to move
    Target[1] = Target[1] + (CommandValue * -StepFactorPerPixel);  // Pixel = how far to move   
    PotentialSpeed[0]=100; // 100% potential speed
    PotentialSpeed[1]=100; // 100% potential speed
    break;
  case 12: // rTurn (right)
    if(myDebug & 2)
    {
      myString = "TurnR:";
      myString = myString + String(CommandValue);
      addBlankSpaces(10);
    }  
    Target[0] = Target[0] + (CommandValue * StepFactorPerDegree);  // how far to turn
    Target[1] = Target[1] + (CommandValue * -StepFactorPerDegree);  // how far to turn
    PotentialSpeed[0]=100; // 100% potential speed
    PotentialSpeed[1]=100; // 100% potential speed    
    break; 
  case 13: // rTurn (left)
    if(myDebug & 2)
    {
      myString = "TurnL:";
      myString = myString + String(CommandValue);
      addBlankSpaces(10);
    }  
    Target[0] = Target[0] + (CommandValue * -StepFactorPerDegree);  // how far to turn
    Target[1] = Target[1] + (CommandValue * StepFactorPerDegree);  // how far to turn    
    PotentialSpeed[0]=100; // 100% potential speed
    PotentialSpeed[1]=100; // 100% potential speed     
    break; 
  default: // ?
    if(myDebug & 2)
    {
      myString = String(Command);
      myString = myString + "??";
      myString = myString + String(CommandValue);
      addBlankSpaces(10);  
    }
    break;         
  }
  if(!myDebug & 128)
  {
    BTPort.write(ResponseValue1);
    BTPort.write(ResponseValue2);  
    BTPort.write(ResponseValue3);
    BTPort.write(ResponseValue4);
    BTPort.write(ResponseValue5);
  }
  if(myDebug & 2)
  {
    myString = myString + "Bump:0x";
    myString = myString + String(ResponseValue1,HEX);
    addBlankSpaces(20);
    row = 0;
    column = 0;
    positionCursor();    
    DebugPort.print(myString);
    //
    myString = "Feel:0x";
    myString = myString + String(ResponseValue2,HEX);
    addBlankSpaces(10);
    myString = myString + "Line:0x";
    myString = myString + String(ResponseValue3,HEX);
    addBlankSpaces(20);
    row = 1;
    column = 0;
    positionCursor();    
    DebugPort.print(myString);
    //  
    myString = "Cmd4:0x";
    myString = myString + String(ResponseValue4,HEX);
    addBlankSpaces(10);          
    myString = myString + "Cmd5:0x";
    myString = myString + String(ResponseValue3,HEX);
    addBlankSpaces(20);
    row = 2;
    column = 0;
    positionCursor();    
    DebugPort.print(myString);
    //
    myString = "                    ";
    row = 3;
    column = 0;
    positionCursor();    
    DebugPort.print(myString);                  
  } 
}
/*--------------------------------------
// end - parse_command()
//------------------------------------*/
/*--------------------------------------
// start - parse_BTcommand()
//------------------------------------*/
void parse_BTCommander()
{
  xyToDifferencial();
  if(buttonStatus & 1)
  {
    myDebug = myDebug | 1;
  }
  else
  {
    myDebug = myDebug & 0b11111110;
  }
  if(buttonStatus & 2)
  {
    myDebug = myDebug | 2;
  }
  else
  {
    myDebug = myDebug & 0b11111101;
  }
  if(buttonStatus & 4)
  {
    myDebug = myDebug | 4;
  }
  else
  {
    myDebug = myDebug & 0b11111011;
  }
  if(buttonStatus & 8)
  {
    myDebug = myDebug | 8;
  }
  else
  {
    myDebug = myDebug & 0b11110111;
  }
  if(buttonStatus & 16)
  {
    myDebug = myDebug | 16;
  }
  else
  {
    myDebug = myDebug & 0b11101111;
  }
}
/*--------------------------------------
// end - parse_BTcommand()
//------------------------------------*/
/*--------------------------------------
// start - update_Speed()
//------------------------------------*/
void update_Speed()
{
  update_PWM(left);
  update_PWM(right);
}
/*--------------------------------------
// end - update_Speed()
//------------------------------------*/
/*--------------------------------------
// start - update_PWM()
//------------------------------------*/
void update_PWM(int index)  // move ramp toward speed
{
  if (PotentialSpeed[index]>Ramp[index]) // if speed > ramp
  {
    Ramp[index]++;  // ramp up
  }
  else
  {
    if (PotentialSpeed[index]<Ramp[index]) // if speed < ramp
    {
      Ramp[index]--;  // ramp down 
    }
    else
    {
      // do nothing
    }
  }
}
/*--------------------------------------
// end - update_PWM()
//------------------------------------*/
/*--------------------------------------
// start - update_Odometers()
//------------------------------------*/
void update_Odometers()
{
  update_Odometer(left);
  update_Odometer(right);
}
/*--------------------------------------
// end - update_Odometers()
//------------------------------------*/
/*--------------------------------------
// start - update_Odometer()
//------------------------------------*/
void update_Odometer(int index) // use ramp to simulate wheel encoders
{
  if (Motor_DIRA[index] == HIGH)   // Forward
  {
    if (Odometer[index]>=Target[index]) // if we reached the target
    {
      PotentialSpeed[index]=0;  // stop moving
      Odometer[index] = 0;
      Target[index] = 0;
    }
  }
  if (Motor_DIRB[index] == HIGH)  // Reverse
  {
    if (Odometer[index]<=Target[index]) // if we reached the target
    {
      PotentialSpeed[index]=0;  // stop moving
      Odometer[index] = 0;
      Target[index] = 0;
    }
  }
}
/*--------------------------------------
// end - update_Odometer()
//------------------------------------*/
/*--------------------------------------
// start - update_Motors()
//------------------------------------*/
void update_Motors()
{
  update_motor(left);
  update_motor(right);
}
/*--------------------------------------
// end - update_Motors()
//------------------------------------*/
/*--------------------------------------
// start - update_motor()
//------------------------------------*/
void update_motor(int index) 
{
  if(Motor[index] == 0)                   // stopped
  {
    digitalWrite(Motor_DIRA[index], LOW);
    digitalWrite(Motor_DIRB[index], LOW);
    analogWrite(Motor_PWM[index], 0 );
  }
  else if(Motor[index] > 0 )             // forward 
  {
    digitalWrite(Motor_DIRB[index], LOW);
    digitalWrite(Motor_DIRA[index], HIGH);
    analogWrite(Motor_PWM[index], Motor[index]);
  }
  else                                  // backward
  {
    digitalWrite(Motor_DIRA[index], LOW);
    digitalWrite(Motor_DIRB[index], HIGH);
    analogWrite(Motor_PWM[index], -Motor[index]);   
  }
}
/*--------------------------------------
// end - update_motor()
//------------------------------------*/
/*--------------------------------------
// start - robotBasicStatus()
//------------------------------------*/
void robotBasicStatus()
{
  if(myDebug & 4)
  {
    myString = "Robot Basic  L - R  ";
    column = 0;
    row = 0;
    positionCursor();
    DebugPort.print(myString);
    //
    myString = "Percent";
    addBlankSpaces(8);
    myString = myString + String(Percent[0]);
    addBlankSpaces(14);
    myString = myString + String(Percent[1]);
    addBlankSpaces(20);
    column = 0;
    row = 1;
    positionCursor();
    DebugPort.print(myString);
    //
    myString = "Ramp";
    addBlankSpaces(8);
    myString = myString + String(Ramp[0]);
    addBlankSpaces(14);
    myString = myString + String(Ramp[1]);
    addBlankSpaces(20);
    column = 0;
    row = 2;
    positionCursor();
    DebugPort.print(myString);
    //
    myString = "Motor";
    addBlankSpaces(8);
    myString = myString + String(Motor[0]);
    addBlankSpaces(14);
    myString = myString + String(Motor[1]);
    addBlankSpaces(20);
    column = 0;
    row = 3;
    positionCursor();
    DebugPort.print(myString);
  }
  if(myDebug & 8)
  {
    myString = "Robot Basic  L - R  ";
    column = 0;
    row = 0;
    positionCursor();
    DebugPort.print(myString);
    //
    myString = "PWM";
    addBlankSpaces(8);
    myString = myString + String(Motor[0]);
    addBlankSpaces(14);
    myString = myString + String(Motor[1]);
    addBlankSpaces(20);
    column = 0;
    row = 1;
    positionCursor();
    DebugPort.print(myString);
    //
    myString = "Ramp";
    addBlankSpaces(8);
    myString = myString + String(Ramp[0]);
    addBlankSpaces(14);
    myString = myString + String(Ramp[1]);
    addBlankSpaces(20);
    column = 0;
    row = 2;
    positionCursor();
    DebugPort.print(myString);
    //
    myString = "Speed";
    addBlankSpaces(8);
    myString = myString + String(PotentialSpeed[0]);
    addBlankSpaces(14);
    myString = myString + String(PotentialSpeed[1]);
    addBlankSpaces(20);
    column = 0;
    row = 3;
    positionCursor();
    DebugPort.print(myString);    
  }
  if(myDebug & 16)
  {
    myString = "Robot Basic  L - R  ";
    column = 0;
    row = 0;
    positionCursor();
    DebugPort.print(myString);
    //
    myString = "Odometer";
    addBlankSpaces(8);
    myString = myString + String(Odometer[0]);
    addBlankSpaces(14);
    myString = myString + String(Odometer[1]);
    addBlankSpaces(20);
    column = 0;
    row = 1;
    positionCursor();
    DebugPort.print(myString);
    //
    myString = "Target";
    addBlankSpaces(8);
    myString = myString + String(Target[0]);
    addBlankSpaces(14);
    myString = myString + String(Target[1]);
    addBlankSpaces(20);
    column = 0;
    row = 2;
    positionCursor();
    DebugPort.print(myString);
    //
    myString = "Speed";
    addBlankSpaces(8);
    myString = myString + String(PotentialSpeed[0]);
    addBlankSpaces(14);
    myString = myString + String(PotentialSpeed[1]);
    addBlankSpaces(20);
    column = 0;
    row = 3;
    positionCursor();
    DebugPort.print(myString);     
  }
}
/*--------------------------------------
// end - robotBasicStatus()
//------------------------------------*/
/*--------------------------------------
// start - positionCursor()
//------------------------------------*/
void positionCursor()
{
  DebugPort.print("?y");
  DebugPort.print(row);
  DebugPort.print("?x");
  if(column < 10)
  {
    DebugPort.print("0");    
  }
  DebugPort.print(column);  
}
/*--------------------------------------
// end - positionCursor()
//------------------------------------*/
/*--------------------------------------
// start - addBlankSpaces()
//------------------------------------*/
void addBlankSpaces(byte s)
{
  while(myString.length() < s)
  {
    myString = myString + " ";
  }
}
/*--------------------------------------
// end - addBlankSpaces()
//------------------------------------*/
/*--------------------------------------
// start - xyToDifferencial()
//------------------------------------*/
void xyToDifferencial()
{
  int x = joyX * -1;
  int y = joyY;
  int rPlusL = (100 - abs(x)) * (y / 100) + y;
  int rMinusL = (100 - abs(y)) * (x / 100) + x;
  Percent[right] = ((rPlusL + rMinusL) / 2 * 2.5);
  constrainMotor(right);
  Percent[left] = ((rPlusL - rMinusL) / 2) * 2.5;
  constrainMotor(left);
  if(myDebug & 1)
  {
    myString = "l%=" + String(Percent[left]) + " r%=" + String(Percent[right]);
    addBlankSpaces(20);
    column = 0;
    row = 3;
    positionCursor();
    DebugPort.print(myString);
  }
}
/*--------------------------------------
// end - xyToDifferencial()
//------------------------------------*/
void constrainMotor(boolean index)
{
  Motor[index] = Percent[index];
  if(Motor[index] < (MIN_MOTOR_VALUE * -1))                // negative
  {
    constrain(Motor[index], (MIN_MOTOR_VALUE * -1), (MAX_MOTOR_VALUE * -1));
  }
  else if(Motor[index] > MIN_MOTOR_VALUE) // positive
  {
    constrain(Motor[index], MIN_MOTOR_VALUE, MAX_MOTOR_VALUE);
  }
  else
  {
    Motor[index] = 0;
  }
}


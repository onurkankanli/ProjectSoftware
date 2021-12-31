 #include <Servo.h>
Servo servo;

#define SHIFT_SRCLK 13  // SRCLK | SH_CP | CLOCK
#define SHIFT_RCLK  12  // RCLK  | ST_CP | LATCH
#define SHIFT_SER   11  // SER   | DS    | DATA
static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3

volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
int dig1 = 10;
int dig2 = 9;
int dig3 = 8;

int button = 4;
int btn_reset = 5;

//int reset_led = 7;
//int yellow_led = 6;

int redPin = A0;
int yellowPin = A1;

String masterkey = "654";
String mkey = "000";
String key = "";

String input1 = "";
String input2 = "";
String input3 = "";

int segment = 1 ;
int state = 0;        //int to hold current state
int buttonPoll = 0; //int to hold button state
int attempt = 0;
int count = 0;    //int to hold current state
int old = 0;      //int to hold last state

boolean turn_on = false;
boolean safe_is_open = false;
const int LONG_PRESS_TIME  = 1000;

int change_password = 0;
int lastState = LOW;
unsigned long pressedTime  = 0;
bool isPressing = false;
bool isLongDetected = false;
int button_reset_Poll = 0;

int Redled = LOW;
int Yellowled = LOW;

void setup() {
  servo.attach(6);
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0, PinA, RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1, PinB, RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  Serial.begin(115200); // start the serial monitor link

  pinMode(SHIFT_RCLK, OUTPUT);
  pinMode(SHIFT_SRCLK, OUTPUT);
  pinMode(SHIFT_SER, OUTPUT);

  pinMode(dig1, OUTPUT);
  pinMode(dig2, OUTPUT);
  pinMode(dig3, OUTPUT);

  pinMode(dig1, OUTPUT); //set as output
  pinMode(dig2, OUTPUT); //set as output
  pinMode(dig3, OUTPUT); //set as output

  pinMode(btn_reset, INPUT); //set Reset btn as input
  pinMode(button, INPUT); //set as input  pinMode;

  //digitalWrite(yellow_led, LOW);
  //digitalWrite(reset_led, LOW);

  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);

  clearSegments();
  ServoClose();
}

// rotary encoder --
void PinA()
{
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && aFlag)
  {
    if (encoderPos == 0)
      encoderPos = 10;
    encoderPos --;
    Serial.println(encoderPos);
    bFlag = 0;
    aFlag = 0;

  }
  else if (reading == B00000100)
    bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

// rotary encoder ++
void PinB()
{
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag)
  {
    if (encoderPos == 9)
      encoderPos = -1;
    encoderPos ++; //increment the encoder's position count
    Serial.println(encoderPos);

    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1;
  sei();
}

void loop() {
  if ((Yellowled == LOW) || (Yellowled == HIGH && Redled == HIGH )) {
    buttonPoll = digitalRead(button); //every loop read the state of switchPin once

    if (buttonPoll == 1 && lastState == LOW) {
      Serial.println("Zmena");
      delay(300);
      count++;
      clearSegments();
    }
  }
  if (count == 0) {

    if (turn_on == false) {
      switchSeg(10);
    }
    digitalWrite(SHIFT_RCLK, LOW);
    shiftOut(SHIFT_SER, SHIFT_SRCLK, MSBFIRST, encoderPos);
    digitalWrite(SHIFT_RCLK, HIGH);
    input1 = encoderPos;
  }

  if (count == 1) {

    if (turn_on == false) {
      switchSeg(9);
    }
    digitalWrite(SHIFT_RCLK, LOW);
    shiftOut(SHIFT_SER, SHIFT_SRCLK, MSBFIRST, encoderPos);
    digitalWrite(SHIFT_RCLK, HIGH);
    input2 = encoderPos;
  }

  if (count == 2) {

    if (turn_on == false) {
      switchSeg(8);
    }
    digitalWrite(SHIFT_RCLK, LOW);
    shiftOut(SHIFT_SER, SHIFT_SRCLK, MSBFIRST, encoderPos);
    digitalWrite(SHIFT_RCLK, HIGH);
    input3 = encoderPos;

  }

  if (count == 3) 
  {
    key = input1 + input2 + input3;
    // if masterkey or key correct = open safe
    if (checkPass(key) == true || key == masterkey) 
    {

      if (safe_is_open == false) 
      {
        Serial.println("Safe Open");
        openSafe();
      }

      // change password
      button_reset_Poll = digitalRead(btn_reset);
      buttonPoll = digitalRead(button); //every loop read the state of switchPin once

      if (button_reset_Poll == 1) 
      {
        Serial.println("Chaning password");
        ServoClose();
        count = 0;
        change_password = 1;
        Redled = HIGH;
        setColorRed(0); // red
        //digitalWrite(reset_led, HIGH);
        delay(300);

      }
      // this si when safe is open
      if (lastState == HIGH  && buttonPoll == LOW) // button is pressed
      {       
        pressedTime = millis();
        isPressing = true;
        isLongDetected = false;
      } 
      else if(lastState == LOW && buttonPoll == HIGH) 
      { 
        isPressing = false; // button is release
      }

      // for closing the safe
      if (isPressing == true && isLongDetected == false) 
      {
        long pressDuration = millis() - pressedTime;

        if ( pressDuration > LONG_PRESS_TIME) 
        {
          ServoClose();
          setColorYellow(255);
          closeSafe();
          
          delay(300);
          isLongDetected = true;

        }
      }

      // save the the last state
      lastState = buttonPoll;

    } 
    else 
    {
      if (change_password == 0)
      {
        attempt++;
        for (int i = 1; i < 4; i++) 
        {
          setColorRed(0); // red
          delay(100);
          setColorRed(255); // red
          delay(100);
        }
        count = 0;
        Serial.println("Wrong pass");
      }
    }
  }


  // changing the password
  if ((count == 3) && ( change_password == 1)) {
    Serial.println("RESET PASWWWWWWWWOOOOOOORD");
    mkey = input1 + input2 + input3;
    Serial.println(key);
    clearSegments();

    Yellowled = LOW;
    Redled = LOW;

    setColorRed(0); // red
    setColorYellow(0);

    for (int i = 1; i < 4; i++) 
    {
      setColorYellow(255);
      setColorRed(255); // red
      delay(750);
      setColorRed(0); // red
      setColorYellow(0);
      delay(750);
    }
    Yellowled = LOW;
    Redled = LOW;
    setColorYellow(255);
    setColorRed(255);

    //digitalWrite(yellow_led, LOW);
    //digitalWrite(reset_led, LOW);

    Serial.println("Set new password");
    attempt = 0;
    count = 0 ;
    safe_is_open = false;
    change_password = 0;
    closeSafe();
  }

  // if the passowrd is put 3 times wrong the safe is going to lock
  if (attempt == 3) 
  {
    Serial.println("Block na 1min");
    clearSegments();
    Redled = HIGH;
    setColorRed(0); // red
    //digitalWrite(reset_led, HIGH);
    delay(10000); // time for blocking the safe after 3 attempts
    Redled = LOW;
    setColorRed(255); // red
    //digitalWrite(reset_led, LOW);
    attempt = 0;
    count = 0;
  }

}
void switchSeg(int seg) 
{
  digitalWrite(seg, HIGH);
  Serial.println(seg);
  turn_on = true;

}


void clearSegments() 
{
  digitalWrite(dig1, LOW);
  digitalWrite(dig2, LOW);
  digitalWrite(dig3, LOW);
  turn_on = false;

}

//check password
boolean checkPass(String key) 
{
  if (key == mkey) return true;
  else return false;
}

// this is for saying that safe is open
void openSafe() 
{
  Yellowled = HIGH;
  ServoOpen();
  //digitalWrite(yellow_led, HIGH);
  setColorYellow(0);
  safe_is_open = true;
 
}

// this is for closing the safe
void closeSafe() 
{
  Yellowled = LOW;
  clearSegments();
  setColorYellow(255);
  attempt = 0;
  count = 0;
  safe_is_open = false;
  Serial.println("Safe Close");
}

void ServoOpen() 
{
  for (int angle = 0; angle < 180; angle++)
  {
    servo.write(angle);
    delay(15);
  }
}

void ServoClose() 
{
  for (int angle = 180; angle >= 0; angle--)
  {
    servo.write(angle);
    delay(15);
  }
}

void setColorRed(int red)
{
  red = 255 - red;
  analogWrite(redPin, red);
  Serial.println("LEEEEEED REEEEEEEEEEEEEEEEDDDDDDDDDDDDDDD");
}

void setColorYellow(int yellow)
{
  Serial.println("LEEEEEED YELLLLOOWWW");
  yellow = 255 - yellow;
  analogWrite(yellowPin, yellow);
}

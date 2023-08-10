#include <QTRSensors.h>
#include <Ultrasonic.h>
#include <LiquidCrystal.h>

#define Kp 0.15 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 5// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define MaxSpeed 250// max speed of the robot
#define BaseSpeed 200 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used

#define speedturn 180

#define rightMotor1 3
#define rightMotor2 4
#define rightMotorPWM 2
#define leftMotor1 6
#define leftMotor2 7
#define leftMotorPWM 5
#define motorPower 13
int dist = 0;

//int PWM_wall = 150;
int PWM = 180;
int true_count = 0, false_count = 0;
int node = 0;

void forward_normal(void);
void true_beep(void);
void false_beep(void);




// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 37, en = 35, d4 = 51, d5 = 49, d6 = 47, d7 = 45;
LiquidCrystal lcd(rs, en, d7, d6, d5, d4);



int PWM_wall = 150;
int left_wall_case = 0;
int right_wall_case = 0;
/*
   Pass as a parameter the trigger and echo pin, respectively,
   or only the signal pin (for sensors 3 pins), like:
   Ultrasonic ultrasonic(13);
*/
Ultrasonic ultrasonic(15, 14);
Ultrasonic ultrasonic1(18, 19);
Ultrasonic ultrasonic2(17, 16);

QTRSensorsRC qtrrc((unsigned char[]) {
  A7, A6, A5, A4, A3, A2, A1, A0
} , NUM_SENSORS, 2500, 15);

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);

  delay(3000);

  int i;
  for (int i = 0; i < 100; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
  {
    //comment this part out for automatic calibration
    /* if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
      {
        move(1, 120, 1);//motor derecho hacia adelante
        move(0, 120, 0);//motor izquierdo hacia atras
      }
      else
      {
        move(1, 120, 0);//motor derecho hacia atras
        move(0, 120, 1);//motor izquierdo hacia adelante
      }/*/
    qtrrc.calibrate();
    delay(20);
  }
  // wait();
  delay(1000); // wait for 2s to position the bot before entering the main loop
}

int lastError = 0;
unsigned int sensors[8];
int position = qtrrc.readLine(sensors);

void loop()
{
  //forward_normal();
  //delay(1000);
  //left_mudja();
  if (node == 0)
  {

    lfr();
  }
  else if (node == 2)
  {
    left_error();
    left_motion();
    if (((sensors[7]) > 800) && ((sensors[6]) > 800 ) && ((sensors[5]) > 800) && ((sensors[4]) > 800) && ((sensors[3]) > 800) && ((sensors[2]) > 800 ) && ((sensors[1]) > 800) && ((sensors[0]) > 800))
    {
      node++;
    }
  }
  else if (node == 1)
  {
    right_error();
    right_motion();
    if (((sensors[7]) > 800) && ((sensors[6]) > 800 ) && ((sensors[5]) > 800) && ((sensors[4]) > 800) && ((sensors[3]) > 800) && ((sensors[2]) > 800 ) && ((sensors[1]) > 800) && ((sensors[0]) > 800))
    {
      node++;
    }
  }
  else if (node == 3)
  {
    lfr_1();
    if (((sensors[7]) > 800) && ((sensors[6]) > 800 ) && ((sensors[5]) > 800) && ((sensors[4]) > 800) && ((sensors[3]) > 800) && ((sensors[2]) > 800 ) && ((sensors[1]) > 800) && ((sensors[0]) > 800))
    {
      node++;
    }

  }
  else if (node == 3)
  {
    lcd.begin(16, 2);
    // Print a message to the LCD.
    lcd.print("true= ");
    lcd.setCursor(6, 0);
    lcd.print(true_count);
    lcd.setCursor(9, 0);
    lcd.print("false= ");
    lcd.setCursor(15, 0);

    lcd.print(false_count);

    lcd.setCursor(0, 1);
    lcd.print("dist");
    lcd.setCursor(5, 1);
    lcd.print(dist);
    while (1)
    {
      rukja();
    }


  }

}

void lfr()
{
  position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.

  
    /* if (((sensors[7]) > 800) && ((sensors[0]) > 800 ) && ((sensors[3]) < 400) && ((sensors[4]) < 400))
    {
     while (((sensors[7]) > 800) && ((sensors[0]) > 800 ) && ((sensors[3]) < 400) && ((sensors[4]) < 400))
     {
       forward_normal();
     }
     delay(2);
     if (((sensors[7]) < 400) && ((sensors[0]) < 400 ) && ((sensors[3]) < 400) && ((sensors[4]) < 400))
     {
       while (((sensors[7]) < 400) && ((sensors[0]) < 400 ) && ((sensors[3]) < 400) && ((sensors[4]) < 400))
       {
         forward_normal();
       }
       delay(2);
       true_beep();
       true_count++;
     }
     else if (((sensors[7]) < 400) && ((sensors[0]) < 400 ) && ((sensors[3]) > 800) && ((sensors[4]) > 800))
     {
       while (((sensors[7]) < 400) && ((sensors[0]) < 400 ) && ((sensors[3]) > 800) && ((sensors[4]) > 800))
       {
         forward_normal();
       }
       delay(2);
       false_beep();
       false_count++;

     }
     forward_normal();
     delay(1000);
    }/*/
  if  (((sensors[7]) > 800) && ((sensors[6]) > 800 ) && ((sensors[5]) > 800) && ((sensors[4]) > 800) && ((sensors[1]) < 400) && ((sensors[0]) < 400))
  {
    left_mudja();
  }
  else if (((sensors[7]) < 400) && ((sensors[6]) < 400 ) && ((sensors[3]) > 800) && ((sensors[2]) > 800 ) && ((sensors[1]) > 800) && ((sensors[0]) > 800))
  {
    right_mudja();
  }
  else if (((sensors[7]) > 800) && ((sensors[6]) > 800 ) && ((sensors[5]) > 800) && ((sensors[4]) > 800) && ((sensors[3]) > 800) && ((sensors[2]) > 800 ) && ((sensors[1]) > 800) && ((sensors[0]) > 800))
  {
    forward_normal();
    delay(1000);

    if (((sensors[7]) > 800) && ((sensors[6]) > 800 ) && ((sensors[5]) > 800) && ((sensors[4]) > 800) && ((sensors[3]) > 800) && ((sensors[2]) > 800 ) && ((sensors[1]) > 800) && ((sensors[0]) > 800))
    {
      // while (((sensors[7]) > 800) && ((sensors[6]) > 800 ) && ((sensors[5]) > 800) && ((sensors[4]) > 800) && ((sensors[3]) > 800) && ((sensors[2]) > 800 ) && ((sensors[1]) > 800) && ((sensors[0]) > 800))
      {
        forward_normal();
      }

      false_beep();
      node++;
    }
    else
    {
      true_beep();
      left_mudja();
    }
    if (position == 3500)
    {
      false_beep();
      node ++;
    }
    else
    {
      true_beep();
      left_mudja();
    }

  }
  else if (position > 6700) {
    left();
    // move(1, speedturn, 1);//motor derecho hacia adelante
    // move(0, speedturn, 0);//motor izquierdo hacia adelante
    return;
  }
  else if (position < 300) {
    right();
    // move(1, speedturn, 0);//motor derecho hacia adelante
    // move(0, speedturn, 1);//motor izquierdo hacia adelante
    return;
  }

  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;

  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;
  if (leftMotorSpeed < 0)leftMotorSpeed = 0;

  forward(leftMotorSpeed, rightMotorSpeed);
}

/*

  void move(int motor, int speed, int direction) {
  digitalWrite(motorPower, HIGH); //disable standby

  boolean inPin1 = HIGH;
  boolean inPin2 = LOW;

  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  if (direction == 0) {
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if (motor == 0) {
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  }
  if (motor == 1) {
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  }
  }
*/

////////////////////////////////////////////
void forward_wall()
{
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, PWM_wall);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, PWM_wall);
}

void back_wall()
{
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, PWM_wall);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, PWM_wall);
}

void rukja()
{
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, PWM_wall);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, PWM_wall);
}


void hleft_wall()
{
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, PWM_wall);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, PWM_wall);
}

void hright_wall()
{
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM,  PWM_wall);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, PWM_wall);
}

void right_wall()
{
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM,  PWM_wall);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, PWM_wall);
}
void left_wall()
{
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM,  PWM_wall);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, PWM_wall);
}

/////////////////////////////////////////////

void forward(int PWM_left, int PWM_right)
{
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, PWM_left);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, PWM_right);
}


void forward_normal()
{
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, PWM);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, PWM);
}


void back()
{
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, PWM);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, PWM);
}



void right()
{
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM,  speedturn);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, speedturn);
}

void left()
{
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM,  speedturn);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, speedturn);
}

void right_mudja()
{
  forward_normal();
  delay(300);
  while (((sensors[0]) < 400))
  {
    right();
  }
  right();
  delay(100);

}

void left_mudja()
{
  forward_normal();
  delay(300);
  while (((sensors[7]) < 400))
  {
    left();
  }
  left();
  delay(100);
}

void true_beep()
{
  digitalWrite(39, HIGH);
  delay(500);
  digitalWrite(39, LOW);
  delay(500);
}

void false_beep()
{
  digitalWrite(39, HIGH);
  delay(250);
  digitalWrite(39, LOW);
  delay(250);
  digitalWrite(39, HIGH);
  delay(250);
  digitalWrite(39, LOW);
  delay(250);
}

void left_error()
{
  lcd.setCursor(0, 1);
  int  dist_left = ultrasonic.distanceRead();
  int  dist_forward = ultrasonic2.distanceRead();
  int  dist_right = ultrasonic1.distanceRead();

  if ((dist_left > 10) && (dist_left < 14) && (dist_forward > 0))
  {
    left_wall_case = 0;
    // forward_wall();
  }
  else if ( (dist_left <= 10) && (dist_forward > 0))
  {
    left_wall_case = 1;
    // hright_wall();
  }
  else if ( (dist_left >= 14) && (dist_forward > 0))
  {
    left_wall_case = 2;
    // hleft_wall();
  }
  else if ( (dist_left > 0) && (dist_forward > 0))
  {
    forward_wall();
    delay(200);
    left_wall_case = 2;
    // hleft_wall();
  }
  else if ( (dist_left >= 0) && (dist_forward < 18))
  {
    left_wall_case = 3;
    back_wall();
    delay(200);
    //  back_wall();
    //  delay(100);
    //  right_wall();
    //  delay(300);
  }
  else
  {
    left_wall_case = left_wall_case;
  }
}

void left_motion()
{
  if (left_wall_case == 0)
  {
    forward_wall();
    //lcd.print("forward");
  }
  else if (left_wall_case == 1)
  {
    hright_wall();
    //lcd.print("right");
  }
  else if (left_wall_case == 2)
  {
    hleft_wall();
    // lcd.print("left");
  }
  else if (left_wall_case == 3)
  {
    // back_wall();
    // delay(100);
    //lcd.print("poora left");
    right_wall();
    delay(200);
  }

}


void right_error()
{
  //lcd.setCursor(0, 1);
  int  dist_left = ultrasonic.distanceRead();
  int  dist_forward = ultrasonic2.distanceRead();
  int  dist_right = ultrasonic1.distanceRead();

  if ((dist_right > 10) && (dist_right < 14) && (dist_forward > 0))
  {
    right_wall_case = 0;
    // forward_wall();
  }
  else if ( (dist_right <= 10) && (dist_forward > 0))
  {
    right_wall_case = 2;
    // hright_wall();
  }
  else if ( (dist_right >= 14) && (dist_forward > 0))
  {
    right_wall_case = 1;
    // hleft_wall();
  }
  else if ( (dist_right > 0) && (dist_forward > 0))
  {
    forward_wall();
    delay(200);
    right_wall_case = 1;
    // hleft_wall();
  }
  else if ( (dist_right >= 0) && (dist_forward < 18))
  {
    right_wall_case = 3;
    back_wall();
    delay(200);
    //  back_wall();
    //  delay(100);
    //  right_wall();
    //  delay(300);
  }
  else
  {
    right_wall_case = right_wall_case;
  }
}

void right_motion()
{
  if (right_wall_case == 0)
  {
    forward_wall();
    //lcd.print("forward");
  }
  else if (right_wall_case == 1)
  {
    hright_wall();
    //lcd.print("right");
  }
  else if (right_wall_case == 2)
  {
    hleft_wall();
    // lcd.print("left");
  }
  else if (right_wall_case == 3)
  {
    // back_wall();
    // delay(100);
    //lcd.print("poora left");
    left_wall();
    delay(200);
  }

}

void lfr_1()
{
  position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.


  if (((sensors[7]) > 800) && ((sensors[0]) > 800 ) && ((sensors[3]) < 400) && ((sensors[4]) < 400))
  {
    while (((sensors[7]) > 800) && ((sensors[0]) > 800 ) && ((sensors[3]) < 400) && ((sensors[4]) < 400))
    {
      forward_normal();
    }
    delay(2);
    if (((sensors[7]) < 400) && ((sensors[0]) < 400 ) && ((sensors[3]) < 400) && ((sensors[4]) < 400))
    {
      while (((sensors[7]) < 400) && ((sensors[0]) < 400 ) && ((sensors[3]) < 400) && ((sensors[4]) < 400))
      {
        forward_normal();
      }
      delay(2);
      true_beep();
      true_count++;
      dist = 0;
      dist = (millis() / 1000);
    }
    else if (((sensors[7]) < 400) && ((sensors[0]) < 400 ) && ((sensors[3]) > 800) && ((sensors[4]) > 800))
    {
      while (((sensors[7]) < 400) && ((sensors[0]) < 400 ) && ((sensors[3]) > 800) && ((sensors[4]) > 800))
      {
        forward_normal();
      }
      delay(2);
      false_beep();
      false_count++;

    }
    forward_normal();
    delay(1000);
  }



  else if (position > 6700) {
    left();
    // move(1, speedturn, 1);//motor derecho hacia adelante
    // move(0, speedturn, 0);//motor izquierdo hacia adelante
    return;
  }
  else if (position < 300) {
    right();
    // move(1, speedturn, 0);//motor derecho hacia adelante
    // move(0, speedturn, 1);//motor izquierdo hacia adelante
    return;
  }

  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;

  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;
  if (leftMotorSpeed < 0)leftMotorSpeed = 0;

  forward(leftMotorSpeed, rightMotorSpeed);
}

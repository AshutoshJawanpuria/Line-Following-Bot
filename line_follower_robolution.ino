// IR Sensors
int R_S = A0;      // Right  sensor
int C_S = A1;// center sensor
int L_S= A2;// Left  sensor
      

// Initial Values of Sensors

// Motor Variables
int ENA = 5;
int MA1 = 8;
int MA2 = 9;
int MB1 = 10;
int MB2 = 11;
int ENB = 6;

//Initial Speed of Motor
int initial_motor_speed = 55;



// PID Constants
float Kp = 30;
float Ki = 0;
float Kd = 15;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

int flag = 0;

void setup()
{
  pinMode(R_S, INPUT);
  pinMode(C_S, INPUT);
  pinMode(L_S, INPUT);
 

  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(9600);                     //setting serial monitor at a default baund rate of 9600
//  delay(500);
//  Serial.println("Started !!");
//  delay(1000);
}
void loop()
{
  read_ERROR();
  Serial.print("error is");
  Serial.println(error);
  if(error==69)
  {Serial.println("stopping bot");//stopping the bot if all the sensors return high 
    stop_bot();
  }
  else if(error==420)
  {Serial.println("slowing bot");//slowing the bot if all the sensors return low
    
    analogWrite(ENA,35);
    analogWrite(ENB,34);
   mover();
  }

else
   { calculate_pid();
    motor_control();}
  
}

void read_ERROR()
{
int LS=digitalRead(L_S);
int CS=digitalRead(C_S);
int RS=digitalRead(R_S);
   if (LS==1&&CS==1&&RS==0)
    error = 2;
  else if ((LS==1)&&(CS==0)&&(RS==0))
    error = 1;
  else if ((LS==0)&&(CS==1)&&(RS==0))
    error = 0;
  else if ((LS==0)&&(CS==0)&&(RS==1))
    error = -1;
  else if ((LS==0)&&(CS==1)&&(RS==1))
    error = -2;
     else if ((LS==1)&&(CS==1)&&(RS==1))
    error = 69;
    else if((LS==0)&&(CS==0)&&(RS==0))
    {
      error=420;
    }

}

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  // Calculating the effective motor speed:
  Serial.println(PID_value);
  int right_motor_speed  = initial_motor_speed - PID_value;
  int left_motor_speed = initial_motor_speed + PID_value;
//  Serial.println(left_motor_speed);
//Serial.println(right_motor_speed);
//delay(2000);

 
  right_motor_speed  = constrain(right_motor_speed, 0, 50);
 left_motor_speed = constrain( left_motor_speed, 0,50);
// Serial.println(left_motor_speed);
//Serial.println(right_motor_speed);
//delay(2000);


  analogWrite(ENB,  right_motor_speed); //Right Motor Speed
  analogWrite(ENA, left_motor_speed ); //Left Motor Speed

  mover();
}

void mover()// this makes the bot go in forward direction with the speed calculated by the pid control system by turning the motors in anticlockwise direction by applying the correct polarity
{
 
  digitalWrite(MA1, LOW);
  digitalWrite(MA2, HIGH);
  digitalWrite(MB1, LOW);
  digitalWrite(MB2, HIGH);
}
void reverse()// this makes the bot go in reverse direction by turning the motors in anticlockwise direction by applying the correct polarity
{
  
  digitalWrite(MA1, HIGH);
  digitalWrite(MA2,LOW);
  digitalWrite(MB1, HIGH);
  digitalWrite(MB2, LOW);
}


void stop_bot()// this function is used to stop the bot by turning all the pins to low
{
  
  digitalWrite(MA1, LOW);
  digitalWrite(MA2, LOW);
  digitalWrite(MB1, LOW);
  digitalWrite(MB2, LOW);
}

float Kp=0.2,Ki=0.0001,Kd=1.0;
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error=0, previous_I=0;
int sensor[5]={0, 0, 0, 0, 0};
int initial_motor_speed=100;

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

// Define the motor control pins
const int enA = 10;  // Enable pin for Motor A
const int in1 = 8;  // Input 1 pin for Motor A
const int in2 = 9;  // Input 2 pin for Motor A
const int enB = 11; // Enable pin for Motor B
const int in3 = 12; // Input 1 pin for Motor B
const int in4 = 13; // Input 2 pin for Motor B

void setup()
{
  // Initialize the motor control pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial.begin(9600); //Enable Serial Communications
}

void loop()
{
    read_sensor_values();
    calculate_pid();
    motor_control();
}

void read_sensor_values()
{
  sensor[0]=digitalRead(A4);
  sensor[1]=digitalRead(A3);
  sensor[2]=digitalRead(A2);
  sensor[3]=digitalRead(A1);
  sensor[4]=digitalRead(A0);

    // Display sensor values on Serial Monitor
  for (int i = 0; i < 5; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensor[i]);
    Serial.print("\t");}
    Serial.println(); // Move to the next line for the next loop
    delay(500);

  
  
  if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1))
  error=4;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))
  error=3;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==0))
  error=2;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==0))
  error=1;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
  error=0;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-1;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-2;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-3;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  error=-4;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
    if(error==-4) error=-5;
    else error=5;

}

void calculate_pid()
{
    P = error;//OK
    //I = I + previous_I; //Master keyword
    //I= I + error; //OK
    I= I + previous_error; //OK
    D = error-previous_error;//OK
    
    PID_value = (Kp*P) + (Ki*I) + (Kd*D); //OK
    /* Kp=0.09;Ki=0.0001;Kd=1.0;*/
    
    previous_I=I; //Master keyword
    //previous_I=error;
    previous_error=error;//OK
}

void motor_control()
{
    // Calculating the effective motor speed:
    int left_motor_speed = initial_motor_speed-PID_value;
    int right_motor_speed = initial_motor_speed-PID_value;
    
    // The motor speed should not exceed the max PWM value
    constrain(left_motor_speed,0,255);
    constrain(right_motor_speed,0,255);
 
    analogWrite(enA,left_motor_speed );   //Left Motor Speed
    analogWrite(enB,right_motor_speed);  //Right Motor Speed
    //following lines of code are to make the bot move forward
    /*The pin numbers and high, low values might be different
    depending on your connections */
//    digitalWrite(4,HIGH);
//    digitalWrite(5,LOW);
//    digitalWrite(6,LOW);
//    digitalWrite(7,HIGH);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  //analogWrite(enA, 255);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  //analogWrite(enB, 255);







}

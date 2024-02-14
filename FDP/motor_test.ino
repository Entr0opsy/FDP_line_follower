// Define the motor control pins
const int enA = 5;  // Enable pin for Motor A
const int in1 = 11;  // Input 1 pin for Motor A
const int in2 = 10;  // Input 2 pin for Motor A
const int enB = 3; // Enable pin for Motor B
const int in3 = 9; // Input 1 pin for Motor B
const int in4 = 8; // Input 2 pin for Motor B

void setup() {
  // Initialize the motor control pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop() {
  // Move both motors forward at full speed
  moveForward();
}

void moveForward() {
  // Set both motors forward at full speed
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 255);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 255);
}

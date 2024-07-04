#include <Servo.h>
#include <PID_v1.h>

// Pin definitions
int PWM_LOWER = 5; // Lower motor (shoulder)
int PWM_UPPER = 6; // Upper motor (elbow)

// Servo objects
Servo lowerServo;
Servo upperServo;

// Arm segment lengths
const float L1 = 100.0;
const float L2 = 100.0;

// Target positions
int x;
int y;


int prevX;
int prevY;


double setpoint1, input1, output1;
double setpoint2, input2, output2;


double Kp = 2.0, Ki = 5.0, Kd = 1.0;


PID pid1(&input1, &output1, &setpoint1, Kp, Ki, Kd, DIRECT);
PID pid2(&input2, &output2, &setpoint2, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  lowerServo.attach(PWM_LOWER);
  upperServo.attach(PWM_UPPER);


  pid1.SetMode(AUTOMATIC);
  pid2.SetMode(AUTOMATIC);
}

void Pos() {
  if(prevX != x || prevY != y) {
    int speedX = x - 90;
    int speedY = y - 90;
  }

  prevX = x;
  prevY = y;
}

void loop() {
  if (Serial.available() > 0) {
    if (Serial.read() == 'X') {
      x = Serial.parseInt();
      if (Serial.read() == 'Y') {
        y = Serial.parseInt();
        Serial.print(x);
        Serial.print(" ");
        Serial.print(y);
        Serial.println();

        float theta2 = acos((x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2));
        float theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));

        int targetAngle1 = theta1 * 180.0 / PI; 
        int targetAngle2 = theta2 * 180.0 / PI; 

        setpoint1 = targetAngle1;
        setpoint2 = targetAngle2;

        input1 = lowerServo.read();
        input2 = upperServo.read();

        pid1.Compute();
        pid2.Compute();

        lowerServo.write(output1);
        upperServo.write(output2);
      }
    }
  }
}

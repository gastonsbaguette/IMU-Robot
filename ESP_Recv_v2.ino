#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

Servo link1, link2, link3, gripper;

int L1 = 50;  // link lengths in mm
int L2 = 50;
int L3 = 75;

int linkChoice = 0; 

bool IK = false;
bool DIR = false;
bool grip = false;
int gripAngle = 0;

// Define a data structure
typedef struct struct_message {
  float pitch_1;
  float pitch_2;
  float armPosX;
  float armPosY;
} struct_message;

struct_message myData;

float theta1, theta2, theta3;
float thetaR1, thetaR2, thetaR3;

int input1 = 90;
int input2 = 90;
int input3 = 90;

float posX, posY;

#define PI 3.141592

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //Serial.print(theta1);
  //Serial.print(",");
  //Serial.print(theta2);
  //Serial.print(",");
  //Serial.println(theta3);
  //printPos();

//  Serial.print(F("Pitch_1, Pitch_2, ArmPosX, ArmPosY: "));
//  Serial.print(myData.pitch_1);
//  Serial.print(",");
//  Serial.print(myData.pitch_2);
//  Serial.print(",");
//  Serial.print(myData.armPosX);
//  Serial.print(",");
//  Serial.println(myData.armPosY);
}

void printPos() {
  Serial.print(F("X: "));
  Serial.print(posX);
  Serial.print(", ");
  Serial.print(F("Y: "));
  Serial.println(posY);    
}

void setup() {
  Serial.begin(115200);
  // Set ESP as Wi-Fi station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  link1.setPeriodHertz(50);
  link2.setPeriodHertz(50);
  link3.setPeriodHertz(50);
  //link4.setPeriodHertz(50);

  link1.attach(16, 500, 2400);
  link2.attach(17, 500, 2400);
  link3.attach(5, 500, 2400); 
  gripper.attach(18, 500, 2400);

  delay(2000);
  Serial.println("Commands: ");
  Serial.println("'R' - resets all joint angles");
  Serial.println("'A' - set joint angle of gripper relative to x-axis");
  Serial.println("'G' - open and close gripper");
  Serial.println("'P' - print the position of the end of the gripper (x,y)");
  Serial.println("'I' - toggle inverse kinematics on or off (default is off)");
  Serial.println("'J' - allows user to control each joint individually");
}

void loop() {

  if (Serial.available()) {
    
    char cmd = Serial.read();

    if (cmd == '!')  {
      Serial.println("\n\nCommands: ");
      Serial.println("'!' - shows command menu");
      Serial.println("'R' - resets all joint angles");
      Serial.println("'A' - set joint angle of gripper relative to x-axis");
      Serial.println("'G' - open and close gripper");
      Serial.println("'P' - print the position of the end of the gripper (x,y)");
      Serial.println("'I' - toggle inverse kinematics on or off (default is off)");
      Serial.println("'J' - allows user to control each joint individually");
      Serial.println("'D' - directly control joints with arm controller");
    }
    
    if (cmd == 'A') {
      Serial.println("Please input an angle for the gripper (relative to the x-axis)");
      while (Serial.available() == 0) {} 
      gripAngle = Serial.parseInt();
      Serial.print("Set angle to ");
      Serial.println(gripAngle);
    }
    
    if (cmd == 'R') {
      input1 = 90;
      input2 = 90;
      input3 = 90;
    }
    if (cmd == 'G') {
      grip = !grip;
      if (grip) {
        gripper.write(50);
      }
      else {
        gripper.write(130);
      }
    }

    if (cmd == 'P') {
      printPos();
    }
    
    if (cmd == 'I' && DIR == false) {
      IK = !IK;
      Serial.print("IK: ");
      Serial.println(IK);
      delay(1000);
    }

    if (cmd == 'D' && IK == false) {
      DIR = !DIR;
      Serial.print("DIR: ");
      Serial.println(DIR);
      delay(1000);
    }
    
    else if (cmd == 'J') {
      Serial.println("Please choose a joint to move: 1, 2, or 3");
      while (Serial.available() == 0) {}
      linkChoice = Serial.parseInt();

      if (linkChoice == 1) {
        Serial.println("Link 1 - Choose a value between 90 -> 180");
        while (Serial.available() == 0) {} 
        int getInput1 = Serial.parseInt();
        if (getInput1 < 90) {
          input1 = 90;
          Serial.println("Set joint to 90");
        }
        else {
          input1 = getInput1;
          Serial.print("Set joint to ");
          Serial.println(input1);
        }
        linkChoice = 0;
      }

      else if (linkChoice == 2) {
        Serial.println("Link 2 - Choose a value between 0 -> 180");
        while (Serial.available() == 0) {}
        input2 = Serial.parseInt();
        Serial.print("Set joint to ");
        Serial.println(input2);
        linkChoice = 0;
      }

      else if (linkChoice == 3) {
        Serial.println("Link 3 - Choose a value between 0 -> 180");
        while (Serial.available() == 0) {}
        input3 = Serial.parseInt();
        Serial.print("Set joint to ");
        Serial.println(input3);
        linkChoice = 0;
      }
      else {
        Serial.println("Please choose a valid number!");
      }
    }
  }


  // IK toggled
  if (IK == true) {
    //link1.write(myData.pitch_1);
    //link2.write(myData.pitch_2);

    float ik_theta1, ik_theta2, ik_theta3, ik_x, ik_y, phi;
    phi = gripAngle;
    ik_x = myData.armPosX;
    //ik_x = 50;
    ik_y = myData.armPosY;
    //ik_y = 70;
    
    ik_theta2 = acos(((ik_x*ik_x)+(ik_y*ik_y)-(L1*L1)-(L2*L2))/(2*L1*L2));
    ik_theta2 = (ik_theta2 * (180/PI));

    
    float ik_theta2_NEW = ik_theta2 + 90;
    ik_theta2_NEW = map(ik_theta2_NEW,0,180,180,0);
    if (ik_theta2_NEW > 180) {
      ik_theta2_NEW = 180;
    }
    else if (ik_theta2_NEW < 0) {
      ik_theta2_NEW = 0;
    }

    ik_theta1 = atan(ik_y/ik_x)-atan((L2*sin(ik_theta2*(PI/180)))/(L1+L2*cos(ik_theta2*(PI/180))));
    ik_theta1 = ik_theta1 * (180/PI);

    float ik_theta1_NEW = map(ik_theta1,0,180,180,0);
    if (ik_theta1_NEW < 90) {
      ik_theta1_NEW = 90;
    }
    else if (ik_theta1_NEW > 180) {
      ik_theta1_NEW = 180;
    }
    
    ik_theta3 = -(phi - ik_theta1 - ik_theta2);
    if (ik_theta3 < 0) {
      ik_theta3 = -(ik_theta3);
    }

    input1 = ik_theta1_NEW;
    input2 = ik_theta2_NEW;
    input3 = ik_theta3;

    link1.write(input1);
    link2.write(input2);
    link3.write(ik_theta3);
    
    Serial.print("Arm Position (X,Y): ");
    Serial.print(myData.armPosX);
    Serial.print(",");
    Serial.print(myData.armPosY);
    Serial.print("     ");
    Serial.print(input1);
    Serial.print(",");
    Serial.print(input2);
    Serial.print(",");
    Serial.print(ik_theta3);
    Serial.print("      ");
    Serial.print(ik_theta1_NEW);
    Serial.print(",");
    Serial.println(ik_theta2_NEW);
  }

  if (DIR == true) {
    link1.write(myData.pitch_1);
    link2.write(myData.pitch_2);
    link3.write(input3);
  }

  if (IK == false && DIR == false) {
    link1.write(input1);
    link2.write(input2);
    link3.write(input3);  
  }

  theta1 = link1.read();       // angle between prev link
  theta2 = link2.read() - 90;  // angle between prev link
  theta3 = link3.read() - 90;  // angle between prev link
  
  thetaR1 = theta1*(PI/180);   // convert to radians
  thetaR2 = theta2*(PI/180);   // convert to radians
  thetaR3 = theta3*(PI/180);   // convert to radians

  posX = -(cos(thetaR1)*L1+L2*(cos(thetaR1)*cos(thetaR2)-sin(thetaR1)*sin(thetaR2))+L3*((sin(thetaR3)*(-cos(thetaR1)*sin(thetaR2)-cos(thetaR2)*sin(thetaR1))+cos(thetaR3)*(cos(thetaR1)*cos(thetaR2)-sin(thetaR1)*sin(thetaR2)))));
  posY = sin(thetaR1)*L1+L2*(cos(thetaR1)*sin(thetaR2)+cos(thetaR2)*sin(thetaR1))+L3*((sin(thetaR3)*(cos(thetaR1)*cos(thetaR2)-sin(thetaR1)*sin(thetaR2))+cos(thetaR3)*(cos(thetaR1)*sin(thetaR2)+cos(thetaR2)*sin(thetaR1))));

  delay(20);
}

#include "MPU9250.h"
#include <esp_now.h>
#include <WiFi.h>

MPU9250 mpu1;
MPU9250 mpu2;

int L1 = 50;
int L2 = 50;
int L3 = 75;

float posX, posY;
float s_theta1, s_theta2, s_theta3; // servo angles
float theta1, theta2, theta3;       // angles b/w links
float thetaR1, thetaR2, thetaR3;    // angles in radians

#define PI 3.141592

// MAC Address of responder
uint8_t broadcastAddress[] = {0x78, 0x21, 0x84, 0x9C, 0x71, 0x74};

// Define a data structure
typedef struct struct_message {
  float pitch_1;
  float pitch_2;
  float armPosX;  // x-coord of wrist
  float armPosY;  // y-coord of wrist
} struct_message;

struct_message myData;

// Peer info
esp_now_peer_info_t peerInfo;

// Callback function called when data is sent
void OnDataSent(const uint8_t *mmac_addr, esp_now_send_status_t status) {
//  Serial.print(F("\r\nLast Packet Send Status:\t"));
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  // Set ESP as Wi-Fi station
  WiFi.mode(WIFI_STA);
  Wire.begin();
  
  if (!mpu1.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println(F("MPU 1 connection failed. Please check your connection with `connection_check` example."));
          delay(5000);
      }
  }
  if (!mpu2.setup(0x69)) {  // change to your own address
      while (1) {
          Serial.println(F("MPU 1 connection failed. Please check your connection with `connection_check` example."));
          delay(5000);
      }
  }  
  // calibrate anytime you want to
  //Serial.println("Accel & Gyro calibration will start in 3 sec.");
  //Serial.println("Please leave the device still on the flat plane.");
    
  mpu1.verbose(false);
  mpu2.verbose(false);
 
  delay(3000);
  mpu1.calibrateAccelGyro();
  Serial.println(F("MPU 1's accel & gyro calibrated"));
  mpu2.calibrateAccelGyro();
  Serial.println(F("MPU 2's accel & gyro calibrated"));

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~ESP-NOW STUFF BELOW~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/  

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("Error initializing ESP-NOW"));
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(F("Failed to add peer"));
    return;
  }
}

void loop() {
  float mpu_pitch1, mpu_pitch2, posX, posY, mapped_pitch1, mapped_pitch2;
  
  if (mpu1.update() && mpu2.update()) {
      static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 25) {
 
        mpu_pitch1 = mpu1.getRoll();
        if (mpu_pitch1 < 0) {
          //myData.pitch_1 = 180;
          s_theta1 = 180;
        }
        else if (mpu_pitch1 >= 0 && mpu_pitch1 <= 180) {
          mapped_pitch1 = map(mpu_pitch1,0,180,180,0);
          if (mapped_pitch1 < 90) {
            //myData.pitch_1 = 90;
            s_theta1 = 90;
          }
          else {
            //myData.pitch_1 = mapped_pitch1;
            s_theta1 = mapped_pitch1; 
          }
        }

        mpu_pitch2 = mpu2.getRoll();
        if (mpu_pitch2 < 0) {
          //myData.pitch_2 = 0;
          s_theta2 = 180;
        }
        else if (mpu_pitch2 >= 0 && mpu_pitch2 <= 180) {
          mapped_pitch2 = map(mpu_pitch2,0,180,180,0);
          //myData.pitch_2 = mapped_pitch2;
          s_theta2 = mapped_pitch2;
        }

        theta1 = s_theta1;
        theta2 = s_theta2 - theta1;
        theta3 = 0;                // hardcode gripper to point upwards

        thetaR1 = theta1*(PI/180); // convert to radians
        thetaR2 = theta2*(PI/180); // convert to radians
        thetaR3 = theta3*(PI/180); // convert to radians


        //posX = -(cos(thetaR1)*L1+L2*(cos(thetaR1)*cos(thetaR2)-sin(thetaR1)*sin(thetaR2))+L3*((sin(thetaR3)*(-cos(thetaR1)*sin(thetaR2)-cos(thetaR2)*sin(thetaR1))+cos(thetaR3)*(cos(thetaR1)*cos(thetaR2)-sin(thetaR1)*sin(thetaR2)))));
        //posY = sin(thetaR1)*L1+L2*(cos(thetaR1)*sin(thetaR2)+cos(thetaR2)*sin(thetaR1))+L3*((sin(thetaR3)*(cos(thetaR1)*cos(thetaR2)-sin(thetaR1)*sin(thetaR2))+cos(thetaR3)*(cos(thetaR1)*sin(thetaR2)+cos(thetaR2)*sin(thetaR1))));

        posX = -(L1*cos(thetaR1)+L2*cos(thetaR1+thetaR2));
        posY = L1*sin(thetaR1)+L2*sin(thetaR1+thetaR2);

        float servo_2 = theta2 + 90;
        if (servo_2 < 0) {
          servo_2 = 0;
        }
        else if (servo_2 > 180) {
          servo_2 = 180;
        }

        myData.pitch_1 = theta1;
        myData.pitch_2 = servo_2;
        
        myData.armPosX = posX;
        myData.armPosY = posY;

        //Serial.print(mpu1.getRoll());
        //Serial.print(",");
        //Serial.print(mpu2.getRoll());
        //Serial.print(",");
        Serial.print(theta1);
        Serial.print(",");
        Serial.print(theta2);
        Serial.print(",");
        //Serial.print(servo_2);
        //Serial.print(",");
        //Serial.print(s_theta1);
        //Serial.print(",");
        //Serial.print(s_theta2);
        //Serial.print(",");
        Serial.print(posX);
        Serial.print(",");
        Serial.println(posY);

        // Send results
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      
//        if (result == ESP_OK) {
//          Serial.println(F("Sending confirmed"));
//        }
//        else {
//          Serial.println(F("Sending error"));
//        }
      prev_ms = millis();
      }
    }
}

#include <WiFi.h>
#include <WiFiClient.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

// Motor pins
const int enA = 19;
const int in1 = 18;
const int in2 = 5;
const int enB = 4;
const int in3 = 17;
const int in4 = 16;

// Max motor speed
const int maxSpeed = 255;

// Wi-Fi credentials(change with yours)
const char* ssid = "Mahipal";
const char* password = "12345678";

// ROS serial server IP address and port
IPAddress server(192, 168, 130, 117);  // Modify with your ROS serial server IP address
const uint16_t port = 11411;           // Modify with your ROS serial server port

// ROS node
ros::NodeHandle nh;

// Variables to track key state
bool keyHeld = false;

// ROS subscriber
void twistCallback(const geometry_msgs::Twist& msg) {
  float linearVel = msg.linear.x;
  float angularVel = msg.angular.z;

  // Calculate motor speeds based on linear and angular velocities
  int motorSpeedA = (linearVel - angularVel*9) * maxSpeed;
  int motorSpeedB = (linearVel + angularVel*9) * maxSpeed;

  // Limit motor speeds within valid range
  motorSpeedA = constrain(motorSpeedA, -maxSpeed, maxSpeed);
  motorSpeedB = constrain(motorSpeedB, -maxSpeed, maxSpeed);

  // Print received data to serial monitor
  Serial.print("LinearVel: ");
  Serial.print(linearVel);
  Serial.print("  AngularVel: ");
  Serial.println(angularVel);

  // Set motor directions based on motor speeds and angular velocity
  if (linearVel > 0 && angularVel == 0) {
    // Moving forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    keyHeld = true;  // Set the flag when the key is held
  } else if (linearVel < 0 && angularVel == 0) {
    // Moving backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    keyHeld = true;  // Set the flag when the key is held
  } else if (linearVel == 0 && angularVel > 0) {
    // Turning left
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    keyHeld = true;  // Set the flag when the key is held
  } else if (linearVel == 0 && angularVel < 0) {
    // Turning right
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    keyHeld = true;  // Set the flag when the key is held
  }  
    else if (linearVel > 0 && angularVel > 0) {
    // Moving diagonally forward left
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    keyHeld = true;
  }
    else if (linearVel > 0 && angularVel < 0) {
    // Moving diagonally forward right
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    keyHeld = true;
  }
    else if (linearVel < 0 && angularVel > 0) {
    // Moving diagonally backward left
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    keyHeld = true;
  }
    else if (linearVel < 0 && angularVel < 0) {
    // Moving diagonally backward right
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    keyHeld = true;
  }  
    else {
    // Stop the motors if no key is held
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    keyHeld = false;  // Clear the flag when the key is released
  }

  // Set motor speeds using analogWrite
  analogWrite(enA, abs(motorSpeedA));
  analogWrite(enB, abs(motorSpeedB));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &twistCallback);

void setup() {
  // Serial communication
  Serial.begin(115200);
  delay(1000);
  Serial.println();

  // Wi-Fi connection
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to Wi-Fi. IP address: ");
  Serial.println(WiFi.localIP());

  // ROS serial connection
  nh.getHardware()->setConnection(server, port);
  nh.initNode();
  nh.subscribe(sub);

  Serial.println("Connected to ROS.");

  // Initialize motor pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop() {
  nh.spinOnce();  // Process ROS callbacks

  // Stop the motors immediately if the key is released
  if (!keyHeld) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    return;  // Exit the loop if the key is released
  }

  delay(1);
}

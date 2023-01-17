#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_PWMServoDriver.h>
#include <vector>

#define SERVOMIN 150
#define SERVOMAX 600
#define SERVO_FREQ 50

std::vector<int> mapToServoRotation(int x, int y, int w, int h);
std::vector<int> parseArray(WiFiClient client);
void blinkCycle(), recvOneChar(), showNewData();


const char* ssid = "samspot_";
const char* password = "1223334444";

int vert =  0;
int hor = 0;

int received;
boolean newData = false;
int turnTo = 90;
WiFiServer server(80);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
 
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  server.begin();
  Serial.println("Server started");
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

void loop() {
    WiFiClient client = server.available();
    if (client) {
        while (client.connected()) {
        if (client.available()) {
            std::vector<int> array = parseArray(client);
            int x = array[0];
            int y = array[1];
            int w = array[2];
            int h = array[3];

            std::vector<int> servo_rotations = mapToServoRotation(x, y, w, h);

            vert = map(servo_rotations[0], 0,180, SERVOMIN, SERVOMAX);
            hor = map(servo_rotations[1], 0,180, SERVOMIN, SERVOMAX);

            pwm.setPWM(0, 0, hor);
            pwm.setPWM(1, 0, vert);
            blinkCycle();


        }
        }
        client.stop();
    }
    blinkCycle();
    recvOneChar();
    showNewData();

    hor = map(turnTo, 0,180, SERVOMIN, SERVOMAX);

    pwm.setPWM(0, 0, hor);
    pwm.setPWM(1, 0, vert);
}

union byteToInt {
    byte array[4];
    int val;
};

std::vector<int> parseArray(WiFiClient client) {
    std::vector<int> array = {0, 0, 0, 0};
    
    byteToInt bti;

    for (int i = 0; i < 4; i++)
    {
        for (int j = 3; j >= 0; j--)
        {
            bti.array[j] = client.read();
        }

        array[i] = bti.val;
    }
    
    Serial.print("x,y,w,h = ");

    for (int i = 0; i < 4; i++)
    {
        Serial.print(array[i]);
        Serial.print(" ");
    }

    Serial.print("\n");
    client.flush();

    return array;
}


std::vector<int> mapToServoRotation(int y, int x, int w, int h) {
    std::vector<int> servo_rotations = {0, 0};
    float x_ratio = (float)y / (float)w;
    float y_ratio = (float)x / (float)h;

    

    servo_rotations[0] = (int)map(x,0,h,108,54);
    servo_rotations[1] = (int)map(y,0,w,90,0);

    Serial.println("Servorot:");
    Serial.println(servo_rotations[0]);
    Serial.println(servo_rotations[1]);

    return servo_rotations;
}


void blinkCycle() {
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 5000) {

        int open = map(0, 0,180, SERVOMIN, SERVOMAX);
        int close = map(45, 0,180, SERVOMIN, SERVOMAX);
        
        pwm.setPWM(2, 0, open);
        pwm.setPWM(3, 0, close);
        delay(100);

        pwm.setPWM(2, 0, close);
        pwm.setPWM(3, 0, open);    
        delay(150);

        pwm.setPWM(2, 0, open);
        pwm.setPWM(3, 0, close);


        Serial.println("Blink");
        previousMillis = currentMillis;
    }
}

void recvOneChar() {
    if (Serial.available() > 0) {
        received = Serial.read();
        newData = true;
    }
}

void showNewData() {
    if (newData == true) {
        switch (received)
        {
        case 51:
            turnTo = 0;
            break;
        case 52:
            turnTo = 18;
            break;
        case 53:
            turnTo = 18 * 2;
            break;
        case 54:
            turnTo = 18 * 3;
            break;
        case 55:
            turnTo = 18 * 4;
            break;
        case 56:
            turnTo = 18 * 5;
            break;
        case 57:
            turnTo = 18 * 6;
            break;
        case 58:
            turnTo = 18 * 7;
            break;
        case 59:
            turnTo = 18 * 8;
            break;
        case 48:
            turnTo = 18 * 9;
            break;
        
        default:
            break;
        }
        Serial.println(received);
        newData = false;
    }
}
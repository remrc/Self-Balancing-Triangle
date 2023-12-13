void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  for (int i = 0; i < 512; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay (3);
  }
  GyZ_offset = GyZ_offset_sum >> 9;
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
}

void angle_calc() {
  
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 4, true);  // request a total of 4 registers
  AcX = Wire.read() << 8 | Wire.read(); 
  AcY = Wire.read() << 8 | Wire.read(); 

  Wire.beginTransmission(MPU6050);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  // request a total of 2 registers
  GyZ = Wire.read() << 8 | Wire.read(); 

  AcXc = AcX - offsets.X;
  AcYc = AcY - offsets.Y;    
  GyZ -= GyZ_offset;

  robot_angle += GyZ * loop_time / 1000.0 / 65.536;       // integrate gyroscope to get angle / 65.536 (bits / (deg/sec))
  Acc_angle = atan2(AcYc, -AcXc) * 57.2958;               // angle from acc. values       * 57.2958 (deg/rad)
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);
}

int sign(int x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  if (x = 0) return 0;
}

float controller(float p_angle, float p_vel, float s_vel, float a_vel) {
  float u =  K1Gain * p_angle + K2Gain * p_vel + K3Gain * s_vel + K4Gain * a_vel;
  if (abs(u) > 70) u = sign(u) * 70;
  return u;
}

double battVoltage(double voltage) {
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
    digitalWrite(INT_LED, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
    digitalWrite(INT_LED, LOW);
  }
  return voltage;
}

#if (USE_BT)
  int Tuning() {
    if (!SerialBT.available())  return 0;
    char param = SerialBT.read();               // get parameter byte
    if (!SerialBT.available()) return 0;
    char cmd = SerialBT.read();                 // get command byte
      switch (param) {
      case 'p':
        if (cmd == '+')    K1Gain += 0.5;
        if (cmd == '-')    K1Gain -= 0.5;
        printValues();
        break;
      case 'd':
        if (cmd == '+')    K2Gain += 0.5;
        if (cmd == '-')    K2Gain -= 0.5;
        printValues();
        break;      
      case 's':
        if (cmd == '+')    K3Gain += 0.1;
        if (cmd == '-')    K3Gain -= 0.1;
        printValues();
        break;
      case 'a':
        if (cmd == '+')    K4Gain += 0.01;
        if (cmd == '-')    K4Gain -= 0.01;
        printValues();
        break;  
      case 'b':
        if (cmd == '+')    bat_divider += 1;
        if (cmd == '-')    bat_divider -= 1;
        printValues();
        break;    
      case 'c':
        if (cmd == '+' && !calibrating) {
          calibrating = true;
          SerialBT.println("Calibrating on");
          SerialBT.println("set robot on first edge (greed led on top)");
        }
        if (cmd == '-' && calibrating) { 
          if (calibrating_step == 1) {
            SerialBT.print("X: "); SerialBT.print(AcX + 16384); SerialBT.print(" Y: "); SerialBT.println(AcY);
            if (abs(AcY) < 5000) {
              offsets.ID = 24;
              offsets.X = AcX + 16384;
              offsets.Y = AcY;
              digitalWrite(BUZZER, HIGH);
              delay(70);
              digitalWrite(BUZZER, LOW);
              calibrating_step = 2;
              SerialBT.println("set robot on left edge (greed led on top)");
            } else {
              calibrating = false;
              SerialBT.println("The angle are wrong!!!");
              SerialBT.println("calibrating off...");
              digitalWrite(BUZZER, HIGH);
              delay(50);
              digitalWrite(BUZZER, LOW);
              delay(70);
              digitalWrite(BUZZER, HIGH);
              delay(50);
              digitalWrite(BUZZER, LOW);
            }
          } else if (calibrating_step == 2 && robot_angle < 140 && robot_angle > 100) {
            offsets.off1 = robot_angle;
            digitalWrite(BUZZER, HIGH);
            delay(70);
            digitalWrite(BUZZER, LOW);
            calibrating_step = 3;
            SerialBT.println("set robot on right edge (greed led on top)");
          } else if (calibrating_step == 3 && robot_angle < -100 && robot_angle > -140) {
            SerialBT.println("calibrating complete.");
            offsets.off2 = robot_angle;
            EEPROM.put(0, offsets);
            EEPROM.commit();
            calibrating = false;
            calibrated = true;
            digitalWrite(BUZZER, HIGH);
            delay(70);
            digitalWrite(BUZZER, LOW);
          }
        }
        break; 
     }
     return 1;    
  }

  void printValues() {
      SerialBT.print("K1: "); SerialBT.print(K1Gain);
      SerialBT.print(" K2: "); SerialBT.print(K2Gain);
      SerialBT.print(" K3: "); SerialBT.print(K3Gain);
      SerialBT.print(" K4: "); SerialBT.println(K4Gain);
      SerialBT.print("Bat_divider: "); SerialBT.println(bat_divider);
  }
#endif  

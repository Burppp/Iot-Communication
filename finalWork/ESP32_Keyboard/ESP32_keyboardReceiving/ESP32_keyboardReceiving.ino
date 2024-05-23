void setup() 
{
  Serial.begin(115200); // 设置串口波特率为9600
  Serial.println("ESP32串口接收已启动");
}

void loop() 
{
  if (Serial.available() > 0) { // 如果串口接收到了数据
    char receivedChar = Serial.read(); // 读取串口数据
    switch (receivedChar) {
      case 'L':
        neopixelWrite(RGB_BUILTIN, 0x00, 0x00, 0x00);
        break;
      case 'R':
        neopixelWrite(RGB_BUILTIN, 0x00, 0x00, 0x00);
        break;
      case 'w':
        neopixelWrite(RGB_BUILTIN, 0x00, 0x00, 0x00);
        break;
      case 'a':
        neopixelWrite(RGB_BUILTIN, 0x40, 0x00, 0x00);
        break;
      case 's':
        neopixelWrite(RGB_BUILTIN, 0x00, 0x40, 0x00);
        break;
      case 'd':
        neopixelWrite(RGB_BUILTIN, 0x00, 0x00, 0x40);
        break;
    }
  }
}

import serial
import keyboard  # 需要安装 keyboard 库：pip install keyboard

# 配置串口
ser = serial.Serial('COM15', 115200)  # 将 'COMx' 替换为你的串口号
# 在Linux或Mac上使用'/dev/ttyUSB0'或'/dev/ttyACM0'

# 监视键盘输入和鼠标点击
while True:
    event = keyboard.read_event()
    if event.event_type == keyboard.KEY_DOWN:
        key = event.name
        if key in ['w', 'a', 's', 'd']:
            print("Key pressed:", key)
            ser.write(key.encode())
    elif event.event_type == keyboard.KEY_UP:
        key = event.name
        if key in ['w', 'a', 's', 'd']:
            print("Key released:", key)
    elif event.event_type == keyboard.KEY_DOWN:
        if event.name == 'left':
            print("Left mouse button clicked")
            ser.write('L'.encode())
        elif event.name == 'right':
            print("Right mouse button clicked")
            ser.write('R'.encode())

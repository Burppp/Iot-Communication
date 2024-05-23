import serial
import threading
import keyboard
from pynput import mouse

# 配置串口
ser = serial.Serial('COM17', 115200)  # 将 'COMx' 替换为你的串口号

# 监视鼠标点击事件的函数
def on_click(x, y, button, pressed):
    if pressed:
        if button == mouse.Button.left:
            print("Left mouse button clicked")
            ser.write(b'L')  # 发送'L'表示左键点击到串口
        elif button == mouse.Button.right:
            print("Right mouse button clicked")
            ser.write(b'R')  # 发送'R'表示右键点击到串口

# 创建一个鼠标监听器
mouse_listener = mouse.Listener(on_click=on_click)

# 开始监听鼠标事件（在新的线程中）
mouse_thread = threading.Thread(target=mouse_listener.start)
mouse_thread.daemon = True
mouse_thread.start()

# 监视键盘输入事件的循环
while True:
    event = keyboard.read_event()
    if event.event_type == keyboard.KEY_DOWN:
        key = event.name
        if key in ['w', 'a', 's', 'd']:
            print("Key pressed:", key)
            ser.write(key.encode())  # 发送键盘按键到串口
    elif event.event_type == keyboard.KEY_UP:
        key = event.name
        if key in ['w', 'a', 's', 'd']:
            print("Key released:", key)

# 主线程将一直运行，直到程序被中断（Ctrl+C）

import serial
import keyboard
import threading
import time
#from pynput import mouse

# 配置串口
ser = serial.Serial('COM20', 115200, timeout=1)

# 初始化按键状态变量
key_states = {'w': False, 'a': False, 's': False, 'd': False}
mouse_left = False
mouse_right = False

# 上一次发送的按键状态和帧
last_sent_key_states = ''
last_sent_frame = ''
sequence_number = 1

# 初始化按键按下时间字典
key_pressed_time = {key: None for key in key_states}

# 发送按键状态到串口
def send_key_states():
    global last_sent_key_states, last_sent_frame, sequence_number
    
    key_state_string = ''.join(['1' if key_states[key] else '0' for key in ['w', 'a', 's', 'd']])
    key_state_string += '1' if mouse_left else '0'
    key_state_string += '1' if mouse_right else '0'
    
    if key_state_string == last_sent_key_states:
        return
    
    frame = f"h{sequence_number}{key_state_string}j"
    ser.write(frame.encode())
    last_sent_key_states = key_state_string
    last_sent_frame = frame
    print("Sent key states:", frame)
    
    while True:
        if ser.in_waiting > 0:
            # 读取串口接收的一个字节
            response = ser.read(1)[0]
            # decoded_response = response.decode('ascii')

            print("response = ", response)
            print("sequense_number = ", sequence_number)

            # 检查接收的字节是否与前一个序列号相同
            if response == sequence_number:
                # 序列号加1，循环从1到9
                sequence_number = sequence_number + 1 if sequence_number < 9 else 1
                break
            else:
                # 重新发送上一帧
                ser.write(last_sent_frame.encode())
                print("Resent key states:", last_sent_frame)
                time.sleep(0.1)  # 等待一小段时间再重发，避免过度占用资源

# 监视鼠标点击事件
# def on_click(x, y, button, pressed):
#     global mouse_left, mouse_right
#     if button == mouse.Button.left:
#         mouse_left = pressed
#         send_key_states()
#     elif button == mouse.Button.right:
#         mouse_right = pressed
#         send_key_states()

# 创建一个鼠标监听器
# mouse_listener = mouse.Listener(on_click=on_click)

# 开始监听鼠标事件
# mouse_thread = threading.Thread(target=mouse_listener.start)
# mouse_thread.daemon = True
# mouse_thread.start()

# LoRa模块 AT 指令发送函数
def LoRa_SendCmd(cmd, result, timeout, isPrintf):
    ser.write(cmd.encode())
    ser.flush()
    time.sleep(timeout / 1000)  # 转换为秒
    reply = ser.read_all().decode()
    if isPrintf:
        print(f"Sent: {cmd.strip()}\nReceived: {reply.strip()}")
    if result in reply:
        print("Success!")
    else:
        print("Fail!")

# LoRa模块透明传输广播模式配置函数
def LoRa_T_V_Attach(isPrintf, isReboot):
    if isReboot:
        time.sleep(1)
        
        LoRa_SendCmd("AT+UART=7,0\r\n", "OK", 1000, isPrintf)   # baudrate=115200，无校验
        LoRa_SendCmd("AT+WLRATE=10,5\r\n", "OK", 1000, isPrintf) # 信道10，19.2kbps
        LoRa_SendCmd("AT+TPOWER=3\r\n", "OK", 1000, isPrintf)   # 20dBm
        LoRa_SendCmd("AT+TMODE=0\r\n", "OK", 1000, isPrintf)    # 透明传输
        LoRa_SendCmd("AT+WLTIME=0\r\n", "OK", 1000, isPrintf)   # 休眠时间1s
        LoRa_SendCmd("AT+CWMODE=0\r\n", "OK", 1000, isPrintf)   # 一般模式
        LoRa_SendCmd("AT+ADDR=FF,FF\r\n", "OK", 1000, isPrintf) # 地址65535
        LoRa_SendCmd("AT+FLASH=1\r\n", "OK", 1000, isPrintf)    # 掉电后保存

        print("Attach!")

# 调用LoRa配置函数（根据需要设置是否打印Log和是否重启）
# LoRa_T_V_Attach(isPrintf=True, isReboot=True)

# 监视键盘输入事件的循环
def keyboard_listener():
    while True:
        event = keyboard.read_event()
        if event.event_type == keyboard.KEY_DOWN:
            key = event.name
            if key in key_states:
                key_pressed_time[key] = threading.Event()
                key_states[key] = True
                send_key_states_thread = threading.Thread(target=send_key_states)
                send_key_states_thread.start()
        elif event.event_type == keyboard.KEY_UP:
            key = event.name
            if key in key_states:
                if key_pressed_time[key] is not None:
                    key_pressed_time[key].set()
                key_states[key] = False
                send_key_states_thread = threading.Thread(target=send_key_states)
                send_key_states_thread.start()
                key_pressed_time[key] = None

# 启动键盘监听线程
keyboard_thread = threading.Thread(target=keyboard_listener)
keyboard_thread.daemon = True
keyboard_thread.start()

# 保持主线程运行
while True:
    time.sleep(1)

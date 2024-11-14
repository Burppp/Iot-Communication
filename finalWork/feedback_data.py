import serial
import struct
from openpyxl import Workbook
import os

current_dir = os.getcwd()

excel_file_path = os.path.join(os.path.expanduser('~'), 'Desktop', 'feedback_data.xlsx')

wb = Workbook()
ws = wb.active
ws.title = "Feedback Data" 
wb.save(excel_file_path) 

ser = serial.Serial(
    port='COM6',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1      
)

if ser.isOpen():
    print("串口已打开，准备接收数据...")
else:
    print("串口打开失败，请检查串口配置。")
    exit()

incomplete_data = b''

try:
    while True:
        data = ser.read(ser.in_waiting)
        

        incomplete_data += data
        
        while len(incomplete_data) >= 4:
            packet = incomplete_data[:4]
            value, = struct.unpack('f', packet)
            print(f"Feedback Data:{value}")
            
            ws.append([value])
            
            incomplete_data = incomplete_data[4:]
        
        wb.save(excel_file_path)

except KeyboardInterrupt:
    print("程序被用户中断。")

finally:
    wb.save(excel_file_path)
    ser.close() 
    print("数据已保存到Excel文件。")
import serial, time
port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_f4d6c14f3473ed11b23e6aeefdf7b791-if00-port0'
ser = serial.Serial(port, 460800, timeout=1)
ser.dtr = False
time.sleep(0.2)
ser.write(b'\xa5\x25')
time.sleep(0.1)
ser.reset_input_buffer()
ser.write(b'\xa5\x20')
time.sleep(0.1)
desc = ser.read(7)
print('SCAN_START desc:', desc.hex())
time.sleep(0.5)
pts = ser.read(100)
print('PTS len:', len(pts), 'hex:', pts.hex())
ser.close()

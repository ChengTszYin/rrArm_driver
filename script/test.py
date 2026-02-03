import serial
import time
import struct

values = [50.5, 100.2, 112.2, 20.5, 33.8, 40.8]
zeros = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
steps = [values, zeros]
val  = [55.0]


SERIAL_PORT = '/dev/ttyACM0' # Change to your port
BAUD_RATE = 115200
TIMEOUT = 1  # seconds

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
while True:
    for step in steps:
        data = struct.pack('<' + 'f' * len(step), *step)
        ser.write(data)     # ← actually send it
        for i, v in enumerate(step):
            # Show each float + its 4 bytes
            chunk = data[i*4 : (i+1)*4]
            print(f"  {v:8.3f} → {chunk.hex(' ').upper()}")
        
        print("-" * 50)
        time.sleep(2)
LAB = (0, 15, -128, 127, -128, 127)#(3, 41, -3, 77, 12, 70)#(0, 95, 1, 34, 36, 80) #(5, 89, 35, 75, -8, 50)# Grayscale threshold for dark things...
import sensor, image, time , math ,struct
from pyb import UART
from struct import pack, unpack
import json
sensor.reset()
#sensor.set_vflip(True)
#sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
#sensor.set_auto_whitebal(False) # must be turned off for color tracking
#sensor.set_windowing([0,20,80,40])
sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
clock = time.clock()                # to process a frame sometimes.
uart = UART(1, 115200)
while(True):
    clock.tick()
    img = sensor.snapshot().binary([LAB])
    line = img.get_regression([(100,100)], robust = True)
    if (line):
        rho_err = abs(line.rho())-img.width()/2 #计算一条直线与图像中央的距离

        #坐标变换  xy轴的角度
        if line.theta()>90:
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()

        img.draw_line(line.line(), color = 127)
        output_str = "%f"%(theta_err)

        sumA = 0
        sumB = 0
        data = bytearray([0x41,0x43])
        uart.write(data)

        data = bytearray([0x02,8])
        for b in data:
            sumB = sumB + b
            sumA = sumA + sumB
        uart.write(data)

        float_value = theta_err
        float_bytes = pack('f', float_value)
        for b in float_bytes:
            sumB = sumB + b
            sumA = sumA + sumB
        uart.write(float_bytes)

        float_value = rho_err*0.1
        float_bytes = pack('f', float_value)
        for b in float_bytes:
            sumB = sumB + b
            sumA = sumA + sumB
        uart.write(float_bytes)

        data = bytearray([sumB, sumA])
        uart.write(data)

        print(float_value,theta_err)
        #print(rho_err*0.1)
    else:
        sumA = 0
        sumB = 0
        data = bytearray([0x41,0x43])
        uart.write(data)

        data = bytearray([0x02,8])
        for b in data:
            sumB = sumB + b
            sumA = sumA + sumB
        uart.write(data)

        float_value = 200
        float_bytes = pack('f', float_value)
        for b in float_bytes:
            sumB = sumB + b
            sumA = sumA + sumB
        uart.write(float_bytes)

        float_value = 0
        float_bytes = pack('f', float_value)
        for b in float_bytes:
            sumB = sumB + b
            sumA = sumA + sumB
        uart.write(float_bytes)

        data = bytearray([sumB, sumA])
        uart.write(data)

        #print(theta_err)
        print(float_value)



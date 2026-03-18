from pyb import Pin
import utime

button = Pin(Pin.cpu.C13, Pin.IN)
last = button.value()

print("Testing blue button on PC13")

while True:
    val = button.value()

    if val != last:
        print("raw =", val)
        if val == 0:
            print("BUTTON PRESSED")
        else:
            print("BUTTON RELEASED")
        last = val

    utime.sleep_ms(50)
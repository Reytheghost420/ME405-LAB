from machine import Pin, ADC
import time

# Create ADC objects for each sensor output pin
adc_pins = [
    ADC(Pin("PC0")),
    ADC(Pin("PC1")),
    ADC(Pin("PB0")),
    ADC(Pin("PA4")),
    ADC(Pin("PC2")),
    ADC(Pin("PC3")),
    ADC(Pin("PC5")),
]

print("QTR 7-Channel Analog Sensor Test")
print("Move sensor over white and black surfaces...\n")

while True:
    values = [adc.read_u16() for adc in adc_pins]

    print(values)

    time.sleep_ms(200)
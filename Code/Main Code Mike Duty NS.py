from machine import Pin, PWM
import time

PWM_PIN = 10  # GPIO pin connected to the ESC's signal wire

# --- PWM CONFIGURATION ---
# ESCs typically work with a standard servo signal: a 50 Hz frequency
# and a pulse width between 1,000 and 2,000 microseconds (us).
# 1,000 us = minimum throttle (stopped)
# 2,000 us = maximum throttle
#
# The PWM duty cycle on MicroPython is a value from 0 to 65535.
# We will use the standard 1000us-2000us range as specified by most manuals.
#
# PWM Period (T) = 1 / Frequency = 1 / 60 Hz = 0.0167 seconds = 16700 us
#
# Duty Cycle = (Pulse Width in us / Period in us) * 65535
#
# Min Duty Cycle (1000 us) = (1000 / 16700) * 65535 = 3924
# Max Duty Cycle (2000 us) = (2000 / 16700) * 65535 = 7848
# Neutral Duty Cycle (1500 us) = (1500 / 16700) * 65535 = 5886

#when shifting and it is rotating in the anti-clockwise dirrection, turning the lever am anti-clockwise direction forces it to shift up
ESC_MIN_DUTY = 5886 - int(10 * 196.2) #max min is at 3924
ESC_MAX_DUTY = 5886 + int(10 * 196.2)#max max is at 7848
NEUTRAL_DUTY = 5886
#2.5 worked on down

ESC_MIN_PW = 800000#935000
ESC_MAX_PW = 2115000
NEUTRAL_PW = 1500000

esc_pwm = PWM(Pin(PWM_PIN))
esc_pwm.freq(61)  # Sets the motor freq to 60
is_pressed = False
counter = 0
rotation_time = 0.1

Upshift_Pin = Pin(2, Pin.IN, Pin.PULL_UP) #define the pin + pull it up bcs of buton configuration.
Downshift_Pin= Pin(3, Pin.IN, Pin.PULL_UP)

while True:
    #checking for =0 bcs when the pin gets low it will return the binary representation of the state, aka: 0 or 1
    if is_pressed == True and (Downshift_Pin.value() == 0 or Upshift_Pin.value() == 0):
        esc_pwm.duty_u16(NEUTRAL_DUTY)
    elif Downshift_Pin.value() == 0 and is_pressed == False: #clockwise spinning
        counter +=1
        is_pressed = True
        print(f"Shift Counter: {counter} - Downshift is pressed!")
        print("Downshifting for 2sec!")
        esc_pwm.duty_ns(ESC_MIN_PW) #this makes it go reverse
        time.sleep(rotation_time)
        
        print("Neutral for 1sec!")
        esc_pwm.duty_ns(NEUTRAL_PW)
        
        
    elif Upshift_Pin.value() == 0 and is_pressed == False:
        counter +=1
        is_pressed = True
        esc_pwm.duty_ns(ESC_MAX_PW)
        print(f"Shift Counter: {counter} - Upshift is pressed!")
        print("Upshifting for 2sec!")
        time.sleep(rotation_time)
        
        print("Neutral")
        esc_pwm.duty_ns(NEUTRAL_PW)
        
    else:
        esc_pwm.duty_ns(NEUTRAL_PW)
        time.sleep(0.1)
        is_pressed = False


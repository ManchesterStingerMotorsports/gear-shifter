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
#Based on the datasheet provided with copperhead 10:
# - Full Reverse: 1.048 ms
# - Neutral: 1.479 ms
# - Full Forward: 1.910 ms
#
# Min Duty Cycle (1048 us) = (1048 / 16700) * 65535 = 4112,62
# Max Duty Cycle (1910 us) = (1910 / 16700) * 65535 = 7495,32
# Neutral Duty Cycle (1479 us) = (1489 / 16700) * 65535 = 5843,21

#when shifting and it is rotating in the anti-clockwise dirrection, turning the lever am anti-clockwise direction forces it to shift up
#
#ESC_MIN_DUTY_ns = 4113
#ESC_MAX_DUTY_ns = 7495
#NEUTRAL_DUTY_ns = 5843
Percentage_Trottle 
ESC_MIN_PW = (1479 - int(0.27 * 431)) * 1000
ESC_MAX_PW = (1479 + int(0.2 * 431)) * 1000
ESC_NEUTRAL_PW = 1479 * 1000

esc_pwm = PWM(Pin(PWM_PIN))
esc_pwm.freq(60)  # Sets the motor freq to 60, should I use a different one

#define the pin + pull it up bcs of buton configuration.
Upshift_Pin = Pin(2, Pin.IN, Pin.PULL_UP) 
Downshift_Pin= Pin(3, Pin.IN, Pin.PULL_UP)

is_pressed = False
counter = 0
rotation_time = 0.1

#Button debounce logic
is_pressed = False

command = "-" #Temporary ------------------------ remove everything to do with it and remove 'or command.lower() == "..."'
while True:
    #checking for =0 bcs when the pin gets low it will return the binary representation of the state, aka: 0 or 1
    if (is_pressed == True and (Downshift_Pin.value() == 0 or Upshift_Pin.value() == 0)):
        esc_pwm.duty_u16(NEUTRAL_DUTY)
    elif (Downshift_Pin.value() == 0 and is_pressed == False) or command.lower() == "d": #clockwise spinning
        counter +=1
        is_pressed = True
        print(f"--- {counter} - Downshift is pressed! ---")
        print(f"Downshifting for {rotation_time} sec!")
        esc_pwm.duty_ns(ESC_MIN_PW) #this makes it go reverse
        time.sleep(rotation_time)
        
        command = "-" #remove when removing the terminal activation
        
    elif (Upshift_Pin.value() == 0 and is_pressed == False) or command.lower() == "u":
        counter +=1
        is_pressed = True
        esc_pwm.duty_ns(ESC_MAX_PW)
        print(f"--- {counter} - Upshift is pressed! ---")
        print(f"Upshifting for {rotation_time} sec!")
        time.sleep(rotation_time)
        
        command = "-" #removed when removing the terminal activation
        
    else:
        esc_pwm.duty_ns(ESC_NEUTRAL_PW)
        time.sleep(0.1)
        is_pressed = False
        command = input("Enter command: ") #temporary remove when doing phisical checks

        
        

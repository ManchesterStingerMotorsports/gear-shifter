# motor_control.py
#
# A MicroPython script for controlling a brushless motor via an ESC
# on a Raspberry Pi Pico.
#
# This version of the script is a dedicated tool to find the precise
# neutral duty cycle for your ESC.

from machine import Pin, PWM
import time

# --- HARDWARE SETUP ---
# Connect the ESC's 3-pin BEC cable to the Raspberry Pi Pico as follows:
# - Ground (black/brown wire) -> Pico GND pin
# - VCC (red wire) -> This is a power line. It should NOT be connected
#   to the Pico's 3V3_OUT pin, but rather used to power the Pico itself.
#   However, if you are powering the Pico via USB, you can leave the VCC
#   wire disconnected from the Pico, and just connect Ground and Signal.
#   Connecting the VCC from the ESC to the Pico can cause damage if the
#   voltages don't match.
# - Signal (white/yellow wire) -> Connect this to a PWM-capable GPIO pin.
#   GPIO 15 is used in this example. You can change this to any
#   other PWM-capable pin (e.g., 0-28).

PWM_PIN = 15  # GPIO pin connected to the ESC's signal wire

# --- PWM CONFIGURATION ---
# ESCs typically work with a standard servo signal: a 50 Hz frequency
# and a pulse width between 1,000 and 2,000 microseconds (us).
#
# The PWM duty cycle on MicroPython is a value from 0 to 65535.
# We will use the standard 1000us-2000us range as specified by most manuals.
#
# PWM Period (T) = 1 / Frequency = 1 / 50 Hz = 0.02 seconds = 20,000 us
#
# Duty Cycle = (Pulse Width in us / Period in us) * 65535
#
# Min Duty Cycle (1000 us) = (1000 / 20000) * 65535 = 3277
# Max Duty Cycle (2000 us) = (2000 / 20000) * 65535 = 6553
# Neutral Duty Cycle (1500 us) = (1500 / 20000) * 65535 = 4915

ESC_MIN_DUTY = 3277
ESC_MAX_DUTY = 6553
NEUTRAL_DUTY = 4915

# Initialize PWM on the specified pin.
pwm = PWM(Pin(PWM_PIN))
pwm.freq(50)  # Set the frequency to 50 Hz

def find_neutral_duty_cycle():
    """
    Interactively finds the correct neutral PWM duty cycle for the ESC.
    This replaces the initial calibration step in the main script.
    """
    print("\n--- NEUTRAL DUTY CYCLE FINDER ---")
    print("\nIMPORTANT: This script is a tool to find the precise neutral point.")
    print("Please follow the instructions below and have the ESC ready.")
    
    # Starting point for neutral duty cycle. It's common for the neutral
    # to be slightly above or below the theoretical 4915.
    current_neutral = NEUTRAL_DUTY - 100
    
    print("\n--- INSTRUCTIONS ---")
    print("1. Hold the SET button on the ESC.")
    print("2. Connect the battery to the ESC. The RED LED on the ESC should start to blink.")
    print("3. Once the RED LED blinks, release the SET button.")
    print("\nThe script will now send an increasing PWM signal.")
    print("Press 'q' and Enter as soon as you see the GREEN light, and the ESC beeps.")
    
    # Give the user a moment to get ready.
    time.sleep(3)
    
    # Loop to test duty cycle values
    while True:
        try:
            pwm.duty_u16(current_neutral)
            print(f"Testing neutral duty cycle: {current_neutral}")
            
            # Wait for user input to either continue or quit
            user_input = input("Press Enter to test the next value, or 'q' to quit: ")
            
            if user_input.lower() == 'q':
                break
                
            current_neutral += 1
            
        except KeyboardInterrupt:
            # Allows for a manual exit with Ctrl+C
            break
            
    print(f"\nFinal Neutral Duty Cycle: {current_neutral}")
    print("Please write this value down and use it to replace NEUTRAL_DUTY in the full calibration script.")

    stop_motor()
    pwm.deinit()
    
def set_speed(speed_percent):
    """
    Sets the motor speed based on a percentage value.
    Note: This is not used in this script but is left here for context.
    """
    # This is a placeholder function, as this script is for finding the
    # neutral point only.
    pass

def stop_motor():
    """
    Stops the motor by setting the duty cycle to the minimum value.
    Note: This is not used in this script but is left here for context.
    """
    # This is a placeholder function.
    pass


# --- MAIN SCRIPT EXECUTION ---
if __name__ == "__main__":
    find_neutral_duty_cycle()
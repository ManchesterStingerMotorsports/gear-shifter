# motor_control.py
#
# A MicroPython script for controlling a brushless motor via an ESC
# on a Raspberry Pi Pico.
#
# IMPORTANT SAFETY WARNING:
# This version of the script performs a guided calibration routine for your ESC,
# based on the provided receiver calibration instructions.
# You MUST follow the instructions printed to the console precisely.
# Failing to do so can result in the motor spinning up unexpectedly.
#
# Ensure your motor and propeller are securely mounted and clear of any
# objects or body parts before running this code. ESCs can spin motors at
# high speeds and can be dangerous. Always test with a low power supply
# or with the propeller removed.

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

ESC_MIN_DUTY = 3924
ESC_MAX_DUTY = 7848
NEUTRAL_DUTY = 5886

# --- FINE-TUNING NEUTRAL DUTY CYCLE ---
# If the ESC does not recognize the neutral signal (the red light keeps
# blinking), the value may be slightly off for your specific ESC.
# Adjust this value in small increments (e.g., +/- 10) to find the
# correct neutral point.
# A positive value increases the pulse width, and a negative value decreases it.
FINE_TUNE_NEUTRAL_DUTY = 0 # Adjust this value as needed

# Initialize PWM on the specified pin.
pwm = PWM(Pin(PWM_PIN))
pwm.freq(60)  # Set the frequency to 50 Hz

def set_speed(speed_percent):
    """
    Sets the motor speed based on a percentage value.

    Args:
        speed_percent (int): An integer from 0 to 100 representing the
                             motor speed. 0 is stopped, 100 is max speed.
    """
    if not 0 <= speed_percent <= 100:
        print("Error: Speed percentage must be between 0 and 100.")
        return

    # Map the input percentage (0-100) to the duty cycle range
    # (ESC_MIN_DUTY to ESC_MAX_DUTY).
    duty_cycle = int(NEUTRAL_DUTY + (ESC_MAX_DUTY - NEUTRAL_DUTY) * (speed_percent / 100))
    pwm.duty_u16(duty_cycle)
    print(f"Setting motor speed to {speed_percent}% (duty_cycle={duty_cycle})")

def stop_motor():
    """
    Stops the motor by setting the duty cycle to the minimum value.
    """
    set_speed(0)

def calibrate_and_arm_esc():
    """
    Guides the user through the ESC calibration and arming process based on the
    Hobbywing manual. This is required for many RC car ESCs.
    """
    print("\n--- ESC CALIBRATION PROCEDURE (AUTOMATED TIMING) ---")
    print("\nIMPORTANT: This script is now automated to prevent timing out.")
    print("Please follow the instructions below quickly and precisely.")
    
    pwm.duty_u16(NEUTRAL_DUTY + FINE_TUNE_NEUTRAL_DUTY)
    print("\n--- INSTRUCTIONS ---")
    print("1. Hold the SET button on the ESC.")
    print("2. Connect the battery to the ESC. The RED LED on the ESC should start to blink.")
    print("3. Once the RED LED blinks, release the SET button. The ESC should make a 'Beep' tone.")
    
    # Step 1: Send neutral signal, wait for user to connect power and press SET button.
    print("\nStep 1: The script is now sending the NEUTRAL signal.")
    
    input('Press enter to continue once neutral is set')  # Give user a few seconds to perform the action
    
    # Step 2: Send max throttle signal
    print("\nStep 2: Sending the FORWARD endpoint signal (max throttle).")
    pwm.duty_u16(ESC_MAX_DUTY)
    
    print("\n--- INSTRUCTIONS ---")
    print("1. Now, quickly press the SET button once on the ESC.")
    print("2. The ESC's GREEN LED should flash once and the motor should make a 'Beep' tone.")
    
    input('Press enter to continue to set backwards signal') # Give a short window for the ESC to register the button press and signal
    
    # Step 3: Send min throttle signal
    print("\nStep 3: Sending the BACKWARD endpoint signal.")
    pwm.duty_u16(ESC_MIN_DUTY)
    
    print("\n--- INSTRUCTIONS ---")
    print("1. Quickly press the SET button one last time to save.")
    print("2. The ESC should make a series of beeps.")
    
    input('Press enter to continue to final setup')
    
    # Final confirmation
    print("\nCalibration is now complete! The ESC is armed.")
    print("The motor is now ready to be controlled.")
    
    time.sleep(3)


# --- MAIN CONTROL LOOP ---
if __name__ == "__main__":
    try:
        # Step 1: Calibrate and arm the ESC automatically
        #calibrate_and_arm_esc()
        
        # Step 2: Now that the ESC is calibrated and armed, we can run a test sequence
        print("\n--- MOTOR TEST SEQUENCE ---")
        print("\nIncreasing motor speed...")
        for speed in range(0, 101, 10):
            set_speed(speed)
            time.sleep(1)
        
        # Step 3: Hold at full speed for a moment
        print("\nRunning at full speed for 2 seconds.")
        time.sleep(2)

        # Step 4: Gradually decrease motor speed back to zero
        print("\nDecreasing motor speed...")
        for speed in range(100, -1, -10):
            set_speed(speed)
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        # Step 5: Always ensure the motor is stopped before exiting
        stop_motor()
        pwm.deinit() # De-initialize PWM to release the pin
        print("Motor stopped and PWM de-initialized.")

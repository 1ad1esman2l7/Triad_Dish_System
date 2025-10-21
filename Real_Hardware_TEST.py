import pyfirmata
import time

# Connect to Arduino
board = pyfirmata.Arduino("COM3")

# Start an iterator thread so pin states are read asynchronously
it = pyfirmata.util.Iterator(board)
it.start()

# Define the button pin
buttonPin = board.get_pin('d:8:i')  # Digital pin 8 as input

# Wait a bit for the iterator to start reading values
time.sleep(1)

print("Monitoring button state... (press Ctrl+C to stop)")

try:
    while True:
        button_state = buttonPin.read()
        if button_state is not None:  # Avoid None readings before iterator syncs
            if button_state:
                print("Button Released!")
            else:
                print("Button Pressed!")  # For pull-up circuits
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    board.exit()
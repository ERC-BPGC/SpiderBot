import time
import datetime
 
def countdown(h=0, m=0, s=60):
 
    # Calculate the total number of seconds
    total_seconds = h * 3600 + m * 60 + s
 
    # While loop that checks if total_seconds reaches zero
    # If not zero, decrement total time by one second
    while total_seconds > 0:
 
        # Timer represents time left on countdown
        timer = datetime.timedelta(seconds = total_seconds)
        
        # Prints the time left on the timer
        print(timer, end="\r")
 
        # Delays the program one second
        time.sleep(1)
 
        # Reduces total time by one second
        total_seconds -=1
countdown()
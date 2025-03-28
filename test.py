import time

def countdown(start):
    for i in range(start, 0, -1):
        print(i)
        time.sleep(1)
    print("Countdown complete!")

countdown(10)

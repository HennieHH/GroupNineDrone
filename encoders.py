from machine import Pin
import time
import math

# --- LEFT ENCODER ---
encoderA_L = Pin(18, Pin.IN)
encoderB_L = Pin(19, Pin.IN)
position_l = 0
last_A_L = encoderA_L.value()

def handle_encoder_left(pin):
    global position_l, last_A_L
    A = encoderA_L.value()
    B = encoderB_L.value()
    if last_A_L == 0 and A == 1:
        if B == 0:
            position_l += 1
        else:
            position_l -= 1
    last_A_L = A

encoderA_L.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=handle_encoder_left)

# --- RIGHT ENCODER ---
encoderA_R = Pin(22, Pin.IN)
encoderB_R = Pin(23, Pin.IN)
position_r = 0
last_A_R = encoderA_R.value()

def handle_encoder_right(pin):
    global position_r, last_A_R
    A = encoderA_R.value()
    B = encoderB_R.value()
    if last_A_R == 0 and A == 1:
        if B == 0:
            position_r += 1
        else:
            position_r -= 1
    last_A_R = A

encoderA_R.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=handle_encoder_right)

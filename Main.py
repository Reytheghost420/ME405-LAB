#main.py

from pyb import Pin, Timer
import pyb
from time import sleep_ms
from encoder import Encoder 
from motor import Motor

#encoder objects 
left_enc = Encoder(5, pyb.Pin('A0'), pyb.Pin('A1'))
right_enc = Encoder(1, pyb.Pin('A8'), pyb.Pin('A9'))

#motor objects 
motor_left  = Motor(Pin.cpu.B7, Pin.cpu.C13, Pin.cpu.C14, 2, 4)  # TIM4
motor_right = Motor(Pin.cpu.A7, Pin.cpu.A6,  Pin.cpu.B6,  2, 3)  # TIM3


#example 
def update_and_print(label, loops=15, step_ms=200):
    """Update encoders repeatedly and print positions."""
    print("\n" + label)
    for _ in range(loops):
        left_enc.update()
        right_enc.update()
        print("L pos:", left_enc.get_position(), " | R pos:", right_enc.get_position())
        sleep_ms(step_ms)

def stop_all():
    motor_left.set_effort(0)
    motor_right.set_effort(0)
    motor_left.disable()
    motor_right.disable()

# -------------------------
# START DEMO
# -------------------------
print("\n=== DEMO START ===")
stop_all()
sleep_ms(500)

# 2) Enable/disable individually + should not move on enable
print("\n[TEST 2] Enable motors: should NOT move when enabled (effort=0)")
motor_left.enable()
motor_right.enable()
sleep_ms(300)

left_enc.zero()
right_enc.zero()
update_and_print("After enable, effort=0 (positions should stay near 0)", loops=8, step_ms=200)

# 1) Each motor forward/back independently with varying speed
print("\n[TEST 1] Right motor only, forward speeds 30 -> 60")
left_enc.zero(); right_enc.zero()

motor_left.set_effort(0)
motor_right.set_effort(30)
update_and_print("Right +30 only (R should change, L ~0)", loops=10)

motor_right.set_effort(60)
update_and_print("Right +60 only (R should change faster)", loops=10)

motor_right.set_effort(0)
sleep_ms(500)

print("\n[TEST 1] Left motor only, backward speeds -30 -> -60")
left_enc.zero(); right_enc.zero()

motor_right.set_effort(0)
motor_left.set_effort(-30)
update_and_print("Left -30 only (L should change, R ~0)", loops=10)

motor_left.set_effort(-60)
update_and_print("Left -60 only (L should change faster)", loops=10)

motor_left.set_effort(0)
sleep_ms(500)

# 3) Encoder position grows positive and negative
print("\n[TEST 3] Both motors forward then backward (positions should go + then -)")
left_enc.zero(); right_enc.zero()

motor_left.set_effort(40)
motor_right.set_effort(40)
update_and_print("Both +40 (positions should increase)", loops=12)

motor_left.set_effort(-40)
motor_right.set_effort(-40)
update_and_print("Both -40 (positions should decrease or wrap)", loops=12)

motor_left.set_effort(0)
motor_right.set_effort(0)
sleep_ms(500)

# 4) Timer reload doesn't affect speed/position (overflow/underflow handling)
# Force position near wrap by spinning a bit, then show it stays smooth across wrap.
print("\n[TEST 4] Overflow/underflow handling (no crazy jump in position)")
left_enc.zero(); right_enc.zero()

motor_left.set_effort(80)
motor_right.set_effort(80)
update_and_print("Run fast a moment", loops=10)

motor_left.set_effort(-80)
motor_right.set_effort(-80)
update_and_print("Reverse fast a moment (should still be smooth)", loops=10)

motor_left.set_effort(0)
motor_right.set_effort(0)
sleep_ms(500)

# 5) Motor/encoder pairing correctness
print("\n[TEST 5] Pairing check: spinning a motor should move its encoder")
left_enc.zero(); right_enc.zero()

motor_left.set_effort(50)
motor_right.set_effort(0)
update_and_print("Left motor only: L should move, R ~0", loops=10)

motor_left.set_effort(0)
motor_right.set_effort(50)
update_and_print("Right motor only: R should move, L ~0", loops=10)

motor_left.set_effort(0)
motor_right.set_effort(0)

# 6) Positive duty corresponds to "drive forward" encoder counts upward
print("\n[TEST 6] Check sign convention: +effort should make forward-driving counts go up")
print("If this is reversed, swap encoder A/B wires OR swap channel(1)/channel(2) in software.")
left_enc.zero(); right_enc.zero()

motor_left.set_effort(40)
motor_right.set_effort(40)
update_and_print("Apply +40 (expect positions to increase for forward direction)", loops=10)

stop_all()
print("\n=== DEMO END ===")



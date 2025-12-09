#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, TouchSensor
from pybricks.parameters import Port, Direction, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import SoundFile


class ClimbingDirections:
  UP = "up"
  DOWN = "down"


def main():
    ev3 = EV3Brick()
    front_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
    back_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
    carriage_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE, [8, 24, 40])

    gyro_sensor = GyroSensor(Port.S1)
    touch_sensor = TouchSensor(Port.S2)

    robot = StairClimberEV3(front_motor, back_motor, carriage_motor, gyro_sensor, touch_sensor, ev3)

    robot.run()

class StairClimberEV3:
  def __init__(self, front_motor, back_motor, carriage_motor, gyro_sensor, touch_sensor, brick: EV3Brick):

    self.front_motor = front_motor
    self.back_motor = back_motor
    self.carriage_motor = carriage_motor
    self.gyro_sensor = gyro_sensor
    self.touch_sensor = touch_sensor
    self.brick = brick

    self.number_of_steps = 3
    self.climbed_steps = 0
    wheel_diameter = 56
    axle_track = 100
    
    self.initialize_carriage_structure()

  def initialize_carriage_structure(self):
    """Recreates the initialization from LEGO’s EV3 example."""
    # Move the lift assembly upward until touch sensor is pressed
    self.back_motor.dc(-20)
    self.carriage_motor.dc(100)

    while not self.touch_sensor.pressed():
      wait(10)

    # Move down a bit
    self.carriage_motor.dc(-100)
    self.back_motor.dc(40)
    wait(50)

    # Move to calibrated zero
    self.carriage_motor.run_angle(-145, 510)
    self.back_motor.hold()
    self.carriage_motor.run_angle(-30, 44)
    self.carriage_motor.reset_angle(0)
    self.gyro_sensor.reset_angle(0)

  # -----------------------------------------------------------------
  # BASIC MOVEMENT
  # -----------------------------------------------------------------
  def move_forward(self, speed=500):
    """Matches Spike version: drive forward until detect_step becomes True."""
    print("EV3: inside move_forward")
    self.back_motor.run(speed)
    self.front_motor.run(speed)

    while not self.detect_step():
      wait(100)

    self.stop_robot()
    print("EV3: finished move_forward")

  def stop_robot(self):
    """Stops horizontal movement."""
    self.front_motor.stop()
    self.back_motor.stop()
    print("EV3: robot stopped")

  # -----------------------------------------------------------------
  # STEP DETECTION
  # -----------------------------------------------------------------
  def detect_step(self):
    """
    Spike version uses ultrasonic distance < 3 cm.
    EV3 version uses gyro angle rising above a threshold.
    """
    print("inside of detect step function")
    angle = self.gyro_sensor.angle()
    step_detected = angle < -24  # Equivalent to “tilting upward”
    print("step detected")
    print(step_detected)
    print("angle: ")
    print(angle)
    return step_detected

  def detect_step_descending(self):
    angle = self.gyro_sensor.angle()
    return angle > 8

  # -----------------------------------------------------------------
  # CARRIAGE CONTROL 
  # -----------------------------------------------------------------
  def operate_carriage(self, direction):
    if direction == ClimbingDirections.UP:
      # Move the lift assembly upward until touch sensor is pressed
      self.carriage_motor.run(400)

      # move until touch sensor hit
      while not self.touch_sensor.pressed():
        wait(5)

      self.carriage_motor.stop()
      print("hit after the while loop in the operate carriage function ")
    else:
      print("in the else statement in the operate_carriage function")
      self.carriage_motor.run(-300)

      while self.carriage_motor.angle() > 0:
        wait(5)

      self.carriage_motor.stop()

  
  def climb_step(self):
    print("EV3: Starting climb_step()")

    # (1) Move forward until angle rises → approaching step
    self.move_forward()

    # (2) Raise carriage while wheels still driving
    self.operate_carriage(ClimbingDirections.UP)

    
    watch = StopWatch()
    while watch.time() < 3000:
      self.back_motor.dc(70)
      self.front_motor.dc(40)

    # (3) Pull robot fully onto step
    self.operate_carriage(ClimbingDirections.DOWN)

    self.climbed_steps += 1

  # -----------------------------------------------------------------
  # DESCENT
  # -----------------------------------------------------------------
  def descend_step(self):
    print("EV3: Starting descend_step()")

    # STEP 1 — Move forward until robot starts tilting downward
    self.drive_base.drive(300, 0)
    while not self.detect_step_descending():
      wait(10)

    self.stop_robot()

    # STEP 2 — Lower carriage to stabilize descent
    self.operate_carriage(ClimbingDirections.DOWN)

    # Run carriage wheel motor while rolling
    watch = StopWatch()
    self.drive_base.drive(200, 0)
    self.back_motor.run(300)
    while watch.time() < 1000:
      wait(10)

    self.back_motor.stop()

    # STEP 3 — Robot should flatten out once it reaches next step
    while self.gyro_sensor.angle() < -2:  # still going down
      self.drive_base.drive(200, 0)
      wait(10)

    self.stop_robot()

    # STEP 4 — Raise carriage to reset posture
    self.operate_carriage(ClimbingDirections.UP)

    # Update step count
    self.number_of_steps -= 1
    


  
  def completed_ascent(self):
    """Spike: detecting black + distance jump. EV3: angle flattening at top."""
    if self.climbed_steps == self.number_of_steps:
      return True
    else:
      return False

  def completed_descent(self):
    """Spike version checks number_of_steps == 0."""
    return self.number_of_steps == 0

  
  def run(self):
    print("EV3: Starting run()")

    while not self.completed_ascent():
      print("EV3 run(): executing climb phase")
      step_detected = self.detect_step()
      if not self.detect_step():
        print("step detected run function: ")
        print(step_detected)
        self.move_forward()
      else:
        print("inside of else statement to climb step")
        self.climb_step()

    while not self.completed_descent():
      if not self.detect_step_descending():
        self.move_forward()
      else:
        self.descend_step()
  
    self.stop_robot()
    print("EV3: ascent complete")




if __name__ == '__main__':
    main()

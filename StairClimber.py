from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, TouchSensor
from pybricks.parameters import Port, Direction, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import SoundFile


class ClimbingDirections:
  UP = "up"
  DOWN = "down"


class StairClimberEV3:
  """
  EV3 version of Hayden's StairClimber class.
  Structure matches EXACTLY the Spike Prime StairClimber class.
  """

  def __init__(self, left_motor, right_motor,
                carriage_wheel_motor, carriage_motor,
                gyro_sensor, touch_sensor, brick: EV3Brick):

    self.left_motor = left_motor
    self.right_motor = right_motor
    self.carriage_wheel_motor = carriage_wheel_motor
    self.carriage_motor = carriage_motor
    self.gyro_sensor = gyro_sensor
    self.touch_sensor = touch_sensor
    self.brick = brick

    self.number_of_steps = 0

    wheel_diameter = 56
    axle_track = 120
    self.drive_base = DriveBase(self.left_motor, self.right_motor, wheel_diameter, axle_track)

    
    self.initialize_carriage_structure()

  def initialize_carriage_structure(self):
    """Recreates the initialization from LEGO’s EV3 example."""
    # Move the lift assembly upward until touch sensor is pressed
    self.right_motor.dc(-20)
    self.carriage_motor.dc(100)

    while not self.touch_sensor.pressed():
        wait(10)

    # Move down a bit
    self.carriage_motor.dc(-100)
    self.right_motor.dc(40)
    wait(50)

    # Move to calibrated zero
    self.carriage_motor.run_angle(-145, 510)
    self.right_motor.hold()
    self.carriage_motor.run_angle(-30, 44)
    self.carriage_motor.reset_angle(0)
    self.gyro_sensor.reset_angle(0)

  # -----------------------------------------------------------------
  # BASIC MOVEMENT
  # -----------------------------------------------------------------
  def move_forward(self, speed=500):
    """Matches Spike version: drive forward until detect_step becomes True."""
    print("EV3: inside move_forward")
    self.drive_base.drive(speed, 0)

    while not self.detect_step():
        wait(10)

    self.stop_robot()
    print("EV3: finished move_forward")

  def stop_robot(self):
    """Stops horizontal movement."""
    self.drive_base.stop()
    print("EV3: robot stopped")

  # -----------------------------------------------------------------
  # STEP DETECTION
  # -----------------------------------------------------------------
  def detect_step(self):
    """
    Spike version uses ultrasonic distance < 3 cm.
    EV3 version uses gyro angle rising above a threshold.
    """

    angle = self.gyro_sensor.angle()
    step_detected = angle > 8  # Equivalent to “tilting upward”
    print(f"EV3 detect_step(): angle={angle}, detected={step_detected}")
    return step_detected

  def detect_step_descending(self):
    angle = self.gyro_sensor.angle()
    return angle < -8

  # -----------------------------------------------------------------
  # CARRIAGE CONTROL 
  # -----------------------------------------------------------------
  def operate_carriage_motor(self, speed, target_angle):
    self.carriage_motor.run_target(speed, target_angle)

  def operate_carriage(self, direction):
    if direction == ClimbingDirections.UP:
      self.operate_carriage_motor(2000, 1000)
    else:
      self.operate_carriage_motor(-250, -1000)

  
  def climb_step(self):
    print("EV3: Starting climb_step()")

    # (1) Move forward until angle rises → approaching step
    self.move_forward()

    # (2) Raise carriage while wheels still driving
    self.operate_carriage(ClimbingDirections.UP)

    # Run carriage wheel motor for 1 second (Spike method)
    watch = StopWatch()
    while watch.time() < 1000:
        self.carriage_wheel_motor.run(200)

    # (3) Pull robot fully onto step
    self.operate_carriage(ClimbingDirections.DOWN)

    self.number_of_steps += 1
    print(f"EV3 climbed step #{self.number_of_steps}")

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
  self.carriage_wheel_motor.run(300)
  while watch.time() < 1000:
      wait(10)

  self.carriage_wheel_motor.stop()

  # STEP 3 — Robot should flatten out once it reaches next step
  while self.gyro_sensor.angle() < -2:  # still going down
      self.drive_base.drive(200, 0)
      wait(10)

  self.stop_robot()

  # STEP 4 — Raise carriage to reset posture
  self.operate_carriage(ClimbingDirections.UP)

  # Update step count
  self.number_of_steps -= 1
  print(f"EV3 descended, remaining steps = {self.number_of_steps}")


  
  def completed_ascent(self):
    """Spike: detecting black + distance jump. EV3: angle flattening at top."""
    angle = self.gyro_sensor.angle()

    if angle < 2:  # robot levels off at top of stairs
      print("EV3: completed_ascent() TRUE")
      return True

    return False

  def completed_descent(self):
    """Spike version checks number_of_steps == 0."""
    return self.number_of_steps == 0

  
  def run(self):
    print("EV3: Starting run()")

    while not self.completed_ascent():
      print("EV3 run(): executing climb phase")
      self.drive_base.drive(500, 0)

      # Run both carriage motors simultaneously (non-blocking)
      self.carriage_motor.run(700)
      self.carriage_wheel_motor.run(700)

      watch = StopWatch()
      while watch.time() < 2000:
        wait(10)

      self.drive_base.stop()
      self.carriage_motor.stop()
      self.carriage_wheel_motor.stop()

      self.drive_base.drive(500, 0)
      self.operate_carriage(ClimbingDirections.DOWN)

    while not self.completed_descent():
      if not self.detect_step_descending():
        self.drive_base.drive(300,0)
      else:
        self.descend_step()
  
    self.stop_robot()
    print("EV3: ascent complete")

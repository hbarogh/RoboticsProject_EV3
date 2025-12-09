from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, TouchSensor
from pybricks.parameters import Port, Direction
from stairclimber_ev3 import StairClimberEV3


def main():
    ev3 = EV3Brick()

    left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    right_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
    carriage_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
    carriage_wheel_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)

    gyro_sensor = GyroSensor(Port.S2)
    touch_sensor = TouchSensor(Port.S3)

    robot = StairClimberEV3(left_motor, right_motor,
                            carriage_wheel_motor, carriage_motor,
                            gyro_sensor, touch_sensor, ev3)

    robot.run()


if __name__ == '__main__':
    main()

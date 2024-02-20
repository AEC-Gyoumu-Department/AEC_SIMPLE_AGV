import RPi.GPIO as GPIO
from gpiozero import DistanceSensor
import time

class Robot:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(17, GPIO.OUT)  # IN1
        GPIO.setup(18, GPIO.OUT)  # IN2
        GPIO.setup(22, GPIO.OUT)  # IN3
        GPIO.setup(23, GPIO.OUT)  # IN4
        self.IN1 = GPIO.PWM(17, 1000)
        self.IN2 = GPIO.PWM(18, 1000)
        self.IN3 = GPIO.PWM(22, 1000)
        self.IN4 = GPIO.PWM(23, 1000)
        self.IN1.start(0)
        self.IN2.start(0)
        self.IN3.start(0)
        self.IN4.start(0)

    def forward(self, v):
        self.IN1.ChangeDutyCycle(v)
        self.IN2.ChangeDutyCycle(0)
        self.IN3.ChangeDutyCycle(v)
        self.IN4.ChangeDutyCycle(0)

    def right(self, v):
        self.IN1.ChangeDutyCycle(v)
        self.IN2.ChangeDutyCycle(v)
        self.IN3.ChangeDutyCycle(v)
        self.IN4.ChangeDutyCycle(0)

    def left(self, v):
        self.IN1.ChangeDutyCycle(v)
        self.IN2.ChangeDutyCycle(0)
        self.IN3.ChangeDutyCycle(v)
        self.IN4.ChangeDutyCycle(v)

    def right_spin(self, v):
        self.IN1.ChangeDutyCycle(0)
        self.IN2.ChangeDutyCycle(v)
        self.IN3.ChangeDutyCycle(v)
        self.IN4.ChangeDutyCycle(0)

    def left_spin(self, v):
        self.IN1.ChangeDutyCycle(v)
        self.IN2.ChangeDutyCycle(0)
        self.IN3.ChangeDutyCycle(0)
        self.IN4.ChangeDutyCycle(v)

    def back(self, v):
        self.IN1.ChangeDutyCycle(0)
        self.IN2.ChangeDutyCycle(v)
        self.IN3.ChangeDutyCycle(0)
        self.IN4.ChangeDutyCycle(v)

    def stop(self):
        self.IN1.ChangeDutyCycle(100)
        self.IN2.ChangeDutyCycle(100)
        self.IN3.ChangeDutyCycle(100)
        self.IN4.ChangeDutyCycle(100)

    def release(self):
        self.IN1.ChangeDutyCycle(0)
        self.IN2.ChangeDutyCycle(0)
        self.IN3.ChangeDutyCycle(0)
        self.IN4.ChangeDutyCycle(0)

class Photo_sensor:
    def __init__(self):
        GPIO.setup(6, GPIO.IN)  # sensor_1
        GPIO.setup(7, GPIO.IN)  # sensor_2
        GPIO.setup(8, GPIO.IN)  # sensor_3

    def read_sensor(self):
        return [GPIO.input(6), GPIO.input(7), GPIO.input(8)]

class Ultrasonic_sensor:
    def __init__(self, trig_pin, echo_pin):
        self.sensor = DistanceSensor(echo=echo_pin, trigger=trig_pin, max_distance=4)  # assumindo max_distance de 4 metros

    def get_distance(self):
        return self.sensor.distance * 100  # convertendo de metros para centímetros

GPIO.cleanup()
GPIO.setwarnings(False)
robot = Robot()
sensor = Photo_sensor()
us_sensor_1 = Ultrasonic_sensor(trig_pin=10, echo_pin=9)  # Exemplo de pinos para sensor ultrassônico 1
#us_sensor_2 = Ultrasonic_sensor(trig_pin=11, echo_pin=12)  # Exemplo de pinos para sensor ultrassônico 2
#us_sensor_3 = Ultrasonic_sensor(trig_pin=13, echo_pin=14)  # Exemplo de pinos para sensor ultrassônico 3
value_old = [0, 0, 0]
time.sleep(1)

while True:
#    print("o codigo esta rodando")
    value = sensor.read_sensor()
#    print("o codigo esta rodando 1.2")
    distance_1 = us_sensor_1.get_distance()
#    print("o codigo esta rodando2")
    if distance_1 < 100:
        robot.stop()
        print("AGV一時停止")
    else:
        # Centro é preto
        if value[1] == 0:
            if value[0] == value[2]:
                robot.forward(80)
                print("AGV真っ直ぐ")
            elif value[0] == 0:
                robot.left_spin(80)
                print("AGV左へ曲がります")
            elif value[2] == 0:
                robot.right_spin(80)
                print("AG右へ曲がります")
# Centro é branco
        else:
            if value[0] == 0:
                robot.left_spin(80)
                print("AGV左")
            elif value[2] == 0:
                robot.right_spin(80)
                print("AGV右")
            else:
                if value_old[0] == 0:
                    robot.left_spin(100)
                    print("AGV左sleep")
                    time.sleep(0.2)
                elif value_old[2] == 0:
                    robot.right_spin(100)
                    print("AGV右sleep")
                    time.sleep(0.2)
                else:
                    robot.forward(80)
                    print("AGVまっすぐ")
    value_old = value

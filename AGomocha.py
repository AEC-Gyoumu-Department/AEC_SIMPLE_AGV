#下記のコードは、以下のページで見つかったコードを改変したものです。
#https://hellobreak.net/raspberry-pi-pico-line-trace-sensor3/
#ラズベリーパイ用のPythonプログラミング言語に変更して、ソナーセンサーに反応するアルゴリズムを追加しました。

import RPi.GPIO as GPIO
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
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        GPIO.setup(trig_pin, GPIO.OUT)
        GPIO.setup(echo_pin, GPIO.IN)

    def get_distance(self):
        GPIO.output(self.trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, False)

        start_time = time.time()
        stop_time = time.time()

        while GPIO.input(self.echo_pin) == 0:
            start_time = time.time()

        while GPIO.input(self.echo_pin) == 1:
            stop_time = time.time()

        time_elapsed = stop_time - start_time
        distance = (time_elapsed * 34300) / 2  # Speed of sound = 34300 cm/s
        return distance

robot = Robot()
sensor = Photo_sensor()
us_sensor_1 = Ultrasonic_sensor(trig_pin=10, echo_pin=9)  # Exemplo de pinos para sensor ultrassônico 1
us_sensor_2 = Ultrasonic_sensor(trig_pin=11, echo_pin=12)  # Exemplo de pinos para sensor ultrassônico 2
us_sensor_3 = Ultrasonic_sensor(trig_pin=13, echo_pin=14)  # Exemplo de pinos para sensor ultrassônico 3
value_old = [0, 0, 0]
time.sleep(1)

while True:
    value = sensor.read_sensor()
    distance_1 = us_sensor_1.get_distance()
    distance_2 = us_sensor_2.get_distance()
    distance_3 = us_sensor_3.get_distance()

    if distance_1 < 100 or distance_2 < 100 or distance_3 < 100:
        robot.stop()
    else:
        # Centro é preto
        if value[1] == 1:
            if value[0] == value[2]:
                robot.forward(80)
            elif value[0] == 1:
                robot.left_spin(80)
            elif value[2] == 1:
                robot.right_spin(80)
                
        # Centro é branco
        else:
            if value[0] == 1:
                robot.left_spin(80)
            elif value[2] == 1:
                robot.right_spin(80)
            else:
                if value_old[0] == 1:
                    robot.left_spin(100)
                    time.sleep(0.2)
                elif value_old[2] == 1:
                    robot.right_spin(100)
                    time.sleep(0.2)
                else:
                    robot.forward(80)
    value_old = value

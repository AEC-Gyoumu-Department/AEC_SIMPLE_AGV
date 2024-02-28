# Line tracer and motor motion reference is based in the code finded in https://hellobreak.net/raspberry-pi-pico-line-trace-sensor3/
#Posted by @KeiMameshiba
#The aruco mark reference, (when it will be implemented) come form openCv for python Docs.
#The sonar code used by us is based on codes from experiments and products developed internally at the company.
#Thanks everyone!! Yours contents was of great help.
#
import RPi.GPIO as GPIO
from gpiozero import DistanceSensor, LED
import time
import curses

class Robot:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(17, GPIO.OUT)  # IN1
        GPIO.setup(18, GPIO.OUT)  # IN2
        GPIO.setup(22, GPIO.OUT)  # IN3
        GPIO.setup(23, GPIO.OUT)  # IN4
        self.IN1 = GPIO.PWM(17, 10000) #OUT1 Left_Motor Negative Pin
        self.IN2 = GPIO.PWM(18, 10000) #OUT2 Left_Motor Positive Pin
        self.IN3 = GPIO.PWM(22, 10000) #OUT3 Right_Motor Positive Pin
        self.IN4 = GPIO.PWM(23, 10000) #OUT4 Right_Motor Negative Pin
        self.IN1.start(0)
        self.IN2.start(0)
        self.IN3.start(0)
        self.IN4.start(0)

    def Left_Motor_forward(self,v):
        self.IN1.ChangeDutyCycle(v)
        self.IN2.ChangeDutyCycle(0)

    def Left_Motor_back(self,v):
        self.IN1.ChangeDutyCycle(0)
        self.IN2.ChangeDutyCycle(v)

    def Left_Motor_stop(self):
        self.IN1.ChangeDutyCycle(100)
        self.IN2.ChangeDutyCycle(100)

    def Right_Motor_forward(self,v):
        self.IN3.ChangeDutyCycle(v)
        self.IN4.ChangeDutyCycle(0)

    def Right_Motor_back(self,v):
        self.IN3.ChangeDutyCycle(0)
        self.IN4.ChangeDutyCycle(v)

    def Right_Motor_stop(self):
        self.IN3.ChangeDutyCycle(100)
        self.IN4.ChangeDutyCycle(100)

    def forward(self, v):
        self.Left_Motor_forward(v)
        self.Right_Motor_forward(v)

    def back(self, v):
        self.Left_Motor_back(v)
        self.Right_Motor_back(v)

    def turn_right(self, v):
        self.Left_Motor_forward(v)
        self.Right_Motor_forward(0)

    def turn_left(self, v):
        self.Left_Motor_forward(0)
        self.Right_Motor_forward(v)

    def right_spin(self, v):
        self.Left_Motor_forward(v)
        self.Right_Motor_back(v)

    def left_spin(self, v):
        self.Right_Motor_forward(v)
        self.Left_Motor_back(v)

    def stop(self):
        self.Right_Motor_stop()
        self.Left_Motor_stop()

    def release(self):
        self.IN1.ChangeDutyCycle(0)
        self.IN2.ChangeDutyCycle(0)
        self.IN3.ChangeDutyCycle(0)
        self.IN4.ChangeDutyCycle(0)

class LEDController:

    def __init__(self, green_pin, red_pin):
        self.green_led = LED(green_pin)
        self.red_led = LED(red_pin)
        self.green_led.off()
        self.red_led.off()
        self.red_is_bliking = 0
        self.green_is_blinking = 0

    def green_on(self):
        if self.green_is_blinking == 0:
            self.green_led.blink(on_time=0.5, off_time=0.5)
            self.green_is_blinking = 1

    def green_off(self):
        self.green_led.off()
        self.green_is_blinking = 0

    def red_on(self):
        if self.red_is_bliking == 0:
            self.red_led.blink(on_time=0.5, off_time=0.5)
            self.red_is_blinking = 1
    def red_off(self):
        self.red_led.off()
        self.red_is_blinking = 0


class Photo_sensor:
    def __init__(self):
        GPIO.setup(6, GPIO.IN)  # sensor_1
        GPIO.setup(7, GPIO.IN)  # sensor_2
        GPIO.setup(8, GPIO.IN)  # sensor_3

    def read_sensor(self):
        sensor_dict = {}
        sensor_dict["left_sensor"] = GPIO.input(8)
        sensor_dict["center_sensor"] = GPIO.input(7)
        sensor_dict["right_sensor"] = GPIO.input(6)
        return sensor_dict

class Ultrasonic_sensor:
    def __init__(self, trig_pin, echo_pin):
        self.sensor = DistanceSensor(echo=echo_pin, trigger=trig_pin, max_distance=4)  # assumindo max_distance de 4 metros

    def get_distance(self):
        return self.sensor.distance * 100  # convertendo de metros para centímetros

GPIO.cleanup()
GPIO.setwarnings(False)
robot = Robot()
photosensor = Photo_sensor()
us_sensor_1 = Ultrasonic_sensor(trig_pin=10, echo_pin=9)  # Exemplo de pinos para sensor ultrassônico 1
#us_sensor_2 = Ultrasonic_sensor(trig_pin=11, echo_pin=12)  # Exemplo de pinos para sensor ultrassônico 2
#us_sensor_3 = Ultrasonic_sensor(trig_pin=13, echo_pin=14)  # Exemplo de pinos para sensor ultrassônico 3
photosensor_value_old = {"left_sensor":0, "center_sensor":0, "right_sensor":0}
led_controller = LEDController(green_pin=24, red_pin=25)
time.sleep(1)
# Configurações iniciais do curses
stdscr = curses.initscr()
curses.curs_set(0)  # Esconde o cursor
stdscr.clear()       # Limpa a tela

while True:
    stdscr.refresh()
    photosensor_value = photosensor.read_sensor()
    distance_1 = us_sensor_1.get_distance()
    if distance_1 < 50:
        robot.stop()
        stdscr.addstr(1, 0, "AGV一時停止")
        led_controller.green_off()
        led_controller.red_on()
    else:
        led_controller.green_on()
        led_controller.red_off()
        # Center is Black
        if photosensor_value["center_sensor"] == 1:
            stdscr.addstr(0, 0, "Center is BLACK")
            if photosensor_value["left_sensor"] == photosensor_value["right_sensor"]:
                robot.forward(80)
                stdscr.addstr(1, 0, "FULL SPEED")
                stdscr.addstr(2, 0, "BLACK Go AHEAD               ")
            elif photosensor_value["left_sensor"] == 1:
                robot.turn_left(90)
                stdscr.addstr(1, 0, "FULL SPEED")
                stdscr.addstr(2, 0,"BLACK Turning LEFT           ")
                time.sleep(0.1)
            elif  photosensor_value["right_sensor"] == 1:
                robot.turn_right(90)
                stdscr.addstr(1, 0, "FULL SPEED")
                stdscr.addstr(2, 0,"BLACK Turning RIGHT          ")
                time.sleep(0.1)
    # Center Is White
        else:
            stdscr.addstr(0, 0, "CENTER IS WHITE")
            if photosensor_value["left_sensor"] == 1:
                robot.turn_left(90)
                stdscr.addstr(1, 0, "FULL SPEED")
                stdscr.addstr(2, 0,"WHITE Turning LEFT           ")
                time.sleep(0.1)
            elif photosensor_value["right_sensor"] == 1:
                robot.turn_right(90)
                stdscr.addstr(1, 0, "FULL SPEED")
                stdscr.addstr(2, 0,"WHITE Turning RIGHT          ")
                time.sleep(0.1)
            else:
                if photosensor_value_old["left_sensor"] == 1:
                    robot.left_spin(90)
                    stdscr.addstr(1, 0, "FULL SPEED")
                    stdscr.addstr(2, 0,"LOST Spinning Left       ")
                    time.sleep(0.1)
                elif photosensor_value_old["right_sensor"] == 1:
                    robot.right_spin(90)
                    stdscr.addstr(1, 0, "FULL SPEED")
                    stdscr.addstr(2, 0,"LOST Spinning Right      ")
                    time.sleep(0.1)
                else:
                    robot.forward(80)
                    stdscr.addstr(1, 0, "SLOW SPEED")
                    stdscr.addstr(2, 0,"LOST slowlyng go ahead")
    photosensor_value_old = photosensor_value
    time.sleep(0.001)

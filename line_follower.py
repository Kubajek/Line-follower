import cv2
import time
import numpy as np
import board
import busio
import adafruit_pca9685
import RPi.GPIO as GPIO
from shape_recognition import detect_shape
from adafruit_servokit import ServoKit
from line_detection import line_follower
from client import call_orders

# Adres I2C modułu PCA9685
PCA9685_I2C_ADDRESS = 0x40

SPEED = 0x3000
TSPEED = 0x8000

# Numer kanału dla silnika DC [Numery wyprowadzeń PCA9685]
PRZ_PR_AENABLE = 0
PRZ_LEW_BENABLE = 1
TYL_PR_AENABLE = 2
TYL_LEW_BENABLE = 3
WIDLY = 15

# Numer pinu GPIO dla kierunku obrotu silnika
PRZ_PR_APHASE = 20
PRZ_LEW_BPHASE = 16
TYL_PR_APHASE = 14
TYL_LEW_BPHASE = 15

# Numery pinów GPIO dla stałego stanu wysokiego [Ustawienie DRV3885 w tryb phase i enable]
PRZ_MODE = 21
TYL_MODE = 18

# Inicjalizacja magistrali I2C
i2c_bus = busio.I2C(board.SCL, board.SDA)

# Inicjalizacja obiektu PCA9685
pca = adafruit_pca9685.PCA9685(i2c_bus, address=PCA9685_I2C_ADDRESS)
pca.frequency = 100  # Ustawienie częstotliwości (w Hz)

kit = ServoKit(channels=16)

# Inicjalizacja pinów GPIO
GPIO.setmode(GPIO.BCM)  # Ustawienie numeracji pinów po numeracji GPIO
GPIO.setup(PRZ_PR_APHASE, GPIO.OUT)
GPIO.setup(PRZ_LEW_BPHASE, GPIO.OUT)
GPIO.setup(TYL_PR_APHASE, GPIO.OUT)
GPIO.setup(TYL_LEW_BPHASE, GPIO.OUT)

GPIO.setup(PRZ_MODE, GPIO.OUT)
GPIO.setup(TYL_MODE, GPIO.OUT)

GPIO.output(PRZ_MODE, GPIO.HIGH)
GPIO.output(TYL_MODE, GPIO.HIGH)


# Funkcja ustawiająca kierunek obrotu silnika
def set_motor_direction(motor, direction):
    if motor == "PRZ_PR":
        if direction == "PRZ":
            GPIO.output(PRZ_PR_APHASE, GPIO.LOW)
        if direction == "TYL":
            GPIO.output(PRZ_PR_APHASE, GPIO.HIGH)
    if motor == "PRZ_LEW":
        if direction == "PRZ":
            GPIO.output(PRZ_LEW_BPHASE, GPIO.HIGH)
        if direction == "TYL":
            GPIO.output(PRZ_LEW_BPHASE, GPIO.LOW)
    if motor == "TYL_PR":
        if direction == "PRZ":
            GPIO.output(TYL_PR_APHASE, GPIO.LOW)
        if direction == "TYL":
            GPIO.output(TYL_PR_APHASE, GPIO.HIGH)
    if motor == "TYL_LEW":
        if direction == "PRZ":
            GPIO.output(TYL_LEW_BPHASE, GPIO.HIGH)
        if direction == "TYL":
            GPIO.output(TYL_LEW_BPHASE, GPIO.LOW)


# Funkcja do jazdy w przód
def move_forward(stime):
    set_motor_direction("PRZ_PR", "PRZ")
    set_motor_direction("PRZ_LEW", "PRZ")
    set_motor_direction("TYL_PR", "PRZ")
    set_motor_direction("TYL_LEW", "PRZ")
    pca.channels[PRZ_PR_AENABLE].duty_cycle = SPEED
    pca.channels[PRZ_LEW_BENABLE].duty_cycle = SPEED
    pca.channels[TYL_PR_AENABLE].duty_cycle = SPEED
    pca.channels[TYL_LEW_BENABLE].duty_cycle = SPEED
    time.sleep(stime)


# Funkcja do jazdy w tył
def move_reverse(stime):
    set_motor_direction("PRZ_PR", "TYL")
    set_motor_direction("PRZ_LEW", "TYL")
    set_motor_direction("TYL_PR", "TYL")
    set_motor_direction("TYL_LEW", "TYL")
    pca.channels[PRZ_PR_AENABLE].duty_cycle = SPEED
    pca.channels[PRZ_LEW_BENABLE].duty_cycle = SPEED
    pca.channels[TYL_PR_AENABLE].duty_cycle = SPEED
    pca.channels[TYL_LEW_BENABLE].duty_cycle = SPEED
    time.sleep(stime)


# Funkcja do jazdy w lewo
def move_left(stime):
    set_motor_direction("PRZ_PR", "TYL")
    set_motor_direction("PRZ_LEW", "PRZ")
    set_motor_direction("TYL_PR", "TYL")
    set_motor_direction("TYL_LEW", "PRZ")
    pca.channels[PRZ_PR_AENABLE].duty_cycle = TSPEED
    pca.channels[PRZ_LEW_BENABLE].duty_cycle = TSPEED
    pca.channels[TYL_PR_AENABLE].duty_cycle = TSPEED
    pca.channels[TYL_LEW_BENABLE].duty_cycle = TSPEED
    time.sleep(stime)


# Funkcja do jazdy w prawo
def move_right(stime):
    set_motor_direction("PRZ_PR", "PRZ")
    set_motor_direction("PRZ_LEW", "TYL")
    set_motor_direction("TYL_PR", "PRZ")
    set_motor_direction("TYL_LEW", "TYL")
    pca.channels[PRZ_PR_AENABLE].duty_cycle = TSPEED
    pca.channels[PRZ_LEW_BENABLE].duty_cycle = TSPEED
    pca.channels[TYL_PR_AENABLE].duty_cycle = TSPEED
    pca.channels[TYL_LEW_BENABLE].duty_cycle = TSPEED
    time.sleep(stime)


def stop_move():
    pca.channels[PRZ_PR_AENABLE].duty_cycle = 0x0000
    pca.channels[PRZ_LEW_BENABLE].duty_cycle = 0x0000
    pca.channels[TYL_PR_AENABLE].duty_cycle = 0x0000
    pca.channels[TYL_LEW_BENABLE].duty_cycle = 0x0000


def box_up():
    angles = list(range(45, -1, -1))
    for angle in angles:
        kit.servo[WIDLY].angle = angle
        time.sleep(0.015)


def box_down():
    angles = list(range(0, 46))
    for angle in angles:
        kit.servo[WIDLY].angle = angle
        time.sleep(0.015)


# Inicjalizacja kamery
cap = cv2.VideoCapture(0)

# Ustawienie rozdzielczości kamery
cap.set(3, 640)  # szerokość
cap.set(4, 480)  # wysokość

path_interrupt = False
wait_tr = False
wait_sq = False

licznik_tr = 0
licznik_omin_tr = 0
licznik_sq = 0
end_route = False

start_programu = True

try:
    while True:
        # Odczytanie obrazu z kamery
        ret, frame = cap.read()

        _, direction = line_follower(frame)
        _, _, path_interrupt = detect_shape(frame, "triangle")
        _, path_stop, _ = detect_shape(frame, "square")

        if (path_stop and licznik_sq == 0):
            stop_move()
            if(end_route):
                move_left(3.75)
                wait_sq = True
                path_stop = False
                end_route = False
                stop_move()
                box_down()
            else:
                if (not(start_programu)):
                    move_reverse(2)
                    box_down()
                    move_forward(2)
                    move_left(3.75)
                while True:
                    route = call_orders('172.16.10.195')

                    if (route != 'wait'):
                        break

                time.sleep(2)
                box_up()
                end_route = True

        elif (path_interrupt and licznik_tr == 0):
            move_reverse(1.8)

            if (len(route) == 1):
                move_left(0)
            if (len(route) == 2):
                if (licznik_omin_tr == 0):
                    move_right(0)
                else:
                    move_reverse(0)

            wait = True
            path_interrupt = False
            time.sleep(1.8)
            stop_move()
            licznik_omin_tr += 1
        else:
                    ######################################
                    ####LINE FOLLOWER ALE JAKO FUNKCJA####
                    ######################################
            if(direction==1):
                print("Turn Left")
                move_left(0.05)
                stop_move()
            elif(direction==2):
                print("On Track")
                move_reverse(0)
            elif(direction==3):
                print("Turn Right")
                move_right(0.05)
                stop_move()
            else:
                print("Nie wiem gdzie jechac")


        if (wait_tr):
            licznik_tr += 1

        if (licznik_tr >= 20):
            licznik_tr = 0
            wait_tr = False

        if (wait_sq):
            licznik_sq += 1

        if (licznik_sq >= 20):
            licznik_sq = 0
            wait_sq = False

        # Zakończenie pętli po naciśnięciu klawisza 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    # Zatrzymanie kamery i zamknięcie okna
    stop_move()
    cap.release()
    cv2.destroyAllWindows()
    pca.deinit()
    GPIO.cleanup()

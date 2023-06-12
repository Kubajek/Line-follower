import board
import time
import busio
import adafruit_pca9685
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit

# Adres I2C modułu PCA9685
PCA9685_I2C_ADDRESS = 0x40

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

stop_move()
from pyPS4Controller.controller import Controller
import time

from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685


# Wires from jetson
# brown - SDA
# white - SCL


i2c = busio.I2C(SCL, SDA)
# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 50

servo1 = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2600)


class ServosController(Controller):
    
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.fraction = 0.3
        servo1.fraction = self.fraction  
    
    # Step down
    def on_x_press(self):
        if self.fraction > 0.2:
            self.fraction -= 0.01
            servo1.fraction = self.fraction
            time.sleep(0.03)

    def on_x_release(self):
        pass
    
    # Step up
    def on_triangle_press(self):
        if self.fraction < 0.8:
            self.fraction += 0.01
            servo1.fraction = self.fraction
            time.sleep(0.03)


    def on_triangle_release(self):
        pass

    # Tilt up
    def on_square_press(self):
        self.fraction = 0.6
        servo1.fraction = self.fraction

    def on_square_release(self):
        pass

    # Tilt down
    def on_circle_press(self):
        self.fraction = 0.2
        servo1.fraction = self.fraction

    def on_circle_release(self):
        pass

    
controller = ServosController(
    interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen(timeout=60)


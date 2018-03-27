import threading, time, socket
import RPi.GPIO as GPIO

TCP_IP = '192.168.0.106'
TCP_PORT = 5005
BUFFER_SIZE = 20

class TCP_Server():
    def __init__(self):
        self.control = Control()
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((TCP_IP, TCP_PORT))
        self.s.listen(1)

    def tcp_server(self):
         while self.run_event.is_set():
             try:
                 self.s.settimeout(5)
                 conn, addr = self.s.accept()
                 data = conn.recv(BUFFER_SIZE)
                 if not data: break
                 else:
                    json_data = json.loads(data)
                    if json_data['command'] == 0x00: self.control.stop()
                    elif json_data['command'] == 0x01: self.control.forward()
                    elif json_data['command'] == 0x02: self.control.back()
                    elif json_data['command'] == 0x03: self.control.left()
                    elif json_data['command'] == 0x04: self.control.right()
                    elif json_data['command'] == 0x05: self.control.motor_veliocity(json_data['value'])
                    elif json_data['command'] == 0x06: self.control.motor_calibration(json_data['value'][0], json_data['value'][1])
                    #elif json_data['command'] == 0x0a: 
                 conn.send(data)
                 conn.close()
            except:
                pass

    def start(self):
        self.run_event = threading.Event()
        self.run_event.set()
        self.t1 = threading.Thread(target = self.tcp_server)
        self.t1.start()

        try:
            while True:
                time.sleep(0.01)
        except KeyboardInterrupt:
                self.run_event.clear()
                self.t1.join()

class Control():
    #pins
    gpio_forward_left = 35
    gpio_forward_right = 36
    gpio_back_left = 37
    gpio_back_right = 38
    gpio_enable_left = 31
    gpio_enable_right = 32
    motor_left = GPIO.PWM(gpio_enable_left, 200)
    motor_right = GPIO.PWM(gpio_enable_right, 200)

    #parameters
    left_motor = 100
    right_motor = 100
    veliocity = 100

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.gpio_forward_left, GPIO.OUT)
        GPIO.setup(self.gpio_forward_right, GPIO.OUT)
        GPIO.setup(self.gpio_back_left, GPIO.OUT)
        GPIO.setup(self.gpio_back_right, GPIO.OUT)
        GPIO.setup(self.gpio_enable_left, GPIO.OUT)
        GPIO.setup(self.gpio_enable_right, GPIO.OUT)
        self.motor_left.start(100)
        self.motor_right.start(100)

    def motor_calibration(self, left_motor, right_motor):
        self.left_motor = ((left_motor / 100) * (self.veliocity / 100) * 100)
        self.right_motor = ((right_motor / 100) * (self.veliocity / 100) * 100)
        self.set_motor_parameters(self)

    def motor_veliocity(self, veliocity):
        self.veliocity = veliocity
        self.set_motor_parameters(self)

    def forward(self):
        GPIO.output(self.gpio_forward_left, GPIO.HIGH)
        GPIO.output(self.gpio_forward_right, GPIO.HIGH)
        GPIO.output(self.gpio_back_left, GPIO.LOW)
        GPIO.output(self.back_right, GPIO.LOW)

    def back(self):
        GPIO.output(self.gpio_forward_left, GPIO.LOW)
        GPIO.output(self.gpio_forward_right, GPIO.LOW)
        GPIO.output(self.gpio_back_left, GPIO.HIGH)
        GPIO.output(self.back_right, GPIO.HIGH)

    def right(self):
        GPIO.output(self.gpio_forward_left, GPIO.HIGH)
        GPIO.output(self.gpio_forward_right, GPIO.LOW)
        GPIO.output(self.gpio_back_left, GPIO.LOW)
        GPIO.output(self.gpio_back_right, GPIO.HIGH)

    def left(self):
        GPIO.output(self.gpio_forward_left, GPIO.LOW)
        GPIO.output(self.gpio_forward_right, GPIO.HIGH)
        GPIO.output(self.gpio_back_left, GPIO.HIGH)
        GPIO.output(self.gpio_back_right, GPIO.LOW)

    def stop(self):
        GPIO.output(self.gpio_forward_left, GPIO.LOW)
        GPIO.output(self.gpio_forward_right, GPIO.LOW)
        GPIO.output(self.gpio_back_left, GPIO.LOW)
        GPIO.output(self.gpio_back_right, GPIO.LOW)
    
    def set_motor_parameters(self):
        self.motor_left.ChangeDutyCycle(self.left_motor)
        self.motor_right.ChangeDutyCycle(self.left_right)

if __name__ == "__main__":
    tcp_server = TCP_Server()
    tcp_server.start()

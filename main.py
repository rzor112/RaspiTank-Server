import threading, time, socket, json
import RPi.GPIO as GPIO

TCP_IP = '192.168.0.114'
TCP_PORT = 5005
BUFFER_SIZE = 1024

class TCP_Server():
    def __init__(self):
        self.control = Control()
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((TCP_IP, TCP_PORT))
        self.s.listen(1)

    def open_cv(self):
        while self.run_event.is_set() and self.control.auto_mode:
            print 'open_cv'

    def tcp_server(self):
         while self.run_event.is_set():
            try:
                self.s.settimeout(2)
                conn, addr = self.s.accept()
                while self.run_event.is_set():
                    data = conn.recv(BUFFER_SIZE)
                    if not data: break
                    else:
                        try:
                            print data
                            json_data = json.loads(data)
                            #------------------------------------------------------- Set Commands
                            if json_data['command'] == 0x00:    #stop
                                if not self.control.auto_mode:
                                    self.control.stop()
                                    conn.send(self.json_builder(True, 0x00, 0))
                                else:
                                    conn.send(self.json_builder(False, 0x00, 0))
                                
                            elif json_data['command'] == 0x01:    #forward
                                if not self.control.auto_mode:
                                    self.control.forward()
                                    conn.send(self.json_builder(True, 0x01, 0))
                                else:
                                    conn.send(self.json_builder(False, 0x01, 0))
                                
                            elif json_data['command'] == 0x02:    #back
                                if not self.control.auto_mode:
                                    self.control.back()
                                    conn.send(self.json_builder(True, 0x02, 0))
                                else:
                                    conn.send(self.json_builder(False, 0x02, 0))
                                
                            elif json_data['command'] == 0x03:    #left
                                if not self.control.auto_mode:
                                    self.control.left()
                                    conn.send(self.json_builder(True, 0x03, 0))
                                else:
                                    conn.send(self.json_builder(False, 0x03, 0))
                                
                            elif json_data['command'] == 0x04:    #right
                                if not self.control.auto_mode:
                                    self.control.right()
                                    conn.send(self.json_builder(True, 0x04, 0))
                                else:
                                    conn.send(self.json_builder(False, 0x04, 0))
                                
                            elif json_data['command'] == 0x05:    #set veliocity
                                if not self.control.auto_mode:
                                    self.control.motor_veliocity(json_data['value'])
                                    conn.send(self.json_builder(True, 0x05, 0))
                                else:
                                    conn.send(self.json_builder(False, 0x05, 0))
                                
                            elif json_data['command'] == 0x06:    #set calibration motor left
                                if not self.control.auto_mode:
                                    self.control.motor_calibration_left(json_data['value'])
                                    conn.send(self.json_builder(True, 0x06, 0))
                                else:
                                    conn.send(self.json_builder(False, 0x06, 0))
                                
                            elif json_data['command'] == 0x07:    #set calibration motor right
                                if not self.control.auto_mode:
                                    self.control.motor_calibration_right(json_data['value'])
                                    conn.send(self.json_builder(True, 0x07, 0))
                                else:
                                    conn.send(self.json_builder(False, 0x07, 0))

                            elif json_data['command'] == 0x08:  #set auto_mode true
                                self.control.auto_mode = True
                                conn.send(self.json_builder(True, 0x08, 0))

                            elif json_data['command'] == 0x09:  #set auto_mode false
                                self.control.auto_mode = False
                                conn.send(self.json_builder(True, 0x09, 0))
                                
                            #------------------------------------------------------- Read Commands
                            
                            elif json_data['command'] == 0xa0:    #read veliocity
                                conn.send(self.json_builder(True, 0xa0, self.control.veliocity))
                            
                            elif json_data['command'] == 0xa1:    #read calibration motor left
                                conn.send(self.json_builder(True, 0xa1, self.control.left_motor_calibration))
                                
                            elif json_data['command'] == 0xa2:    #read calibration motor right
                                conn.send(self.json_builder(True, 0xa2, self.control.right_motor_calibration))
                            
                            elif json_data['command'] == 0xa3:    #read auto_mode
                                 conn.send(self.json_builder(True, 0xa3, self.control.auto_mode))   

                            elif json_data['command'] == 0xff:  #ping
                                conn.send(self.json_builder(True, 0xff, 0))
                                
                        except Exception as e:
                            print e
                conn.close()
                self.control.stop()
            except Exception as e:
                pass
                
    def json_builder(self, status, command, value):
        json_data = {'ResponseStatus': status, 'data': {'command': command, 'value': value}}
        json_data = json.dumps(json_data)
        return str(json_data)
                
    def start(self):
        self.run_event = threading.Event()
        self.run_event.set()
        self.t1 = threading.Thread(target = self.tcp_server)
        self.t2 = threading.Thread(target = self.open_cv)
        self.t1.start()
        self.t2.start()

        try:
            while True:
                time.sleep(0.01)
        except KeyboardInterrupt:
                self.run_event.clear()
                self.t1.join()

class Control():
    #pin
    GPIO.setmode(GPIO.BOARD)
    gpio_forward_left = 13
    gpio_forward_right = 16
    gpio_back_left = 15
    gpio_back_right = 18
    gpio_enable_left = 11
    gpio_enable_right = 12

    #parameters
    left_motor_calibration = 100
    right_motor_calibration = 100
    veliocity = 100
    
    #definitions
    auto_mode = False

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.gpio_forward_left, GPIO.OUT)
        GPIO.setup(self.gpio_forward_right, GPIO.OUT)
        GPIO.setup(self.gpio_back_left, GPIO.OUT)
        GPIO.setup(self.gpio_back_right, GPIO.OUT)
        GPIO.setup(self.gpio_enable_left, GPIO.OUT)
        GPIO.setup(self.gpio_enable_right, GPIO.OUT)
        self.motor_left = GPIO.PWM(self.gpio_enable_left, 200)
        self.motor_right = GPIO.PWM(self.gpio_enable_right, 200)
        self.motor_left.start(100)
        self.motor_right.start(100)

    def motor_calibration_right(self, multipler):
        self.right_motor_calibration = int(multipler)
        self.set_motor_parameters()
        
    def motor_calibration_left(self, multipler):
        self.left_motor_calibration = int(multipler)
        self.set_motor_parameters()

    def motor_veliocity(self, veliocity):
        self.veliocity = veliocity
        self.set_motor_parameters()

    def forward(self):
        GPIO.output(self.gpio_forward_left, GPIO.HIGH)
        GPIO.output(self.gpio_forward_right, GPIO.HIGH)
        GPIO.output(self.gpio_back_left, GPIO.LOW)
        GPIO.output(self.gpio_back_right, GPIO.LOW)

    def back(self):
        GPIO.output(self.gpio_forward_left, GPIO.LOW)
        GPIO.output(self.gpio_forward_right, GPIO.LOW)
        GPIO.output(self.gpio_back_left, GPIO.HIGH)
        GPIO.output(self.gpio_back_right, GPIO.HIGH)

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
        self.motor_left.ChangeDutyCycle(((self.veliocity / 100.0) * (self.left_motor_calibration / 100.0)) * 100.0)
        self.motor_right.ChangeDutyCycle(((self.veliocity / 100.0) * (self.right_motor_calibration / 100.0)) * 100.0)

if __name__ == "__main__":
    tcp_server = TCP_Server()
    tcp_server.start()

import threading, time, socket, json
import RPi.GPIO as GPIO
import numpy as np
import cv2
import urllib2
from markers_module import *
from open_cv_module import *

TCP_IP = '192.168.0.114'
TCP_PORT = 5005
BUFFER_SIZE = 1024
camera_address = 'http://192.168.0.114:8081/'

def order_points(points):
    s = points.sum(axis=1)
    diff = np.diff(points, axis=1)
     
    ordered_points = np.zeros((4,2), dtype="float32")
 
    ordered_points[0] = points[np.argmin(s)]
    ordered_points[2] = points[np.argmax(s)]
    ordered_points[1] = points[np.argmin(diff)]
    ordered_points[3] = points[np.argmax(diff)]
 
    return ordered_points

def max_width_height(points):
 
    (tl, tr, br, bl) = points
 
    top_width = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    bottom_width = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    max_width = max(int(top_width), int(bottom_width))
 
    left_height = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    right_height = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    max_height = max(int(left_height), int(right_height))
 
    return (max_width,max_height)

class TCP_Server():
    def __init__(self):
        self.control = Control()
        self.manual_left_motor_calibration = self.control.left_motor_calibration
        self.manual_right_motor_calibration = self.control.right_motor_calibration
        self.manual_velocity = self.control.veliocity
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((TCP_IP, TCP_PORT))
        self.s.listen(1)

    def open_cv(self):
        bytes = ''
        while self.run_event.is_set():
            if self.control.auto_mode:
                try:
                    bytes += self.stream.read(1024)
                    a = bytes.find('\xff\xd8')
                    b = bytes.find('\xff\xd9')
                    if a!=-1 and b!=-1:
                        jpg = bytes[a:b+2]
                        bytes= bytes[b+2:]
                        im = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.IMREAD_COLOR)
                        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
                        edges = cv2.Canny(gray, 100, 200)
                        im2, contours, hierarchy = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:10]
                        for contour in contours:
                            perimeter = cv2.arcLength(contour, True)
                            approx = cv2.approxPolyDP(contour, 0.01*perimeter, True)
                            if len(approx) == 4:
                                topdown_quad = get_topdown_quad(gray, approx.reshape(4, 2))
                                resized_shape = resize_image(topdown_quad, 100.0)
                                if resized_shape[5, 5] > 120: continue
                                glyph_pattern = get_glyph_pattern(resized_shape, 100, 135)
                                glyph_found, glyph_rotation, glyph_substitute = match_glyph_pattern(glyph_pattern)
                                if glyph_found:
                                    dst = approx.reshape(4, 2)
                                    dst = order_points(dst)
                                    (tl, tr, br, bl) = dst
                                    min_x = min(int(tl[0]), int(bl[0]))
                                    min_y = min(int(tl[1]), int(tr[1]))
                                    for point in dst:
                                        point[0] = point[0] - min_x
                                        point[1] = point[1] - min_y
                                    (max_width,max_height) = max_width_height(dst)
                                    
                                    #engine controlling
                                    
                                    x_position = ((min_x + max_width)-70)-120

                                    if -50 < x_position <= 0:
                                        self.control.forward()
                                        self.control.motor_calibration_right(100)
                                        self.control.motor_calibration_left(100 - (x_position * -1))

                                    elif 0 < x_position < 50:
                                        self.control.forward()
                                        self.control.motor_calibration_left(100)
                                        self.control.motor_calibration_right(100 - x_position)

                                    elif -84 < x_position <= -50:
                                        self.control.forward()
                                        self.control.motor_calibration_right(100)
                                        self.control.motor_calibration_left(0)

                                    elif 50 <= x_position < 84:
                                        self.control.forward()
                                        self.control.motor_calibration_left(100)
                                        self.control.motor_calibration_right(0)

                                    elif x_position <= -84:
                                        self.control.left()
                                        self.control.motor_calibration_left(100)
                                        self.control.motor_calibration_right(100)

                                    elif 84 <= x_position:
                                        self.control.right()
                                        self.control.motor_calibration_left(100)
                                        self.control.motor_calibration_right(100)
                                    
                        if cv2.waitKey(1) ==27:
                            exit(0)
                except Exception as e:
                    print e
            else:
                time.sleep(0.1)

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
                                self.stream = urllib2.urlopen(camera_address, timeout = 2)
                                self.manual_left_motor_calibration = self.control.left_motor_calibration
                                self.manual_right_motor_calibration = self.control.right_motor_calibration
                                self.manual_velocity = self.control.veliocity
                                self.control.left_motor_calibration = 100
                                self.control.right_motor_calibration = 100
                                self.control.veliocity = 100
                                self.control.auto_mode = True
                                conn.send(self.json_builder(True, 0x08, 0))

                            elif json_data['command'] == 0x09:  #set auto_mode false
                                self.control.auto_mode = False
                                self.control.left_motor_calibration = self.manual_left_motor_calibration
                                self.control.right_motor_calibration = self.manual_right_motor_calibration
                                self.control.veliocity = self.manual_velocity
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
        self.control.thread_start()
        self.t1.start()
        self.t2.start()

        try:
            while True:
                time.sleep(0.01)
        except KeyboardInterrupt:
                self.control.run_event.clear()
                self.run_event.clear()
                self.control.t1.join()
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
    timeout = 0

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
        if self.auto_mode:
            self.timeout = 10

    def back(self):
        GPIO.output(self.gpio_forward_left, GPIO.LOW)
        GPIO.output(self.gpio_forward_right, GPIO.LOW)
        GPIO.output(self.gpio_back_left, GPIO.HIGH)
        GPIO.output(self.gpio_back_right, GPIO.HIGH)
        if self.auto_mode:
            self.timeout = 10

    def right(self):
        GPIO.output(self.gpio_forward_left, GPIO.HIGH)
        GPIO.output(self.gpio_forward_right, GPIO.LOW)
        GPIO.output(self.gpio_back_left, GPIO.LOW)
        GPIO.output(self.gpio_back_right, GPIO.HIGH)
        if self.auto_mode:
            self.timeout = 10

    def left(self):
        GPIO.output(self.gpio_forward_left, GPIO.LOW)
        GPIO.output(self.gpio_forward_right, GPIO.HIGH)
        GPIO.output(self.gpio_back_left, GPIO.HIGH)
        GPIO.output(self.gpio_back_right, GPIO.LOW)
        if self.auto_mode:
            self.timeout = 10

    def stop(self):
        GPIO.output(self.gpio_forward_left, GPIO.LOW)
        GPIO.output(self.gpio_forward_right, GPIO.LOW)
        GPIO.output(self.gpio_back_left, GPIO.LOW)
        GPIO.output(self.gpio_back_right, GPIO.LOW)
    
    def set_motor_parameters(self):
        self.motor_left.ChangeDutyCycle(((self.veliocity / 100.0) * (self.left_motor_calibration / 100.0)) * 100.0)
        self.motor_right.ChangeDutyCycle(((self.veliocity / 100.0) * (self.right_motor_calibration / 100.0)) * 100.0)

    def thread_start(self):
        self.run_event = threading.Event()
        self.run_event.set()
        self.t1 = threading.Thread(target = self.timeout_stop)
        self.t1.start()

    def timeout_stop(self):
        while self.run_event.is_set():
            if self.timeout > 0:
                self.timeout -= 1
                time.sleep(0.1)
            elif self.timeout == 0:
                self.timeout = -1
                self.stop()
            else:
                time.sleep(0.1)

if __name__ == "__main__":
    tcp_server = TCP_Server()
    tcp_server.start()

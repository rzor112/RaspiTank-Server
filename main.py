import threading, time, socket
import RPi.GPIO as GPIO

TCP_IP = '192.168.0.106'
TCP_PORT = 5005
BUFFER_SIZE = 20

GPIO.setmode(GPIO.BOARD)

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
                if data == "on": self.control.on()
                if data == "off": self.control.off()
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
    def __init__(self):
        GPIO.setup(40, GPIO.OUT)
    
    def on(self):
        GPIO.output(40, GPIO.HIGH)

    def off(self):
        GPIO.output(38, GPIO.LOW)

if __name__ == "__main__":
    tcp_server = TCP_Server()
    tcp_server.start()

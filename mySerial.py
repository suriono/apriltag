import serial,time

class MySerial:
   ser = serial.Serial()
   ser.port = '/dev/ttyACM0'

   # ---------------------------
   #def __init__(self, portcom):
   def __init__(self):
      #self.ser.baudrate = 115200
      self.ser.baudrate = 57600
      self.ser.timeout = 1

   # ---------------------------
   def findPort(self):  # find Arduino serial port
      def testConnection():
         try:
            self.ser.open()
            self.serWrite('{"cmd":"TestConnection"}')
            readline = self.ser.readline()
            print(readline.decode())
            self.ser.close()

            return (readline.decode().rstrip() == "OK")
         except:
            return False

      self.ser.port = '/dev/ttyACM0'
      print(self.ser.port)
      if testConnection():
         return "Port: " + self.ser.port
      self.ser.port = '/dev/ttyACM1'
      print(self.ser.port)
      if testConnection():
         return "Port: " + self.ser.port
         return
      self.ser.port = '/dev/ttyACM2'
      print(self.ser.port)
      if testConnection():
         return "Port: " + self.ser.port
         return
      self.ser.port = '/dev/ttyACM0'

   # ---------------------------
   def serOpen(self):
      try: 
         self.ser.open()
         print("Port: ", self.ser.port)
      except Exception as e:
         # print ("Error open serial port: " , str(e))
         self.findPort()
         # self.ser.close()
         try: 
            self.ser.open()
            print("Port: ", self.ser.port)
         except Exception as e:
            # print ("Error open serial port: " , str(e))
            exit()
      time.sleep(2.0)

   # ---------------------------
   def serClose(self):
      self.ser.close()

   # ---------------------------
   def serWrite(self, message):
      if self.ser.isOpen():
         #time.sleep(0.001)
         time.sleep(0.1)
         try:
            self.ser.flushInput() #flush input buffer, discarding all its contents
            self.ser.flushOutput()
            #time.sleep(0.1)
            time.sleep(0.2)
#            print("sending serial: ", message)
            self.ser.write(str.encode(message))  
            #self.ser.write(message)  
         except Exception as e:
            print ("Error to write communication ...")
      else:
         print("Cannot open serial port")

   # ---------------------------
   def serWrite_and_Read(self, message, wait_sec=0):
      self.serWrite(message)
      if wait_sec > 0: time.sleep(wait_sec)
      return self.serReadline_keep_trying()

   # ---------------------------
   def serReadline(self):
      if self.ser.isOpen():
         return self.ser.readline()
      else:
         print("Cannot open serial port")

   # ---------------------------
   def serReadline_keep_trying(self):
      instr = ""
      readstr = self.serReadline()
      while len(readstr) > 0:
        instr = instr + str(readstr)
        readstr = self.serReadline()
      return instr

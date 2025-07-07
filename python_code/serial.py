import serial
import serial.tools.list_ports
import traceback


class ArduinoCommunication:
    def __init__(self):
        self.ports = list(serial.tools.list_ports.comports())

        self.portsInfo()
        self.COM_port = self.ports[0].device

        self.ser = self.connectSerial()

    def portsInfo(self):
        for port in self.ports:
            print(f"Порт: {port.device}")
            print(f"Описание: {port.description}")
            print(f"Производитель: {port.manufacturer}")
        
        if len(self.ports) != 1: raise("Непредусмотренное количество портов")


    def connectSerial(self, baudrate=115200):
        return serial.Serial(port=self.COM_port, baudrate=baudrate, timeout=3)


    def disconectSerial(self):
        if self.ser.isOpen():
            self.ser.close()


    def readArduinoData(self, wait=False):
        if not self.ser.inWaiting() and not wait:
            print("P.msg. > Сообщений нет. Буфер пуст")
            return
        while(self.ser.inWaiting() or wait):
            print(self.ser.readline().decode(), end='')
            if wait: wait=False


    def sendArduinoData(self, data):
        if isinstance(data, str):
            data = data.encode()
        elif not isinstance(data, bytes):
            raise Exception("unknown message type")
        
        self.ser.write(data)

            
    def setAnglesServo(self, angles):
        command = '<s'
        for angle in angles:
            command += f"{angle:03}"
        command += '>'
        self.ser.write(command.encode())


    def getData(self):
        self.ser.write(b'<d>')
        mpu_data = self.ser.readline()
        mpu_data = mpu_data.decode().strip()
        dist = self.ser.readline()
        dist = dist.decode().strip()
        data = '\t'.join([mpu_data, dist])
        return [float(a) for a in data.split('\t')]

if __name__ == "__main__":
    arduino = ArduinoCommunication()
    
    try:
        arduino.readArduinoData(wait=True)

        arduino.setAnglesServo([110, 45, 110, 135, 110, 135, 110, 45])

        print(arduino.getData())
        arduino.readArduinoData(wait=True)
    except Exception as e:
        print(f"Произошла ошибка: {e}")
        print(traceback.format_exc())
    finally:
        arduino.disconectSerial()

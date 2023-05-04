import pydirectinput
import serial
import argparse
import time
import logging
from pycaw.pycaw import AudioUtilities

class MyControllerMap:
    def __init__(self):
        self.button = {'VERDE': 1, 'VERMELHO': 2, 'AMARELO': 3, 'AZUL': 4, 'START': 5, 'UP': 6, 'DOWN': 7, 'LEFT': 8, 'RIGHT': 9} # Fast forward (10 seg) pro Youtube

class SerialControllerInterface:
    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.incoming = '0'
        pydirectinput.PAUSE = 0  ## remove delay
    
    def update(self):
        ## Sync protocol
        while self.incoming != b'X':
            self.incoming = self.ser.read()
            print("esperando X")
            logging.debug("Received INCOMING: {}".format(self.incoming))

        data = self.ser.read()
        logging.debug("Received DATA: {}".format(data))
        # Protocolo do Handshake
        if data == b'W':
            self.ser.write(b'w')
            print("Mandando w de volta pro microship")
            logging.info("Mandando w de volta pro microship") 
        elif data == b'1':
            logging.info("PRESS T")
            pydirectinput.keyDown('t')
        elif data == b'A':
            logging.info("UNPRESS T")
            pydirectinput.keyUp('t')
        elif data == b'2':
            logging.info("PRESS Y")
            pydirectinput.keyDown('y')
        elif data == b'B':
            logging.info("UNPRESS Y")
            pydirectinput.keyUp('y')
        elif data == b'3':
            logging.info("PRESS g")
            pydirectinput.keyDown('g')
        elif data == b'C':
            logging.info("UNPRESS g")
            pydirectinput.keyUp('g')
        elif data == b'4':
            logging.info("PRESS Y")
            pydirectinput.keyDown('h')
        elif data == b'D':
            logging.info("UNPRESS Y")
            pydirectinput.keyUp('h')
        elif data == b'5':
            logging.info("PRESS W")
            pydirectinput.keyDown('w')
        elif data == b'E':
            logging.info("UNPRESS W")
            pydirectinput.keyUp('w')
        elif data == b'6':
            logging.info("PRESS S")
            pydirectinput.keyDown('s')
        elif data == b'F':
            logging.info("UNPRESS S")
            pydirectinput.keyUp('s')
        elif data == b'7':
            logging.info("PRESS A")
            pydirectinput.keyDown('a')
        elif data == b'G':
            logging.info("UNPRESS A")
            pydirectinput.keyUp('a')
        elif data == b'8':
            logging.info("PRESS D")
            pydirectinput.keyDown('d')
        elif data == b'H':
            logging.info("UNPRESS D")
            pydirectinput.keyUp('d')
        elif data == b'13':
            sessions = AudioUtilities.GetAllSessions()
            for session in sessions:
                interface = session.SimpleAudioVolume
                if session.Process and session.Process.name() == self.process_name:
                    # only set volume in the range 0.0 to 1.0
                    # self.volume = min(1.0, max(0.0, decibels))
                    interface.SetMasterVolume(self.volume, None)
                    print('Volume set to', self.volume)  # debug

        self.incoming = self.ser.read()


class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()

    def update(self):
        pydirectinput.keyDown(self.mapping.button['A'])
        time.sleep(0.1)
        pydirectinput.keyUp(self.mapping.button['A'])
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()

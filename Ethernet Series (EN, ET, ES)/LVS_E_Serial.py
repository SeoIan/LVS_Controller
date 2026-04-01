"""
파일명 : 
    - LVS_E_Serial.py
    
의존성 : 
    - pyserial      (pip install pyserial)
    - threading
    - time
    - queue
    
설명   :
    - 시리얼 통신을 관리하고 데이터를 송수신하는 클래스
    
버전 기록 : 
    - v1.0.0 (2026-03-30) 최초 작성
    
TODO    :
    - 시리얼 통신 예외 처리 추가
    - 수신된 데이터를 처리하는 콜백 함수 지원 
      (일부 커맨드는 ACK, NACK만 보내지만, 일부 커맨드는 데이터를 반환하는 경우가 존재함.)
      (수신된 데이터 처리 방법도 추가 필요함.)
"""

#region Command List
""" Command List """
SOH             = b'\x01' # Start of Header
EOT             = b'\x04' # End of Transmission
OPCODE_WRITE    = b'\x00' # Write Operation
OPCODE_READ     = b'\x01' # Read Operation
Data_Length_1    = b'\x01' # Data Length 1 byte
Data_Length_2    = b'\x02' # Data Length 2 bytes
Data_Length_4    = b'\x04' # Data Length 4 bytes

""" 
Description     : Read Test Register
Data Length     : 2 byte 
Command Type    : Read Only
"""
REG_RTR         = b'\x00' 

""" 
Description     : Read Write Test Register
Data Length     : 2 byte 
Command Type    : Read Write
"""
REG_RWTR        = b'\x04'

""" 
Description     : Channel Select Register 
Data Length     : 1 byte 
Command Type    : Read Write
bit 0-7         : Channel Number (1-8)
"""
REG_CSR         = b'\x20' 

""" 
Description     : Step Value Register
Data Length     : 2 byte 
Command Type    : Read Write
bit 0-7         : Step Value (0-255)
bit 8-15        : Reserved (256 - 65535)
"""
REG_SVR         = b'\x28'

""" 
Description     : System Control Register
Data Length     : 1 byte 
Command Type    : Write Only
bit 4           : Save Flag (Active High)
bit 3           : Page Hold Flag (Active High) (Strobe Controller Only)
bit 2           : Confirm Flag (Active High)
bit 1           : EThernet Mode (0 : UDP, 1 : TCP/IP)
bit 0           : Ethernet Re-Start (Active High)
"""
REG_SCR         = b'\x2C'

""" 
Description     : Reset Control Register 
Data Length     : 1 byte 
Command Type    : Write Only
bit 7           : Global Reset (Active High)
bit 6-5         : Reserved
bit 4           : Calibration Reset (Active High)
bit 3           : Max-Page Reset (Active High)
bit 2           : EThernet Address Reset (Active High)
bit 1           : On-Time Reset (Active High)
bit 0           : Intensity Reset (Active High)
"""
REG_RCR         = b'\x2D'

""" 
Description     : System Option Register
Data Length     : 1 byte 
Command Type    : Read Write
bit 7           : All Message (0 : On, 1 : Off)
bit 6-2         : Reserved
bit 1           : Save Debug Message (0 : Off, 1 : On)
bit 0           : Auto Save (0 : Auto-Save Enable, 1 : Auto-Save Disable)
"""
REG_SOR         = b'\x2E'

""" 
Description     : Channel On/Off Register
Data Length     : 1 byte 
Command Type    : Read Write
bit 7-0         : Channel On/Off (0 : Off, 1 : On)
"""
REG_COR         = b'\x34'

""" 
Description     : Constant Current Register (EN Controller Only)
Data Length     : 1 byte 
Command Type    : Write Only
bit 7-0         : Constant Current Mode (0 : Off, 1 : On)
"""
REG_CCM         = b'\x36'

""" 
Description     : Page Control Register
Data Length     : 1 byte 
Command Type    : Write Only
bit 7-2         : Reserved
bit 1           : Trigger Mode (0 : SEL_TRIG, 1:PRO_TRIG)
bit 0           : Burst mode Enable Flag (0 : Disable, 1 : Enable)
"""
REG_PCR         = b'\x38'

""" 
Description     : Page Count Register
Data Length     : 1 byte 
Command Type    : Read Write
bit 7-0         : Page Count (1-8)
"""
REG_PACR        = b'\x39'

""" 
Description     : Page Select Register
Data Length     : 1 byte 
Command Type    : Read Write
bit 7-0         : Page Number (1-8)
"""
REG_PASR        = b'\x3C'

""" 
Description     : Ethernet Environment Access Register
Data Length     : 1 byte 
Command Type    : Read Write
bit 7           : Client Port Number
bit 6           : MAC Address (Engineer Only)
bit 5           : Server Port Number
bit 4           : Server IP Address
bit 3           : Domain Name System Address 
bit 2           : Subnet Mask Address
bit 1           : Gateway Address
bit 0           : Client IP Address
"""
REG_EEAR        = b'\x40'

""" 
Description     : Ethernet Environment Data Register
Data Length     : 4 byte 
Command Type    : Read Write
"""
REG_EEDR        = b'\x44' 

""" 
Description     : Ethernet Control Register
Data Length     : 1 byte 
Command Type    : Read Write
bit 7-2         : Reserved
bit 1           : Ethernet TCP IP Mode (0 : Client , 1 : Host)
bit 0           : Ethernet Mode (0 : UDP, 1 : TCP/IP)
"""
REG_ECR         = b'\x4C' 

""" 
Description     : Test Trigger Register
Data Length     : 1 byte 
Command Type    : Write Only
"""
REG_TTR         = b'\xD8' 

""" 
Command Examples 
CMD_READ_CSR = SOH + OPCODE_READ + Data_Length_1 + REG_CSR + EOT
CMD_WRITE_CSR = SOH + OPCODE_WRITE + Data_Length_1 + REG_CSR + b'\x04' + EOT
"""
#endregion

import serial 
import threading
import time
import queue 

class LVS_Serial:
    def __init__(self, port : str = None,
                 baudrate : int = 9600, bytesize : int = serial.EIGHTBITS, 
                 parity : int = serial.PARITY_NONE, stopbits :int = serial.STOPBITS_ONE):
        self._communicator = serial.Serial(port=port, baudrate=baudrate, bytesize=bytesize, parity=parity, stopbits=stopbits)
        
        self._worker : threading.Thread = None
        self._stop_event : threading.Event = threading.Event()
        self._lock : threading.Lock = threading.Lock()
        
    def open(self):
        if not self._communicator.is_open:
            self._communicator.open()
        
        self._stop_event.clear()
        self._worker = threading.Thread(target=self.__Worker)
        self._worker.start()
        
    def close(self):
        if self._worker is not None:
            self._stop_event.set()
            self._worker.join()
            
        self._communicator.close()
        
    def write(self, message: bytes):
        if not isinstance(message, bytes):
            return 

        self._communicator.write(message)
        

    def read(self) -> bytes:
        available = self._communicator.in_waiting
        if available == 0:
            return b''

        data = self._communicator.read(available)
        return data
        
    def __Worker(self):
        while not self._stop_event.is_set():
            with self._lock:
                data = self.read()
                if data != b'':
                    print(f"Received data: {data.hex()}")
            
    def int2bytes(self, value: int, length: int) -> bytes:
        if not isinstance(value, int):
            return b''
        if value < 0:   
            return b''
        return value.to_bytes(length, byteorder='little')
    
    def bytes2int(self, value: bytes, length : int) -> int:
        if not isinstance(value, bytes):
            return 0
        return int.from_bytes(value[:length], byteorder='little')
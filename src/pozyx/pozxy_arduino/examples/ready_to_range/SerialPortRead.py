import serial 


def init_serial():
    with serial.Serial('COM7',115200, bytesize =serial.EIGHTBITS, parity= serial.PARITY_NONE, timeout = 2) as ser:
        try:
            ser.isOpen()
            print('serial port is open')
        except:
            print('Error opening')
            exit()
        if(ser.isOpen()):
            try:
                while(1):
                    print(ser.readline())
            except Exception:
                print'error'
        else:
            print 'cannot open serial port'
        x = ser.read(20)
        print('data: '+ser.readline())

def getData():
    return ser.realine()

    

if __name__ == '__main__':
    init_serial()
    

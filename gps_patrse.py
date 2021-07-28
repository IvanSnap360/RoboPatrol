import serial

port = serial.Serial('/dev/ttyTHS1',9600,timeout=3)
print("begin")

port.write(b'PMTK314')

while(1):
    
    line = str(port.readline())
    # print(line)
    line_array = line.split(',')
    # print(line_array)
    index = line_array[0]
    if (index == "$GNGGA"):
        sat = line_array[7]
        if (sat != ' '):
            sat = float(sat)
            if (sat >= 4):
                lat = float(line_array[2])
                lon = float(line_array[4])
                print(lat,lon,sat)
            else:
                print("wait coordinates... \n sattelite count", sat)
        else:
            print("wait satelites...")
    
    # print(port.readline())
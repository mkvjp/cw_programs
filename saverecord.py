# imports libs
import pandas as pd
import datetime
import serial
import os


def main(baudrate:int = 9600):
    # sets comport number and baudrate
    comname = "/dev/tty.usbserial-14110" #winならcom13とか
    com = serial.Serial(comname,baudrate)
    os.makedirs(f"record",exist_ok=True)
    foldername = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    os.makedirs(f"record/"+foldername, exist_ok=True)
    df = pd.DataFrame(columns=['timestamp','voltage'])
    df = df.set_index("timestamp",drop=True)
    while True:

        now = datetime.datetime.now()
        file_name = "record/"+foldername+"/record.csv"
        print("start!")
        while True:
            # gets now time
            now = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            voltage = str(com.readline().decode("utf-8"))
            df.loc[now] = [voltage]
            df.to_csv(file_name)
            print(voltage)
        
if __name__ == '__main__':
    main()

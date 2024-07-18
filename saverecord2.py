# imports libs
import pandas as pd
import datetime
import serial #pyserial
import os
import time

def main(baudrate:int = 9600):
    # sets comport number and baudrate
    comname = "/dev/tty.usbmodem141201" #winならCOM13とか
    com = serial.Serial(comname, baudrate)
    os.makedirs(f"record", exist_ok=True)
    file_name = "record/" +  comname+datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + ".csv"
    df = pd.DataFrame(columns=['timestamp', 'voltage'])
    df = df.set_index("timestamp", drop=True)

    start_time = time.time()
    duration = 60 * 60  # 1 hour in seconds

    print("start!")
    while True:
        if time.time() - start_time > duration:
            print("1 hour has passed. Ending loop.")
            break

        file_name = "record/" + foldername + "/record.csv"
        # gets now time
        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        voltage = str(com.readline().decode("utf-8"))
        df.loc[now] = [voltage]
        df.to_csv(file_name)
        print(voltage)

if __name__ == '__main__':
    main()

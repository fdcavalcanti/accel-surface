#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 12 22:46:51 2021

@author: filipe
"""

import serial
import numpy as np
import pandas as pd

MCU_PORT = '/dev/ttyACM0'
MCU_BAUD = 57600

ser = serial.Serial(None,
                    MCU_BAUD, 
                    serial.EIGHTBITS, 
                    serial.PARITY_NONE,
                    serial.STOPBITS_ONE, 
                    timeout = 2) 


ser.port = MCU_PORT
ser.open()
ser.reset_input_buffer()

ask1 = input("Enter Y to start read from {}.\n>> ".format(MCU_PORT))

data_raw = []
data_proc = []
READY_1 = False
READY_2 = False

def process_line(line):
    global READY_1
    line = line.decode("utf-8")
    split = line.split("\t")[:-1]
    if 'sample' in split:
        READY_1 = True
        READY_2 = False
        return split
        
    else:    
        data_num = np.array(split, dtype=float)
        if len(data_num) == 5:
            return data_num.T


if ask1 in (['Y', 'y']):
    try:
        while(True):
            line = ser.readline()
            data_raw.append(line.decode("utf-8"))
            data_filt = process_line(line)
            data_proc.append(data_filt)
            
            if (READY_1 is True and READY_2 is False):
                if type(data_filt[0]) is np.float64 and data_filt[0] >= 100:
                    READY_2 = True
                    READY_1 = False

            print(READY_1, READY_2, data_filt)
            
    except KeyboardInterrupt:
        ser.close()        
        print("Exiting")


#%%
def raw_2_df(raw_data):
    default_columns = ['sample', 'terrain', 'acc_x', 'acc_y', 'acc_z']
    
    df = pd.DataFrame(raw_data)
    df = df[0].str.split('\t', expand = True)
    df = df.drop(axis = 1, labels = [5])
    df = df.astype(float)
    df.rename(columns = {0: default_columns[0], 1: default_columns[1], 2: default_columns[2],
                         3: default_columns[3], 4: default_columns[4]}, inplace = True)
    
    return df

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri May 18 20:27:59 2018

@author: paul
"""

import pandas as pd
import matplotlib.pyplot as plt
from os import listdir

DATA_PATH='./data/'

def plot_and_save(filename,plot_name):
    df=pd.read_csv(DATA_PATH+filename)
    df.drop(0,inplace=True)
    df.time=df.time.astype(float)
    df.Position=df.Position.astype(float)
    df.Velocity=df.Velocity.astype(float)
    df.Effort=df.Effort.astype(float)
    plt.figure('Phase plot'+filename)
    plt.title('Phase plot '+plot_name)
    plt.plot(df.Position,df.Velocity)
    plt.xlabel('Position');
    plt.ylabel('Velocity');
    plt.savefig('Phase_'+plot_name+'.png', bbox_inches='tight')
    
    plt.figure('Time plot'+filename)
    t=df.time[df.time<30]  
    plt.plot(t,df.Position[t.index],label='Position')
    plt.plot(t,df.Velocity[t.index],label='Velocity')
    plt.legend()
    plt.xlabel('Time [s]')

    plt.title('Time plot '+plot_name)
    #plt.savefig('Time_'+plot_name+'.png', bbox_inches='tight')

if __name__=='__main__':
    plot_and_save('sine_01.csv','sinus excitation (A=1, f=0.1Hz)')
    plot_and_save('sine_02.csv','sinus excitation (A=1, f=0.2Hz)')
    plot_and_save('sine_05.csv','sinus excitation (A=1, f=0.5Hz)')
    plot_and_save('perturbation.csv','with external perturbation')
    
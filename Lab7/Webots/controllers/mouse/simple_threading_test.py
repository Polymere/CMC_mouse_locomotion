#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon May 21 22:12:36 2018

@author: paul
"""
from PyQt5.QtCore import QThread, QObject, pyqtSignal, pyqtSlot
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import time,sys


class Simulation(QObject):
    def __init__(self,params):
        super(Simulation, self).__init__()
        self.params=params
    def set_params(self,params):
        self.params=params

    def run(self):
        self.run_flag=True
        while self.run_flag:
            time.sleep(2)
            self.params.print_params()
            
class ReflexParams():
    transitions={'Hip angle liftoff': -0.1235,'Ankle unloading liftoff':0.8,
     'Hip angle touchdown':0.4, 'Ankle unloading touchdown':-10.25}
    # MUSCLE ACTIVATION CONSTANTS
    #stance_2_lift_off
    activation={'Stance to lift off':[0.05, 0.05, 0.05, 0.05,0.05,0.05],
    'Swing to touch down':[0.05,0.05,0.05],
    'Touch down to stance':[0.05,0.05],
    'Lift off to swing':[0.05,0.05]}
    
    enable={'Stance to lift off':False,'Swing to touch down':False,
    'Touch down to stance':False,'Lift off to swing':False}
    
    def __init__(self):
            print 'Initiate reflexes param'

            
    def set_activation(self,key,idx,value):
        print value
        try:
            self.activation[key][idx]=float(value)/100.0
            print key,idx,value
        except IndexError:
            print 'Valid indexes are between 0 and %d', len(self.activation[key])
        except KeyError:
            print 'Valid keys are '
            for key in self.activation.keys():
                print '\t %s',key

    def toggle(self,key):
        try:
            self.enable[key]= not self.enable[key]
            print 'Togeule',key
            return
        except KeyError:
            print 'Valid keys are '
            for key in self.enable.keys():
                print '\t %s',key
        except:
            print 'Error in toggle'

    def set_transitions(self,key,value):
        try:
            self.transitions[key]=value
        except KeyError:
            print 'Valid keys are '
            for key in self.transitions.keys():
                print '\t %s',key
        except:
            print 'Error in set transitions'

    def print_params(self):
        print 'Activation values :'
        for step,activation in self.activation.items():
            print step
            print activation
        print 'Transition triggers :'
        for trigger,transition in self.transitions.items():
            print trigger
            print transition
        print 'Activated steps are :'
        for step,activated in self.enable.items():
            if activated:
                print step
class MainWindow(QMainWindow):
    def __init__(self,params_obj):
        super(MainWindow, self).__init__()

        self.params_cb=params_obj
        self.sim=Simulation(params_obj)
        self.sim.set_params(params_obj)
        grid = QGridLayout()
        grid.addWidget(self.createActivation('Stance to lift off',6,
                                            params_obj.set_activation,params_obj.toggle), 0, 0)
        grid.addWidget(self.createActivation('Swing to touch down',3,
                                            params_obj.set_activation,params_obj.toggle), 0, 1)
        grid.addWidget(self.createActivation('Touch down to stance',2,
                                            params_obj.set_activation,params_obj.toggle), 1, 0)
        grid.addWidget(self.createActivation('Lift off to swing',2,
                                            params_obj.set_activation,params_obj.toggle), 1, 1)

        grid.addWidget(self.createReflex(key='Hip angle liftoff'), 2, 0)
        grid.addWidget(self.createReflex(key='Ankle unloading liftoff'), 2, 1)
        grid.addWidget(self.createReflex(key='Hip angle touchdown'), 3, 0)
        grid.addWidget(self.createReflex(key='Ankle unloading touchdown'), 3, 1)

        self.sim_thread=QThread()
        self.sim.moveToThread(self.sim_thread)
        self.sim_thread.started.connect(self.sim.run)
        self.sim_thread.start()
        b=QPushButton('Run')
        b.pressed.connect(self.sim_thread.start)
        grid.addWidget(b,3,2)
        self.w=QWidget()
        self.w.setLayout(grid)
        self.setCentralWidget(self.w)
        self.setWindowTitle("Parameter tuning")
        self.resize(400, 300)
        self.show()

    def createActivation(self,key,n_values,value_cb,toggle_cb):
        print key
        groupBox = QGroupBox(key)
        radio1 = QRadioButton('Enable ?')
        radio1.setChecked(False)
        radio1.toggled.connect(lambda : toggle_cb(key))
        vbox = QVBoxLayout()
        vbox.addWidget(radio1)
        for i in range(n_values):
            vbox.addWidget(self.get_slider(key,value_cb,i))
            vbox.addStretch(1)
        groupBox.setLayout(vbox)
        return groupBox
    def get_slider(self,key,cb_method,idx=0):
        slider = QSlider(Qt.Horizontal)
        slider.setFocusPolicy(Qt.StrongFocus)
        slider.setTickPosition(QSlider.TicksBothSides)
        slider.setTickInterval(10)
        slider.setTickPosition(1)
        slider.setSingleStep(1)
        slider.sliderReleased.connect(lambda :cb_method(key,idx,slider.value()))
        return slider
    def createReflex(self,key='Default'):
        groupBox = QGroupBox(key)
        vbox = QVBoxLayout()
        slider = QSlider(Qt.Horizontal)
        slider.setFocusPolicy(Qt.StrongFocus)
        slider.setTickPosition(QSlider.TicksBothSides)
        slider.setTickInterval(10)
        slider.setSingleStep(1)
        vbox.addWidget(slider)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)
        return groupBox

if __name__ == '__main__':

    
    reflex_params=ReflexParams()
    app = QApplication([])
    win=MainWindow(reflex_params)
   
    #mouse.run(reflex_params)
    app.exec_()    
"""This file implements reflex controller."""
# Default modules
import os

# Webots modules
from controller import Supervisor
from controller import Keyboard

# MusculoSkeletal system
from musculoskeletal import MusculoSkeletalSystem

from reflexes import Reflexes
from reflexesParams import ReflexParams

# Muscle Visualization
from muscle_visualization import MuscleVisualization

import numpy as np



from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtCore import QThread, QObject, pyqtSignal, pyqtSlot


 

# multithread, qt master thread 
# pygmo for optim


# VARIABLE TO TURN ON/OFF MUSCLE VISUALIZATION
MUSCLE_VISUALIZATION = True

# TRAJECTORIES BUFFER SIZE, CAN BE INCREASED FOR LONGER SIMULATIONS
BUFFER_SIZE_TRAJECTORIES = int(1e6)

# RESULTS DIRECTORY (INSIDE /Lab7/Webots/controllers/mouse/Results by default)
RESULTS_DIRECTORY = os.path.join(".", "Results", "")

class ThreadMouse(QObject):
    def __init__(self,params):
        super(ThreadMouse, self).__init__()
        self.mouse=Mouse()
        self.mouse.set_params(params)

    def run(self):
        print 'Run thread mouse'
        self.mouse.params.print_params()
        self.mouse.run()
class Mouse(Supervisor):
    """Main class for Mouse control. """

    def __init__(self):

        super(Mouse, self).__init__()
        self.biomech = MusculoSkeletalSystem.MusculoSkeletalSystem(
            os.path.join('musculoskeletal', 'mouse.json')
        )
        self.TIMESTEP = int(self.getBasicTimeStep())
        self.motors = {}
        self.position_sensors = {}
        self.ground_contact_sensors = {}
        self.joint_positions = {}
        self.muscle_forces = {}
        self.ground_contacts = {}

        # Initialize webots
        self.key_press = None
        self.initialize_webots_motors()
        self.initialize_webots_position_sensors()
        self.initialize_webots_ground_contact_sensors()
        self.initialize_webots_keyboard()

        # Muscle visualization
        self.gps = {}
        self.mv_transform = {}
        self.mv_color = {}
        self.mv_geom = {}
        self.muscle_viz = None
        self.initialize_gps()
        self.initialize_muscle_visualizations()

        # Foot Trajectories [time, axis]
        self.ankle_r_trajectory = np.zeros([BUFFER_SIZE_TRAJECTORIES, 3])
        self.ankle_l_trajectory = np.zeros([BUFFER_SIZE_TRAJECTORIES, 3])

        self.iteration = 0

        

    def __del__(self):
        """ Deletion """
        try:
            os.stat(RESULTS_DIRECTORY)
        except:
            os.mkdir(RESULTS_DIRECTORY)
        np.save(
            RESULTS_DIRECTORY + "ankle_l_trajectory.npy",
            self.ankle_l_trajectory[:self.iteration, :]
        )
        np.save(
            RESULTS_DIRECTORY + "ankle_r_trajectory.npy",
            self.ankle_r_trajectory[:self.iteration, :]
        )

        return
        
    def set_params(self,params):
        print 'Setting params to '
        params.print_params()
        self.params=params
        
    def run(self):
        """ Run """
        print 'Running mouse model with '
        self.params.print_params()
        reflex = Reflexes(self.biomech.sim_muscle_names,self.params)

        while self.step(self.TIMESTEP) != -1:

            # Reflex model
            reflex.step(
                self.update_joint_positions(),
                self.update_muscle_forces(),
                self.update_ground_contacts()
            )

            activations = reflex.activations

            # Update the biomechanical model
            self.biomech.update(
                self.TIMESTEP / 1000.,
                self.update_joint_positions(),
                activations
            )

            # Get the Torque
            torque = self.biomech.joint_torque()

            for name, motor in self.motors.iteritems():
                motor.setTorque(torque[name])

            if MUSCLE_VISUALIZATION:
                self.muscle_viz.step(activations, viz=True)
            else:
                self.muscle_viz.step(activations, viz=False)

            # Save data
            self.iteration += 1
            if self.iteration%100==0:
                self.params.print_params()

            self.ankle_l_trajectory[self.iteration, :] = (
                self.gps["LH_G1_ANKLE"].getValues()
            )
            self.ankle_r_trajectory[self.iteration, :] = (
                self.gps["RH_G1_ANKLE"].getValues()
            )

    def initialize_webots_keyboard(self):
        """ Initialize webots keyboard """
        self.key_press = Keyboard()
        self.key_press.enable(self.TIMESTEP * 25)

    def initialize_webots_motors(self):
        """Set-up leg joints in the system."""
        for joint in self.biomech.sim_joints:
            #print('Initializing webots motor : {}'.format(joint))
            self.motors[joint] = self.getMotor(str(joint))

    def initialize_webots_position_sensors(self):
        """Set-up leg joints in the system."""
        for joint in self.biomech.sim_joints:
            #print('Initializing webots motor : {}'.format(joint))
            self.position_sensors[joint] = self.getPositionSensor(
                str(joint) + '_POS'
            )
            self.position_sensors[joint].enable(self.TIMESTEP)

    def initialize_webots_ground_contact_sensors(self):
        """Initialize groung contact sensors."""
        #print('Initializing webots ground contact sensors ')
        self.ground_contact_sensors['LEFT_TOE_TOUCH'] = self.getTouchSensor(
            'LEFT_TOE_TOUCH'
        )
        self.ground_contact_sensors['LEFT_TOE_TOUCH'].enable(self.TIMESTEP)
        self.ground_contact_sensors['RIGHT_TOE_TOUCH'] = self.getTouchSensor(
            'RIGHT_TOE_TOUCH'
        )
        self.ground_contact_sensors['RIGHT_TOE_TOUCH'].enable(self.TIMESTEP)

    # MUSCLE VISUALIZATION

    def initialize_muscle_visualizations(self):
        """Initialize necessary attributes for muscle visualization."""

        # Get muscle transform, appearance and geom
        for muscle in self.biomech.sim_muscle_names:
            muscle_split = muscle.split('_')
            side = muscle_split[0]
            name = muscle_split[-1]

            # TRANSFORM
            transform = str(side + '_MV_TRANSFORM_' + name)
            self.mv_transform[transform] = self.getFromDef(transform)
            # COLOR
            appearance = str(side + '_MV_COLOR_' + name)
            self.mv_color[appearance] = self.getFromDef(appearance)
            # GEOM
            geom = str(side + '_MV_GEOM_' + name)
            self.mv_geom[geom] = self.getFromDef(geom)
            # Creat muscle muscle visualization object
            self.muscle_viz = MuscleVisualization(
                self.muscle_visualization_attachment(),
                self.gps,
                self.mv_transform,
                self.mv_color,
                self.mv_geom)
        return

    def initialize_gps(self):
        """Initialize gps nodes for muscle visualization."""

        GPS_NAMES = ['G1_PELVIS', 'G2_PELVIS',
                     'G1_HIP', 'G2_HIP',
                     'G1_KNEE', 'G2_KNEE',
                     'G1_ANKLE']

        sides = ['LH', 'RH']

        for side in sides:
            for gps in GPS_NAMES:
                name = side + '_' + gps
                self.gps[name] = self.getGPS(name)
                self.gps[name].enable(self.TIMESTEP)
        return

    def muscle_visualization_attachment(self):
        """Returns the dictionaries muscle origin and
        insertion with respect to GPS."""

        muscle_attach = {
            'PMA': ['G1_PELVIS', 'G1_HIP'],
            'CF': ['G2_PELVIS', 'G1_HIP'],
            'SM': ['G2_PELVIS', 'G1_KNEE'],
            'POP': ['G1_HIP', 'G1_KNEE'],
            'RF': ['G1_HIP', 'G2_HIP'],
            'TA': ['G1_KNEE', 'G1_ANKLE'],
            'SOL': ['G1_KNEE', 'G2_KNEE'],
            'LG': ['G1_HIP', 'G2_KNEE']
        }
        return muscle_attach

    def update_joint_positions(self):
        """ Initialize the array to store joint positions."""
        for name, sensor in self.position_sensors.iteritems():
            self.joint_positions[name] = (
                sensor.getValue()
                + self.biomech.sim_joints[name].reference_angle
            )
        return self.joint_positions

    def update_muscle_forces(self):
        """ Initialize the array to store joint positions."""
        for muscle in self.biomech.sim_muscles:
            self.muscle_forces[muscle] = self.biomech.sim_muscles[
                muscle
            ].tendonForce
        return self.muscle_forces

    def update_ground_contacts(self):
        """ Update ground contacts """
        self.ground_contacts['L'] = self.ground_contact_sensors[
            'LEFT_TOE_TOUCH'
        ].getValue()
        self.ground_contacts['R'] = self.ground_contact_sensors[
            'RIGHT_TOE_TOUCH'
        ].getValue()
        return self.ground_contacts

        




class MainWindow(QMainWindow):
    def __init__(self,params_obj):
        super(MainWindow, self).__init__()

        self.params_cb=params_obj

        self.th_mouse=ThreadMouse(params_obj)
        self.sim_thread=QThread()
        self.th_mouse.moveToThread(self.sim_thread)
        self.sim_thread.started.connect(self.th_mouse.run)
        
        
        grid = QGridLayout()
        grid.addWidget(self.createActivation('Stance to lift off',6,
                                            params_obj), 0, 0)
        grid.addWidget(self.createActivation('Swing to touch down',3,
                                            params_obj), 0, 1)
        grid.addWidget(self.createActivation('Touch down to stance',2,
                                            params_obj), 1, 0)
        grid.addWidget(self.createActivation('Lift off to swing',2,
                                            params_obj), 1, 1)

        grid.addWidget(self.createReflex(params_obj,key='Hip angle liftoff'), 2, 0)
        grid.addWidget(self.createReflex(params_obj,key='Ankle unloading liftoff'), 2, 1)
        grid.addWidget(self.createReflex(params_obj,key='Hip angle touchdown'), 3, 0)
        grid.addWidget(self.createReflex(params_obj,key='Ankle unloading touchdown'), 3, 1)
        
        
        radio_rev=QRadioButton('Revert simulation')
        radio_rev.setAutoExclusive(False)
        radio_rev.setChecked(False)
        radio_rev.toggled.connect(self.revert)
        grid.addWidget(radio_rev,5,1)

        self.w=QWidget()
        self.w.setLayout(grid)
        self.setCentralWidget(self.w)
        self.setWindowTitle("Parameter tuning")
        self.resize(400, 300)
        self.show()
        
        self.sim_thread.start()
        
    def revert(self):
        self.params_cb.save()
        self.th_mouse.mouse.simulationRevert()
        
    def createActivation(self,key,n_values,params_obj):
        
        #print key
        value_cb=params_obj.set_activation
        but_cb=params_obj.set_enable
        
        sl_values=params_obj.activation[key]
        radio_value=params_obj.enable[key]
        groupBox = QGroupBox(key)
        radio1 = QRadioButton('Force ?')
        #radio1.setAutoExclusive(False)
        radio1.setChecked(radio_value)
        radio1.toggled.connect(lambda : but_cb(key,radio1.isChecked()))
        vbox = QVBoxLayout()
        vbox.addWidget(radio1)
        for i in range(n_values):
            vbox.addWidget(self.get_slider(key,value_cb,init_value=sl_values[i],idx=i))
            vbox.addStretch(1)
        groupBox.setLayout(vbox)
        return groupBox
        
    def get_slider(self,key,cb_method,init_value=5,idx=0):
        slider = QSlider(Qt.Horizontal)
        slider.setFocusPolicy(Qt.StrongFocus)
        slider.setTickPosition(QSlider.TicksBothSides)
        slider.setTickInterval(10)
        slider.setTickPosition(1)
        slider.setValue(init_value*100)
        slider.setSingleStep(1)
        slider.sliderReleased.connect(lambda :cb_method(key,idx,slider.value()))
        return slider
        
    def createReflex(self,params_obj,key='Default'):
        value_cb=params_obj.set_transitions
        value=params_obj.trans_val_to_percent(params_obj.transitions[key],key)
        groupBox = QGroupBox(key)
        vbox = QVBoxLayout()
        slider = QSlider(Qt.Horizontal)
        slider.setFocusPolicy(Qt.StrongFocus)
        slider.setTickPosition(QSlider.TicksBothSides)
        slider.setTickInterval(10)
        slider.setTickPosition(5)
        slider.setSingleStep(1)
        slider.setValue(value)
        slider.sliderReleased.connect(lambda :value_cb(key,slider.value()))
        vbox.addWidget(slider)
        vbox.addStretch(1)
        groupBox.setLayout(vbox)
        return groupBox

        

if __name__ == '__main__':



    reflex_params=ReflexParams()
    app = QApplication([])
    slider_win=MainWindow(reflex_params)
    app.exec_()


"""This file implements reflex controller."""
# Default modules
import os

# Webots modules
from controller import Supervisor
from controller import Keyboard

# MusculoSkeletal system
from musculoskeletal import MusculoSkeletalSystem

from reflexes import Reflexes

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

class ThreadMouse(QThread):
    def __init__(self,mouse):
        self.mouse=mouse

    def run(self):
        QThread.__init__(self)
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
            print('Initializing webots motor : {}'.format(joint))
            self.motors[joint] = self.getMotor(str(joint))

    def initialize_webots_position_sensors(self):
        """Set-up leg joints in the system."""
        for joint in self.biomech.sim_joints:
            print('Initializing webots motor : {}'.format(joint))
            self.position_sensors[joint] = self.getPositionSensor(
                str(joint) + '_POS'
            )
            self.position_sensors[joint].enable(self.TIMESTEP)

    def initialize_webots_ground_contact_sensors(self):
        """Initialize groung contact sensors."""
        print('Initializing webots ground contact sensors ')
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
                
class Worker(QObject):
    finished = pyqtSignal()
    intReady = pyqtSignal(int)


    @pyqtSlot()
    def procCounter(self): # A slot takes no params
        for i in range(1, 100):
            time.sleep(1)
            self.intReady.emit(i)

        self.finished.emit()                
class MainWindow(QMainWindow):
    def __init__(self,params_obj,th_mouse):
        super(MainWindow, self).__init__()

        self.params_cb=params_obj
        
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
        b=QPushButton('Run')
        b.pressed.connect(th_mouse.run)
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

    mouse = Mouse()

    reflex_params=ReflexParams()
    mouse.set_params(reflex_params)
    thread_mouse=ThreadMouse(mouse)
    app = QApplication([])
    slider_win=MainWindow(reflex_params,thread_mouse)
   
    #mouse.run(reflex_params)
    sys.exit(app.exec_())
    #mouse.run(reflex_params)

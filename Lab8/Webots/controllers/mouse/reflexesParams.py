import json
class ReflexParams():
    transitions={'Hip angle liftoff': -0.1235,
                 'Ankle unloading liftoff':0.8,
                 'Hip angle touchdown':0.4, 
                 'Ankle unloading touchdown':-10.25} # Default values
    
    # transitions for each step have default value, min value, max value.
    
    transition_boundaries={'Hip angle liftoff': [-0.2,0.2],
                           'Ankle unloading liftoff':[-1.0,1.0],
                           'Hip angle touchdown':[-1.0,2.0], 
                           'Ankle unloading touchdown':[-20.,20.]}
                 
    activation={'Stance to lift off':[0.05, 0.05, 0.05, 0.05,0.05,0.05],
                'Swing to touch down':[0.05,0.05,0.05],
                'Touch down to stance':[0.05,0.05],
                'Lift off to swing':[0.05,0.05]}
    
    enable={'Stance to lift off':False,
            'Swing to touch down':False,
            'Touch down to stance':False,
            'Lift off to swing':False,
            'Coupling':True,
            'Hip extension rule':True,
            'Ankle unloading rule':True}
    
    def __init__(self):
        print 'Initiate reflexes param'
        try :
            f=open('temp.json','rb')
            l=json.load(f)
            f.close()
            self.activation=l['activation']
            self.enable=l['enable']
            self.transition_boundaries=l['transition_boundaries']
            self.transitions=l['transitions']
            print '########################'
            self.print_params()
            print '########################'
        except:
            print 'No parameters, starting with default'
            self.print_params()
    def set_activation(self,key,idx,value):
        try:
            self.activation[key][idx]=float(value)/100.0
            print key+'K'+str(idx+1)+' set to '+str(float(value)/100.0)
            self.save()
        except IndexError:
            print 'Valid indexes are between 0 and %d', len(self.activation[key])
        except KeyError:
            print 'Valid keys are '
            for key in self.activation.keys():
                print '\t %s',key
    
    def set_enable(self,key,value):
        try:
            self.enable[key]= value
            print '------------------------------'
            print 'Setting', key, 'to',str(value)
            print '------------------------------'
            self.save()
            return
        except KeyError:
            print 'Valid keys are '
            for key in self.enable.keys():
                print '\t %s',key
        except:
            print 'Error in set enable'
    
    def set_transitions(self,key,value):
        try:
            max_val=self.transition_boundaries[key][1]
            min_val=self.transition_boundaries[key][0]
            self.transitions[key]=value*(max_val-min_val)/100+min_val
            # linear between max_val and min_val (value ranges from 0 to 100)
            print key +' set to ' + str(self.transitions[key])
            self.save()
        except KeyError:
            print 'Valid keys are '
            for key in self.transitions.keys():
                print '\t %s',key
                
    def trans_val_to_percent(self,val,key):
        max_val=self.transition_boundaries[key][1]
        min_val=self.transition_boundaries[key][0]
        return 100*(val-min_val)/(max_val-min_val)
    
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
    def as_dict(self):
        return {'transitions':self.transitions,
                'transition_boundaries': self.transition_boundaries,
                'activation':self.activation,
                'enable':self.enable}
    def save(self):
      print 'Saving parameters'
      f=file('temp.json','wb')
      json.dump(self.as_dict(),f)
      f.close()
      #self.print_params()
      print 'Parameters saved'
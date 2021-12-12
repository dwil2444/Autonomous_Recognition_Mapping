class StateMachine:
    """General Purpose State Machine"""
    
    def __init__(self,states,default_state=None,**global_kwargs):
        self.states=states
        if default_state==None:
            self.state=states[0] #Default to whatever the first state provided is
        else:
			if default_state in self.states:
				self.state=default_state
			else:
				raise Exception("Received Unknown State: %s, try using one of %s, or adding the state"%(default_state,self.states))
        self.params=global_kwargs
    
    def addParam(self,**kwargs):
        self.params.update(kwargs)

    def getParam(self,param):
        return self.params[param]

    def setParam(self,param,val):
        self.params[param]=val

    def getState(self):
        return self.state

    def setState(self,state):
        if state in self.states:
            self.state=state
        else:
            raise Exception("Received Unknown State: %s, try using one of %s, or adding the state"%(state,self.states))

    def addState(self,state):
        if state not in self.states:
            self.states.append(state)
        else:
            raise Exception("%s already exists. Choose a statename not in %s"%(state,self.states))

if __name__=='__main__':
    states=['a','b','c']
    default_state='b'
    globs={'foo':1,'bar':2}
    SM=StateMachine(states,default_state=default_state,**globs)

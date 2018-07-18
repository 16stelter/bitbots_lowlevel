import rospy 
import humanoid_league_msgs.msg
from bitbots_stackmachine.abstract_action_element import AbstractActionElement
import bitbots_hcm.hcm_stack_machine.hcm_connector


class AbstractStay(AbstractActionElement):
    """
    Abstract class to create actions which just stay on top of the stack.
    This can be used to stay in a certain state till some precondition changes.
    Implementations can be used to change the name
    """

    def __init__(self, connector, _):
        super(AbstractStay, self).__init__(connector)

    def perform(self, connector, reevaluate=False):        
        # just do nothing
        return 

class StayControlable(AbstractStay):
    pass

class StayWalking(AbstractStay):
    pass

class StayAnimationRunning(AbstractStay):
    pass

class StayPickedUp(AbstractStay):
    pass

class StayMotorsOff(AbstractStay):
    pass

class StayInPenalty(AbstractStay):
    pass

class StayShutDown(AbstractStay):
    pass
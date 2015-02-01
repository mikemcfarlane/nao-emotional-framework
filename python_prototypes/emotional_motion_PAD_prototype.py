""" nao emotional framework - prototype NAO motions to represent emotions, using the PAD model.

"""

import sys
import time

import numpy as np
import math

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule

from optparse import OptionParser

NAO_IP = "169.254.216.239"


# Global variable to store module instances
emotion_motion = None
memory = None

class motion_module(ALModule):
    """ A simple module to change the head position and stance to represent emotions.

    """
    def __init__(self, name):
        ALModule.__init__(self, name)

        # Create proxies for the instance.
        self.motion = ALProxy("ALMotion")
        
        # Run behaviour when a tactile touched.
        global memory
        memory = ALProxy("ALMemory")
        memory.subscribeToEvent("TouchChanged", self.getName(), "emotive_motion")

    def emotive_motion(self, *_args):
        """ Driven by:
            Torso position (hip pitch, ankle pitch, shoulder pitch) = proportional to dominance AND pleasure 
            Position on ground = proportional to dominance AND pleasure, only activated by strong values.
            Stance openness (shoulder roll) = proportional to pleasure
            Head pitch = inversely proportional to arousal.
            Height = directly proportional to arousal.
            Motion speed = proportional to arousal            
                        
        """

        memory.unsubscribeToEvent("TouchChanged", self.getName())
        
        motion_names = list()
        motion_times = list()
        motion_keys = list()

        current_emotion = memory.getData("Emotion/Current")
        pleasure = current_emotion[0][0]
        arousal = current_emotion[0][1]
        dominance = current_emotion[0][2]

        # Motion speed - proportional to arousal.
        # Controlled by setting relative time for each motion action where the relative time is scaled,
        # with arousal and a scaling factor. Time inversely proportional to arousal.
        dummy_motion_time_increment = 1.0
        time_scale_factor = 10
        relative_motion_time_increment = dummy_motion_time_increment - arousal / time_scale_factor
        # In this proptype code there are three motions, hence three times required.
        t1 = relative_motion_time_increment
        t2 = t1 + relative_motion_time_increment
        t3 = t2 + relative_motion_time_increment

        # Torso position (hip pitch, ankle pitch, shoulder pitch) = proportional to dominance AND pleasure
        # Shoulders have a pitch of +2 to -2 radians.
        # Used in absolute mode, central pitch value is 1.45726radians.
        shoulder_pitch_position_default = 1.45726
        shoulder_pitch_range = 0.5
        shoulder_pitch_scaled_to_dominance = dominance * shoulder_pitch_range
        shoulder_pitch_scaled_to_pleasure = pleasure * shoulder_pitch_range

        shoulder_pitch_dominance = shoulder_pitch_position_default - shoulder_pitch_scaled_to_dominance
        shoulder_pitch_pleasure = shoulder_pitch_position_default - shoulder_pitch_scaled_to_pleasure
        shoulder_pitch = (shoulder_pitch_dominance + shoulder_pitch_pleasure) / 2
        # TODO: does shoulder_pitch need limited?
        

        motion_names.append("LShoulderPitch")
        motion_times.append([t1, t2, t3])
        motion_keys.append([shoulder_pitch_position_default, shoulder_pitch, shoulder_pitch_position_default])

        motion_names.append("RShoulderPitch")
        motion_times.append([t1, t2, t3])
        motion_keys.append([shoulder_pitch_position_default, shoulder_pitch, shoulder_pitch_position_default])

        # Ankles have a pitch of approx +0.9 to -1.1radians.
        # Used in absolute mode, central pitch value is 0.08radians.
        # Scaled: current dominance / max dominance = current range / max range
        ankle_pitch_position_default = 0.06
        ankle_pitch_range = 0.1
        ankle_pitch_range_scaled_to_dominance = dominance * ankle_pitch_range 
        ankle_pitch_range_scaled_to_pleasure = pleasure * ankle_pitch_range

        ankle_pitch_dominance = ankle_pitch_position_default - ankle_pitch_range_scaled_to_dominance
        ankle_pitch_pleasure = ankle_pitch_position_default - ankle_pitch_range_scaled_to_pleasure
        ankle_pitch = (ankle_pitch_dominance + ankle_pitch_pleasure) / 2
        # Limit ankle pitch to prevent falls
        ankle_pitch_limit = 0.12
        if ankle_pitch > ankle_pitch_limit:
            ankle_pitch = ankle_pitch_limit

        motion_names.append("LAnklePitch")
        motion_times.append([t1, t2, t3])
        motion_keys.append([ankle_pitch_position_default, ankle_pitch, ankle_pitch_position_default])

        motion_names.append("RAnklePitch")
        motion_times.append([t1, t2, t3])
        motion_keys.append([ankle_pitch_position_default, ankle_pitch, ankle_pitch_position_default])

        # Position on ground = proportional to dominance AND pleasure, only activated by strong values.
        walk_activation_threshold = 0.8
        walk_distance = 0.1 # meters
        if (abs(dominance) > walk_activation_threshold):
            walk_dominance = math.copysign(walk_distance, dominance)
        if (abs(pleasure) > walk_activation_threshold):
            walk_pleasure = math.copysign(walk_distance, pleasure)
        
        walk = walk_dominance + walk_pleasure

        # Stance openness (shoulder roll) = proportional to pleasure
        # Shoulder roll ranges -1.3265 to 0.3142radians.
        # Used in absolute mode, central roll value is right -0.16radians, left 0.16radians.
        shoulder_roll_move = 0.3

        shoulder_roll_right = -0.16 - shoulder_roll_move * pleasure
        shoulder_roll_left = 0.16 + shoulder_roll_move * pleasure

        motion_names.append("RShoulderRoll")
        motion_times.append([t1, t2, t3])
        motion_keys.append([-0.16, shoulder_roll_right, -0.16])

        motion_names.append("LShoulderRoll")
        motion_times.append([t1, t2, t3])
        motion_keys.append([0.16, shoulder_roll_left, 0.16])

        # Head pitch - directly proportional to arousal.
        # Head pitch has a range of approx +0.5 to -0.5 radians so divide normalised arousal value by 2.
        head_pitch = arousal / 2

        motion_names.append("HeadPitch")
        motion_times.append([t1, t2, t3])
        motion_keys.append([0.0, head_pitch, 0.0])

        # TODO: height, how ??? Whole Body Balancer ??


        # TODO: Might be better to use the reactive methods -  ALMotionProxy::setAngles(). or Whole Body Balancer.
        # TODO: add walk, whilst running motion.angleInterpolation in background.
        try:
            self.motion.angleInterpolation(motion_names, motion_keys, motion_times, True)
        except Exception, e:
            print "Motion exception, ", e
        
        print "pleasure: ", pleasure
        print "arousal: ", arousal
        print "dominance: ", dominance
        print "t1: ", t1
        print "shoulder_pitch: ", shoulder_pitch
        print "ankle_pitch: ", ankle_pitch
        print "walk: ", walk
        print "shoulder_roll_right: ", shoulder_roll_right
        print "shoulder_roll_left: ", shoulder_roll_left
        print "head_pitch: ", head_pitch
        print "---------- done ----------"

        memory.subscribeToEvent("TouchChanged", self.getName(), "emotive_motion")


def main():
    """ Main entry point

    """
    parser = OptionParser()
    parser.add_option("--pip",
        help="Parent broker port. The IP address or your robot",
        dest="pip")
    parser.add_option("--pport",
        help="Parent broker port. The port NAOqi is listening to",
        dest="pport",
        type="int")
    parser.set_defaults(
        pip=NAO_IP,
        pport=9559)

    (opts, args_) = parser.parse_args()
    pip   = opts.pip
    pport = opts.pport

    myBroker = ALBroker("myBroker",
       "0.0.0.0",   
       0,           
       pip,         
       pport)       

    
    global emotion_motion
    global memory

    emotion_motion = motion_module("emotion_motion")

    # set some ALMemory values
    pleasure = -1.0
    arousal = -1.0
    dominance = -1.0
    current_emotion = [(pleasure, arousal, dominance)]
    memory.insertData("Emotion/Current", current_emotion)


    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print
        print "Interrupted by user, shutting down"
        myBroker.shutdown()
        sys.exit(0)

if __name__ == "__main__":
    main()
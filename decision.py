import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function


# Helper function called to keep track of when rover should go into 
# pickle mode. 
# Rover:            Rover Data Structure
# time_limit:       The elapsed time tolerated before triggering pickle mode
def pickle(Rover, time_limit):
    # if the rover isn't moving, or is moving really slow
    if Rover.vel < 0.1:
        
        # if Rover.stopped_time is defined, then we are tracking an event already
        # so determine if we have exceeded time limit or not
        if Rover.stopped_time:
            if (Rover.total_time - Rover.stopped_time >= time_limit):
                Rover.mode = 'pickle'
                Rover.stopped_time = None
        
        # else, we arent tracking any current event so record the time the rover
        # stopped moving.
        else:
            Rover.stopped_time = Rover.total_time
            
    # Rover is not stopped, so just set the stopped time to None
    else:
        Rover.stopped_time = None
    return Rover
def decision_step(Rover):
# Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    # returning home
    if Rover.samples_collected == 4 or Rover.mapping_percentage > 95.0:
        print('RETURNING HOME')
        if abs(Rover.pos[0] - Rover.start_pos[0]) < 20 and abs(Rover.pos[1] - Rover.start_pos[1]) < 20:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            print('congratulations! rover at home')
            return Rover
        
        
    if Rover.mode == 'pickle':
        
        # PICKLE: Intended to ensure that the Rover always reliably 'gets out of
        # a pickle', by sweeping 45 degrees of terrain at a time until it finds 
        # a good nav solution to follow and resume Forward state. 45 degrees is a good
        # choice as it ensures that if the rover collided with a wall, it finds a solution
        # close to it's original trajectory, vs. potentially finding a best nav solution
        # by reversing its tract. Although it will eventually reverse track if a solution
        # can not be found in the first few 45 degree sweeps, so that it can get itself 
        # out of box canyons reliably.
        
        print("==================================PICKLE===========================")
        # If we dont have any nav angles at all, then just turn left continuously, until
        # we at least have SOME nav angles. Speeds up the getting to the boundary of a
        # navigable region rather than just going in 45 degree increments.
        if Rover.nav_angles is None:
            Rover.steer = 15.
            Rover.throttle = 0.
            return Rover
        
        # If we are moving at all in either direction, then put on the brake until
        # we are stopped
        if Rover.vel > abs(.1) or Rover.throttle > 0:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            return Rover
        
        # If we get here, we are stopped, so take off the brake 
        if Rover.brake != 0:
            Rover.brake = 0
            return Rover
        
        # Initialize some properties the first time through to keep track of things
        #    stopped_angle : Initial yaw condition of rover prior to sweep
        #    tgt_angle     : The angle we will sweep to while turning left
        #    bst_nav       : Tracks the highest number of nav pixels found during
        #                    45 degree sweep. Initialized with the minimum go forward
        #                    pixel count value so that if bst is not better than the minimum
        #                    we will not select it.
        #    bst_angle     : The angle where bst_nav was found. Initialized with the 
        #                    tgt_angle so that if no bst_nav solution is found, rover
        #                    will turn to the tgt_angle and start another sweep
        if not Rover.stopped_angle: 
            Rover.stopped_angle = Rover.yaw
            Rover.tgt_angle = (Rover.yaw + 45) % 360
            Rover.bst_nav = Rover.go_forward
            Rover.bst_angle = Rover.tgt_angle
            return Rover
        
        # Turn this bad boy left
        Rover.steer = 15
        
        # If we find an angle that has more pixels than bst_nav, 
        # then update bst_nav and record the current angle.
        if  Rover.nav_angles.size > Rover.bst_nav: 
            Rover.bst_nav = Rover.nav_angles.size
            Rover.bst_angle = Rover.yaw
        
        # If we are within +/- 5 degrees of the target angle then
        # stop and turn to the bst_angle by switching to azimuth mode
        if abs(Rover.tgt_angle - Rover.yaw) < 5:
            Rover.tgt_angle = Rover.bst_angle
        
        # In certain situations, the rover has a clear view to nice navigable
        # field, but terain outside of the rovers FOV is blocking it. In those
        # cases, make sure to guarantee that we turn a minimum of 20 degrees
        # per pickle so that we don't infinitely choose a bad route because we
        # can't detect the obstacles from the current POV.
            delta = Rover.tgt_angle - Rover.stopped_angle
            # Some strange looking maths to handle the case where we are crossing
            # zero degrees during our i.e. tgt angle = 20deg and stopped angle = 335deg
            if  (delta < -352) or (delta < 10 and delta > 0):
                Rover.tgt_angle = (Rover.tgt_angle + 20) % 360
        
        # Leving the pickle, set the pickle state properties back to initial
        # state for next time through. 
            Rover.bst_nav = 0
            Rover.stopped_angle = None
            Rover.mode = 'azimuth'
        print ("===================LEAVING PICKLE===================")
        return Rover
    
    
    
    
    
    # There were no nav angles present...go straight into a pickle
    else:
        print("Else Pickle at End")
        Rover.mode = 'pickle'
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    print("Function Return, Mode: ", Rover.mode)
    return 


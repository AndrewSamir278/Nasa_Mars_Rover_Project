import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function


# Collision Adjustment=====================================================
# Attempts to over-ride wall / nav pixel navigation decisions 
# In the event that there is un-navigable terrain in the path 
# of the rover
# Steer:     The steer angle detmermined by Nav/Wal pixel navigaiton
# c_angles:  List of angles to non navigable terrain we want to avoid
def collision_adj(steer, c_angles):
    # Only make adjustments if there is more than 10 pixels of non
    # navigable terrain
    if c_angles.size > 40:
        # Determine mean angle to the non-navigable pixels
        col_angle_mean = np.mean(c_angles) * 180./np.pi
        
        # There is a chance that the non navigable pixe mean
        # is straight ahead ( < 1 degree). If that is the case then
        # force the terrain to appear on the left, so that some
        # action is taken to break the decision.
        if abs(col_angle_mean) < 1.0: col_angle_mean = 15.
            
        # adjust the steering anngle scaled to the number of collision pixels 
        adj_angle = steer - col_angle_mean * c_angles.size/200 
        adj_angle = np.clip(adj_angle, -15.,15.)
    else:
        adj_angle = steer

    return adj_angle

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
    
  








    if Rover.mode == 'sample':
        
        # In sample mode we try and pick up  the target rocks. Sample just 
        # uses the pixel angle averaging technique for the gold rocks to 
        # aim the rover. Then uses telemetry reading near_sample to trigger
        # sending a pickup command to the rover.
        #
        # picked_up:         set to True after rover is finished picking up
        # sample_detected:   set to True afet a sample has been detected
        #
        print ("===================ENTERING SAMPLE===================")
        
        # call pickle in case we get stuck for more than 5 seconds we can
        # get ourselves unstuck. If we do trigger a pickle, return.
        Rover = pickle(Rover, 5)
        if Rover.mode == 'pickle': return Rover
        
        # The rocks are not always detectable on every scan, so make srue
        # there is valid data in the array, tehn caluclate the steer angle
        # to the target.
        if Rover.tgt_angles.size:
            Rover.steer = np.mean(Rover.tgt_angles * 180./np.pi)
            
        # If this is the first pass through on this mode, do some things
        # like stopping the rover completely, and setting picked_up to false
        if not Rover.sample_detected:
            if abs(Rover.vel) >= .1:
                Rover.brake_set = 10
                Rover.brake = Rover.brake_set
                return Rover
        
        # If stopped, then set sample_detected to True 
            else:
                Rover.sample_detected = True
        
        # Take off the brake and get rover up to max .5 velocity
        Rover.brake = 0
        if Rover.vel < .5:
            Rover.throttle = .1
        else:
            Rover.thottle = 0.
        
        # If the rover is near a sample based on telemetry feedback:
        # First make sure it is completely stopped before doing anything
        # else
        if Rover.near_sample:
            if abs(Rover.vel) >= .1:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                print ("=1=================LEAVING SAMPLE===================")
                return Rover
            
            # Now that it is stopped, send the pickup command to the rover
            Rover.send_pickup = True
            
            # Spin here while the rover is picking up. Go ahead and set 
            # sample_detected, send_picku to false to prepare for next
            # sample detection event, and assume a successful pickup by 
            # setting picked_up to True.
            while Rover.picking_up:
                print ("=2=================LEAVING SAMPLE===================")
                Rover.sample_detected = False
                Rover.send_pickup = False
                Rover.picked_up = True
                return Rover
        
        # If picked up is True then we must be done. Set picked up to False,
        # and mode to pickle so that we can gracefully leave the sample location.
        if Rover.picked_up:
            print ("=3================LEAVING SAMPLE===================")
            Rover.mode = 'pickle'
            Rover.picked_up = False
        print ("=4================LEAVING SAMPLE===================")
        return Rover
        
        
        
        
        
        
        
        
        
         
    
    
    
    if Rover.mode == 'azimuth':
        
    # In this state, the Rover will stop and turn until it's yaw is approx = to the Rover.tgt_angle
    # Only currently used in pickle mode to have the rover seek the tgt_angle after it is found. 
        print("==================================AZIMUTH===========================")
    
    # Check to make sure tgt_angle is valid before proceeding. If not, go into forward mode
        if np.isnan(Rover.tgt_angle):
            Rover.mode = 'forward'
            return Rover

        # first make sure that the rover is stopped
        if abs(Rover.vel) >= .1:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            return Rover
        
        # rover is stopped, release the brake and turn right. Turning right
        # makes sense since we want to turn back to the bst_angle found by 
        # pickle 
        else:
            Rover.brake = 0
            Rover.steer = -15
        
        # if we are within 3 degrees of teh tgt_angle, then that's good enough
        # put the rover in forward and leave azimuth mode
        if abs(Rover.yaw - Rover.tgt_angle) < 3:
            Rover.mode = 'forward'
        print("========================LEAVING AZIMUTH===========================")
        return Rover    
    
     # Do we have any valid Nav agles? We could just be looking at a black wall.
    if Rover.nav_angles is not None:
        
        if Rover.mode == 'forward':
            # Forward mode does several steps as follows at a high level:
            # 
            #   Step 1: Set max velocity, scaled off of the number of wall contour
            #           pixels detected. More pixels = longer contour = go faster
            #   
            #   Step 2: Calculate the preference (afdlya) for navigation based on naviable pixels  over wall contour navigation (p_n).
            #            
            #
            #            --> a: If there are any wall contour pixels available for navigation
            #                   navigate using the wall pixel angle combined with the navigable pixel mean, weighted by the number of navigable pixels.
            #                   Rover.steer = (Rover.steer + np.clip(nav_angle_mean * p_n  + (wal_angle_mean) * (1 - p_n),-15,15))/2
            #                   
            #                   More nav pixels = more bias away from the wall toward the naviable pixels. 
            #                   
            #            --> b: final collision adjustment done by the function
            #                   collistion_adj, that will over-ride a and b completely
            #                   if there appears to be enough of a collision hazard
            #                   directly in front of the rover.
            #            --> c: If there are any obstacles in front of the 
            #                   rover, increase the preference for navigating towards
            #                   navigable pixels based on how many navigable pixels 
            #                   are detected.
            #
            #   Step 3: Look for samples, and go into sample mode if any are seen
            #
            #
            
            print("==================================FORWARD===========================")
            # use pickle() to make srue we don't stay in forward, not moving forever.
            Rover = pickle(Rover, Rover.stopped_time_limit)
            
            if Rover.col_angles.size  > 80:	# was 60
            	Rover.mode = 'pickle'
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	print("////////////////////////////////////////////////////////")
            	return Rover
            	
            
            
            # if number of pixels of area that can be naviabled is greater than 300 go forward
            if len(Rover.nav_angles) >= Rover.stop_forward:
                
                # Determine mean distance of the wal contour pixels available for navigation
                wal_length = np.mean(Rover.wal_dists)                
                # Set the default velocity
                Rover.max_vel = 1.0   
                             
                # if the mean length is longer than 20, go faster
                # scaled by how much longer than than 20 it is.
                if wal_length >= 20:
                    Rover.max_vel = np.clip(wal_length/22, 0,2.0)	#wa Rover.max_vel = np.clip(wal_length/20, 0,3.0)
                    
                
                
                  
                # if going slower than max, throttle
                if Rover.vel < Rover.max_vel:
                    Rover.throttle = Rover.throttle_set
                else:
                    Rover.throttle = 0
                    
                # if going faster than,  brake lightly
                if Rover.vel > Rover.max_vel and Rover.vel > 1.0:
                    Rover.throttle = 0
                    Rover.brake = .03
                else:
                    Rover.brake = 0
                
                    
                # If there are any wal contour pixels available for naviagtion, use those
                # but weight the final result based off how much 'open' terrain there is represented by the navigable pixel count.
                # A wide open white field of navigable pixels is ~ 15,000 (we have get this number by looping over world map and get we have 15000 white pixels)
                if Rover.wal_angles.size: 
                    wal_angle_mean = np.clip(np.mean(Rover.wal_angles * 180/np.pi) + 10,-15,15) # Offset to keep off of wall
                    p_n = np.clip(Rover.nav_angles.size/12000.,.1,.9)
                    
                    # if the wal angle mean looks like we are actually detecting the contour of the warped
                    # fov, then navigate off of nav pixels entirely.
                    # if wal_angle_mean < -35.: p_n = .8
                
                # else there are no wal contour pixels, so navigate on navigable pixels only for now:
                else:
                    wal_angle_mean = 0
                    p_n = 1.
                print('Preference for Nav %i' % p_n)
                
                
                # IF there are any pixels in front of us that look like a collision, set a preference
                # for naviable pixel based navigation relative to the number of collidable pixels seen.
                # else preference for nav pixel navigation to zero.
                if Rover.col_angles.size: 
                    p_n = np.clip(Rover.nav_dists.size/40.,0.1,.9) 
                    
                    
                # Determine the mean angle of the navigable pixels
                nav_angle_mean = np.clip(np.mean(Rover.nav_angles * 180/np.pi),-15,15)
                
                # Set steering by determinig weighted average of wall contour and navigable pixels means
                # Include the previous Rover.Steer value in teh average to smooth response.
                print('Preference for Nav %f' % p_n)
                print('Wal angle Mean %f' % wal_angle_mean)
                print('Nav angle Mean %f' % nav_angle_mean)
                
                Rover.steer = (         Rover.steer + np.clip(nav_angle_mean * p_n  + (wal_angle_mean) * (1 - p_n),-15,15)        ) / 2

                # If we see any gold nuggets, go into sample mode now!
                if Rover.tgt_angles.any():
                    print('-------Decision: Sample-------')
                    Rover.mode = 'sample'
                    return Rover
                
                # Make final adjustments to steering decision to avoid clear and present obstacles
                # dreicetly in front of the rover.
                Rover.steer = collision_adj(Rover.steer, Rover.col_angles)
                
            # If there's a lack of navigable terrain pixels then go to 'picke' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'pickle'
                    Rover.stopped_time = None
        print("=========================LEAVING==FORWARD===========================")   
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


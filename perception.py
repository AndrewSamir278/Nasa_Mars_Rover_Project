import numpy as np
import numpy.ma as ma
import cv2



def get_contours(img, rgb_thresh=(170, 170, 170)):
    
    # get_contours finds all the contours present in the warped image. 
    # with the goal being to find edge between the sand and teh wall.
    # img:           wapred camera image
    # rgb_thresh:    threshold for determing sand / not sand
    
    
    
    # Create an array of zeros same xy size as img, but single channel
    thresh_img = np.zeros_like(img[:,:,0]) 
    # Require that each pixel be above 2 of the three threshold values in RGB
    # above_thresh will now contain a boolean array with 0-3. 
    above_thresh = (img[:,:,0] > rgb_thresh[0]).astype(int) + \
                   (img[:,:,1] > rgb_thresh[1]).astype(int) + \
                   (img[:,:,2] > rgb_thresh[2]).astype(int)
    # create the selection mask where above_thresh is used to select pixels where at least 2 of 3 RGB channels are above threshold
    # assign all elements in thresh_img wehre color_select = 1, to 255
    color_select = (above_thresh > 1).astype(int)
    thresh_img[color_select] = 255
    
    
    #convert contour image to grayscale
    imgray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY) 
    # convert grayscale image for contours to binary image, return imbin as binary image 
    (thresh, imbin) = cv2.threshold(imgray, np.average(rgb_thresh).astype(int), 255, cv2.THRESH_BINARY)
    

    # contours is a list of coordiantes of our contour.
    # the CHAIN_APPROX_NONE ensures we get all points without compression/ extrapolation.
    imcont = np.copy(imbin)	# since cv2.findcontours will modify image passed as first parameter
    contours, hierarchy = cv2.findContours(imcont,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    
    return thresh_img, imbin, contours

def color_thresh(img, rgb_thresh=(160, 160, 160), tgt=False, tol=(40,40,40)):
    # img                  transformed (warped) camera image array
    # rgb_thresh           tgt==False (default) return array where > rgb_thresh = True (1)
    #                      tgt==True  return array where rgb_thresh +/- rgb_thresh * tol = True (1)
    # tol                  when tgt==True, sets the tolerance +/- for a pixel to return true
    
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    
    # if tgt == True we are looking for samples so look for +/- from rgb_thresh
    if tgt:
        bool_array = (abs(img[:,:,0] - rgb_thresh[0]) < tol[0]) \
                   & (abs(img[:,:,1] - rgb_thresh[1]) < tol[1]) \
                   & (abs(img[:,:,2] - rgb_thresh[2]) < tol[2]) 
    
    # tgt is false, we are looking for nav terrain/obstacles
    else:    
        # Require that each pixel be above all three threshold values in RGB
        # above_thresh will now contain a boolean array with "True"
        # where threshold was met
        bool_array = (img[:,:,0] > rgb_thresh[0]) \
                   & (img[:,:,1] > rgb_thresh[1]) \
                   & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[bool_array] = 1
    # Return the binary image
    return color_select


def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # call separate function to convert to rover centric this way we can
    # re-use for the contour conversion step as well
    x_pixel, y_pixel = rover_coords_(xpos, ypos, binary_img, 0)
    return x_pixel, y_pixel

def rover_coords_(xpos, ypos, binary_img, offset):
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 + offset ).astype(np.float)
    return x_pixel, y_pixel

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
# TODO: Modify to account for non-square worlds, i.e. world_size[0,1]
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # NOTE: camera image is coming to you in Rover.img
    
    # 1) Define source and destination points for perspective transform
    source = np.float32([[13,140], [302,140], [200,96], [118,96]])  #four pixel coords from source image
    destination = np.float32([[155,155],[165,155],[165,145],[155,145]]) #four pixel coords from dest image
    
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)
    
    # change [:,:] to mask out portions of warped image if desired
    roi = np.copy(warped[:, :])  #120, 60:220 worked good for fidelity but mapped slowly so stopped using.
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    #GOLD ROCK ~ rgb = 189,144,19 --> 213,183,25 --> 255,219,54
    #OBSTACLES ~ rgb = 13,0,0
    nav_threshold = (190, 180, 160)
    tgt_threshold = (185, 140, 15)
    obs_threshold = (100, 100, 100)
    tgt_img = color_thresh(warped, tgt_threshold, tgt=True) # used for finding colored rocks
    obs_img = color_thresh(warped, obs_threshold) # used for finding obstacles
    threshedroi = color_thresh(roi, nav_threshold)   # color threshed roi for mapping.
    
    # Below determines the array used for detecting and trying to prevent collisions
    # it masks off only the section right in front of the rover.
    coll_roi = np.copy(warped[130:150, 150:170])             # was 130:150, 150:170
    coll_roi = cv2.bitwise_not(coll_roi)
    coll_roi = color_thresh(coll_roi,obs_threshold)
    

    

    # 3.5) Retrieve the contours for determining navigation
    cont_source = np.copy(warped)
    nav_img, imbin, contours= get_contours(cont_source, nav_threshold)
    # get the biggest contour (assume that is the one that is navigable, smaller ones are likley obstables / anomolies)
    # if no contours present, then return and hope we can pick one up next scan. Pickle logic will kick in eventually
    try:
        contour = max(contours, key = cv2.contourArea)
    except:
        #no controus returned... happens occasionally
        return Rover
    
    # Rover is a right wall follower, so mask out parts of the contour
    # that aren't on the right wall:
    nav_roi = contour[contour[:,:,0] > 160] #started at 160
    nav_roi = nav_roi[nav_roi[:,1] > 90] # started at 100
    nav_roi = nav_roi[nav_roi[:,1] < 150] # started at 140 
    xpos_w, ypos_w = nav_roi[:,0], nav_roi[:,1] #Separate out into x & y pixels (w stands for wall here)
    imbin_inv = np.invert(imbin)
    
     
    # 5) Convert map image pixel values to rover-centric coords
    xpix_rvr_wal, ypix_rvr_wal = rover_coords_(xpos_w, ypos_w, imbin, -3)   #for navigation (contour roi points in rover coordinates)
    xpix_rvr_nav, ypix_rvr_nav = rover_coords(nav_img)
    xpix_rvr_tgt, ypix_rvr_tgt = rover_coords(tgt_img) #for rock targets
    xpix_rvr_col, ypix_rvr_col = rover_coords(coll_roi)                          # -10 kept rover too far from wall
    
    xpix_rvr_obs, ypix_rvr_obs = rover_coords(obs_img) #for obstacles
    xpix_rvr_msk, ypix_rvr_msk = rover_coords(threshedroi) #for mappinig

    # 6) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    Rover.wal_dists, Rover.wal_angles = to_polar_coords(xpix_rvr_wal, ypix_rvr_wal)
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix_rvr_nav, ypix_rvr_nav)
    Rover.tgt_dists, Rover.tgt_angles = to_polar_coords(xpix_rvr_tgt, ypix_rvr_tgt)
    Rover.col_dists, Rover.col_angles = to_polar_coords(xpix_rvr_col, ypix_rvr_col)
    
    
    # 7) Convert rover-centric pixel values to world coordinates)
    xpos, ypos = Rover.pos
    yaw = Rover.yaw
    world_size = Rover.worldmap.shape[0]
    scale = 20 # was 100 --> 60% mapping     40 --> 80% mapping
    #xpix_wrld_nav, ypix_wrld_nav = pix_to_world(xpix_rvr_nav, ypix_rvr_nav, xpos, ypos, yaw, world_size, scale)
    xpix_wrld_tgt, ypix_wrld_tgt = pix_to_world(xpix_rvr_tgt, ypix_rvr_tgt, xpos, ypos, yaw, world_size, scale)
    xpix_wrld_obs, ypix_wrld_obs = pix_to_world(xpix_rvr_obs, ypix_rvr_obs, xpos, ypos, yaw, world_size, scale)
    xpix_wrld_msk, ypix_wrld_msk = pix_to_world(xpix_rvr_msk, ypix_rvr_msk, xpos, ypos, yaw, world_size, scale)
    
    
    
    # 8) Update Rover worldmap (to be displayed on right side of screen)
    # Only if roll and pitch are within tolerance (roll, pitch)
    tolerance = (1.5,1)
    if ((Rover.roll < tolerance[0]) or \
       (Rover.roll > (360.0 - tolerance[0]))) and \
       ((Rover.pitch < tolerance[1]) or \
       (Rover.pitch > (360.0 - tolerance[1]))):
        
        Rover.worldmap[ypix_wrld_obs.astype(int), xpix_wrld_obs.astype(int), 0] = 255
        Rover.worldmap[ypix_wrld_tgt.astype(int), xpix_wrld_tgt.astype(int), 1] = 255
        Rover.worldmap[ypix_wrld_msk.astype(int), xpix_wrld_msk.astype(int), 2] = 255

   
    
    
    
    if Rover.start_pos is None:
        Rover.start_pos = (Rover.pos[0], Rover.pos[1])
        print('STARTING POSITION IS: ', Rover.start_pos)
    
    
    
    
    
    # update an image to include our navigation data on HUD
    # Draw the entire contour on imgwcontour
    imgwcontour = cv2.drawContours(cont_source, contour,-1, (255,0,0), 1)
    
    # highlight the wall pixels we are navigating to 
    #(should match up exactly with a portion of the contour drawn above)
    imgwcontour[ypos_w,xpos_w, 1:2] = 255
    
    # (Show the current nav angle to the wall pixels
    cv2.putText(imgwcontour,"NavAngle To Wall: " + str(np.mean(Rover.wal_angles * 180/np.pi))[:4], (0, 20), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255), 1)
    # Show the average distances to the masked wall pixels. [:4] limits the string to 3 significant digits
    cv2.putText(imgwcontour,"NavPixels: " + str(len(Rover.nav_dists)), (0, 40), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255), 1)   
    cv2.putText(imgwcontour,"Mode: " + Rover.mode, (0, 60), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(imgwcontour,"Pickle: " + str(Rover.pickle), (0, 80), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(imgwcontour,"col_pix: " + str(Rover.col_angles.size), (0, 100), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(imgwcontour,"near_sample: " + str(Rover.near_sample), (0, 120), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 255), 1)
    if tgt_img.any():
        cv2.putText(imgwcontour,"SAMPLE DETECTED", (0, 140),cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
    #imgwcontour[obs_nav_xpix.astype(int), obs_nav_ypix.astype(int), 0:2] = 255
    Rover.vision_image = imgwcontour
    
    
    return Rover

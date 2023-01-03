# Nasa Rover Project
This project is modeled after the NASA Sample Return Robot Challenge and we can get experience with the three essential elements of robotics, which are perception, decision making and actuation. We will carry out this project in a simulator environment built with the Unity game engine.



![rover_image](https://user-images.githubusercontent.com/92469329/206799438-fbabbb93-415d-46ab-8715-c638a9858daa.jpg)

## 1.Traning Mode
We can test the simulator using "Traning Mode" by using Keyboard and Mouse


## 2.Autonomous mode
To navigate autonomously we have to call drive_rover.py. We have to complete perception.py and decision.py.

## 3.Data Analysis
Included in Jupyter Notebook to perform various steps of the project

## Pipeline

### Pipeline of Preception Step:

1-      size of input image(image came from camera) is 160*320 pxiels   
        160 pixels vertically
        320 pixels horizontally
        
2- 	Define source and destination points for generating mask which will helps to get prespective transform 
	the destination points will be 10 pixels* 10 pixels but we know that the size of grid 1 pixel *1 pixel
	so we need scale to be 10  --> but we did not make this scale as this will decrease fidelity  
	we choose scale between 15 and 20 so that we have good fidelity  (pixel will be represented in less that one pixel)









3-	Apply presective transform of camera image using perspect_transform and called the result  warped

	warped is image representing plane of camera image (top view)      
	      --> image like camera image but represent top view
	      
	perspect_transform is function take 3 paraments 
		parameter1 : image that we want to get prespective fot it (camera image)
		parameter2 : source points
		parameter3 : destination points      



4-	Take copy of wraped image using np.copy and called it roi
	roi -->  prescetive transform of camera image	
	
	copy is function from np library take image or part of image
	ex: copy(image)   =   copy(image[:,:])   
	--> copyed image is identical to original image
	
	ex: copy(image[130:140,150:170])    --> copy from image from 130 to 140 (vertically)
					    --> copy from image from 150 to 170 (horizontally)
					    --> copyed image is 10 pixels in height    and   20 pixels in width

5-	Apply color threshold on wraped image
	we apply color threshold (on wraped image) three times with different threshold values to identify navigable terrain/obstacles/rock samples in top view image
	
	we apply color threshold (on wraped image) with values 90, 180, 160  to identify navigable terrain in top view image and called result threshedroi image
	we apply color threshold (on wraped image) with values 185, 140, 15  to identify rock samples      in top view image and called result tgt_img image
	we apply color threshold (on wraped image) with values 100, 100, 100 to identify obstacles         in top view image and called result obs_img image



	we get color threshold using color_thresh function
	
	color_thresh is function take 3 parameters
		parameter1 : image that we want to apply threshold on it (wrapped image)
		parameter2 : tuple of three elements.each element represents threshold value for one channel of 3 channels
		parameter3 : boolean represent we applying threshold for identify rock samples



6-	we need to prevent collision by looking two meters square in front  of rover
	as each meter square is represented by 10*10      note: we do not apply scaling now  (i know that we do not apply scaling) 
	
	our idea is
	a-	take copy of part of wraped image ( from 130 to 150 (vertically) and from 150 to 170 (horizontally) )  which represent 2 m^2 infront of rover
		using np.copy and called it coll_roi
		
		coll_roi -->  part of prescetive transform of camera image which represent 2 m^2 infront of rover
		
	b-	applying not operation on coll_roi (part of prescetive transform of camera image which represent 2 m^2 infront of rover)
		we applying not operation using cv2.bitwise_not
		
		bitwise_not is function from cv2 library take one parameter
		parameter1 : image that we want to not operation on it (part of prescetive transform of camera image which represent 2 m^2 infront of rover)
		
	c-	applying thresholding to to identify obstacles from coll_roi (part of prescetive transform of camera image which represent 2 m^2 infront of rover )
		and called the result is color_thresh

7-	Finding edge pixels between the sand and teh wall using get_contours function
	
	get_contours is function take 2 parametes which is used to get array of arrays (each inner array contain pixels for a contour in image)
		parameter1: wraped image
		parameter2: threshold values of nagivable trainin for determing sand / not sand	
		
	get_contours is function returns 3 variables
		variable1: binary image that identify navigable terrain in top view image
		variable2: binary image of wraped	
		variable3: array of arrays represents pixels of each contours
	
	we find contours in wraped image using  cv2.findContours

	findContours is function of cv2 library, take 3 parameter
		parameter1: copy of binary representation of image that get want to find its contouts
		parameter2: cv2.RETR_TREE
		parameter3: CHAIN_APPROX_NONE --> get all points without compression/ extrapolation

                        

8. Update Rover worldmap (to be displayed on right side of screen and # update an image to include our navigation data on HUD and draw the entire contour on imgwcontour


















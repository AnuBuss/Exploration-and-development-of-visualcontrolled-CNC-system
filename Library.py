# -*- coding: utf-8 -*-
"""
Created on Thu Jun  3 12:28:40 2021

Library for VisualCNC

@author: aleks
"""
############################################################################
################################# Imports ##################################
############################################################################

import serial
import time
import sys
import glob
import os
import numpy as np
import cv2
import math
import tkinter as tk
from PIL import Image
from scipy.interpolate import splprep, splev  
import easygui
from datetime import datetime
from tkinter import filedialog

markers_found = False   # Dont touch this. 

############################################################################
########################## User Variables ##################################
############################################################################
depth_of_cut = 21 #mm
clearance = 10  #mm
feed_xy = 500   #mm/min
feed_z = 100     #mm/min
spindle_speed = 20000 #RPM
dz = 1.5 #mm
bit_radius = 1.5 #mm
gap_constant = 2 # gap in amount of bit radius, in pix
step_over = 0.75 # Percent of overlap between each pass

# Grid and step size for analyzing workspace
stepsize_x = 30 #mm
stepsize_y = 30 #mm
gridsize_x = 240 #mm
gridsize_y = 240 #mm

n_tabs = 5
tab_size = 3 #mm


correct_line = "middle" # "middle", "inner" and "outer" is possible. 
                        # This will determine whether the inner, 
                        # middle or outer contour is the correct to use 

cut_type = "Part"       #"Engrave" "Hole" "Part" 
                        # On Line cuts on the  line, this is good for engraving.
                        # Hole cuts the inside of the contour
                        # Part cuts the outside of the contour   

disregard_morph = True  # True if possible, False if extra help is needed to 
                        # connect contours                         

thresh_area = 51       # Variables to change if thresholding is doing weird stuff
thresh_bias = 7
# default used for debugging
scaling = 0.07122507122507123 #mm/pixel     
inv_scaling = 14 #pixel/mm

debugOutput = False     # Prints debug outputs to files.

resize = False          # Debugging and test tool.


############################################################################
########################## CNC control methods #############################
############################################################################

def init_grbl(ser):
    ##
    # Wake up grbl/marlin 
    ##
    string = "\r \n \r \n"
    ser.write(string.encode())
    time.sleep(2)   # Wait for grbl to initialize
    ser.flushInput()  # Flush startup text in serial input

def send_gcode(ser, file):
    ##
    # Send all lines in gcode file.
    ##
  
    for line in file:
        inp = line.strip() # Strip all EOL characters for consistency
        print('Sending: ' + inp)
        string = inp + '\n'
        ser.write(string.encode()) # Send g-code block to grbl
        while(True):
            grbl_out = ser.readline().decode() # Wait for marlin
            print( ' : ' + grbl_out.strip())
            if (grbl_out == "ok\n"):
                break
        
def serial_ports():
    
    ## 
    # Lists serial ports available
    ##
    
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def choose_COM():
    ## 
    # Open serial communication to CNC, ask user if more than one is available
    ## 
    
    ser_ports = serial_ports()
    if len(serial_ports()) == 0:
        print("No serial connection")
        return 0
        
    print("Serial ports found: ")
    for i in range(len(ser_ports)):
        print(ser_ports[i] + "   ")
    print("\n")
    
    if (len(ser_ports) > 1):
        for i in range(len(ser_ports)):
            print(str(i) + " = " + ser_ports[i] + "   ")
        inp = int(input("choose COM port: "))
        ser_port = ser_ports[inp]
    else:
        ser_port = ser_ports[0]
        
    return ser_port

def sendcode(string, ser):
    ##
    # Function for sending single g-code trough serial connection.
    ##
    
    inp = string.strip() # Strip all EOL characters for consistency
    print('Sending: ' + inp)
    string = inp + '\n'
    ser.write(string.encode()) # Send g-code block to grbl
    out = []
    while(True):
        grbl_out = ser.readline().decode() # Wait for marlin
        out.append(grbl_out)
        print( ' : ' + grbl_out.strip())
        if (grbl_out == "ok\n"):
            break
    return out

def gcodeStream(filename = " "):
    ##
    # Send codes sequentially to CNC machine, might add pause and feed button
    ##
    
    ser_port = choose_COM()
 
    ser = serial.Serial(port=ser_port, baudrate=115200)
    
    if filename == " ": 
        filename = easygui.fileopenbox() #open('testMarlin.gcode', 'r');
    
    file = open(filename,'r')

    init_grbl(ser)
    send_gcode(ser, file)

    ser.close()
    file.close()
    
############################################################################
######################## Image Processing methods ##########################
############################################################################

def binaryMorph(thresh):
    ##
    # Binary morphology operations
    ## 
    
    canny = cv2.Canny(thresh, 80, 255, 1)
    #debugDisplay(canny, debugOutput, "3_Canny")
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    #opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=5)
    opening = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel, iterations=5)
    
    dilate = cv2.dilate(opening, kernel, iterations=2)
    
    erode = cv2.erode(dilate,kernel, iterations=2)
    
    #debugDisplay(erode, debugOutput, "4_After_Morphology_operations")
    return erode

def analyzeMarkers(image, size): 
    ##
    # Find markers and calculate relevant parameters from this.
    ##
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_16h5)
    arucoParams = cv2.aruco.DetectorParameters_create()


    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,parameters=arucoParams)

    mean_dx = 0 
    mean_dy = 0
    
    try :
        ids[0]
    except :
        print("no markers found")
        return None, None, (None, None), None
    
    print(" %i Markers found!" % len(ids))
    for sets in corners:
        
        min_x = min(sets[0,:,0])
        min_y = min(sets[0,:,1])
        max_x = max(sets[0,:,0])
        max_y = max(sets[0,:,1])
        dx = max_x - min_x
        dy = max_y - min_y 
        mean_dx += dx
        mean_dy += dy
        
    mean_dx/=len(ids)
    mean_dy/=len(ids)
    
    scale_x = size/mean_dx
    scale_y = size/mean_dy
    scale_mean = (scale_x + scale_y)/2
    
    
    
    return ids, corners, (scale_x, scale_y), scale_mean

def RemoveMarks(image, corners):
    ##
    # Crops image with to remove all things outside region of interest, and 
    # fiducial markers
    ##
    x_vals = []
    y_vals = []
    for c in corners:
        x_vals.append(max(c[0,:,0]))
        x_vals.append(min(c[0,:,0]))
        y_vals.append(max(c[0,:,1]))
        y_vals.append(min(c[0,:,1]))
        
        # Remove feducial:
        image[int(min(c[0,:,1])):int(max(c[0,:,1])),int(min(c[0,:,0])):int(max(c[0,:,0]))].fill(255)
    
    cropMin_x = int(min(x_vals))
    cropMax_x = int(max(x_vals))
    cropMin_y = int(min(y_vals))
    cropMax_y = int(max(y_vals))
    
    cropped = image[cropMin_y:cropMax_y, cropMin_x:cropMax_x]
    
    #debugDisplay(cropped)
    return cropped
    
    
    

def create_blank(width, height, rgb_color=(255, 255, 255), gray=False):
    ##
    # Create new image filled with certain color in RGB
    ##
    image = np.zeros((height, width, 3), np.uint8)

    
    # Since OpenCV uses BGR, convert the color first
    color = tuple(reversed(rgb_color))
    # Fill image with color
    image[:] = color
    if gray == True:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
    return image


def Whiten(gray):
    ##
    # Normalises intensity values in grayscale image 
    # to set max to 255 and minimum to 0. Whiten might have been a bad name
    ##
    min_val = np.amin(gray)
    gray = gray - min_val
    max_val = np.amax(gray)
    gray = np.around(np.multiply(gray,255/max_val)).astype(np.uint8)
    return gray
    

def Sharpening(img):
    ##
    # Sharpening using simple linear filter.
    ##
    kernel = np.array([[-1,-1,-1], 
                       [-1, 9,-1],
                       [-1,-1,-1]])
    img = cv2.filter2D(img, -1, kernel)
    return img

def Skeletonize(img):
    ##
    # Tested skeletonize for acquiring midcontours, that didn't work
    ##
    skel = np.zeros(img.shape, np.uint8)

    # Get a Cross Shaped Kernel
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))

    # Repeat steps 2-4
    while True:
        #Step 2: Open the image
        open = cv2.morphologyEx(img, cv2.MORPH_OPEN, element)
        #Step 3: Substract open from the original image
        temp = cv2.subtract(img, open)
        #Step 4: Erode the original image and refine the skeleton
        eroded = cv2.erode(img, element)
        skel = cv2.bitwise_or(skel,temp)
        img = eroded.copy()
        # Step 5: If there are no white pixels left ie.. the image has been completely eroded, quit the loop
        if cv2.countNonZero(img)==0:
            break
    return skel

def blobfilter(image):
    ##
    # Blob Filter. Takes the worst noise that 
    # may interfere in the image processing
    ## 
    
    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 200


    # Filter by Area.
    params.filterByArea = True
    params.minArea = 0
    params.maxArea = 1000

    # Filter by Circularity
    params.filterByCircularity = False
    params.maxCircularity = 0.1
    
    # Filter by Convexity
    params.filterByConvexity = False
    params.maxConvexity = 0.7

    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.01

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    keypoints = detector.detect(image)

    for keypoint in keypoints:
        cv2.circle(image, (int(keypoint.pt[0]), int(keypoint.pt[1])), int(keypoint.size), (255,255,255), -1)

    return image

def cvStitch():
    ##
    # Performs worse than ghettostitching, leaving unfinished and distorted 
    # images due to bad features at the current resolution, performs well with
    # few images that is overlapping about 40%
    ##
    stitch_dir = filedialog.askdirectory()
    images = loadDataset(stitch_dir)
    stitcher = cv2.Stitcher.create(1) #cv2.Stitcher_SCANS
    stitcher.setPanoConfidenceThresh(1)
    status, result = stitcher.stitch(images)

    if status == 1 : 
        print("need more images!")
    elif status == 2 : 
        print("Homography estimation failed!")
    elif status == 3 :
        print("Camera Estimation Failed!")
    return result
    

############################################################################
######################## Contour Processing methods ########################
############################################################################

def closest_point_idx(point, contour): 
    ##
    # Used to find closest point in other contour pair 
    # for calculating midcontours
    ##
    xydist = (contour - point)
    absdist = np.sum(np.square(xydist),axis=2)
    return np.argmin(absdist)

def midcontour(contpair):
    ##
    # Calculate midcontours using vector mathematics
    ##
    midcnts = []
    for i in range(len(contpair[0])):
        closest = closest_point_idx(contpair[0][i],contpair[1])
        mid = np.round(contpair[0][i] + (contpair[1][closest] - contpair[0][i])/2)
        midcnts.append(mid)
    return np.array(midcnts,dtype=np.int32) 
        
def cntrMidpoint(cnt):
    ##
    # Calculate contours midpoints
    ##
    midpoints = []
    for i in range(len(cnt)):
        midpoints.append(sum(cnt[i])/len(cnt[i])) 
        
    return midpoints

def contoursmooth(cnt, threshold=100):
    ##
    # In testing phase: use interpolation to smooth contours. 
    ##
    for i in range(cnt)-1:
        nextdist = np.square(cnt[0][i] - cnt[0][i+1],axis=2)
        if nextdist > threshold :
            cnt[0].pop(i+1)
    return cnt
    
def CntSmooth(contours, points):
    ##
    # In testing phase: use interpolation to smooth contours. 
    ##
    smoothened = []
    for contour in contours:
        print(contour)
        x,y = contour.T
        # Convert from numpy arrays to normal arrays
        x = x.tolist()[0]
        y = y.tolist()[0]
        
        tck, u = splprep([x,y], u=None, s=1.0, per=1)
        # https://docs.scipy.org/doc/numpy-1.10.1/reference/generated/numpy.linspace.html
        
        u_new = np.linspace(u.min(), u.max(), points)
        # https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.interpolate.splev.html
        
        x_new, y_new = splev(u_new, tck, der=0)
        # Convert it back to numpy format
        
        res_array = [[[int(i[0]), int(i[1])]] for i in zip(x_new,y_new)]
        smoothened.append(np.asarray(res_array, dtype=np.int32))
    return smoothened



def contour_process(goodContours): 
    ##
    # Takes all found pairs and compute their midcontours
    ##
    pairs = []
    for i in range(len(goodContours)):
        if i%2 == 0:
            pairs.append([goodContours[i], goodContours[i+1]])


    ## Main contour script
    
    midcnts = []
    for i in range(len(pairs)):
   
        midcnt = midcontour(pairs[i])
        
        midcnts.append([midcnt])
    
    return midcnts

def outputContours(midcnts, image, outname):
    ##
    # Saves found contour for later use and debugging
    ##
    for i in range(len(midcnts)):
        cv2.drawContours(image, midcnts[i], -1, (0, 0, 0), 2)

    debugDisplay(image)

    ret, out = cv2.threshold(image,1,255,cv2.THRESH_BINARY)

    debugDisplay(out)
    cv2.imwrite(outname + '.jpg', out)

    cv2.destroyAllWindows()
    return out

def getMinDistFromMids(mids):
    ##
    # calculates minimum distance from contour midpoints 
    # to another contour midpoint 
    ##
    min_dists = []
    
    for i in range(len(mids)):
        xydist = (mids - mids[i])
        xydist[i] = 10000    # Dont worry, this is a trick to make sure that the closest is not itself :D 
        absdist = np.sum(np.square(xydist),axis=2)
        min_dists.append(min(absdist)) 

    return min_dists, absdist

def stripContourDuplicates(cnts):
    ##
    # Removes duplicate contours, this did raise errors in a few tests, so
    # it has been removed for now, but will be added soon
    ##
    print("Trying to strip %i Contours" % len(cnts))
    out = cnts.copy()
    dup_idx = []
    for i in range(len(cnts)):
        for j in range(i+1,len(cnts)):
            if len(cnts[i])==len(cnts[j]):
                dup_idx.append(j)

    dup_idx.sort(reverse=True)
    for i in range(len(dup_idx)):
        print(dup_idx)
        out.pop(dup_idx[len(dup_idx) - i - 1])
        
    return out

def getInnerOuterContours(cnts):
    ##
    # It is assumed that contours has been processed to pairs at this point
    # And has been stripped of duplicates
    ##
    inner = []
    outer = []
    for i in range(0,len(cnts),2):
        outer.append([cnts[i]])
        inner.append([cnts[i+1]])
    return inner, outer
    

def findClosedContourMatches(goodContours):
    ##
    # Compares all contours to establish closed contours from 
    # inner and outer contours. More info in thesis report.
    ##
    
    mids = cntrMidpoint(goodContours)[:]
    min_dists, absdist = getMinDistFromMids(mids)
    print(min_dists)

    threshold = 50000

    # If the minimum distance to the closest mid of contour is small and is agreed upon from two contours, then they make a outer inner pair
    pairs = []

    for i in range(len(absdist)):
        comparer = min_dists==min_dists[i]
        if min_dists[i] < threshold and sum(sum(comparer)) == 2 : #Found a pair!
            
            for j in range(len(min_dists)):
                if comparer[j] == True:
                    pairs.append(goodContours[j])
                
    closedContours = []
    
    # Original: [closedContours.append(x) for x in pairs if x not in closedContours]

    # Now:
    for cntr in pairs:
        closedContours.append(cntr)

    #Doesnt quite work... or yes?
    print(len(goodContours))
    return closedContours

def calcScaling(midcnts, max_size):
    ##
    # Calculates scaling from size specified by user if no fiducial is used
    ##
    
    # Max size scaling
    max_x = 0
    max_y = 0
    min_x = 10000000
    min_y = 10000000
    # find max and min pixel from bw image
    for i in range(len(midcnts)):
        cntr_x = midcnts[i][0][:,0,0]
        cntr_y = midcnts[i][0][:,0,1]
        
        if max(cntr_x) > max_x:
            max_x = max(cntr_x)
        if max(cntr_y) > max_y:    
            max_y = max(cntr_y)
        if min(cntr_x) < min_x:    
            min_x = min(cntr_x)
        if min(cntr_y) < min_y:
            min_y = min(cntr_y)
    
    # Default for logitech camera: 0.07122507122507123 cm/pix or 14 pix/cm
    print([min_x, max_x, min_y, max_y])
    scaling = min([max_size[0]/(max_x-min_x), max_size[1]/(max_y-min_y)]) #Scales according to critical axis
    return scaling

############################################################################
######################### File Conversion Methods ##########################
############################################################################

def potrace(Directory, filename):
    ##
    # In development: Use potrace to smooth curves with Bézier Curves
    ##
    os.chdir(Directory)
    os.system('potrace ' + filename + '.bmp -b svg')



def jpg2bmp(Directory, filename):
    ##
    # Simple file conversion from JPG to BMP
    ##
    os.chdir(Directory)
    Image.open(filename + '.jpg').save(filename + '.bmp')



def jpg2svg(Directory, filename):
    ##
    # Use potrace to convert jpg to svg
    ##
    jpg2bmp(Directory, filename)
    potrace(Directory, filename)


############################################################################
################################ GUI Methods ###############################
############################################################################

def openFile():
    ##
    # Lets user open file.
    ##
    filepath = filedialog.askopenfilename(initialdir="C:/Users/aleks/Desktop/DTU/Speciale - Visual CNC/Tests",
                                          title="Choose file to open",
                                          filetypes= (("Image files","*"),
                                          ("all files","*.*")))
    return filepath

def getHeightWidthFromUser():
    ##
    # Function for prompting size of contour if user dont have fiducials
    ##
    master = tk.Tk()
    tk.Label(master, text="No scaling has been calculated \n Enter height and width of contour in mm:").grid(row=0)
    tk.Label(master, text="Height: ").grid(row=1)
    tk.Label(master, text="Width: ").grid(row=2)


    e1 = tk.Entry(master)
    e2 = tk.Entry(master)
    
    e1.grid(row=1, column=1)
    e2.grid(row=2, column=1)
    
    tk.Button(master, 
              text='Submit', 
              command=master.quit).grid(row=3, 
                                        column=0, 
                                        sticky=tk.W, 
                                        pady=4)

    master.mainloop()

    h_str = e1.get()
    w_str = e1.get()
    try:
        h = float(h_str)
        w = float(w_str)
    except:
        print("YOU DUMBFUCK! Go fuck around somewhere else!")
        sys.exit()

    master.destroy()
    return [h, w]

############################################################################
############################## G-code Methods ##############################
############################################################################

class Tab:
    ##
    # Tab class. This was the best structure i could come up with to store 
    # tabs in a good way.
    ##
    def __init__(self, x, y, z, normx, normy, tab_size, gap):
        self.x = x
        self.y = y
        self.z = z
        self.normx = normx
        self.normy = normy
        self.size = tab_size
        # associated line:
        self.a = normy / normx
        self.b = y - self.a * x
        self.gap = gap
        self.maxPointDistXY = math.sqrt(gap**2 + self.size**2)
    def clearOfTab(self, x, y, z):
        point_distxy = math.sqrt((x-self.x)**2 + (y-self.y)**2)
        if point_distxy > self.maxPointDistXY : 
            return True
        
        dist_xy = abs(self.a * x + self.b - y)/math.sqrt(self.a**2 + 1)
        dist_xyz = math.sqrt(dist_xy**2 + ((z - self.z))**2)
        return dist_xyz > self.size

def get_tabs(midcnts, n_tabs, tab_size, tab_z=-depth_of_cut, scaling=scaling, gap=gap_constant*bit_radius): 
    ##
    # finds the tabs from the mid countours found before, 
    # the amount of tabs on each contour can be set, as well as their size
    ##
    
    tabs = []
    for i in range(len(midcnts)):
        length = len(midcnts[i][0])
        tab_idx = list(range(int(round(length/(n_tabs+1))),length,int(round(length/(n_tabs+1))))) # The indexes of where the tabs should be placed
        for idx in tab_idx[0:n_tabs]:
            tab_x = midcnts[i][0][idx][0][0] * scaling
            tab_y = midcnts[i][0][idx][0][1] * scaling
            tab_normx = -(midcnts[i][0][idx+5][0][1] * scaling - tab_y)
            tab_normy = (midcnts[i][0][idx+5][0][0] * scaling - tab_x)
            norm_len = math.sqrt(math.pow(tab_normx, 2) + math.pow(tab_normy, 2))
            tab_normx = tab_normx / norm_len
            tab_normy = tab_normy / norm_len
            tab = Tab(tab_x, tab_y, tab_z, tab_normx, tab_normy, tab_size, gap)
            tabs.append(tab)
    return tabs


def writeInitSequence(File, feedxy, clearance, speed):
    ##
    # Defines the current position as (0,0,0), parses the feed rate,
    # and starts the spindle
    ##
    File.write("G92 X0 Y0 Z0 \n")
    File.write("G01 Z%f F%f  \n" % (clearance, feedxy))
    File.write("M3 S%f \n" % (speed))

def writeToolChange(File, feedxy, clearance, Tool_name):
    ##
    # Stops spindle, raises to clearance plane, prompts a tool change,
    # User are meant to place the CNC on (0,0,0) before pressing ready
    ##
    File.write("M05 \n")
    File.write("G00 X0 Y0 Z50 \n")
    File.write("M18 \n")
    File.write("M00 ; Change Tool: %s \n" % Tool_name)
    File.write("G92 X0 Y0 Z0 \n")
    File.write("M17 \n")
    File.write("G01 Z%f F%f  \n" % (clearance, feedxy))
    File.write("M3 \n")

def writeEndSequence(File, clearance):
    ##
    # Raises to clearance plane and stops spindle. 
    ##
    File.write("G00 Z%f \n" % (clearance))
    File.write("M05 \n")
    
def extractCurrentPos(ser):
    ##
    # This kinda doesn't work, as marlin records the next position as the
    # command is passed to the stepper drivers. Leading to a reading for
    # where the machine is going. But can still be utilized to some degree
    # with some tricks in acquisition.
    ##
    out = sendcode("M114", ser)
    out = out[0]
    try:
        current_x = float(out[out.find(':')+1:out.find(' ')])
    except ValueError:
        return -1, -1, -1
    current_x = float(out[out.find(':')+1:out.find(' ')])
    out = out[out.find(' ')+1:-1]
    current_y = float(out[out.find(':')+1:out.find(' ')])
    out = out[out.find(' ')+1:-1]
    current_z = float(out[out.find(':')+1:out.find(' ')])
    return current_x, current_y, current_z


#% Write to file
def WriteGcode(File, cut_type, midcnts, innerCntr, outerCntr, depth_of_cut, dz, feed_xy, feed_z, clearance, tabs, scaling):
    ##
    #  Write paths found to .gcode file. See flowchart in thesis to get explanation
    ##
    writeInitSequence(File, feed_xy, clearance, spindle_speed)
    
    checker = True
    for j in np.arange(0, -depth_of_cut ,-dz):
        print(j)
        if cut_type == "Engrave":
            for i in range(len(midcnts)):
                contour = midcnts[i]
                x = round(contour[0][0][0][0]*scaling, 2)
                y = round(contour[0][0][0][1]*scaling, 2)
                z = j
                
                if len(midcnts)!=1: # for one contour this is unnecessary
                    File.write("G00 X%f Y%f Z%f \n" % (x, y, clearance))
                    
                File.write("G01 X%f Y%f Z%f F%f \n" % (x, y, z, feed_z))
                File.write("G01 F%f \n" % (feed_xy))
                for k in range(len(contour[0])):
                    x = round(contour[0][k][0][0]*scaling, 2)
                    y = round(contour[0][k][0][1]*scaling, 2)
                    #string = ("G00 X%f Y%f \n",contour[0][i][0][0], contour[0][i][0][1])
                    #string = ''.join(string)
                    for tab in tabs:
                        if tab.clearOfTab(x,y,z) == False:
                            File.write("G00 X%f Y%f Z%f \n" % (x, y, -depth_of_cut + tab.size))
                            checker = False
                            print("I avoided the tab!")
                            
                    if checker  == True :
                        File.write("G00 X%f Y%f Z%f \n" % (x, y, z))
                    else: checker = True
                    
                    if k == len(contour[0]) - 1:
                        x = round(contour[0][0][0][0]*scaling, 2)
                        y = round(contour[0][0][0][1]*scaling, 2)
                        z = j
                        File.write("G00 X%f Y%f Z%f \n" % (x,y,z))
                
                if len(midcnts)!=1: # for one contour this is unnecessary
                    File.write("G00 X%f Y%f Z%f \n" % (x, y, clearance))
        
        elif cut_type == "Hole":
            for i in range(len(innerCntr)):
                contour = innerCntr[i]
                x = round(contour[0][0][0]*scaling, 2)
                y = round(contour[0][0][1]*scaling, 2)
                z = j
                
                if len(midcnts)!=1: # for one contour this is unnecessary
                    File.write("G00 X%f Y%f Z%f \n" % (x, y, clearance))
                    
                File.write("G01 X%f Y%f Z%f F%f \n" % (x, y, z, feed_z))
                File.write("G01 F%f \n" % (feed_xy))
                for k in range(len(contour)):
                    x = round(contour[k][0][0]*scaling, 2)
                    y = round(contour[k][0][1]*scaling, 2)
                    #string = ("G00 X%f Y%f \n",contour[0][i][0][0], contour[0][i][0][1])
                    #string = ''.join(string)
                    for tab in tabs:
                        if tab.clearOfTab(x,y,z) == False:
                            if z > -depth_of_cut + tab.size:
                                print("I Avoided tab at ")
                                print((tab.x, tab.y))
                                tabs.remove(tab)
                            else:
                                File.write("G00 X%f Y%f Z%f \n" % (x, y, -depth_of_cut + tab.size))
                                checker = False
                           
                            
                    if checker  == True :
                        File.write("G00 X%f Y%f Z%f \n" % (x, y, z))
                    else: checker = True
                    #print(string)
                    
                    if k == len(contour) - 1:
                        x = round(contour[0][0][0]*scaling, 2)
                        y = round(contour[0][0][1]*scaling, 2)
                        z = j
                        File.write("G00 X%f Y%f Z%f \n" % (x,y,z))
                
                if len(midcnts)!=1: # for one contour this is unnecessary
                    File.write("G00 X%f Y%f Z%f \n" % (x, y, clearance))
    
        elif cut_type == "Part":
            for i in range(len(outerCntr)):
                contour = outerCntr[i]
                x = round(contour[0][0][0]*scaling, 2)
                y = round(contour[0][0][1]*scaling, 2)
                z = j
                
                if len(midcnts)!=1: # for one contour this is unnecessary
                    File.write("G00 X%f Y%f Z%f \n" % (x, y, clearance))
                    
                File.write("G01 X%f Y%f Z%f F%f \n" % (x, y, z, feed_z))
                File.write("G01 F%f \n" % (feed_xy))
                for k in range(len(contour)):
                    x = round(contour[k][0][0]*scaling, 2)
                    y = round(contour[k][0][1]*scaling, 2)
                    #string = ("G00 X%f Y%f \n",contour[0][i][0][0], contour[0][i][0][1])
                    #string = ''.join(string)
                    for tab in tabs:
                        if tab.clearOfTab(x,y,z) == False:
                            if z > -depth_of_cut + tab.size:
                                print("I Avoided tab at ")
                                print((tab.x, tab.y))
                                tabs.remove(tab)
                            else:
                                File.write("G00 X%f Y%f Z%f \n" % (x, y, -depth_of_cut + tab.size))
                                checker = False
                    if checker  == True :
                        File.write("G00 X%f Y%f Z%f \n" % (x, y, z))
                    else: checker = True
                    #print(string)
                    
                    if k == len(contour) - 1:
                        x = round(contour[0][0][0]*scaling, 2)
                        y = round(contour[0][0][1]*scaling, 2)
                        z = j
                        File.write("G00 X%f Y%f Z%f \n" % (x,y,z))
                
                if len(midcnts)!=1: # for one contour this is unnecessary
                    File.write("G00 X%f Y%f Z%f \n" % (x, y, clearance))
    
    
    writeEndSequence(File, clearance)
    File.close()
    


    
############################################################################
############################# Utility Methods ##############################
############################################################################

def camTest(idx): 
    ##
    # Function much like fotoshoot(), but without image saving, 
    # meant for checking cam setup
    ##
    cam = cv2.VideoCapture(idx)

    cv2.namedWindow("Capture")

    while True:
        ret, frame = cam.read()
        if not ret:
            print("failed to grab frame")
            break
        cv2.imshow("test", frame)
        k = cv2.waitKey(1)
        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
    cv2.destroyAllWindows()

def ResizeWithAspectRatio(image, width=None, height=None, inter=cv2.INTER_AREA):
    ##
    # Resizes the image with aspect ratio intact. This is useful in the 
    # debugDisplay() Function, for presenting an image in a meaningful way. 
    ##
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    return cv2.resize(image, dim, interpolation=inter)

def debugDisplay(img, out=False, outname="Debug_out"):
    ##
    # Display function for debugging. This is also how the subresults are output
    # Will be used to present the results to the user in a finished usecase.
    ##
    im2 = img.copy()
    resized = ResizeWithAspectRatio(im2, width=400)
    cv2.imshow('Debug display (Press Q to resume program)',resized)
    
    if out == True:
        time.sleep(1) # to make sure we down have the same time twice
        path = "C:/Users/aleks/Desktop/DTU/Speciale - Visual CNC/Tests/Debug_Out"
        os.path.join(path , 'waka.jpg')
        name = outname + "_" + str(datetime.now().hour) +"_"+ str(datetime.now().minute) +"_"+ str(datetime.now().second)
        cv2.imwrite(os.path.join(path , name + '.jpg'), img)
        print("Debug Display Saved")
    
    while(True):
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

def Fotoshoot(idx): 
    ##
    # Camera check function used for testing the camera.
    # Usage: Fotoshoot(0) for builtin webcam, Fotoshoot(1) for USB Webcam
    ##
    cam = cv2.VideoCapture(idx)

    cv2.namedWindow("Capture")

    img_counter = 0

    while True:
        ret, frame = cam.read()
        if not ret:
            print("failed to grab frame")
            break
        cv2.imshow("test", frame)
        
        k = cv2.waitKey(1)
        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
        elif k%256 == 32:
            # SPACE pressed
            img_name = "opencv_frame_{}.jpg".format(img_counter)
            cv2.imwrite(img_name, frame)
            print("{} written!".format(img_name))
        
            img_counter += 1

    cv2.destroyAllWindows()

def SampleImages(images):
    ##
    #  Sample a dataset to make it smaller. Used for debugging when working on 
    #  stitching
    ##
    for i in range(0,len(images), 3):
        print(i)
        cv2.imwrite(("image_%i.jpg" % i), images[i])

############################################################################
######################## Grayscale to carve G-code #########################
############################################################################

def carve3d():
    ##
    #  Feature for using an image to carve 3-dimensional lithophane style images. 
    #  This approach could also be usefull for doing 3D Cam operations in the future.
    #  Not a direct part of the Visual CNC system.
    ##
    x_size = easygui.integerbox('Enter maximum x size in mm (max 670):', 'x size', lowerbound=0, upperbound=670)
    y_size = easygui.integerbox('Enter maximum y size in mm (max 670), if zero the image height will follow chosen x:', 'y size', lowerbound=0, upperbound=670)
    z_size = easygui.integerbox('Enter maximum z size in mm (max 150), this will be the depest the allowed cut', 'y size', lowerbound=0, upperbound=150)
    clearance = 5
    image_filename = openFile()
    image = cv2.imread(image_filename)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    img_size = gray.shape
    gray = Whiten(gray)
    
    if y_size == 0 :
        y_size = img_size[1] * x_size/img_size[0]
    
    coordinate_grid = np.zeros((img_size[0],img_size[1],3))
    for x in range(img_size[0]) :
        for y in range(img_size[1]) : 
            coordinate_grid[x,y,0] = x * (x_size/img_size[0])
            coordinate_grid[x,y,1] = y * (y_size/img_size[1])
            coordinate_grid[x,y,2] = -gray[x,y] * (z_size/255)
            
    outName = easygui.enterbox("Name of output file:") + ".gcode"
    
    File = open(outName,"w")
    writeInitSequence(File, 1000, clearance, 12000)
    allowance = 0.5 #mm
    dz_rough = 1 #mm
    dz_fine = 0.1 #mm
    bit_size = 3 #mm
    
    up = True
    for z in range(-dz_rough,-z_size-1,-dz_rough):
        print(z+allowance)
        for x in range(img_size[0]):
            if x % 2 == 1:
                for y in range(img_size[1]-1,-1,-1):
                    if -coordinate_grid[x,y,2] < z + allowance:
                        if up == True:
                            File.write("G00 X%f Y%f" % (coordinate_grid[x,y,0], coordinate_grid[x,y,1]))
                            
                        up = False
                        File.write("G01 X%f Y%f Z%f  \n" % (coordinate_grid[x,y,0], coordinate_grid[x,y,1], z + allowance))
                    elif up == False: 
                        up = True
                        File.write("G01 Z%f \n " % (clearance))
            else:
                for y in range(img_size[1]):
                    if -coordinate_grid[x,y,2] < z + allowance:
                        if up == True:
                            File.write("G00 X%f Y%f" % (coordinate_grid[x,y,0], coordinate_grid[x,y,1]))
                            
                        up = False
                        File.write("G01 X%f Y%f Z%f  \n" % (coordinate_grid[x,y,0], coordinate_grid[x,y,1], z + allowance))
                    elif up == False: 
                        up = True
                        File.write("G01 Z%f \n " % (clearance))
    
    writeToolChange(File, 1000, clearance, "2mmBall")
    
    ### For debugging finishing by itself:
    File.close()
    File = open("FineTest.gcode", "w")
    ###
    
    up = True
    for x in range(img_size[0]):
        for y in range(img_size[1]):
            if coordinate_grid[x,y,2] > 0.05:
                File.write("G01 X%f Y%f Z%f  \n" % (coordinate_grid[x,y,0], coordinate_grid[x,y,1], -coordinate_grid[x,y,2]))
                
    File.close()
    
############################################################################
######################## Workspace Analysis scripts ########################
############################################################################

def ImageAcquisition(directory, gridsize_x, gridsize_y, stepsize_x, stepsize_y, ser=None):
    ##
    #  This script runs the user through the process of acquiring the images of the workspace.
    #  Images is taken in a grid of gridsize_x by gridsizy_y, with a stepsize of stepsize_x by stepsize_y
    #  taken in the direction of left to right, top to bottom. See documentation image.
    ##
    
    current_dir = os.getcwd()
    
    try:
        os.mkdir(directory)
        os.chdir(directory)
    except OSError:
        print ("Creation of the directory %s failed" % directory)
    else:
        print ("Successfully created the directory %s " % directory)
        
    current_x = 0
    current_y = 0
    
    cameraIndex = 0
    counter = 0
    
    if ser == None:
        ser_port = choose_COM()
        ser = serial.Serial(port=ser_port, baudrate=115200)
    
    sendcode("M05", ser) #Stop spindle 
    sendcode("M18", ser) #Disable Steppers
    
    cam = cv2.VideoCapture(cameraIndex)
    
    input("Place CNC on origin, Press Enter to continue...") # User sets a wanted origin.
    
    sendcode("M17", ser) #Enable Steppers
    sendcode("G92 X0 Y0 Z0", ser) # Declares current position as origin
    sendcode("G00 Z30", ser) # Raise Z axis so that there is no obstruction
    
    for i in range(int(gridsize_x/stepsize_x)+1):
        for j in range(int(gridsize_y/stepsize_y)+1):
            img_name = ("VisCNC_%i_%i.jpg" % (i,j))
            counter+=1
            sendcode("G00 X%.2f Y%.2f \n" % (j*stepsize_x, i*stepsize_y), ser)
            sendcode("M400 \n", ser)
            moving = True
            while moving == True : 
                current_x, current_y, current_z = extractCurrentPos(ser)
                if current_x == j*stepsize_x and current_y == i*stepsize_y:
                    moving = False
                    time.sleep(2)
                    ret, frame = cam.read()
                    cv2.imwrite(img_name, frame)
                    print("Picture taken at coordinate %.2f, %.2f" % (current_x, current_y) )
    os.chdir(current_dir)

def loadDataset(directory):
    ##
    #  Loads all images in the directory named VisCNC. 
    ##
    current_dir = os.getcwd()
    os.chdir(directory)
    images = []
    img_names = []

    for img in glob.glob("VisCNC*.jpg"):
        img_names.append(img)
        img1 = cv2.imread(img)
        images.append(img1)

    os.chdir(current_dir)
    return images

def GhettoStitch(images, gridsize_x, gridsize_y, stepsize_x, stepsize_y, kx, ky):
    #% GhettoStitching
    # stitchOut = False #flag for saving after stitching. False when debugging
    i = 0
    j = 0 
    kw = stepsize_x*4 + kx
    kh = stepsize_x*4 + ky
    grid = [int(gridsize_x/stepsize_x),int(gridsize_y/stepsize_y)] 
    
    
    height, width, channels = images[0].shape
    res = np.zeros((grid[0]*kh+height,grid[1]*kw+width),dtype='uint8')

    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,31,10)
        thresh = cv2.erode(thresh, None)
        
        # Res is constructed from the middle most part of each picture, which is the part with the most overlap to adjacent pictures
        temp = thresh[int((height-kh)/2):int((height+kh)/2),int((width-kw)/2):int((width+kw)/2)]
        res[i*kh + int((height-kh)/2) : i*kh + int((height+kh)/2), j*kw + int((width-kw)/2) : j*kw+int((width+kw)/2)] = temp
        j += 1
        if j == grid[1]:
            j = 0
            i += 1
            print(i)
            
    res = blobfilter(res)
    debugDisplay(res)
    
    return res

#% Camera Acquisition Script
def acquireAndStitch(stepsize_x, stepsize_y, gridsize_x, gridsize_y, kx, ky, acquire=False, stitchOut=False):
    ##
    #  acquire value states whether a new dataset needs to be acquired, if not, the user has to open the dataset directory
    #  stitchOut should be true if you want to save the stitched image, otherwise set to false.
    #  mostly useful for debugging.
    ##
    
    if acquire == True:
        stitch_dir = "Temp"
        ImageAcquisition(stitch_dir, gridsize_x, gridsize_y, stepsize_x, stepsize_y)
    else: 
        stitch_dir = filedialog.askdirectory()
    
    images = loadDataset(stitch_dir)
    
    res = GhettoStitch(images, gridsize_x, gridsize_y, stepsize_x, stepsize_y, kx, ky)
    
    if stitchOut == True:
        i = len(glob.glob("GhettoStitched*.jpg"))
        cv2.imwrite(("GhettoStitched_%i.jpg" % i), res)
    
    return res

############################################################################
############################ Engraving workflow ############################
############################################################################

def EngraveImageProcess(image):
    ##
    #  Engraving requires very high contrast contours, as to not be confused
    #  with contour noise in the image.
    ##
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray,150,255,cv2.THRESH_BINARY)
    debugDisplay(thresh)


if __name__=='__main__':
    # Placeholder for loading the library
    ...

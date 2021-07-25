# -*- coding: utf-8 -*-
"""
Created on Thu Jun  3 12:23:10 2021

@author: aleks

Created related to the work of the Master Thesis: Exploration and development
Of Computer Vision Based CNC System. 

As part of VisualCNC.

This script performs the operations described in 
"Image processing and Contour extraction" and 
"Path planning and G-code generation". 
All custom functions used are stored in Library.py

To run VisualCNC, use VisualCNC_GUI.py

debugDisplays can be uncommented to see internal image states.
"""

def run(runfile):
  with open(runfile,"r") as rnf:
    exec(rnf.read())

#% Import dependencies
import os
os.chdir("C:/Users/aleks/Desktop/DTU/Speciale - Visual CNC/Python_Code")
from Library import *
os.chdir("C:/Users/aleks/Desktop/DTU/Speciale - Visual CNC/Tests")

#% Begin flow

#% Import image, segment, and find contours 
image_filename = openFile()

if "GhettoStitched" in image_filename:
    Stitched = True # stitched is only relevant for ghettostitch, AutoStitch is handled automatically
    resize = False
else:
    Stitched = False

image = cv2.imread(image_filename)

size = image.shape

# Some debugging tetsts are performed from images taken with other cameras. 
# The resize is then a tool to better resemble the expected data 

if resize == True:
    image = cv2.resize(image, (2048, int(2048*size[0]/size[1])), interpolation = cv2.INTER_AREA)

#debugDisplay(image, False, "1_Scaled_Image")

# Analyze Apriltag markers
ids, corners, (scale_x, scale_y), scale_mean = analyzeMarkers(image, 50) # Output is only good if resize = False
try :
    ids[1]
    markers_found = True
except : 
    markers_found = False

if Stitched == False:
    image = cv2.bilateralFilter(image,10,125,125) # Can be uncommented if it is too much

    #debugDisplay(image)

    blur = image #blur = cv2.medianBlur(image, 5) 
    
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    gray = Whiten(gray)                               # Normalizes the gray scale image.

    thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, thresh_area, thresh_bias) # 101, 7 for pano16
    
    # If apriltags were found, remove them and crop the image according to their outer corners
    if markers_found == True: 
        thresh = RemoveMarks(thresh, corners)

    #print("Check 2")
    #debugDisplay(thresh)
    thresh = blobfilter(thresh)
    thresh = blobfilter(thresh)
    #print("Check 3")
    #debugDisplay(thresh)

    invthresh = cv2.bitwise_not(thresh)
    
    #debugDisplay(thresh, debugOutput, "2_Threshold_image") ## Adaptive Thresholding is sick bruh


else: 
    # The image transform is done within the stitching, so we just have to grayscale it.
    thresh = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

morphed = binaryMorph(thresh)
#debugDisplay(morphed)


if disregard_morph == True: # Better filter and drawing makes for higher precision, but lower robustness
    cnts = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
else :
    cnts = cv2.findContours(morphed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


cnts = cnts[0] if len(cnts) == 2 else cnts[1]

if Stitched == False:
    min_area = 5000         # Area descriptor is decent
    min_len = 150           # Length is hard to judge by
else:
    min_area = 8000         # Area descriptor is decent
    min_len = 500           # Length is hard to judge by

                        # the better noise removal, the smaller these can be

goodContours = []
for c in cnts:
    area = cv2.contourArea(c)
    length = cv2.arcLength(c,True)
    if area > min_area and length > min_len:
        cv2.drawContours(image, [c], -1, (255, 0, 0), 5)
        goodContours.append(c)
        #debugDisplay(image)
debugDisplay(image, debugOutput, "5_Outer_Contours_Found")
cv2.destroyAllWindows()



#% Contour processing :: Remove non-closed and stray contours.
goodContours = findClosedContourMatches(goodContours) # returns a pair for each good contour, this gives duplicates
#goodContours = stripContourDuplicates(goodContours) # This needs debugging to be added again.

if len(goodContours) == 0:
    print("Could not find any contours, change settings and try again!")
    sys.exit()

#% Find mid contours and show on image
image = create_blank(image.shape[1],image.shape[0],(255,255,255)) # cv2.imread(image_filename) # Reload image to disregard outer contours

if resize == True:
    image = create_blank(2048,int(2048*size[0]/size[1]),(255,255,255))

midcnts = contour_process(goodContours) # Calculates mid contours

## If the correct line to follow is the inner or outer, this is triggered
if correct_line == "inner" : 
    midcnts, _ = getInnerOuterContours(goodContours)
elif correct_line == "outer" : 
    _ , midcnts = getInnerOuterContours(goodContours)


# Draw contours and output image to OutputMidContours.jpg for skipping steps 
# up to this point in possible later attempts, this also makes the image to use
# in the g-code generation. 
outpic = outputContours(midcnts, image, "OutputMidContours") # used to create g-code later
outpic = cv2.cvtColor(outpic, cv2.COLOR_BGR2GRAY)

debugDisplay(outpic, debugOutput, "6_Midcontours_Established")


##############################################################################
#% G-code Generation :: use the extracted information to write a 
#% g-code script for the cut.
##############################################################################
if markers_found == True: # If markers were analyzed, then use that scaling
    scaling = scale_mean

# If no markers were used or for some reason could not be analysed, 
# get a size from user:
else : 
    try : 
        ids[0]
        scaling = scale_mean
    except :
        max_size = getHeightWidthFromUser()
        scaling = calcScaling(midcnts, max_size)

# Scaling is mm/pixel
inv_scaling = 1/scaling 
bit_radius = bit_radius * inv_scaling #pix
bit_diameter = 2 * bit_radius
gap = gap_constant * bit_radius #pix

#% Dilation operations for path planning
contour_orders = []
for order in range(int(np.round((gap - bit_radius)/(bit_radius*step_over)))+1):
    
    if order == 0 : # If order is zero, do complete clearance
        kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(int(np.round(bit_diameter)),int(np.round(bit_diameter)))) 
        eroded = cv2.erode(outpic,kernel,iterations=1)
        #debugDisplay(eroded, debugOutput, "7_Clearance_erosions")
    
    else : # if not, scale by stepover constant. 
        kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(int(np.round(bit_diameter*step_over)),int(np.round(bit_diameter*step_over))))
        eroded = cv2.erode(eroded,kernel,iterations=1)
        #debugDisplay(eroded, debugOutput, "7_Clearance_erosions")
    
    #canny = cv2.Canny(eroded, 20, 125)
    offset_cntrs = cv2.findContours(eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    offset_cntrs = offset_cntrs[0] if len(offset_cntrs) == 2 else offset_cntrs[1]
    offset_cntrs.pop(0)
    print(len(offset_cntrs))
    #for i in range(len())
    
    contour_orders.append(offset_cntrs)
    # debugDisplay(canny)

#% Divide result from waterline program into inner and outer contours
outerCntr = []
innerCntr = []
tests = []
    
for orders in contour_orders:
    mids = cntrMidpoint(orders)[:]
    min_dists, absdist = getMinDistFromMids(mids)
    test = findClosedContourMatches(orders)
    for i in range(int(len(test)/2)):
        length1 = cv2.arcLength(test[i*2],True)
        #print(length1)
        #print(mid)
        length2 = cv2.arcLength(test[i*2+1],True)
        #print(length2)
        if length1 > length2:
            outerCntr.append(test[i*2])
            innerCntr.append(test[i*2+1])
        else:
            innerCntr.append(test[i*2])
            outerCntr.append(test[i*2+1])
    tests.append(test)
    print(min_dists)
    #for c in orders : 
    #    cv2.drawContours(outpic, [c], -1, (255, 0, 0), 5)

#to see what is happening, reload outpic in rgb
outpic = outputContours(midcnts, image, "OutputMidContours")

for c in innerCntr:
    cv2.drawContours(outpic, [c], -1, (0, 255, 0), 5)
for c in outerCntr:
    cv2.drawContours(outpic, [c], -1, (0, 0, 255), 5)
# debugDisplay(outpic, debugOutput, "8_Inner_And_Outer_Cuts_found")

#% Output .gcode file from contour
gcodefile = image_filename.split("/")[-1].split(".")[0] + ".gcode"


#Write G-code
File = open(gcodefile,"w")

#% find tab placements
tabs = get_tabs(midcnts, n_tabs, tab_size, -depth_of_cut, scaling, gap)


WriteGcode(File, cut_type, midcnts, innerCntr, outerCntr, depth_of_cut, dz, feed_xy, feed_z, clearance, tabs, scaling)

print("G-code ready")

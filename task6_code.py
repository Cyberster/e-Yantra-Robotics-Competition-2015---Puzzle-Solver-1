'''
* Team Id: 			eYRCPlus-PS1#2678
* Author List: 		Avick Dutta, Arpan Das, Subhendu Hazra, Asesh Basu
* Filename: 		task5_code.py
* Theme: 			Puzzle Solver Robot (GLCD) - eYRCPlus
* Functions: 		sortContoursByArea, sortContoursByPosition, cropDigits, digitCount, detectDigit, imgToNumber,
*					cropBiggestContourAsImage, drawBorderAroundDigits, play, matcher, generate_output, substract_list,
*					get_result_set, puzzle, generate_traversal_path
* Global Variables:	<List of global variables defined in this file, none if no global variables>
'''

import numpy as np
import cv2
from operator import itemgetter
import itertools
from serial import Serial

######################### Task1 Functions start #######################
#######################################################################

######################### my functions ##########################

# sorting contours by area (bigger first)
'''
* Function Name: 	sortContoursByArea
* Input: 			contours -- is a list of contours as input argument #0, 
*					rev -- (reverse) is a boolean as input argument #1 : default True for bigger first
* Output: 			contours_ret -- is a list of sorted contours
* Logic: 			Iterate throught all contours and check for area of each one. Then we store area and coutour
*					pair in array variable area_list. Then we sort it by area using sorted function. After that
*					we append only contours from area_list to contours_ret. Then we return contours_ret
* Example Call: 	sortContoursByArea(contours, True)
'''
def sortContoursByArea (contours, rev=True): 
	area_list = []
	for contour in contours:
		item = [cv2.contourArea(contour), contour]
		area_list.append(item)
	area_list = sorted(area_list, key=itemgetter(0), reverse=rev) # 0=area, 1=contour
	
	contours_ret = []
	for i in range(0, len(area_list)):
		contours_ret.append(area_list[i][1]) 
	
	return contours_ret
# sortContoursByArea end

# sorting contours by position
'''
* Function Name: 	sortContoursByPosition
* Input: 			contours -- is a list of contours as input argument #0,
*					rev -- (reverse) is a boolean as input argument #1 : default True for bigger first,
*					sortBy -- (sort by direction) is a string as input argument #2 : default 'x'
* Output: 			contours_ret -- is a list of sorted contours
* Logic: 			We iterate through contours and get positions of each one. We append 3-tuple i.e.
*					x-coordinate, y-coordinate, and contour in array variable position_list.
*					Now, if sortBy is "x", then we sort position_list by x-coordinate with given rev
*					else, we sort position_list by y-coordinate with given rev. After that, we iterate
*					through position_list and append contours only from it to array variable contours_ret.
*					Then we return contours_ret
* Example Call: 	sortContoursByPosition(contours, True, 'x')
'''
def sortContoursByPosition (contours, rev=True, sortBy='x'):
	position_list = []
	for contour in contours:
		# finding centroid of a particular contour
		M = cv2.moments(contour)
		
		try: 
			cx = int(M["m10"]/M["m00"])
			cy = int(M["m01"]/M["m00"])
		except: pass
		
		item = [cx, cy, contour]
		position_list.append(item)
	if sortBy == "x": position_list = sorted(position_list, key=itemgetter(0), reverse=rev) # 0=cx, 1=cy
	else: position_list = sorted(position_list, key=itemgetter(1), reverse=rev) # 0=cx, 1=cy
	
	contours_ret = []
	for i in range(0, len(position_list)):
		contours_ret.append(position_list[i][2])
	
	return contours_ret
# sortContoursByPosition end
'''
* Function Name: 	cropDigits
* Input: 			img -- is an image as input argument #0,
* Output: 			returns crops -- is a 2d list containing cropped digits
* Logic: 			We convert the input image to grey scale and fetch contours from it. Then we crop output
*					the actial digit with rectangular shape by using cv2.boundingRect() function and append
*					the cropped portion to array variable crops. Then we return crops.
* Example Call: 	cropDigits(img, 1)
'''
# crop digits from an image with digits ony
def cropDigits (img):
	# getting contours
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU) # cv2.THRESH_BINARY = 0
	contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	contours = sortContoursByPosition(contours)
	
	# crop digit and store them into list crops
	crops = []
	for contour in contours:
		x, y, w, h = cv2.boundingRect(contour)
		crops.append(img[y:y+h, x:x+w])
	
	return crops
# cropDigits end

# count number of digit from a image (only with digits)
'''
* Function Name: 	digitCount
* Input: 			img -- is an image as input argument #0
* Output: 			contour_count -- is an integer, returns number of contours
* Logic: 			If img is a string i.e. image's file name, then we read the image, else its already an image.
*					Then, if the image is extreme black or extreme white i.e. the image is blank, then return 0.
*					Then we convert the input image to grey scale and fetch contours from it. After that, we count
*					the number of elements on variable contours return it.
* Example Call: 	digitCount(img)
'''
def digitCount (img):
	if type(img) == str: img = cv2.imread(img)
	h, w, c = img.shape
	
	# handling blank images
	if np.sum(img[:, :, :]) == 0 or np.sum(img[:, :, :]) == h*w*c*255: return 0
	
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU) # cv2.THRESH_BINARY = 0
	contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	contour_count = len(contours)
	
	return contour_count
# blankOrOneOrTwoDigits end

# detect a single digit from a image (only with one digit)
'''
* Function Name: 	detectDigit
* Input: 			img1 -- is an image as input argument #0
* Output: 			retVal -- is an integer, which is the recogized digit
* Logic: 			If img is a string i.e. image's file name, then we read the image, else its already an image.
*					Note that img1 is needle and img2 is haystack. We store height and width in img1 in h1 and w1 respectively.
*					We iterate through variable i in range 0-9 and indented process below is for each iteration. (Please see the 
*					directory "data" which has numbers stored as images with the number as file name. This is the memory of the program). 
*						We read the image with file name and stores height and width in variables h2 and w2 
*						respectively. Then we calculate average height and width and store them into hAvg and wAvg respectively. 
*						After that we resize both the images with average height and width i.e. hAvg and wAvg respectively. Then
*						we calculates the image difference by XOR-ing two images using cv2.bitwise_xor() method and store the 
*						result into variable diff. Then we sum up all pixels of diff (black=0, white=255) and store the result
*						in the variable sumOfDiff. After that we append pair of i and sumOfDiff into array matched_digit_list.
*					After all abobe process, we sort the matched_digit_list in ascending order and stores the first element i.e.
*					the least difference into variable retval and then we return retval.
*					
* Example Call: 	detectDigit (img)
'''
def detectDigit (img1): # img1 = needle, img2 = haystack
	if type(img1) == str: img1 = cv2.imread(img1)
	h1, w1, _ = img1.shape
	
	matched_digit_list = [] # it will contain list of index and difference magnitude

	for i in range (0, 10):
		img2 = cv2.imread("data/%d.png" % i)
		h2, w2, _ = img2.shape
		
		# resize both image of their average resolution
		hAvg, wAvg = (h1+h2)/2, (w1+w2)/2
		img1 = cv2.resize(img1, (wAvg, hAvg))
		img2 = cv2.resize(img2, (wAvg, hAvg))

		#diff = img1 - img2 # img2 - img1 not working
		diff = cv2.bitwise_xor(img1, img2) # Bitwise-AND mask and original image
		
		# applying morph: erosion operation
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
		diff = cv2.erode(diff, kernel, iterations=1)

		# applying median blur
		diff = cv2.medianBlur(diff, 3)

		# calculating sum of exclusive different pixels
		sumOfDiff = np.sum(diff[:, :, :])
		#print sumOfDiff
		
		matched_digit_list.append([i, sumOfDiff])
	# end for
		
	matched_digit_list = sorted(matched_digit_list, key=itemgetter(1)) # sort asc
	retval = matched_digit_list[0][0] # get most least difference
	
	return retval # return most matched
# end detectDigit

# comverts image containing 2 digits number to equivalent integer
# detect and convert image to number (with any no of digits)
# 1. count digits -> 2. crops them -> 3. detect each digit -> 4. calculate total value
'''
* Function Name: 	imgToNumber
* Input: 			img -- is an image as input argument #0
* Output: 			retval -- is an integer
* Logic: 			This function comverts image containing 2 digits number to equivalent integer. At first we check 
*					if img is a string i.e. image's file name, then we read the image, else its already an image.
*					Then we count digits using function digitCount() and store the result in the variable digCnt.
*					After that we crop img using function cropDigits() and store the result in variable d. Then we iterate i 
*					in range from 0 to digCnt and repeat the indented steps below.
*						We detect i-th digit from array variable d ans stores current digit	in the variable thisDigit.
*						Then we append thisDigit * 10^i to integer variable retval.
*					int variable retval will be returned.
* Example Call: 	imgToNumber(img)
'''
def imgToNumber (img):
	if type(img) == str: img = cv2.imread(img)
	h, w, c = img.shape
	
	digCnt = digitCount(img) # 1. count digits
	if digCnt == 0: return None
	
	d = cropDigits(img) # 2. crops them
	
	# 4. calculate total value
	retval = 0
	for i in range(0, digCnt):
		thisDigit = detectDigit(d[i]) # 3. detect each digit
		retval += thisDigit * pow(10, i) # because 123 = 3*(10^0) + 2*(10^1) + 1*(10^2)
	
	return retval
# imgToNumber end

# crop and get the biggest external contour from an image
'''
* Function Name: 	cropBiggestContourAsImage
* Input: 			img -- is an image as input argument #0
*					extraPadding -- (extra padding) is an integer as input argument #1 : default 0,
*					cropWidth -- (crop by width) is a boolean as input argument #2,
*					cropHeight -- (crop by height) is a boolean as input argument #3,
*					howMany -- (how many biggest contour to be returned) is an integer as input argument #4,
*					threshMode -- (threshholding mode) is an integer as input argument #5	
* Output: 			crops -- is a 2d list containing cropped digits
* Logic: 			If img is a string i.e. image's file name, then we read the image, else its already an image.
*					Then, if the image is extreme black or extreme white i.e. the image is blank, then return the img itself.
*					Then we convert the input image to grey scale and fetch contours from it and also count number of contours
*					and store it to variable contour_count. Then we sort contours by area using sortContoursByArea() function.
*					If howMany > contour_count, then we print error and return them img itself.
*					Now we iterate i in range from 0 to howMany and repeat the indenteded steps below:
*						We get top-left co-ordinate of i'th contour using cv2.boundingRect() method.
*						Then we add some padding according to boolean variables cropWidth and cropHeight and append the
*						cropped image to array variable crops.
*					All after above process, we return crops.
* Example Call: 	cropBiggestContourAsImage(img, extraPadding, threshMode=cv2.THRESH_OTSU)
'''
def cropBiggestContourAsImage (img, extraPadding=0, cropWidth=True, cropHeight=True, howMany=1, threshMode=cv2.THRESH_OTSU): # find the biggest contour using 
	if type(img) == str: img = cv2.imread(img)
	h, w, c = img.shape

	# handling blank images
	if np.sum(img[:, :, :]) == 0 or np.sum(img[:, :, :]) == h*w*c*255: return [img]
	
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(gray, 0, 255, threshMode) # cv2.THRESH_BINARY = 0
	contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	contour_count = len(contours)
	
	contours = sortContoursByArea(contours) # need sortContoursByArea() but if digit, need sortContoursByPosition()
	
	if howMany > contour_count:
		print "Error: howMany should be less than contour_count!"
		return [img]
	crops = []
	for i in range (0, howMany):	
		x, y, wd, ht = cv2.boundingRect(contours[i])
		if cropWidth == True and cropHeight == True: crops.append(img[y-extraPadding:y+ht+extraPadding, x-extraPadding:x+wd+extraPadding])
		elif cropWidth == True and cropHeight == False: crops[i].append(img[:, x-extraPadding:x+wd+extraPadding])
		elif cropWidth == False and cropHeight == True: crops[i].append(img[y-extraPadding:y+ht+extraPadding, :])
		else: crops[i].append(img)
	
	return crops
# cropBiggestContourAsImage end

# draw borders around digits for image
'''
* Function Name: 	drawBorderAroundDigits
* Input: 			img -- is an image as input argument #0,
*					color -- is a rgb color tuple as input argument #1,
*					borderWidth -- (border width) is an integer as input argument #2,
*					extraPadding -- (extra padding) is an integer as input argument #3 : default 0
* Output: 			img -- is an image
* Logic: 			Invert the img, crop out the biggest contour and store back to img. Then invert back the img.
*					Then we convert the input image to grey scale and fetch contours from it. Then we iterate 
*					through each contour and follow repeat the indented steps below.
*						Get contour area by using cv2.contourArea() method and if the area > 400 or area < 2800
*						then we draw green contour border of 2 pixels using cv2.drawContours() method
*					All after above process, we return img
* Example Call: 	drawBorderAroundDigits(original, (0, 255, 0), 2, extraPadding=10)
'''
def drawBorderAroundDigits (img, color, borderWidth, extraPadding=0):
	cv2.bitwise_not(img, img) 	# invert the image
	img = cropBiggestContourAsImage(img, extraPadding, threshMode=cv2.THRESH_OTSU)[0]
	cv2.bitwise_not(img, img) 	# invert the image back
	
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU) # cv2.THRESH_BINARY = 0
	contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	for i, contour in enumerate(contours):
		area = cv2.contourArea(contour)
		areaLowerBound = int(round((img.size/3) * 0.11083/100)) # 400 with ratio of current test image size
		areaUpperBound = int(round((img.size/3) * 0.77581/100)) # 2800 with ratio of current test image size
		if area > areaLowerBound and area < areaUpperBound:
			cv2.drawContours(img, contours, i, color, 2)
			
	return img
# drawBorderAroundDigits end
#################################################################

'''
* Function Name: 	play
* Input: 			img -- a single test image as input argument
* Output: 			No_pos_D1 -- List containing detected numbers in Division D1,
*					No_pos_D2 -- List of pair [grid number, value]
* Logic: 			At first we backup the img into variable orinal and invert img using cv2.bitwise_not()
*					Then we calculated the required padding and store it into variable padding. After that we get
*					the biggest contour from img and store it into variable arena. Then we get height and width
*					of arena and store them into variables h and w respectively. After that we draw a vertical
*					line to separate D1 and D2 and the line has distance 42% from left of the arena.
*					Then we applied morphing i.e. arode the arena.
*					Then we convert the image arena to grey scale and fetch contours from it. Then we reverse contours.
*					Then we iterate through each contour and follow repeat the indented steps below. (i=iteration counter)
*						if i > 37, then break the loop
*						calculate contour area and store it into variable area
*						get area bound i.e. allowed area for contours (size of a box) and store it into variable areaBound
*						if area < areaBound, then skip the rest part of cussrent iteration 
*						Then we get crop and cut the digit only, and convert it into actual integer number and store it 
*						to variable digit. 
*						Now, if i >= 0 and i <= 11, then append digit to array variable No_pos_D1
*						else if i >= 13 and i <= 36, then do as indented steps below:
*							If digit is valid, then append pair of i-13 and digit to variable pair.
*							After that append variable pair to array variable No_pos_D2.
*						i = i + 1
*					After that, draw 2px green border around digits using function drawBorderAroundDigits and show img
*					At the end, return No_pos_D1 and No_pos_D2
* Example Call: 	play(img)
'''
def play (img):
	original = img.copy() # making backup of original img
	cv2.bitwise_not(img, img) 	# invert the image
	padding = int(round((img.size/3) * 0.00138/100)) # 5px with ratio of current test image size
	arena = cropBiggestContourAsImage(img, extraPadding=padding)[0] # get the biggest external contour
	h, w, _ = arena.shape

	# draw line to separate D1 and D2 (image, start, end, color, thickness)
	cv2.line(arena, (int(w*42/100), 0), (int(w*42/100), h), (0, 0, 0), 1) # for extraPadding=30

	# apply morphing: erode arena
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
	erosion = cv2.erode(arena, kernel, iterations=1) # erosion
	arena = erosion

	## fetch divisions start
	gray = cv2.cvtColor(arena, cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
	contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	contours.reverse()

	arena_for_show = arena.copy() # backup
	No_pos_D1 = [] # format for D1 = [1, 2, 1, 3, 7, 6, 2, 9, 8, 1, 5, 4]
	No_pos_D2 = [] # format for D2 = [ [5,10], [6,11], [15,15] ]
	i=0
	for contour in contours:
		# value of i: 0-11=12 boxes of D1, 12=whole D1, 13-36=24 boxes of D2, 37=whole D2
		if i > 37: break
		area = cv2.contourArea(contour)
		# get area bound i.e. allowed area for contours (size of a box)
		#areaBound = int(round((arena.size/3) * 0.69268/100)) # 2500px with ratio of current test image size
		areaBound = int(round((arena.size/3) * 1/100)) # 2500px with ratio of current test image size
		if area < areaBound: continue
			
		x, y, w, h= cv2.boundingRect(contour)
		np = int(round((arena.size/3) * 0.00138/100)) # negative padding 5px with ratio of current test image size
		box = arena[y+np:y+h-np, x+np:x+w-np, :].copy() # h, w, c
		digit = imgToNumber(box)
		
		if i >= 0 and i <= 11: # get left div i.e. D1
			try: # getting error, some contour h=0 w=0 coming sometimes
				No_pos_D1.append(digit)
				## export single digits as file to build the memory/template
				# crop = cropBiggestContourAsImage(box)[0];
				# cv2.imwrite("data12/"+str(i)+".png", crop)
			except: pass
			# you may uncomment below and imshow("show", arena_for_show) to check the operational img
			#cv2.putText(arena_for_show, str(i), (x+60, y+80), 0, 0.5, (255, 255, 255), 1, cv2.CV_AA)
		elif i>=13 and i <= 36: # get right div i.e. D2
			try: # getting error, some contour h=0 w=0 coming sometimes				
				if not(digit == None): # if the digit is not none
					pair = [i-13, digit]
					No_pos_D2.append(pair)	
			except: pass
			# you may uncomment below and imshow("show", arena_for_show) to check the operational img
			#cv2.putText(arena_for_show, str(i-13), (x+60, y+80), 0, 0.5, (255, 255, 255), 1, cv2.CV_AA)	
		i += 1
	# end for
	#cv2.imshow("arena_for_show", arena_for_show)
	
	## drawing contours border around the digits and showing output image
	padding = int(round((arena.size/3) * 0.00692/100)) # 25px with ratio of current test image size
	img = drawBorderAroundDigits(original, (0, 255, 0), 2, extraPadding=padding)
	cv2.imshow("contour", img)
	
	# returning D1, D2 lists
	#No_pos_D1 = "D1 = " + str(No_pos_D1)
	#No_pos_D2 = "D2 = " + str(No_pos_D2)
	return No_pos_D1, No_pos_D2
######################### Task1 Functions end #########################
#######################################################################

######################### Task2 Functions start #######################
#######################################################################
########################################### My functions ###############################################
########################################################################################################

'''
* Function Name: 	matcher
* Input: 			Given_array -- is an integer list as input argument #0,
*					Required_number -- is an integer number as input argument #1 : default True for bigger first,
*					operands -- is an integer number as input argument #2 : default 1	
* Output: 			matcher.results -- is a static list of integer numbers : initialized with []
* Logic: 			If matcher.results is not empty or operands > 12, then return
*					Generate all unique possible combinations of Given_array with operands
*					Then iterating through each combinations, repeat the following indented steps below:
*						sum = 0
*						iterating throught range from 0 to operands, taking i as iteration counter, repeat dedention below:
*							sum = sum + i-th combination
*						if sum is equals to required number, then append conbination to array variable matcher.results
*					If matcher.results is empty, then recursively call function matcher() with operand+1
*					else sort matcher.results in reverse order.
* Example Call: 	matcher(D1, number)
'''
def matcher (Given_array, Required_number, operands=1): ################################################
	## recursion breaking condition: (matcher.results is not empty or operands > 12)
	if (matcher.results or operands > 12): return
	
	## generating all possible combinations
	combinations = list(itertools.combinations(Given_array, operands)) # returns combinations as list of tuples
	combinations = list(set(combinations)) # make combinations unique by convert to set and then list
	combinations = [list(x) for x in combinations] # convert tuples into lists
	
	for combination in combinations: # iterate through every distinct combinations
		sum = 0
		for i in range(0, operands): sum += combination[i] # calculating sum of n digit combinations
		if (sum == Required_number): matcher.results.append(combination) # append matched to results
	
	## if no result found i.e. matcher.results is empty for n operands, then recursively try with n+1
	if (not matcher.results): matcher(Given_array, Required_number, operands+1)
	else: matcher.results.sort(reverse=True)
matcher.results = [] # static variable of matcher() function
# end of matcher #######################################################################################

'''
* Function Name: 	generate_output
* Input: 			number -- is an integer numbers as input argument #0,
*					operands -- is an integer list as input argument #1	
* Output: 			output -- is a string with format x = y + z
* Logic: 			Concatenate number following " = " to initially blank string variable output.
*					If operand is not empty, then iterate through range from 0 to no of elements in operands and repeat indention below:
*						concatenate i-th operand from operands array to output string.
*						If i is not equal to no of elements in operands - 1 then concatenete " + " to output string.
*					else concatenate "Incomplete" to output string.
*					Return th output string variable.
* Example Call: 	
'''
def generate_output (number, operands): ################################################################
	output = str(number) + " = "
	
	if (operands): # if operand is not empty
		# forming the output strnig from operands list
		for i in range(0, len(operands)):
			output += str(operands[i])
			if not(i == len(operands)-1): output += " + "
	else:
		output += "Incomplete"
		
	return output
# end of generate_output ###############################################################################

# substract lit b from list a
'''
* Function Name: 	substract_list
* Input: 			a -- is an integer list as input argument #0,
*					b -- is an integer list as input argument #1	
* Output: 			a -- is an integer list
* Logic: 			Remove x from a for each x found in b.						
* Example Call: 	substract_list(D1, result)
'''
def substract_list (a, b): #############################################################################
	try:
		[a.remove(x) for x in b]
	except Exception as e: 
		pass
		#print e.args, e.message
	return a
# end of substract_list ################################################################################


'''
* Function Name: 	get_result_set
* Input: 			D1 -- is an integer list as input argument #0,
*					D2_values -- is an integer list as input argument #1	
* Output: 			result_set -- is an integer list in form of [x, y, z] if x = y + z,
*					priority -- is an integer which is the priority of a result set by no of operand used and incomplete.
* Logic: 			Iterate through each number present in D2_values and repeat the indented steps below:
*						Call matcher(D1, number)
*						If matcher.results is not empty then set variable result = matcher.results and reset matcher.results
*						If result is found, then remove the result from D1
*						Make array of one element i.e. number and result and append the pair into result_set (result_set 
*						will have lists that each list contains list thatthe first element is the number [0], and rest of 
*						others [1:]are the operands used the make the sum of that number)
*					Initialize new variable operand_used = 0, incomplete = 0, priority = 0, priority_one_var = 0, 
*					priority_two_or_more_var = 0, i.e. all equals to 0 initially.
*					Now, iterate through each result present in result_set and follow the indented steps below:
*						operand_used = operand_used + number of elements in result - 1
*						If number of elements in result is equals to 1, then incomplete = incomplete + 1 (i.e. no = incomplete (no operand))
*						Else if number of elements in result is equals to 2, then priority_one_var = priority_one_var + 1 (i.e. no = x (one operand))
*						Else priority_two_or_more_var = priority_two_or_more_var + 1 (i.e. no = x+y+... (two or more operands))
*					operand_used = 12 - operand_used (complement it as lower is better (12 = no of element in D1))
*					incomplete = number of element in D2_values - incomplete (complement it too as lower is better)
*					incomplete = incomplete * incomplete (giving more -ve priority: as using more operands is better than leave incomplete)
*					Then finally priority = (incomplete * operand_used) - priority_one_var + priority_two_or_more_var
*					All after abobe process, we return result_set and priority.
* Example Call: 	get_result_set(list(D1), combination)
'''
def get_result_set (D1, D2_values): ####################################################################
	## getting result set for all different combinations of D2
	result_set = []
	for number in D2_values: # getting each values of D2
		try:
			result = [] # stores the best result

			## find result
			matcher(D1, number)
			if (matcher.results):
				result = matcher.results[0]
				matcher.results = []
				
			## remove result from D1
			if result:
				D1 = substract_list(D1, result)
			
			# result_set will have lists that each list contains list that
			#	the first element is the number [0], and rest of others [1:]
			# 	are the operands used the make the sum of that number
			result_set.append([number] + result)
			#print generate_output(number, result)
		except RuntimeError as re: print re.args, re.message
	# end for loop
	
	## calculate priority of the result set
	operand_used = 0
	incomplete = 0
	priority = 0
	priority_one_var = 0 # priority decrementer
	priority_two_or_more_var = 0 # priority incrementer
	
	for result in result_set:
		operand_used += len(result)-1
		if (len(result) == 1): incomplete += 1 # no = incomplete (no operand)
		elif (len(result) == 2): priority_one_var += 1 # no = x (one operand)
		else: priority_two_or_more_var += 1 # no = x+y+... (two or more operands)
	# end for loop

	operand_used = 12 - operand_used # complement it as lower is better (12 = no of element in D1)
	incomplete = len(D2_values)-incomplete # complement it too as lower is better
	incomplete *= incomplete # giving more -ve priority: as using more operands is better than leave incomplete
	priority = (incomplete * operand_used) - priority_one_var + priority_two_or_more_var
	
	return result_set, priority
# end of get_result_set ################################################################################
########################################################################################################

'''
* Function Name: 	puzzle
* Input: 			D1 -- is an integer list as input argument #0,
*					D2 -- is an integer list as input argument #1
* Output: 			retVal -- is a 2D integer list
* Logic: 			Sort D1 in desc order. Then get only values without positions from D2 and sort by descending order.
*					Then we get all possible combinations of D2. Initially we set integer variable max_priority = 0.
*					Now we iterate through each combination in D2_combinations and do the indented steps below:
*						get result_set, priority from function get_result_set(list(D1), combination)
*						If priority >= max_priority, then set max_priority = priority and best_result_set = result_set.
*					Then sorting back best_result_set by original positions of D2 though it doesn't matter as e-yantra said.
*					Now, iterate through each result in results_sorted and taking i as iterator counter, repeat as indented follows:
*						Append result to retVal
*					After all the above process, return retVal.
* Example Call: 	puzzle(D1, D2)
'''
def puzzle (D1, D2):
	## sort D1 in desc order
	D1.sort(reverse=True)
	
	## get only values without positions from D2 and sort by DESC order
	D2_values = []
	#D2_pos_value_pair = []
	for i in xrange(1, len(D2), 2): # getting each values of D2
		D2_values.append(D2[i])
		#D2_pos_value_pair.append([D2[i], D2[i-1]])

	## get all possible combinations of D2
	# returns combinations as list of tuples
	D2_combinations = list(itertools.permutations(D2_values))
	# convert them into list of lists
	D2_combinations = [list(x) for x in D2_combinations]
	
	## get through all combinations -> calculate priority -> choose result with highest priority
	max_priority = 0
	best_result_set = []
	for combination in D2_combinations:
		result_set, priority = get_result_set(list(D1), combination)
		if (priority >= max_priority):
			max_priority = priority
			best_result_set = result_set
	
	## sorting back best_result_set by original positions of D2 though it doesn't matter as e-yantra said
	# best_result_set.sort(key=lambda x: D2_values.index(x[0])) # this cant sort if duplicate found on D2
	results_sorted = []
	for i in range(len(D2_values)): # iterate through every value of D2_values
		j = 0 # index for D2
		# until best_result_set become empty, find first occurance and append to results_sorted
		while (len(best_result_set) > 0):
			if (D2_values[i] == best_result_set[j][0]): # if match found
				results_sorted.append(best_result_set[j]) # append it to results_sorted
				del best_result_set[j] # delete j-th result from best_result_set
				break # break after first occurance
			j += 1
		# end while
	
	## print best_result_set
	retVal = []
	for i, result in enumerate(results_sorted):
		#print generate_output(result[0], result[1:])
		retVal.append(result)
	return retVal
# end of puzzle

######################### Task2 Functions end #########################
#######################################################################

######################### Task4 Functions start #######################
#######################################################################

'''
* Function Name: 	generate_traversal_path
* Input: 			D1 -- is an integer list as input argument #0,
*					D2_1D -- is an integer list as input argument #1,
*					results -- is a 2D integer list as input argument #3	
* Output: 			output_str -- is a string vaariable which stores the traversal path
* Logic: 			Prioritize D1 to get nearest number among ambigious numbers by manually sorting position-value pair in D1
*					and store the result pairs into D1_prioritized_kv. Then get only values from D1_prioritized_kv and store
*					the result into D1_prioritized_v. Now we need to make values invalid, so the keys i.e. positions will be 
*					only valid. Copy D2_1D into D2_copy. Then iterate through D2_copy, for each element present in D2_copy and
*					taking i as iteration counter repeat as indented steps follows:
*						if i is even i.e. it a value, then set i-th element of D2_copy to -1 to make it invalid.
*					Now we generate solution se in this form: D2 position, D2 value, D1 position, D1 value, D1 position, D1 
*					value... (rest operands in D1); D2 value, D1 position, D1 value, D1 position, D1 value...; ... (rest 
*					solution set in D2, D1). e.g. - 1,16,3,8,10,8;10,14,1,9,2,5;23,10,6,7,7,3. To do that,
*					terate through each element i.e. solution in results taking i as iteration counter repeat indention below:
*						Iterate through each number present in each solution taking j as iteration counter repeat indention below:
*							If j is equal to 0, then do as indention below:
*								Get the position index of first occurance of y in D2_copy and stor it into variable pos.
*								Concatenate pos-1 th element of D2_1D into output_str following a comma.
*								Concatenate y into output_str.
*								Set -1 to pos th element in D2_1D to make a used number invalid.
*								Set -1 to pos th element in D2_copy mark the current D2 number as used so that the robot will not fill it again.
*							Else, do as indention below:
*								Get the position index of first occurance of y in D1_prioritized_v and stor it into variable pos.
*								Get the actual position from D1_prioritized_kv and store it into variable actual_pos.
*								Concatenate actual_pos into output_str following a comma then concatenate y into it.
*								Set -1 to pos th element in D1_prioritized_v to make a used number invalid.
*							If j is not equal to (number of elements in x) - 1, then concatenate comma into output_str.
*						If j is not equal to (number of elements in result) - 1, then concatenate semicolon into output_str.
*					Now we are going to generating the traversal path of the robot in comma separated format i.e. D1 position, D1 value, 
*					D2 position, D2 value, D1 position, D1 value ... e.g. - 3,8,1,16,10,8,1,16,1,9,10,14,2,5,10,14,6,7,23,10,7,3,23,10.
*					To do that, we split output_str by semicolon and save the returned array into variable results_array and reset output_str.
*					Now, we iterate through each result present in results_array and do the indented steps below:
*						We split result by comma and save the returned array back into variable result. Initialize variable j = 0.
*						We iterate through each number present in result[2:] i.e. 2nd element to last element if result, follow indention below:
*							If j is even do as following indented steps:
*								Concatenate 2+j th element of result into output_str following a comma.
*								Concatenate 2+j+1 th element of result into output_str following a comma.
*								Concatenate 0 th element of result into output_str following a comma.
*								Concatenate 1 st element of result into output_str following a comma.
*							j = j + 1
*					After that removing the last comma from output_str and return output_str.
* Example Call: 	generate_traversal_path(D1, D2_1D, results)
'''
def generate_traversal_path (D1, D2_1D, results):
	output_str = ""
	
	# prioritize D1 to get nearest number among ambigious numbers
	D1_prioritized_kv = [] # holds manually sorted position-value pair
	D1_prioritized_kv.append([7, D1[7]])
	D1_prioritized_kv.append([11, D1[11]])
	D1_prioritized_kv.append([10, D1[10]])
	D1_prioritized_kv.append([6, D1[6]])
	D1_prioritized_kv.append([3, D1[3]])
	D1_prioritized_kv.append([2, D1[2]])
	D1_prioritized_kv.append([9, D1[9]])
	D1_prioritized_kv.append([5, D1[5]])
	D1_prioritized_kv.append([1, D1[1]])
	D1_prioritized_kv.append([8, D1[8]])
	D1_prioritized_kv.append([4, D1[4]])
	D1_prioritized_kv.append([0, D1[0]])
	
	# get only values from D1_prioritized_kv
	D1_prioritized_v = []
	for x in D1_prioritized_kv:
		D1_prioritized_v.append(x[1])
	
	# we need to make values invalid, so the keys i.e. positions will be only valid
	D2_copy = list(D2_1D) # backing up D2_1D
	for i, x in enumerate(D2_copy):
		if (i%2 == 0): D2_copy[i] = -1
		
	# generate solution se in this form: D2 position, D2 value, D1 position, D1 value, D1 position, D1 value... (rest operands in D1);
	#									D2 value, D1 position, D1 value, D1 position, D1 value...; ... (rest solution set in D2, D1)
	#	e.g. - 1,16,3,8,10,8;10,14,1,9,2,5;23,10,6,7,7,3
	for i, x in enumerate(results):
		for j, y in enumerate(x):
			if (j == 0): # number of D2
				pos = D2_copy.index(y)
				output_str += str(D2_1D[pos-1])
				output_str += ','
				output_str += (str(y))
				D2_1D[pos] = -1
				D2_copy[pos] = -1 # mark the current D2 number as used so that the robot will not fill it again
			else: # operands from D1
				pos = D1_prioritized_v.index(y)
				actual_pos = D1_prioritized_kv[pos][0]
				output_str += str(actual_pos)
				output_str += ','
				output_str += (str(y))
				D1_prioritized_v[pos] = -1;
			if (j != len(x)-1): output_str += ','
		if (i != len(results)-1): output_str += ';'
		
	#print "\noutput_str: ", output_str
	
	# generating the traversal path of the robot in comma separated format
	# 	D1 position, D1 value, D2 position, D2 value, D1 position, D1 value ...
	#	e.g. - 3,8,1,16,10,8,1,16,1,9,10,14,2,5,10,14,6,7,23,10,7,3,23,10
	results_array = output_str.split(";")
	output_str = ""
	
	for result in results_array:
		result = result.split(",")
		
		j=0
		for num in result[2:]:
			if j%2 == 0:
				output_str += result[2+j] + "," 	# concatenating D1 position
				output_str += result[2+j+1] + ","	# concatenating D1 value
				output_str += result[0] + ","		# concatenating D2 position
				output_str += result[1] + ","		# concatenating D2 value
			j += 1
		#output_str += "-1," # to indicate completion of a number in D2
		
	output_str = output_str[:-1] # removing the last comma
	return output_str
# end of generate_traversal_path

######################### Task4 Functions end #########################
#######################################################################

'''
* Function Name: 	main
* Input: 			"Bonus/Puzzle Solver 1.jpg" as image path and the COM PORT number with which the robot is connected.
* Output: 			D1, D2
* Logic: 			We read the image and pass it to the play() function to do OCR and store the results in variable D1 and D2.
*					Then we convert 2d list into 1d list i.e. from D2 to D2_1D.
*					Now we run our puzzle solver algorithm i.e. function puzzle() store the result into variable results.
*					Then we generate output that will be sent over serial communication terminal software or pyserial to the robot
*					before pressing the boot key i.e. interrupt key using generate_traversal_path() function.
* Example Call: 	Called automatically by the Operating System.
'''
if __name__ == "__main__":
	img = cv2.imread('Bonus/Puzzle Solver 1.jpg')
	com_port = 4 # COM PORT
	
	D1, D2 = play(img)
	
	print 'D1 =', D1
	print 'D2 =', D2
	
	# converting 2d list into 1d list
	D2_1D = []
	for i in D2:
		for j in i:
			D2_1D.append(j)
	D2_1D
	
	results = puzzle(list(D1), D2_1D)
	
	
	# this string will be sent over serial communication terminal software or pyserial to the robot
	# 	before pressing the boot key i.e. interrupt key
	output_str = generate_traversal_path(D1, D2_1D, results)
	#print '\nResults =', results
	#print '\nString to be sent: ', output_str
	
	try:
		# send output_str to the robot using pyserial
		ser = Serial("com"+str(com_port), 9600) # parameters: comport, baudrate
		ser.write(output_str) #send file
	except:
		print "\nError: \tCould not open COM Port COM"+str(com_port)
		print "\tPlease connect the robot to the PC via USB to Serial cable and retry"
	
	cv2.waitKey(0)
	cv2.destroyAllWindows()

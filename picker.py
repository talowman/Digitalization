import serial 
import time
import cv2
import numpy as np
import pprint

# Specify the paths for the 2 files 
protoFile = "pose_deploy_linevec_faster_4_stages.prototxt"
weightsFile = "pose_iter_160000.caffemodel"
# Read the network into Memory 
net = cv2.dnn.readNetFromCaffe(protoFile, weightsFile)
POSE_PAIRS = [
	[0,1],
	[1,2],
	[1,5],
	[2,3],
	[3,4],
	[5,6],
	[6,7],
	[1,14],
	[14,8],
	[14,11],
	[8,9],
	[9,10],
	[11,12],
	[12,13]
]


def getPosePrediction(image):
	# Specify the input image dimensions
	inWidth = 368
	inHeight = 368

	# Prepare the frame to be fed to the network 
	inpBlob = cv2.dnn.blobFromImage( 
		image, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False) 

	# Set the prepared object as the input blob of the network 
	net.setInput(inpBlob)
	return net.forward()

def overlayPrediction(output, frame):
	H = output.shape[2] 
	W = output.shape[3]
	threshold = 0.1
	height, width, _ = frame.shape
	# Empty list to store the detected keypoints 
	points = []
	for i in range(15): 
		# confidence map of corresponding body's part. 
		probMap = output[0, i, :, :] 

		# Find global maxima of the probMap. 
		minVal, prob, minLoc, point = cv2.minMaxLoc(probMap)

		# Scale the point to fit on the original image 
		x = (width * point[0]) / W 
		y = (height * point[1]) / H 

		if prob > threshold: 
			cv2.circle(frame, (int(x), int(y)), 15, (0, 255, 255), thickness=-1, lineType=cv2.FILLED) 
			cv2.putText(frame, "{}".format(i), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3, lineType=cv2.LINE_AA) 
			points.append((int(x), int(y))) 
		else: 
			points.append(None)
         
	for pair in POSE_PAIRS: 
		partA = pair[0]
		partB = pair[1] 

		if points[partA] and points[partB]: 
			cv2.line(frame, points[partA], points[partB], (0, 255, 0), 3)
	return points

# Do prediction on image obj
def getImagePoints(image):
  imageCopy = image["rawImage"].copy()
  output = getPosePrediction(imageCopy)
  points = overlayPrediction(output, imageCopy)
  image["points"] = points
  image["overlay"] = imageCopy

# View prediction
def showImage(name, img):
  while(True):
    cv2.imshow(name, img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
  cv2.destroyAllWindows()

def calculateAngle(anglePoints):
  firstPoint = anglePoints[0]
  secondPoint = anglePoints[1]
  thirdPoint = anglePoints[2]
  
  ba = firstPoint - secondPoint
  bc = thirdPoint - secondPoint
  cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
  return np.degrees(np.arccos(cosine_angle))

# get the points needed for angles
def calculateImageAngles(image):
	# get points from wrist, shoulder, and hip
  angles = {
      "left_hip_shoulder_wrist": { "pointsIdx": [11,5,7] },
      "right_hip_shoulder_wrist": { "pointsIdx": [8,2,4] },
      "left_shoulder_hip_vertical": { "pointsIdx": [5,11], "extendPoint": 11 },
      "right_shoulder_hip_vertical": { "pointsIdx": [2,8], "extendPoint": 8 }
  }
  points = image["points"]
  for angle in angles:
      pIdxs = angles[angle]["pointsIdx"]
      anglePoints = []
      for pIdx in pIdxs:
        if points[pIdx] != None:
            anglePoints.append(np.array(points[pIdx]))

      if "extendPoint" in angles[angle]:
        extPointIdx = angles[angle]["extendPoint"]
        if points[extPointIdx] != None:
          extPoint = list(points[extPointIdx][:])
          extPoint[1] += 30
          anglePoints.append(np.array(extPoint))
      
      if len(anglePoints) != 3:
        angles[angle]["calculated"] = None
      else:
        result = calculateAngle(anglePoints)
        angles[angle]["calculated"] = result
  image["angles"] = angles

#define angle ranges as optimal, okay, or suboptimal
def getLeastOptimal(arg1, arg2, t):
  if arg1 and arg2:
    if t == "L":
      return max([arg1, arg2])
    elif t == "S":
      return min([arg1, arg2])
    elif arg1:
      return arg1
  return arg2

def defineErgonomics(image):
  leftWS = image["angles"]["left_hip_shoulder_wrist"]["calculated"]
  rightWS = image["angles"]["right_hip_shoulder_wrist"]["calculated"]
  leftHS = image["angles"]["left_shoulder_hip_vertical"]["calculated"]
  rightHS = image["angles"]["right_shoulder_hip_vertical"]["calculated"]

  WSCalc = getLeastOptimal(leftWS, rightWS, "L")
  HSCalc = getLeastOptimal(leftHS, rightHS, "S")

  WS = ""
  if WSCalc == None or WSCalc > 90:
    WS = "okay"
  else:
    WS = "optimal"

  HS = ""
  if HSCalc == None or HSCalc < 120:
    HS = "suboptimal"
  elif 120 <= HSCalc <= 160:
    HS = "okay"
  else:
    HS = "optimal"

    #categorize combination of movements into best, okay, or worst heights
  score = ""
  if HS == "suboptimal":
    score = 0
  elif HS == "optimal" and WS == "optimal":
    score = 2
  else: 
    score = 1
    
  image["ergonomics"] = {"score": score, "WSscore": WS, "HSscore": HS }

# Will return a number corresponding to the bin number
# Returns None if no reading from arduino or no picking detection
lastBins = []
def getArduinoReading(arduino):
  # Take reading from arduino
  data = arduino.readline()
  global lastBins
  if data:
    bins = data.decode('ascii').strip().split(' ')
    if bins != lastBins:
      lastBins = bins
      for i in range(len(bins)):
        b_num = int(bins[i])
        # If any sensor gets activated
        if b_num == 1:
          # Record incident bin number and time
          return i
    return None

# Mock arduino output
looper = 0
def getArduinoReadingMock():
  global looper
  looper += 1
  return looper % 3


# 1. COLLECTION
print("Starting camera and arduino")
arduino = serial.Serial(port='COM7', baudrate=9600, timeout=.1)
cam = cv2.VideoCapture(1)

detections = []
print("Starting data collection")
while len(detections) < 11:
  reading = getArduinoReading(arduino)
  if reading != None:
    result, image = cam.read()
    if result:
      print("reading detected", reading)
      detections.append({
        "bin": reading,
        "time": time.ctime(time.time()),
        "image": {
          "rawImage": image
        }})
    else:
      print("Error capturing image")
    time.sleep(0.05)

cam.release()
arduino.close()

# 2. PROCESSING
# On end, process all pictures and give score to each
for d in detections:
  image = d["image"]
  getImagePoints(image)
  calculateImageAngles(image)
  defineErgonomics(image)
  print(image["ergonomics"])
  showImage("detection overlay", image["overlay"])

# 3. EVALUATION
# Go through all detections and evaluate final outcome
# Give readout of best ordering

# get frequency of bin picking
bins = {}
for d in detections:
  binScore = d["image"]["ergonomics"]["score"]
  bin = d["bin"]
  if not bin in bins:
    bins[bin] = {
      "count": 1,
      "score": binScore
    }
  else:
    bins[bin]["count"] += 1
    bins[bin]["score"] = (bins[bin]["score"] + binScore) / bins[bin]["count"]

pprint.pprint(bins)

lowestBinCount = None
lowestBin = None
# Find bin that's been picked least frequently
for bin, binInfo in bins.items():
  count = binInfo["count"]
  if lowestBinCount == None:
    lowestBinCount = count
    lowestBin = bin
  if count < lowestBinCount:
    lowestBinCount = count
    lowestBin = bin

highestBinCount = None
highestBin = None
# Find bin that's been picked most frequently
for bin, binInfo in bins.items():
  count = binInfo["count"]
  if highestBinCount == None:
    highestBinCount = count
    highestBin = bin
  if count > highestBinCount:
    highestBinCount = count
    highestBin = bin
        
print(lowestBinCount, " Bin picked least frequently: ", lowestBin)
print(highestBinCount, " Bin picked most frequently: ", highestBin)

worstPosture = None
worstPostureBin = None
# Find bin that's been picked least frequently
for bin, ergInfo in bins.items():
  posture = ergInfo["score"]
  if worstPosture == None:
    worstPosture = posture
    worstPostureBin = bin
  if posture < worstPosture:
    worstPosture = posture
    worstPostureBin = bin

bestPosture = None
bestPostureBin = None
# Find bin that's been picked most frequently
for bin, ergInfo in bins.items():
  posture = ergInfo["score"]
  if bestPosture == None:
    bestPosture = posture
    bestPostureBin = bin
  if posture > bestPosture:
    bestPosture = posture
    bestPostureBin = bin
        
print(worstPosture, " Bin in the worst ergonomic position: ", worstPostureBin)
print(bestPosture, " Bin in the best ergonomic position: ", bestPostureBin)


# Get associated ergonomic position and bin number
# Pair positions of best ergonomics with the bin number picked most frequently
if highestBin != bestPostureBin:
  print("Bin ", highestBin, " should be moved to the position of bin ", bestPostureBin)
if lowestBin != worstPostureBin:
  print("Bin ", lowestBin, " should be moved to the position of bin ", worstPostureBin)
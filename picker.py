import cv2
import time
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

print("Starting camera...")
cam = cv2.VideoCapture(1)
print("camera found!")
while(True):
  result, image = cam.read()
  if result:
    t1 = time.time()
    output = getPosePrediction(image)
    print("Predict time: ", time.time() - t1)
    overlayPrediction(output, image)
    cv2.imshow("pose-prediction", image)

  # If captured image is corrupted, moving to else part 
  else:
      print("No image detected. Please! try again")
  if cv2.waitKey(1) & 0xFF == ord('q'):
      cv2.imwrite("sample.png", image)
      break

cam.release()
cv2.destroyAllWindows()
import cv2


def getBallPos(cap, lower_hsv, upper_hsv):
    ret, frame = cap.read()
    if not ret:
        return

    #Blur, convert to HSV
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    #Create a mask for the target color, then clean it up
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    #Find contours in the mask
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    if not contours:
        return

    #Pick the largest contour and fit a circle
    c_largest = max(contours, key=cv2.contourArea)
    (x, y), radius = cv2.minEnclosingCircle(c_largest)

    #Only proceed if the radius is large enough (filter noise)
    if radius > 10:
        #draw the circle and centroid on the frame
        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
        return (x, y)

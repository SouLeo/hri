import numpy as np
import cv2

cap = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, image = cap.read()

    # define the list of boundaries
    boundaries = [
        ([17, 15, 100], [50, 56, 200])
        # ([86, 31, 4], [220, 88, 50])
    ]
    # loop over the boundaries
    for (lower, upper) in boundaries:
        # create NumPy arrays from the
        # boundaries
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")

        # find the colors within the specified
        #  boundaries and apply the mask
        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask=mask)

        imageOut = np.hstack([image, output])

        # Display the resulting frame
        cv2.imshow('RGB',imageOut)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

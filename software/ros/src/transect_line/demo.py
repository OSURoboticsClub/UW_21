import cv2
import numpy as np
import path_algo as pa

while True:
    ret, frame = cap.read()
    cols, rows, _ = frame.shape
    buffer = int(rows * 0.1)
    split_size = int((rows- (2*buffer)) / 3)
    bounds = pa.draw_grid(frame, split_size, 5, buffer)
    command = pa.check_bounds(frame, bounds, 20)
    print(command)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


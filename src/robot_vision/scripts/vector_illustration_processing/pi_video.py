#!/usr/bin/env python

import cv2
import numpy as np

def combine_images(img_1, img_2, padding=5):
    if not pi_video.is_cv_image_valid(img_1):
        return None
    
    if not pi_video.is_cv_image_valid(img_2):
        return None
    
    if not (len(img_1.shape) == len(img_2.shape)):
        return None
    
    im_1_width = 0
    im_1_height = 0
    im_2_width = 0
    im_2_height = 0

    if len(img_1.shape) == 2:
        im_1_height, im_1_width = img_1.shape
        im_2_height, im_2_width = img_2.shape
    elif len(img_1.shape) == 3:
        im_1_height, im_1_width, _ = img_1.shape
        im_2_height, im_2_width, _ = img_2.shape
    
    im_width = 0
    im_height = 0

    if im_1_width > im_1_height:
        im_width = max(im_1_width, im_2_width)
        im_height = im_1_height + padding + im_2_height
    else:
        im_width = im_1_width + padding + im_2_width
        im_height = max(im_1_height, im_2_height)
    
    frame = np.zeros((im_height, im_width, 3), np.uint8)

    if im_1_width > im_1_height:
        frame[:im_1_height, :, :] = img_1
        frame[im_1_height+padding:, :im_2_width, :] = img_2
    else:
        frame[:, :im_1_width, :] = img_1
        frame[:im_2_height, im_1_width+padding:, :] = img_2
    
    try:
        rospy.loginfo("[INFO] frame {} with {}".format(type(frame), frame.shape))
    except:
        pass
    
    return frame

def is_cv_image_valid(cv_image):
	if not isinstance(cv_image, np.ndarray):
		return False
	
	if len(cv_image.shape) == 2:
		n_rows, n_cols = cv_image.shape

		if (n_rows <= 0) or (n_cols <= 0):
			return False
		
		return True

	elif len(cv_image.shape) == 3:

		if cv_image.shape[2] != 3:
			return False

		n_rows, n_cols, depth = cv_image.shape
		if (n_rows <= 0) or (n_cols <= 0):
			return False
		
		return True
	else:
		return False

class Camera(object):
	_started = False
	_cap = None
	_device_id = 0
	_frame = None

	def __init__(self, device_id=1):
		self._device_id = device_id

	def start(self):
		if not self._started:
			self._cap = cv2.VideoCapture(self._device_id)

			if self._cap is None:
				self._started = False
			elif not self._cap.isOpened():
				self._started = False
			else:
				self._started = True

		return self._started

	def stop(self):
		if self._started:
			self._started = False
			self._cap.release()

		return not self._started

	def get_next_frame(self):
		if self._started:
			return self._cap.read()[1]

	def get_prev_frame(self):
		return self._frame


class Viewer(object):
	_started = False
	_title = ""
	_fms = 0

	def __init__(self, fps=30.0, title="frame"):
		self._title = title
		self._fms = int(1000.0/fps);

	def start(self):
		if not self._started:
			self._started = True
			cv2.namedWindow(self._title, cv2.WINDOW_AUTOSIZE) # // c++ will use CV_WINDOW_AUTOSIZE not cv2.WIN...

		return self._started

	def stop(self):
		if self._started:
			self._started = False
			cv2.destroyAllWindows()

		return not self._started

	def show_frame(self, frame):
		if self._started:
			cv2.imshow(self._title, frame)
			return cv2.waitKey(self._fms) & 0xFF

		return -1

class StaticImage(object):
	_started = False
	_frame = None
	_image_name = ""

	def __init__(self, image_name=""):
		self._image_name = image_name

	def start(self):
		if len(self._image_name) > 0:
			_frame = cv2.imread(self._image_name)
			self._started = True

		return self._started

	def stop(self):
		self._started = False
		return not self._started

	def get_next_frame(self):
		return self._frame

	def get_prev_frame(self):
		return self._frame

## // TODO: Complete this - import pi_color, etc ....
# def simplify_image(frame, color_array):
# 	r = b = g = 0

# 	for row in range(len(frame.rows)):
# 		for col in range(len(frame.cols)):
# 			b,g,r = frame[row, col]
# 			color = RGBColor(r, g, b)

# 			color = simplify(color, color_array)
# 			frame[row, col] = color.get_as_bgr()

# 	return frame



def test():
	cam = Camera(0)
	view = Viewer()

	cam.start()
	view.start()

	while True:
		key = view.show_frame(cam.get_next_frame())
		
		if key in (27, 113):
			break
		elif key > 0 and key < 255:
			print ("key", key)

	view.stop()
	cam.stop()

if __name__ == '__main__':
	test()
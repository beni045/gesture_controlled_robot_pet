import os
import cv2
import numpy as np
import argparse
import sys

sys.path.append('../')

from src.pose_decode import decode_pose
from acl_model import Model

heatmap_width = 64
heatmap_height = 64

############################################
############### HAND GESTURE ###############
############################################

class handpose_ModelProcessor:
    
    def __init__(self, acl_resource, params):
        self._acl_resource = acl_resource
        self.params = params
        self._model_width = params['width']
        self._model_height = params['height']

        assert 'model_dir' in params and params['model_dir'] is not None, 'Review your param: model_dir'
        assert os.path.exists(params['model_dir']), "Model directory doesn't exist {}".format(params['model_dir'])
            
        # load model from path, and get model ready for inference
        self.model = Model(acl_resource, params['model_dir'])

    def predict(self, cropped_img, xmin, xmax, ymin, ymax, img_original):
        
        #preprocess image to get 'model_input'
        model_input = self.preprocess(cropped_img)

        # execute model inference
        result = self.model.execute([model_input]) 

        # postprocessing: use the heatmaps (the second output of model) to get the joins and limbs for human body
        # Note: the model has multiple outputs, here we used a simplified method, which only uses heatmap for body joints
        #       and the heatmap has shape of [1,14], each value correspond to the position of one of the 14 joints. 
        #       The value is the index in the 92*92 heatmap (flatten to one dimension)

        # calculate the scale of original image over heatmap, Note: image_original.shape[0] is height
        scale = np.array([int(cropped_img.shape[1] / heatmap_width), int(cropped_img.shape[0]/ heatmap_height)])

        # Canvas is for presenter server, hg_command is for communicating hand gesture command to Raspberry Pi
        canvas, hg_command = decode_pose(result[0][0], scale, xmin, xmax, ymin, ymax, img_original, cropped_img)

        return canvas, hg_command

    def preprocess(self,img_original):
        '''
        preprocessing: resize image to model required size, and normalize value between [0,1]
                       and convert from RGB to BGR
        '''
        preprocessed_img = cv2.resize(img_original, (self._model_width, self._model_height))
        preprocessed_img = cv2.cvtColor(preprocessed_img, cv2.COLOR_RGB2BGR)

        return preprocessed_img

##############################################
############### HAND DETECTION ###############
##############################################

class hand_detection_ModelProcessor:
    
    def __init__(self, acl_resource, params):
        self._acl_resource = acl_resource
        self.params = params

        assert 'model_dir' in params and params['model_dir'] is not None, 'Review your param: model_dir'
        assert os.path.exists(params['model_dir']), "Model directory doesn't exist {}".format(params['model_dir'])
            
        # load model from path, and get model ready for inference
        self.model = Model(acl_resource, params['model_dir'])

    def predict(self, img_original):
        
        #preprocess image to get 'model_input'
        model_input = self.PreProcessing(img_original)

        # execute model inference
        resultList  = self.model.execute([model_input])

	# postprocessing and save inference results
        canvas = self.PostProcessing(img_original, resultList)

        return canvas


    def PreProcessing(self, image):
        # resize image to 300*300, and convert from BGR to RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (300,300))
        # type conversion to UINT8
        image = image.astype(np.uint8).copy()
        return image


    # draw the bounding boxes for all detected hands with confidence greater than a set threshold
    def PostProcessing(self, image, resultList, threshold=0.6):
        num_detections = resultList[0][0].astype(np.int)
        scores = resultList[2]
        boxes = resultList[3]
        bbox_num = 0
        rightmost_x_min = -1 # for selecting the right-most hand (hand gesture model works on right hands)
        crop_img = image

        # For recording bbox coordinates where the hand is in the original frame
        xmin = 0
        xmax = 0
        ymin = 0
        ymax = 0
	
        # loop through all the detections and get the confidence and bbox coordinates
        for i in range(num_detections):
                det_conf = scores[0,i]
                det_ymin = boxes[0,i,0]
                det_xmin = boxes[0,i,1]
                det_ymax = boxes[0,i,2]
                det_xmax = boxes[0,i,3]

                bbox_width = det_xmax - det_xmin
                bbox_height = det_ymax - det_ymin
                # the detection confidence and bbox dimensions must be greater than a minimum value to be a valid detection
                if threshold <= det_conf and 1>=det_conf and bbox_width>0 and bbox_height > 0 and int(round(det_xmin * image.shape[1])) > rightmost_x_min:
                        bbox_num += 1
                        xmin = int(round(det_xmin * image.shape[1]))
                        ymin = int(round(det_ymin * image.shape[0]))
                        xmax = int(round(det_xmax * image.shape[1]))
                        ymax = int(round(det_ymax * image.shape[0]))

                        # Extend bbox coords on all sides for better hand gesture prediction
                        x_extend = int((xmax - xmin) / 8)
                        y_extend = int((ymax - ymin) / 8)

                        # Correct if any bbox coordinates are outside of frame
                        xmin = xmin - x_extend
                        if xmin < 0:
                            xmin = 0

                        xmax = xmax + x_extend
                        if xmax > 1280:
                            xmax = 1280

                        ymin = ymin - y_extend
                        if ymin < 0:
                            ymin = 0

                        ymax = ymax + y_extend
                        if ymax > 720:
                            ymax = 720

                        # record hand's xmin bbox coordinate to determine which hand is right-most in frame
                        rightmost_x_min = xmin

                        crop_img = image[ymin:ymax, xmin:xmax]
                else:
                        continue

        print("detected bbox num:", bbox_num)
        # Bbox coordinates are returned for later drawing a rectangle on the original image for presenter server
        return crop_img, xmin, xmax, ymin, ymax

##############################################
############### FACE DETECTION ###############
##############################################

class face_detection_ModelProcessor:
    
    def __init__(self, acl_resource, params):
        self._acl_resource = acl_resource
        self.params = params

        assert 'model_dir' in params and params['model_dir'] is not None, 'Review your param: model_dir'
        assert os.path.exists(params['model_dir']), "Model directory doesn't exist {}".format(params['model_dir'])
            
        # load model from path, and get model ready for inference
        self.model = Model(acl_resource, params['model_dir'])


    def predict(self, img_original):
        
        #preprocess image to get 'model_input'
        model_input = self.PreProcessing(img_original)

        # execute model inference
        resultList  = self.model.execute([model_input])

	# postprocessing and save inference results
        canvas = self.PostProcessing(img_original, resultList)

        return canvas

    def PreProcessing(self, image):
        # resize image to 300*300 
        image = cv2.resize(image, (300,300))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # type conversion to float 32
        image = image.astype('float32')
        # convert channel format from NHWC to NCHW
        image = np.transpose(image, (2, 0, 1)).copy()
        return image


    # draw the bounding boxes for all detected faces with confidence greater than a set threshold
    def PostProcessing(self, image, resultList, threshold=0.9):
        detections = resultList[1]
        bbox_num = 0
        command = "nothing"
        biggest_face = []
        biggest_face_area = 0
	
        # loop through all the detections and get the confidence and bbox coordinates
        for i in range(detections.shape[1]):
                det_conf = detections[0,i,2]
                det_xmin = detections[0,i,3]
                det_ymin = detections[0,i,4]
                det_xmax = detections[0,i,5]
                det_ymax = detections[0,i,6]
                bbox_width = det_xmax - det_xmin
                bbox_height = det_ymax - det_ymin
                # the detection confidence and bbox dimensions must be greater than a minimum value to be a valid detection
                if threshold <= det_conf and 1>=det_conf and bbox_width>0 and bbox_height > 0:
                        bbox_num += 1
                        xmin = int(round(det_xmin * image.shape[1]))
                        ymin = int(round(det_ymin * image.shape[0]))
                        xmax = int(round(det_xmax * image.shape[1]))
                        ymax = int(round(det_ymax * image.shape[0]))
                        
                        # We only care about the biggest face in the frame (closest to the camera)
                        if (not biggest_face) or biggest_face_area < ( (xmax - xmin) * (ymax - ymin) ):
                            biggest_face = []
                            biggest_face.extend([xmin, xmax, ymin, ymax])
			
                        # For presenter server
                        cv2.rectangle(image,(xmin,ymin),(xmax,ymax),(0,255,0),1)
                else:
                        continue

        # Determine if servo motors should adjust camera angle to bring face closer to the centre of the frame
        if len(biggest_face) != 0:

            if (biggest_face[0] + ((biggest_face[1] - biggest_face[0]) / 2)) < ((1280 / 2) - 150):
                command = "r" # Move servo right

            elif (biggest_face[0] + ((biggest_face[1] - biggest_face[0]) / 2)) > ((1280 / 2) + 150):
                command = "l" # Move servo left

            elif (biggest_face[2] + ((biggest_face[3] - biggest_face[2]) / 2)) < ((720 / 2) - 75):
                command = "u" # Move servo up

            elif (biggest_face[2] + ((biggest_face[3] - biggest_face[2]) / 2)) > ((720 / 2) + 75):
                command = "d" # Move servo down

        return image, command





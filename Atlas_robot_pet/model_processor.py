import os
import cv2
import numpy as np
import argparse
import sys

sys.path.append("../")

from src.body_pose_decode import decode_body_pose
from acl_model import Model
from src.dataloader import letterbox
from src.multitracker import JDETracker
from src import visualization as vis

heatmap_width = 64
heatmap_height = 64
body_heatmap_width = 92
body_heatmap_height = 92

########################################
########### BODY POSE GESTURE ##########
########################################


class body_pose_ModelProcessor:
    def __init__(self, acl_resource, params):
        self._acl_resource = acl_resource
        self.params = params
        self._model_width = params["width"]
        self._model_height = params["height"]

        assert (
            "model_dir" in params and params["model_dir"] is not None
        ), "Review your param: model_dir"
        assert os.path.exists(params["model_dir"]), "Model directory doesn't exist {}".format(
            params["model_dir"]
        )

        # load model from path, and get model ready for inference
        self.model = Model(acl_resource, params["model_dir"])

    def predict(self, img_original, canvas, active):

        # preprocess image to get 'model_input'
        model_input = self.preprocess(img_original)

        # execute model inference
        result = self.model.execute([model_input])

        # postprocessing: use the heatmaps (the second output of model) to get the joins and limbs for human body
        # Note: the model has multiple outputs, here we used a simplified method, which only uses heatmap for body joints
        #       and the heatmap has shape of [1,14], each value correspond to the position of one of the 14 joints.
        #       The value is the index in the 92*92 heatmap (flatten to one dimension)
        heatmaps = result[1]
        # calculate the scale of original image over heatmap, Note: image_original.shape[0] is height
        scale = np.array(
            [
                img_original.shape[1] / body_heatmap_width,
                img_original.shape[0] / body_heatmap_height,
            ]
        )

        canvas, hg_command, box, rotate = decode_body_pose(heatmaps[0], scale, canvas, active)

        return canvas, hg_command, box, rotate

    def preprocess(self, img_original):
        """
        preprocessing: resize image to model required size, and normalize value between [0,1]
        """
        scaled_img_data = cv2.resize(img_original, (self._model_width, self._model_height))
        preprocessed_img = np.asarray(scaled_img_data, dtype=np.float32) / 255.0

        return preprocessed_img


##############################################
############### OBJECT TRACKING ##############
##############################################


class object_tracking_ModelProcessor:
    def __init__(self, acl_resource, params):
        self._acl_resource = acl_resource
        self.params = params

        assert (
            "model_dir" in params and params["model_dir"] is not None
        ), "Review your param: model_dir"
        assert os.path.exists(params["model_dir"]), "Model directory doesn't exist {}".format(
            params["model_dir"]
        )

        # load model from path, and get model ready for inference
        self.model = Model(acl_resource, params["model_dir"])

        self.height = 608
        self.width = 1088

        self.left_count = 0
        self.right_count = 0
        self.motor_count = 2

        # Parameters passed to the Tracker
        args = argparse.ArgumentParser()
        args.conf_thres = 0.35
        args.track_buffer = 30
        args.min_box_area = 100
        args.K = 100
        args.mean = [0.408, 0.447, 0.470]
        args.std = [0.289, 0.274, 0.278]
        args.down_ratio = 4
        args.num_classes = 1
        self.args = args

        # initialize tracker
        self.tracker = JDETracker(args, self.model, frame_rate=30)
        self.frame_id = 0

    def predict(self, img_original, coords, curr_tid):
        # preprocess image to get 'model_input'
        img, img0 = self.PreProcessing(img_original)

        # list of Tracklet; see multitracker.STrack
        online_targets = self.tracker.update(np.array([img]), img0)

        # prepare for drawing, get all bbox and id
        online_tlwhs = []
        online_ids = []
        min_diff = float("inf")
        if curr_tid == -1:
            # If the user's id to track is unkown

            for t in online_targets:
                tlwh = t.tlwh
                tid = t.track_id
                vertical = tlwh[2] / tlwh[3] > 1.6
                if tlwh[2] * tlwh[3] > self.args.min_box_area and not vertical:

                    tlwh[2] = tlwh[0] + tlwh[2]
                    tlwh[3] = tlwh[1] + tlwh[3]

                    # Find a bbox that's closest to the coords where "FOLLOW" command is detected
                    coords = np.array(coords).reshape((4,))
                    diff = np.sum(np.square(np.subtract(tlwh, coords)))
                    tlwh[2] = tlwh[2] - tlwh[0]
                    tlwh[3] = tlwh[3] - tlwh[1]
                    if diff < min_diff:
                        min_diff = diff
                        curr_tid = tid
                        online_tlwhs = [tlwh]
                        online_ids = [tid]

        else:
            # Keep following the id of the tracked user
            online_tlwhs = [t.tlwh for t in online_targets if t.track_id == curr_tid]
            if len(online_tlwhs) > 0:
                online_ids = [curr_tid]

        # draw bbox and id
        canvas = vis.plot_tracking(img0, online_tlwhs, online_ids, frame_id=self.frame_id, fps=1.0)
        self.frame_id += 1

        canvas, command = self.PostProcessing(canvas, online_tlwhs)

        if len(online_tlwhs) > 0:
            # Keep follow
            next_tid = curr_tid
        else:
            # Not found in this frame. Keep trying in next iteration
            next_tid = -1
        return canvas, command, next_tid

    def PreProcessing(self, img0):
        # img:  h w c; 608 1088 3
        # img0: c h w; 3 608 1088

        # Padded resize
        img, _, _, _ = letterbox(img0, height=self.height, width=self.width)

        # Normalize RGB
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img, dtype=np.float32)
        img /= 255.0

        return img, img0

    # Determine the command for camera to center the bounding box of the detected person
    def PostProcessing(self, image, bboxes):
        command = "STOP"

        if len(bboxes) > 0:
            bbox = bboxes[0]

            xmin, ymin, w, h = bbox

            x_center = xmin + w / 2
            y_center = ymin + h / 2

            if x_center < ((1280 / 2) - 150):
                command = "RIGHT"  # Move car right

            elif x_center > ((1280 / 2) + 150):
                command = "LEFT"  # Move servo left

            elif y_center < ((720 / 2) - 75):
                command = "UP"  # Move servo up

            elif y_center > ((720 / 2) + 100):
                command = "DOWN"  # Move servo down

            elif w < 200 or h < 480:
                command = "FORWARDS"

            elif w > 400 or h > 600:
                command = "BACKWARDS"

        return image, command

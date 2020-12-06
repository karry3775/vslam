#!/usr/bin/env python2

"""
Author : kartikprakash3775@gmail.com
Purpose : Write a pipeline to do visual odometry on kitti dataset
"""
import rospy
import cv2
import argparse
import logging
import numpy as np

import pykitti # for parsing kitti data set
# source: https://github.com/utiasSTARS/pykitti

class KittiParser():
    # A class built on top of pykitti
    def __init__(self, base_dir, date, drive):
        self.base_dir = base_dir
        self.date = date
        self.drive = drive

    def setAllKittiData(self):
        self.data = pykitti.raw(self.base_dir, self.date, self.drive, frames=(0, 50, 0))
        point_velo = np.array([0.0, 0.0, 0.0, 1.0])
        self.point_cam0 = self.data.calib.T_cam0_velo.dot(point_velo)
        point_imu = np.array([0, 0, 0, 1])
        self.point_w = [o.T_w_imu.dot(point_imu) for o in self.data.oxts]

    def showImages(self):
        for cam0_image in self.data.cam0:
            cv2.imshow(camo_image)

class VisualOdometry():

    def __init__(self):
        self.video_stream_file = None

    def setVideoStreamFilePath(self, video_stream_file_path):
        self.video_stream_file = video_stream_file_path

    def playVideoStream(self):
        # Code to stream the video stream
        pass

def main():
    # create a parser
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-video-file-path")
    args = parser.parse_args()


    visual_odometry = VisualOdometry()

    # set the file path of the incoming video stream
    visual_odometry.setVideoStreamFilePath(args.input_video_file_path)

    # sanity check
    logging.info("The file for the videostream is: {}".format(visual_odometry.video_stream_file))


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

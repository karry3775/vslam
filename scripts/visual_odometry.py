#!/usr/bin/env python2

"""
Author : kartikprakash3775@gmail.com
Purpose : Write a pipeline to do visual odometry on kitti dataset
"""
import rospy
import cv2
import argparse
import logging


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


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

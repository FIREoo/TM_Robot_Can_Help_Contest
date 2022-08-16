#!/usr/bin/env python3

from __future__ import print_function

import sys

import numpy as np

import cv2

if __name__ == "__main__":
    cv2.namedWindow('show')
    key = 0
    while (key != ord('q')):
        key = cv2.waitKey(10) & 0xFF
        if key != 255:
            print(key)

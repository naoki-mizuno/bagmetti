#!/usr/bin/env python

import rospy
import rostest

import unittest

from bagmetti.rules.rename import RenameRule


class TestRename(unittest.TestCase):
    pass


if __name__ == '__main__':
    rostest.rosrun('bagmetti', 'rename.py', TestRename)

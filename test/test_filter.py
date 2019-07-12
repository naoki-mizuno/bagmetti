#!/usr/bin/env python

from bagmetti.rules.filter import FilterRule

import rospy
import rostest
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

import six
import unittest


class TestFilter(unittest.TestCase):
    def setUp(self):
        self.make_inc_rules()
        self.make_exc_rules()

    @staticmethod
    def all_true(rules, func, args=()):
        """
        Checks whether the result of calling a given function to the given
        rules (with the also given arguments) is all true
        :param rules:
        :type rules: list[FilterRule]
        :param func: name or the reference to the boolean function to call
        :type func: str|func
        :param args: arguments, if any, to pass to the function
        :return: True if all function calls returns True, False otherwise
        :raises: AttributeError
        """
        if isinstance(func, six.string_types):
            func_name = func
        else:
            func_name = func.__name__
        return all([getattr(r, func_name)(*args) for r in rules])

    @staticmethod
    def all_false(rules, func, args=()):
        """
        Pretty much same as TestFilter::all_true, except all False
        :param rules:
        :type rules: list[FilterRule]
        :param func: name or the reference to the boolean function to call
        :type func: str|func
        :param args: arguments, if any, to pass to the function
        :return: True if all function calls returns True, False otherwise
        :raises: AttributeError
        """
        if isinstance(func, six.string_types):
            func_name = func
        else:
            func_name = func.__name__
        return all([not getattr(r, func_name)(*args) for r in rules])

    @staticmethod
    def make_tf(parent, child, time, is_static=False, topic_prefix=''):
        tform = TransformStamped()
        tform.header.stamp = rospy.Time().from_sec(time)
        tform.header.frame_id = parent
        tform.child_frame_id = child

        tf_msg = TFMessage()
        tf_msg.transforms.append(tform)

        topic_name = topic_prefix + '/tf'
        if is_static:
            topic_name += '_static'

        return topic_name, tf_msg, tform.header.stamp

    def assert_all_true(self, rules, func_name, args=()):
        self.assertTrue(self.all_true(rules, func_name, args))

    def assert_all_false(self, rules, func_name, args=()):
        self.assertTrue(self.all_false(rules, func_name, args))

    def make_inc_rules(self):
        inc_rules_desc = """
        tf:
          inc:
            - map -> odom  # 0
            - map, odom    # 1
            - map odom     # 2
        topic:
          inc:
            - inc_topic    # 3
            - foo_topic    # 4
        time:
          inc:
            - 0 -> 10      # 5
            - 20, 30       # 6
            - 40 50        # 7
        """
        self.inc_rules = FilterRule.parse(inc_rules_desc)

    def make_exc_rules(self):
        exc_rules_desc = """
        tf:
          exc:
            - map -> odom_arrow
            - map, odom_comma
            - map odom_space
        topic:
          exc:
            - inc_topic
        time:
          exc:
            - 0 -> 10
            - 20, 30
            - 40 50
        """
        self.exc_rules = FilterRule.parse(exc_rules_desc)

    def test_is_include(self):
        self.assert_all_true(self.inc_rules, FilterRule.is_include)
        self.assert_all_false(self.exc_rules, FilterRule.is_include)

    def test_is_exclude(self):
        self.assert_all_true(self.exc_rules, FilterRule.is_exclude)
        self.assert_all_false(self.inc_rules, FilterRule.is_exclude)

    def test_is_tf(self):
        # Doesn't matter whether it's inc_rules or exc_rules
        r = self.inc_rules
        # tf
        self.assertTrue(r[0].is_tf())
        self.assertTrue(r[1].is_tf())
        self.assertTrue(r[2].is_tf())
        # topic
        self.assertFalse(r[3].is_tf())
        self.assertFalse(r[4].is_tf())
        # time
        self.assertFalse(r[5].is_tf())
        self.assertFalse(r[6].is_tf())
        self.assertFalse(r[7].is_tf())

    def test_is_topic(self):
        # Doesn't matter whether it's inc_rules or exc_rules
        r = self.inc_rules
        # tf
        self.assertFalse(r[0].is_topic())
        self.assertFalse(r[1].is_topic())
        self.assertFalse(r[2].is_topic())
        # topic
        self.assertTrue(r[3].is_topic())
        self.assertTrue(r[4].is_topic())
        # time
        self.assertFalse(r[5].is_topic())
        self.assertFalse(r[6].is_topic())
        self.assertFalse(r[7].is_topic())

    def test_is_time(self):
        # Doesn't matter whether it's inc_rules or exc_rules
        r = self.inc_rules
        # tf
        self.assertFalse(r[0].is_time())
        self.assertFalse(r[1].is_time())
        self.assertFalse(r[2].is_time())
        # topic
        self.assertFalse(r[3].is_time())
        self.assertFalse(r[4].is_time())
        # time
        self.assertTrue(r[5].is_time())
        self.assertTrue(r[6].is_time())
        self.assertTrue(r[7].is_time())

    def test_match_tf(self):
        data = self.make_tf('map', 'odom', 10, topic_prefix='/foo')
        # tf
        self.assertTrue(self.inc_rules[0].msg_match(*data))
        self.assertTrue(self.inc_rules[1].msg_match(*data))
        self.assertTrue(self.inc_rules[2].msg_match(*data))
        # topic
        self.assertFalse(self.inc_rules[3].msg_match(*data))
        self.assertFalse(self.inc_rules[4].msg_match(*data))
        # We only compare TF and topic because time is different

    def test_match_topic(self):
        # Topic name must match
        data = ('/inc_topic', Imu(), rospy.Time(45))
        # tf
        self.assertFalse(self.inc_rules[0].msg_match(*data))
        self.assertFalse(self.inc_rules[1].msg_match(*data))
        self.assertFalse(self.inc_rules[2].msg_match(*data))
        # topic
        self.assertTrue(self.inc_rules[3].msg_match(*data))
        self.assertFalse(self.inc_rules[4].msg_match(*data))

    def test_match_time(self):
        data = self.make_tf('map', 'odom_arrow', 10, topic_prefix='/foo')
        self.assertTrue(self.inc_rules[5].msg_match(*data))
        self.assertFalse(self.inc_rules[6].msg_match(*data))
        self.assertFalse(self.inc_rules[7].msg_match(*data))


if __name__ == '__main__':
    rostest.rosrun('bagmetti', 'test_filter.py', TestFilter)

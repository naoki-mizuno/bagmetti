#!/usr/bin/env python2

from bagmetti.rules import RenameRule
from rosbag import Bag

import sys


def usage():
    return 'rename.py <in bag> <out bag> <conf file>'


def modify_topic(topic, rules):
    for r in rules:
        topic, changed = r.rename(topic)
        if changed:
            # Make sure whatever we changed starts with a slash
            return '/' + topic.lstrip('/')
    return topic


def modify_tf(msg, rules):
    for r in rules:
        changed_p = False
        changed_c = False
        for tf in msg.transforms:
            tf.header.frame_id, changed_p = r.rename(tf.header.frame_id)
            tf.child_frame_id, changed_c = r.rename(tf.child_frame_id)
        if changed_p or changed_c:
            return msg
    return msg


def process_bag(bag_in_fn, bag_out_fn, conf_file_fn):
    bag_in = Bag(bag_in_fn)
    bag_out = Bag(bag_out_fn, 'w')
    topic_rules, tf_rules = RenameRule.parse(conf_file_fn)

    for topic, msg, t in bag_in.read_messages():
        if topic == '/tf' or topic == '/tf_static':
            new_topic = topic
            new_msg = modify_tf(msg, tf_rules)
        else:
            new_topic = modify_topic(topic, topic_rules)
            new_msg = msg
        bag_out.write(new_topic, new_msg, t)

    bag_in.close()
    bag_out.close()


if __name__ == '__main__':
    if len(sys.argv) < 4:
        sys.stderr.write(usage() + '\n')
        sys.exit(1)
    process_bag(*sys.argv[1:4])
else:
    sys.stderr.write('rename.py needs to be run as a script\n')
    sys.exit(1)
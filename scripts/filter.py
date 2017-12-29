#!/usr/bin/env python2

from bagmetti import Rule
from rosbag import Bag
from rospy import Time
import sys


if __name__ != '__main__':
    sys.stderr.write('bagmetti needs to be run as a script')
    sys.exit(1)


def usage():
    return 'filter.py <in bag> <out bag> <conf file>\n'


def get_begin_end(rules, t_start, t_end):
    t_min = None
    t_max = None
    for r in rules:
        if r.is_time() and r.is_include():
            if t_min is None or r.token_from < t_min:
                t_min = r.token_from
            if t_max is None or r.token_to > t_max:
                t_max = r.token_to

    t_min = t_start if t_min is None else t_min + t_start
    t_max = t_end if t_max is None else t_max + t_start
    return Time(t_min), Time(t_max)


def get_topics(rules, all_topics=None):
    topics = set()
    topics_to_include = set(all_topics) if all_topics is not None else None
    for r in rules:
        if r.is_topic() and r.is_include():
            topics.add(r.token_from)
        elif r.is_topic() and r.is_exclude():
            if r.token_from not in topics_to_include:
                continue
            topics_to_include.remove(r.token_from)
        if r.is_tf():
            topics.add('/tf')
    if len(topics - {'/tf'}) == 0:
        topics = topics_to_include
    return topics


if len(sys.argv) < 4:
    sys.stderr.write(usage())
    sys.exit(1)

bag_in_fn, bag_out_fn, conf_file_fn = sys.argv[1:4]

bag_in = Bag(bag_in_fn)
bag_out = Bag(bag_out_fn, 'w')

# Read in rules
rules = []
time_rules = []
with open(conf_file_fn) as conf_file:
    for line in conf_file:
        rule = Rule.parse(line)
        if rule is None:
            continue
        if rule.is_time():
            time_rules.append(rule)
        else:
            rules.append(rule)
Rule.set_begin_time(bag_in.get_start_time())

# Find start and end times so that we can speed up the reading as much as possible
t_start, t_end = get_begin_end(time_rules, bag_in.get_start_time(), bag_in.get_end_time())
# Same thing for topics
all_topics = bag_in.get_type_and_topic_info().topics.keys()
topics = get_topics(rules, all_topics)

for topic, msg, t in bag_in.read_messages(topics=topics, start_time=t_start, end_time=t_end):
    # Normal rules
    inclusion_ok = False
    exclusion_ok = True
    for r in rules:
        if r.is_include() and r.is_ok_with(topic, msg, t):
            inclusion_ok = True
        elif r.is_exclude() and not r.is_ok_with(topic, msg, t):
            exclusion_ok = False
    # Time rules
    time_ok = True
    for r in time_rules:
        if not r.is_ok_with(topic, msg, t):
            time_ok = False
    # Write to file
    if (inclusion_ok or exclusion_ok) and time_ok:
        bag_out.write(topic, msg, t)

bag_out.close()

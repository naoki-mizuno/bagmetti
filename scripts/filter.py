#!/usr/bin/env python

from bagmetti.rules import FilterRule
from rosbag import Bag
from rospy import Time
import sys


def usage():
    return 'filter.py <in bag> <out bag> <conf file>'


def get_begin_end(time_rules, bag_file):
    # Find the min/max relative time (0 at the beginning of the bag file)
    t_min = None
    t_max = None
    for r in time_rules:
        if r.is_time() and r.is_include():
            if t_min is None or r.token_from < t_min:
                t_min = r.token_from
            if t_max is None or r.token_to > t_max:
                t_max = r.token_to

    t_start, t_end = bag_file.get_start_time(), bag_file.get_end_time()
    t_min = t_start if t_min is None else t_min + t_start
    t_max = t_end if t_max is None else t_max + t_start
    return Time(t_min), Time(t_max)


def get_topics(rules, bag_topics):
    # Read all topics from bag files
    if FilterRule.DEFAULT_ENFORCEMENT_TOPIC == FilterRule.INCLUDE:
        return None

    topics = set()
    sample_set = set(bag_topics)

    if FilterRule.DEFAULT_ENFORCEMENT_TF == FilterRule.INCLUDE:
        topics = topics.union(FilterRule.TF_TOPICS)

    for r in rules:
        if r.is_topic():
            if r.is_include():
                topics.add(r.token_from)
            elif r.is_exclude() and r.token_from in sample_set:
                sample_set.remove(r.token_from)
        if r.is_tf():
            topics = topics.union(FilterRule.TF_TOPICS)

    if len(topics - FilterRule.TF_TOPICS) == 0:
        topics = None
    return topics


def read_rules(conf_file_fn):
    # Read in rules
    include_rules = []
    exclude_rules = []
    time_rules = []

    all_rules = FilterRule.parse(conf_file_fn)
    for r in all_rules:
        if r.is_time():
            time_rules.append(r)
        elif r.is_include():
            include_rules.append(r)
        elif r.is_exclude():
            exclude_rules.append(r)
        else:
            raise 'Unexpected rule found: {0}'.format(str(r))

    return include_rules, exclude_rules, time_rules


def process_bag(bag_in_fn, bag_out_fn, conf_file_fn):
    bag_in = Bag(bag_in_fn)
    bag_out = Bag(bag_out_fn, 'w')

    include_rules, exclude_rules, time_rules = read_rules(conf_file_fn)
    topic_rules = include_rules + exclude_rules

    # Set the time UNIX time at the start of the bag file
    for r in topic_rules:
        r.set_begin_time(bag_in.get_start_time())
    for r in time_rules:
        r.set_begin_time(bag_in.get_start_time())

    # Find start and end times that are actually required
    t_start, t_end = get_begin_end(time_rules, bag_in)
    # Find topics that are actually required
    bag_topics = bag_in.get_type_and_topic_info().topics.keys()
    topics = get_topics(topic_rules, bag_topics)

    messages = bag_in.read_messages(topics=topics,
                                    start_time=t_start,
                                    end_time=t_end,
                                    return_connection_header=True)
    for topic, msg, t, conn_header in messages:
        # Check default enforcement for this message
        if FilterRule.is_tf_topic(topic):
            default = FilterRule.DEFAULT_ENFORCEMENT_TF
        else:
            default = FilterRule.DEFAULT_ENFORCEMENT_TOPIC

        # When default is to include, only check whether the exclusion
        # rules are satisfied, and if all of them are ok, write it out
        if default == FilterRule.INCLUDE:
            # Check exclusions
            ok = True
            for r in exclude_rules:
                if r.msg_match(topic, msg, t):
                    ok = False
        # When default is to exclude, check if the message matches any
        # of the inclusion rules and write it out if it does
        else:
            # Check inclusions
            ok = False
            for r in include_rules:
                if r.msg_match(topic, msg, t):
                    ok = True

        # Time rules
        time_ok = True
        for r in time_rules:
            if not r.is_ok_with(topic, msg, t):
                time_ok = False

        # Write to file
        if ok and time_ok:
            bag_out.write(topic, msg, t, connection_header=conn_header)

    bag_out.close()


if __name__ == '__main__':
    if len(sys.argv) < 4:
        sys.stderr.write(usage() + '\n')
        sys.exit(1)
    process_bag(*sys.argv[1:4])
else:
    sys.stderr.write('filter.py needs to be run as a script\n')
    sys.exit(1)

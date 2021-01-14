#!/usr/bin/env python

from rospy import Time
import re
import yaml
import six


class FilterRule:
    TF_TOPICS = {'/tf', '/tf_static'}

    TOKEN_SEP = [',', '->', ' ']
    INCLUDE = '+'
    EXCLUDE = '-'
    DEFAULT_ENFORCEMENT_TOPIC = INCLUDE
    DEFAULT_ENFORCEMENT_TF = INCLUDE
    DEFAULT_ENFORCEMENT_TIME = INCLUDE
    RULE_TYPE_TIME = 'time'
    RULE_TYPE_TOPIC = 'topic'
    RULE_TYPE_TF = 'tf'
    RULE_TYPES = (RULE_TYPE_TIME, RULE_TYPE_TOPIC, RULE_TYPE_TF)
    DEFAULT_RULE_TYPE = RULE_TYPE_TOPIC

    # Time at the beginning of the file
    begin_time = 0

    def __init__(self):
        self.rule_type = None
        self.enforcement = None
        self.token_from = None
        self.token_to = None
        self.begin_time = 0

    def __str__(self):
        to_ret = '{0} ({1})'.format(self.rule_type, self.enforcement)
        if self.token_to is not None:
            to_ret += ': {0} -> {1}'.format(self.token_from, self.token_to)
        else:
            to_ret += ': {0}'.format(self.token_from)
        return to_ret

    def set_begin_time(self, t):
        if t is Time:
            self.begin_time = t.to_sec()
        else:
            self.begin_time = float(t)

    def is_include(self):
        return self.enforcement == FilterRule.INCLUDE

    def is_exclude(self):
        return self.enforcement == FilterRule.EXCLUDE

    def is_topic(self):
        return self.rule_type == FilterRule.RULE_TYPE_TOPIC

    def is_tf(self):
        return self.rule_type == FilterRule.RULE_TYPE_TF

    def is_time(self):
        return self.rule_type == FilterRule.RULE_TYPE_TIME

    def is_ok_with(self, topic, msg, t):
        is_match = self.msg_match(topic, msg, t)
        return is_match if self.is_include() else not is_match

    def msg_match(self, topic, msg, t):
        # Subtract offset
        t = t.to_sec() - self.begin_time
        if self.is_topic():
            return self.__msg_match_topic__(topic, msg, t)
        elif self.is_tf():
            return self.__msg_match_tf__(topic, msg, t)
        elif self.is_time():
            return self.__msg_match_time__(topic, msg, t)

    def __msg_match_topic__(self, topic, msg, t):
        return self.token_from == topic

    def __msg_match_tf__(self, topic, msg, t):
        if not FilterRule.is_tf_topic(topic):
            return False

        is_match = False
        # TODO: Only filter transforms that are bad
        for transform in msg.transforms:
            parent_frame = transform.header.frame_id.lstrip('/')
            child_frame = transform.child_frame_id.lstrip('/')

            if self.token_from is None and self.token_to == child_frame:
                is_match = True
            elif self.token_to is None and self.token_from == parent_frame:
                is_match = True
            elif self.token_from == parent_frame and self.token_to == child_frame:
                is_match = True

        return is_match

    def __msg_match_time__(self, topic, msg, t):
        is_in_range = True
        # TODO: Look at header.stamp for tf?
        if self.token_from is not None and t < self.token_from:
            is_in_range = False
        elif self.token_to is not None and t > self.token_to:
            is_in_range = False

        return is_in_range

    @staticmethod
    def is_tf_topic(topic):
        """
        Checks whether the given topic name is a TF topic name

        This is done by checking the suffix of the topic name
        :param topic:
        :return: True if it is a TF topic, False otherwise
        :rtype: bool
        """
        return any([topic.endswith(name) for name in FilterRule.TF_TOPICS])

    @staticmethod
    def change_default(msg_type, enforcement):
        if msg_type == FilterRule.RULE_TYPE_TF:
            FilterRule.DEFAULT_ENFORCEMENT_TF = enforcement
        elif msg_type == FilterRule.RULE_TYPE_TOPIC:
            FilterRule.DEFAULT_ENFORCEMENT_TOPIC = enforcement
        elif msg_type == FilterRule.RULE_TYPE_TIME:
            FilterRule.DEFAULT_ENFORCEMENT_TIME = enforcement

    @staticmethod
    def make_rules(doc, msg_type, enforcement):
        # Make sure what we get is a list
        # tf:
        #   include:
        #     # These lines
        #     - foo -> bar
        #     - baz -> qux
        lines = doc[msg_type][enforcement]
        # One-line strings
        # time:
        #   include: -> 200
        if isinstance(lines, six.string_types):
            lines = [lines]

        to_ret = []

        for line in lines:
            rule = FilterRule()
            rule.enforcement = enforcement
            if msg_type == FilterRule.RULE_TYPE_TF:
                rule = FilterRule.__parse_tf__(rule, line)
            elif msg_type == FilterRule.RULE_TYPE_TOPIC:
                rule = FilterRule.__parse_topic__(rule, line)
            elif msg_type == FilterRule.RULE_TYPE_TIME:
                rule = FilterRule.__parse_time__(rule, line)
            to_ret.append(rule)

        return to_ret

    @staticmethod
    def parse(yaml_fn):
        try:
            fh = open(yaml_fn)
            doc = yaml.safe_load(fh)
        except IOError:
            # Not a file name, probably a YAML string
            doc = yaml.safe_load(yaml_fn)

        rules = []
        for msg_type in doc.keys():
            if msg_type not in FilterRule.RULE_TYPES:
                raise 'Invalid message type {0}'.format(msg_type)

            if 'default' in doc[msg_type]:
                FilterRule.change_default(msg_type, doc[msg_type]['default'])
                continue
            else:
                enforcement = None
                FilterRule.__normalize_keys__(doc[msg_type])
                if FilterRule.INCLUDE in doc[msg_type]:
                    if FilterRule.EXCLUDE not in doc[msg_type]:
                        FilterRule.change_default(msg_type, FilterRule.EXCLUDE)
                    enforcement = FilterRule.INCLUDE
                elif FilterRule.EXCLUDE in doc[msg_type]:
                    if FilterRule.INCLUDE not in doc[msg_type]:
                        FilterRule.change_default(msg_type, FilterRule.INCLUDE)
                    enforcement = FilterRule.EXCLUDE

                rules += FilterRule.make_rules(doc, msg_type, enforcement)

        return rules

    @staticmethod
    def __normalize_keys__(msg):
        for key in msg.keys():
            # Non-'+' inclusions
            if key.startswith('i'):
                msg[FilterRule.INCLUDE] = msg.pop(key)
            # Non-'-' exclusions
            elif key.startswith('e'):
                msg[FilterRule.EXCLUDE] = msg.pop(key)

    @staticmethod
    def __parse_time__(rule, line):
        rule.rule_type = FilterRule.RULE_TYPE_TIME
        # Seconds from start of file
        tokens = FilterRule.__token_sep__(line)
        rule.token_from = float(tokens[0]) if tokens[0] is not None else None
        rule.token_to = float(tokens[1]) if tokens[1] is not None else None
        return rule

    @staticmethod
    def __parse_tf__(rule, line):
        rule.rule_type = FilterRule.RULE_TYPE_TF
        tokens = FilterRule.__token_sep__(line)
        rule.token_from, rule.token_to = tokens[0:2]
        return rule

    @staticmethod
    def __parse_topic__(rule, line):
        rule.rule_type = FilterRule.RULE_TYPE_TOPIC
        rule.token_from = line.lstrip('/')
        # Then add it
        rule.token_from = '/' + rule.token_from
        return rule

    @staticmethod
    def __token_sep__(line):
        pat = '\s*(?:{0})\s*'.format('|'.join(FilterRule.TOKEN_SEP))
        tokens = re.split(pat, line.strip())
        # Put only non-empty elements in the list
        return list(map(lambda t: None if t == '' else t.strip(), tokens))

#!/usr/bin/env python2

from rospy import Time
import re


class FilterRule:
    TOKEN_SEP = [',', '->', ' ']
    ENFORCEMENT_INCLUDE = '+'
    ENFORCEMENT_EXCLUDE = '-'
    DEFAULT_ENFORCEMENT = ENFORCEMENT_INCLUDE
    DEFAULT_ENFORCEMENT_TOPIC = DEFAULT_ENFORCEMENT
    DEFAULT_ENFORCEMENT_TF = DEFAULT_ENFORCEMENT
    DEFAULT_ENFORCEMENT_TIME = DEFAULT_ENFORCEMENT
    RULE_TYPE_TIME = 'time'
    RULE_TYPE_TOPIC = 'topic'
    RULE_TYPE_TF = 'tf'
    DEFAULT_RULE_TYPE = RULE_TYPE_TOPIC

    TIME = 0
    TF = 1
    TOPIC = 2

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
        return self.enforcement == FilterRule.ENFORCEMENT_INCLUDE

    def is_exclude(self):
        return self.enforcement == FilterRule.ENFORCEMENT_EXCLUDE

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
        if topic.lstrip('/') != 'tf':
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
    def change_default(line):
        tokens = re.split('\s*[:, ]\s*', line.lower())

        for token in tokens[1:]:
            if token == '+':
                FilterRule.DEFAULT_ENFORCEMENT = FilterRule.ENFORCEMENT_INCLUDE
            elif token == '-':
                FilterRule.DEFAULT_ENFORCEMENT = FilterRule.ENFORCEMENT_EXCLUDE
            else:
                # '+tf' -> ['+', 'tf']
                enforcement = token[0]
                what = token.strip('+-')
                if what == 'rule':
                    FilterRule.DEFAULT_ENFORCEMENT = enforcement
                elif what == FilterRule.RULE_TYPE_TF:
                    FilterRule.DEFAULT_ENFORCEMENT_TF = enforcement
                elif what == FilterRule.RULE_TYPE_TOPIC:
                    FilterRule.DEFAULT_ENFORCEMENT_TOPIC = enforcement
                elif what == FilterRule.RULE_TYPE_TIME:
                    FilterRule.DEFAULT_ENFORCEMENT_TIME = enforcement
        return None

    @staticmethod
    def parse(line):
        # Comment line
        if line.startswith('#'):
            return None
        if line.startswith('default'):
            FilterRule.change_default(line)
            return None

        to_ret = FilterRule()

        # + or -?
        if line.startswith(FilterRule.ENFORCEMENT_INCLUDE):
            to_ret.enforcement = FilterRule.ENFORCEMENT_INCLUDE
        elif line.startswith(FilterRule.ENFORCEMENT_EXCLUDE) or line.startswith('!'):
            to_ret.enforcement = FilterRule.ENFORCEMENT_EXCLUDE
        else:
            to_ret.enforcement = FilterRule.DEFAULT_ENFORCEMENT
        # Remove +, -, and ! if any
        line = line.lstrip(FilterRule.ENFORCEMENT_INCLUDE + FilterRule.ENFORCEMENT_EXCLUDE + '!')

        if ':' in line:
            rule_type, rest = line.strip().split(':', 1)
        else:
            # Topic
            rule_type = FilterRule.DEFAULT_RULE_TYPE
            rest = line.strip()
        rule_type = rule_type.strip()

        # Parse according to rule type
        to_ret.rule_type = rule_type.lower()
        if to_ret.rule_type == FilterRule.RULE_TYPE_TIME:
            return FilterRule.__parse_time__(to_ret, rest)
        elif to_ret.rule_type == FilterRule.RULE_TYPE_TF:
            return FilterRule.__parse_tf__(to_ret, rest)
        elif to_ret.rule_type == FilterRule.RULE_TYPE_TOPIC:
            return FilterRule.__parse_topic__(to_ret, rest)

        raise ValueError('Invalid rule type {0} passed to parse'.format(rule_type))

    @staticmethod
    def __parse_time__(rule, line):
        # Seconds from start of file
        tokens = FilterRule.__token_sep__(line)
        if len(tokens) < 2:
            rule.token_from = float(tokens[0])
        else:
            rule.token_from = float(tokens[0]) if tokens[0] is not None else None
            rule.token_to = float(tokens[1]) if tokens[1] is not None else None
        return rule

    @staticmethod
    def __parse_tf__(rule, line):
        tokens = FilterRule.__token_sep__(line)
        if len(tokens) < 2:
            rule.token_from = tokens[0]
        else:
            rule.token_from, rule.token_to = tokens[0:2]
        return rule

    @staticmethod
    def __parse_topic__(rule, line):
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

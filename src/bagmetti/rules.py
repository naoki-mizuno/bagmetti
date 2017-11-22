#!/usr/bin/env python2

from rospy import Time


class Rule:
    TOKEN_SEP = [',', '->', ' ']
    ENFORCEMENT_INCLUDE = '+'
    ENFORCEMENT_EXCLUDE = '-'
    DEFAULT_ENFORCEMENT = ENFORCEMENT_INCLUDE
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

    def __str__(self):
        to_ret = '{0} ({1})'.format(self.rule_type, self.enforcement)
        if self.token_to is not None:
            to_ret += ': {0} -> {1}'.format(self.token_from, self.token_to)
        else:
            to_ret += ': {0}'.format(self.token_from)
        return to_ret

    def is_include(self):
        return self.enforcement == Rule.ENFORCEMENT_INCLUDE

    def is_exclude(self):
        return self.enforcement == Rule.ENFORCEMENT_EXCLUDE

    def is_topic(self):
        return self.rule_type == Rule.RULE_TYPE_TOPIC

    def is_tf(self):
        return self.rule_type == Rule.RULE_TYPE_TF

    def is_time(self):
        return self.rule_type == Rule.RULE_TYPE_TIME

    def is_ok_with(self, topic, msg, t):
        # Subtract offset
        t = t.to_sec() - Rule.begin_time
        if self.is_topic():
            return self.__ok_topic__(topic, msg, t)
        elif self.is_tf():
            return self.__ok_tf__(topic, msg, t)
        elif self.is_time():
            return self.__ok_time__(topic, msg, t)

    def __ok_topic__(self, topic, msg, t):
        match = (self.token_from == topic)
        if self.is_include():
            return match
        else:
            return not match

    def __ok_tf__(self, topic, msg, t):
        if topic.lstrip('/') != 'tf':
            if self.is_include():
                return False
            else:
                return True

        all_match = True
        any_match = False
        # TODO: Only filter transforms that are bad
        for transform in msg.transforms:
            parent_frame = transform.header.frame_id
            child_frame = transform.child_frame_id

            if self.token_from is None and self.token_to == child_frame:
                any_match = True
            elif self.token_to is None and self.token_from == parent_frame:
                any_match = True
            elif self.token_from == parent_frame and self.token_to == child_frame:
                any_match = True

            if self.token_from is not None and self.token_from != parent_frame:
                all_match = False
            elif self.token_to is not None and self.token_to != child_frame:
                all_match = False

        if self.is_include():
            return all_match
        else:
            return not any_match

    def __ok_time__(self, topic, msg, t):
        is_in_range = True
        # TODO: Look at header.stamp for tf?
        if self.token_from is not None and t < self.token_from:
            is_in_range = False
        elif self.token_to is not None and t > self.token_to:
            is_in_range = False

        if self.is_include():
            return is_in_range
        else:
            return not is_in_range

    @staticmethod
    def set_begin_time(t):
        if t is Time:
            Rule.begin_time = t.to_sec()
        else:
            Rule.begin_time = float(t)

    @staticmethod
    def parse(line):
        # Comment line
        if line.startswith('#'):
            return None

        to_ret = Rule()

        # + or -?
        if line.startswith(Rule.ENFORCEMENT_INCLUDE):
            to_ret.enforcement = Rule.ENFORCEMENT_INCLUDE
        elif line.startswith(Rule.ENFORCEMENT_EXCLUDE) or line.startswith('!'):
            to_ret.enforcement = Rule.ENFORCEMENT_EXCLUDE
        else:
            to_ret.enforcement = Rule.DEFAULT_ENFORCEMENT
        # Remove +, -, and ! if any
        line = line.lstrip(Rule.ENFORCEMENT_INCLUDE + Rule.ENFORCEMENT_EXCLUDE + '!')

        if ':' in line:
            rule_type, rest = line.strip().split(':', 1)
        else:
            # Topic
            rule_type = Rule.DEFAULT_RULE_TYPE
            rest = line.strip()
        rule_type = rule_type.strip()

        if rule_type == 'default':
            what = rest.strip().lower()
            if what == 'plus' or what == 'include':
                what = Rule.ENFORCEMENT_INCLUDE
            elif what == 'minus' or what == 'exclude':
                what = Rule.ENFORCEMENT_EXCLUDE

            if what != '+' and what != '-':
                raise ValueError('Expected +, -, plus, minus for default')

            Rule.DEFAULT_ENFORCEMENT = what
            return None

        # Parse according to rule type
        to_ret.rule_type = rule_type.lower()
        if to_ret.rule_type == Rule.RULE_TYPE_TIME:
            return Rule.__parse_time__(to_ret, rest)
        elif to_ret.rule_type == Rule.RULE_TYPE_TF:
            return Rule.__parse_tf__(to_ret, rest)
        elif to_ret.rule_type == Rule.RULE_TYPE_TOPIC:
            return Rule.__parse_topic__(to_ret, rest)

        raise ValueError('Invalid rule type {0} passed to parse'.format(rule_type))

    @staticmethod
    def __parse_time__(rule, line):
        # Seconds from start of file
        rule.token_from, rule.token_to = map(float, Rule.__token_sep__(line)[0:2])
        return rule

    @staticmethod
    def __parse_tf__(rule, line):
        rule.token_from, rule.token_to = Rule.__token_sep__(line)[0:2]
        return rule

    @staticmethod
    def __parse_topic__(rule, line):
        # Strip the leading / from topic names
        rule.token_from = line.lstrip('/')
        # Then add it
        rule.token_from = '/' + rule.token_from
        return rule

    @staticmethod
    def __token_sep__(line):
        for sep in Rule.TOKEN_SEP:
            line = line.replace(sep, '\0')
        tokens = line.split('\0')
        # Put only non-empty elements in the list
        return list(filter(lambda t: t != '', map(lambda t: t.strip(), tokens)))

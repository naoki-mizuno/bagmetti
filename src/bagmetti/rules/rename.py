#!/usr/bin/env python

import re
import yaml


class RenameRule:
    def __init__(self, pat_from, pat_to):
        self.pat_from = pat_from
        self.pat_to = pat_to
        self.__pat__ = RenameRule.__glob_to_re__(self.pat_from)
        self.__fmt__ = RenameRule.__new_fmt__(self.pat_to)

    def rename(self, old_str):
        """
        Creates the new string from the old string, replacing according to
        the rule defined in this instance.
        :param old_str: the string to be replaced
        :return: new string after replacement, true if old_str != new_str
        """
        m = re.search(self.__pat__, old_str)
        # No match found
        if m is None:
            return old_str, False

        try:
            new_str_partial = self.__fmt__.format(*m.groups())
            new_str = re.sub(self.__pat__, new_str_partial, old_str)
        except IndexError:
            # Probably too many placeholders in "to"
            # For example:
            #   from: foo/*
            #   to:   bar/*/*/*/*/*
            raise ValueError('Invalid placeholder index')
        return new_str, old_str != new_str

    def __str__(self):
        s = ''
        s += 'From : {0}\n'.format(self.pat_from)
        s += 'To   : {0}'.format(RenameRule.__new_fmt__(self.pat_to))
        return s

    @staticmethod
    def parse(yaml_fn):
        """
        Parses the YAML file and gets the rules
        :param yaml_fn: name of the YAML file to parse
        :return: lists of topic rules and tf rules
        """
        with open(yaml_fn) as fh:
            doc = yaml.load(fh)
        tf_rules = []
        topic_rules = []

        if 'topic' in doc:
            topic_rules = RenameRule.__make_rules__(doc['topic'])
        if 'tf' in doc:
            tf_rules = RenameRule.__make_rules__(doc['tf'])
        return topic_rules, tf_rules

    @staticmethod
    def __make_rules__(doc):
        """
        Creates RenameRule instances from a YAML doc.
        :param doc: YAML dict or list-of-dict
        :return: the list of RenameRule instances
        """
        yaml_list = []
        if type(doc) is not list:
            # If doc has only one dict element
            yaml_list.append(doc)
        else:
            yaml_list = doc
        rules = []
        for yaml_rule in yaml_list:
            f = yaml_rule['from']
            t = yaml_rule['to']
            if f is None:
                raise ValueError('from cannot be empty')
            if t is None:
                t = ''
            rules.append(RenameRule(f, t))
        return rules

    @staticmethod
    def __glob_to_re__(glob_pat):
        """
        Converts glob symbols such as * and + to regex
        :param glob_pat: string containing the glob pattern
        :return: compiled regex pattern
        """
        re_pat = glob_pat
        re_pat = re_pat.replace('*', '(.*)')
        re_pat = re_pat.replace('+', '(.+)')
        return re.compile(re_pat)

    @staticmethod
    def __new_fmt__(glob_pat):
        """
        Creates a format string for the "from" pattern, replacing all
        asterisks with {X} where X corresponds to the Xth asterisk, with the
        first asterisk in the glob_pat being 0th (i.e. is 0-based).
        Thus, foo/*/*/{3} becomes foo/{0}/{1}/{3}.
        :param glob_pat: the pattern containing asterisks
        :return: pattern with asterisks replaced with {X}
        """
        pat = glob_pat
        for i in range(glob_pat.count('*')):
            pat = pat.replace('*', '{' + str(i) + '}', 1)
        return pat

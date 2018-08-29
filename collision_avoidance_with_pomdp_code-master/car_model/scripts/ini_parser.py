#!/usr/bin/python

import ConfigParser


class IniParser:
    def __init__(self):
        self.config = ConfigParser.ConfigParser()
        self.config.read("/home/albert/ias_ros/src/car_model/config/config.ini")

    def get_config(self):
        return self.config

    def get(self, section):
        dict1 = {}
        options = self.config.options(section)
        for option in options:
            try:
                dict1[option] = self.config.get(section, option)
                if dict1[option] == -1:
                    print("skip: %s" % option)
            except IOError:
                print("exception on %s!" % option)
                dict1[option] = None
        return dict1

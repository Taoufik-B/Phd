import configparser
import os
import logging

def load_configuration(path):
    configParser = configparser.ConfigParser()
    configParser.read(path)
    return configParser

if __name__ == '__main__':
    logging.info("Loading the configuration file")
    config = load_configuration('./configs/basic.cfg')
    logging.info("Printing configuration file")
    print(dict(config))
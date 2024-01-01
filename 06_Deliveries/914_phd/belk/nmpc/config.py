import configparser
import os

def load_configuration(path):
    configParser = configparser.ConfigParser()
    configParser.read(path)
    return configParser


if __name__ == 'main':
    config = load_configuration('./configs/test.cfg')
    print(config)
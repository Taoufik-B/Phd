import configparser
import os
import logging

def load_configuration(path):
    configParser = configparser.ConfigParser()
    configParser.read(path)
    return configParser


def main():
    logging.info("Loading the configuration file")
    config = load_configuration('./configs/test.cfg')
    logging.info("Printing configuration file")
    print(config)

if __name__ == 'main':
    main()
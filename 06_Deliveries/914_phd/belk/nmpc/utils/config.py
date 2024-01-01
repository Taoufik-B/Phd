import configparser
import yaml
import os
import logging

def load_configuration(path):
    configParser = configparser.ConfigParser(converters={'list': lambda x: [i.strip() for i in x.split(',')]})
    configParser.read(path)
    return configParser

def get_list(config, key, value, type=float):
    return list(map(type, config.getlist(key,value)))

def load_yaml_config(path):
    with open(path) as f:
        return yaml.load(f, Loader=yaml.FullLoader)

if __name__ == '__main__':
    logging.info("Loading the configuration file")
    config = load_configuration('./configs/basic.cfg')
    yaml_config = load_yaml_config('./configs/basic.yaml')
    logging.info("Printing configuration file")
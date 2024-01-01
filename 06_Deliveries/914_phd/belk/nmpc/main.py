# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

import logging
import argparse
from utils.config im


def main():
    argparser = argparse.ArgumentParser(
        description='NMPC Simulation')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '-c','--config',
        metavar='cfg',
        default='./configs/basic.yaml',
        help='configuration file in yaml form under ./configs')


    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(asctime)s %(levelname)-8s: %(message)s', level=log_level, datefmt='%Y-%m-%d %H:%M:%S')

    # logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        config = 
        ## prepare the environement
        ## run the environement
        ## store the results
        ## plot the results if required
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')



if __name__ == '__main__':
    main()
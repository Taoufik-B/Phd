# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

import logging
import argparse


def main():
    argparser = argparse.ArgumentParser(
        description='NMPC Simulation')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--config',
        metavar='c',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(asctime)s %(levelname)-8s: %(message)s', level=log_level, datefmt='%Y-%m-%d %H:%M:%S')

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:
        print(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')



if __name__ == '__main__':
    main()
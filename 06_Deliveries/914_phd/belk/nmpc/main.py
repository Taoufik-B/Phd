# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

import logging
import argparse
from utils.config import load_yaml_config
from utils.trajectory import ReferenceTrajectory
from controllers.nmpc_controller import NMPCController
from models.kinematics import VehicleKinematicModel
from utils.visualization import simulate


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
        ## prepare the environement
        config = load_yaml_config(args.config)
        print(config)
        trajectory = ReferenceTrajectory(**config['NMPC.environment']['trajectory'])
        vehicle_model = VehicleKinematicModel(**config['NMPC.externals']['vehicle'])
        nmpc = NMPCController(model=vehicle_model,trajectory=trajectory, **config['NMPC.internals'], bounds=config['NMPC.externals']['bounds'])
        t=[]
        reference = trajectory.get_reference()
        mpciter=0
        t0 = 0
        ## run the environement
        while (True):
            u_opt  = nmpc.compute_controls(mpciter, config['NMPC.externals']['u_ref'])
            nmpc.run_step(u_opt)
            distance = np.linalg.norm(trajectory.xs[0:2]-nmpc.x0[0:2])
            distance_p = np.linalg.norm(trajectory.xs[0:2]-nmpc.p_history[0:2,0,mpciter])
            logging.info(f"Distance iteration {mpciter} : {distance}")
            mpciter += 1     
            t.append(t0)
            if distance < 0.5:
                break
            if distance_p <0.5:
                break
            
        ## store the results
        ## plot the results if required
        simulate(trajectory.path, nmpc.p_history, nmpc.x_history, nmpc.u_opt_history, t, nmpc.dt, nmpc.N,reference, False)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')



if __name__ == '__main__':
    main()
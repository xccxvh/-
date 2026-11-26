"""Run one Nano experiment using the protocol codec shipped for the G3519 firmware."""
import argparse
import math
from pathlib import Path

from host_experiment import BoardController, run_auto, run_remote
from servo_scan import make_plan


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--serial', required=True)
    sub = ap.add_subparsers(dest='experiment')
    auto = sub.add_parser('auto')
    auto.add_argument('--target-cm', type=float, default=0.)
    auto.add_argument('--kp', type=float, required=True)
    auto.add_argument('--kd', type=float, required=True)
    auto.add_argument('--seconds', type=float, required=True)
    scan = sub.add_parser('scan')
    scan.add_argument('--min-ticks', type=int, required=True, help='field-confirmed safe limit')
    scan.add_argument('--max-ticks', type=int, required=True, help='field-confirmed safe limit')
    scan.add_argument('--neutral', type=int, required=True)
    scan.add_argument('--step', type=int, required=True)
    scan.add_argument('--hold-ms', type=int, required=True)
    scan.add_argument('--direction', choices=('up','down'), default='up')
    args = ap.parse_args()
    if args.experiment == 'auto':
        if not all(math.isfinite(v) for v in (args.target_cm,args.kp,args.kd,args.seconds)) or args.seconds <= 0:
            ap.error('auto needs finite target, gains and positive duration')
    elif args.experiment == 'scan':
        if not args.min_ticks <= args.neutral <= args.max_ticks or args.hold_ms < 200:
            ap.error('neutral must be inside confirmed stroke, hold-ms >= 200')
        plan = make_plan('ticks', args.min_ticks, args.max_ticks, args.step, args.hold_ms, args.direction)
    else:
        ap.error('choose auto or scan')

    # The firmware team supplies the 0x03 payload and 0x83 result definitions.
    # Fail before opening any devices until a matching Nano codec is available.
    try:
        import g3519_command_codec as codec
    except ModuleNotFoundError as exc:
        if exc.name != 'g3519_command_codec':
            raise
        ap.exit(2,'G3519 0x03/0x83 protocol adapter unavailable; no device opened.\n')

    def experiment(port, log, cancelled):
        if args.experiment == 'auto':
            board = BoardController(port.execute, port.send)
            run_auto(board, port.next_status, log, args.target_cm, args.kp, args.kd,
                     args.seconds, cancelled=cancelled)
        else:
            board = BoardController(port.execute, port.send, (args.min_ticks,args.max_ticks))
            run_remote(board, port.next_debug, log, plan, args.hold_ms, args.neutral,
                       cancelled=cancelled)

    from main import main as vision
    vision_args = ['--config', str(Path(__file__).with_name('config_static_fast_c.json')),
                   '--csv', '--serial', args.serial, '--label','host_'+args.experiment]
    if args.experiment == 'auto':
        vision_args.append('--uart-enable-valid')
    vision(argv=vision_args, experiment=experiment, codec=codec)


if __name__ == '__main__':
    main()

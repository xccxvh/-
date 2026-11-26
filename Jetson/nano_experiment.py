"""Run one Nano experiment using the protocol codec shipped for the G3519 firmware."""
import argparse
import json
import math
from pathlib import Path

from host_experiment import BoardController, run_auto, run_remote
from servo_scan import make_plan, board_profile, Target, ticks_to_degrees


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
    scan.add_argument('--neutral', type=int, help='optional cross-check of board neutral')
    route = scan.add_mutually_exclusive_group(required=True)
    route.add_argument('--step', type=int)
    route.add_argument('--platforms', type=int, nargs='+', help='ordered absolute PWM plateaus')
    scan.add_argument('--record', action='store_true', help='save video and raw plateau samples')
    scan.add_argument('--hold-ms', type=int, required=True)
    scan.add_argument('--direction', choices=('up','down'), default='up')
    adaptive = sub.add_parser('adaptive')
    adaptive.add_argument('--min-ticks', type=int, required=True)
    adaptive.add_argument('--max-ticks', type=int, required=True)
    adaptive.add_argument('--seconds', type=float, default=15.)
    for mode in (adaptive, scan):
        mode.add_argument('--board-profile', help='PWM stroke for the flashed G3519 build')
    adaptive.add_argument('--probe-sign', type=int, choices=(-1, 1), help='probe one PWM direction then STOP')
    adaptive.add_argument('--polarity', type=int, choices=(-1, 1), default=-1,
                          help='sign of measured x acceleration under increasing PWM')
    identify = adaptive.add_mutually_exclusive_group()
    identify.add_argument('--identify-platforms', nargs='+', type=int, help='ordered absolute PWM targets with ball')
    identify.add_argument('--identify-offset', type=int, help='signed PWM offset from live neutral')
    adaptive.add_argument('--brake-ticks', type=int, help='verified opposite-side brake target')
    adaptive.add_argument('--model', help='directional acceleration gains and braking parameters JSON')
    adaptive.add_argument('--record', action='store_true', help='save timestamped experiment video')
    args = ap.parse_args()
    if args.experiment in ('auto', 'adaptive'):
        if not math.isfinite(args.seconds) or args.seconds <= 0:
            ap.error('duration must be finite and positive')
        if args.experiment == 'auto' and not all(math.isfinite(v) for v in (args.target_cm, args.kp, args.kd)):
            ap.error('auto needs finite target and gains')
    elif args.experiment == 'scan':
        if args.hold_ms < 200:
            ap.error('hold-ms must be >= 200')
    else:
        ap.error('choose auto, scan or adaptive')

    # The firmware team supplies the 0x03 payload and 0x83 result definitions.
    # Fail before opening any devices until a matching Nano codec is available.
    try:
        import g3519_command_codec as codec
    except ModuleNotFoundError as exc:
        if exc.name != 'g3519_command_codec':
            raise
        ap.exit(2,'G3519 0x03/0x83 protocol adapter unavailable; no device opened.\n')

    def experiment(port, log, cancelled):
        if args.experiment in ('scan', 'adaptive'):
            profile = (board_profile(args.board_profile) if args.experiment == 'scan' and args.board_profile
                       else port.actuator_profile())
            neutral = profile['neutral_ticks']
            limits = (profile['min_ticks'], profile['max_ticks'])
            if not limits[0] <= args.min_ticks <= neutral <= args.max_ticks <= limits[1]:
                raise ValueError('experiment stroke exceeds board limits or excludes neutral')
            if args.experiment == 'adaptive' and not args.min_ticks < neutral < args.max_ticks:
                raise ValueError('adaptive limits must straddle neutral')
        if args.experiment == 'auto':
            board = BoardController(port.execute, port.send)
            run_auto(board, port.next_status, log, args.target_cm, args.kp, args.kd,
                     args.seconds, cancelled=cancelled)
        elif args.experiment == 'adaptive':
            from adaptive_return import run
            port.adaptive = True
            board = BoardController(port.execute, port.send, (args.min_ticks,args.max_ticks))
            model = None
            if args.model:
                with open(args.model) as stream:
                    model = json.load(stream)
            run(board, port, log, args.seconds, cancelled, profile, args.probe_sign, args.polarity,
                args.identify_offset, model, args.identify_platforms, args.brake_ticks)
        else:
            board = BoardController(port.execute, port.send, (args.min_ticks,args.max_ticks))
            if args.neutral not in (None, neutral):
                raise ValueError('neutral differs from board')
            if args.platforms:
                # 完整序列在使能前核对，避免执行到一半才发现下一平台越界。
                if any(not args.min_ticks <= ticks <= args.max_ticks for ticks in args.platforms):
                    raise ValueError('plateau outside field-confirmed stroke')
                plan = [Target(ticks_to_degrees(t), t) for t in args.platforms]
            else:
                plan = make_plan('ticks', args.min_ticks, args.max_ticks, args.step, args.hold_ms,
                                 args.direction, limits=limits)
            run_remote(board, port.next_debug, log, plan, args.hold_ms, neutral,
                       cancelled=cancelled)

    from main import main as vision
    vision_args = ['--config', str(Path(__file__).with_name('config_static_fast_c.json')),
                   '--csv', '--serial', args.serial, '--label','host_'+args.experiment]
    if args.experiment in ('auto', 'adaptive'):
        vision_args.append('--uart-enable-valid')
    if args.experiment == 'scan' and args.record:
        vision_args.append('--ruler-samples')
    if args.experiment in ('adaptive', 'scan') and args.record:
        vision_args.extend(['--record', '--record-fps', '8', '--record-scale', '.5'])
    vision(argv=vision_args, experiment=experiment, codec=codec)


if __name__ == '__main__':
    main()

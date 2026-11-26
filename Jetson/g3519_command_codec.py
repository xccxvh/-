"""Control payloads for G3519 vision_link.h at 03991c7."""
import struct

COMMANDS = ('SET_MODE', 'START_AUTO', 'STOP', 'SET_TARGET', 'SET_PD', 'START_REMOTE')
MODES = {'MANUAL': 0, 'UART': 2, 'REMOTE': 5}
RESULTS = ('OK', 'BAD_MODE', 'VISION_INVALID', 'PARAM_RANGE', 'FAULT', 'REJECTED')


def encode(name, args):
    mode = MODES[args[0]] if name == 'SET_MODE' else 0
    a = args[0] if name in ('SET_TARGET', 'SET_PD') else 0.
    b = args[1] if name == 'SET_PD' else 0.
    return struct.pack('<BBff', COMMANDS.index(name) + 1, mode, a, b)


def ack(header_seq, payload):
    seq, op, result, mode, state = struct.unpack('<HBBBB', payload)
    name = COMMANDS[op-1] if 1 <= op <= len(COMMANDS) else 'UNKNOWN'
    reason = RESULTS[result] if result < len(RESULTS) else str(result)
    return seq, name, result == 0, '%s mode=%d state=%d' % (reason, mode, state)

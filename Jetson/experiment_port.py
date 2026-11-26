"""Single-owner UART for Scheme C feedback and board commands (Python 3.6)."""
import queue
import struct
import threading
import time

from position_feedback import crc16, LinkJournal


class ExperimentPort:
    """Wrap the open SerialFeedback. Pass this object to both vision and controller.

    codec.encode(name, args) supplies the official command payload.
    codec.ack(header_seq, payload) returns (command_seq, name, accepted, reason).
    No command payload or result code is inferred from the legacy firmware.
    """
    def __init__(self, port, codec):
        self.port = port
        self.codec = codec
        self.tx_lock = threading.Lock()
        self.next_command = int(time.monotonic()*1000) & 65535
        self.acks = queue.Queue()
        self.status = queue.Queue()
        self.debug = queue.Queue()
        self.buffer = bytearray()
        self.crc_errors = 0

    @property
    def fd(self):
        return self.port.fd

    def send(self, packet):
        with self.tx_lock:
            self.port.send(packet)

    def execute(self, name, *args):
        payload = self.codec.encode(name, args)
        seq = self.next_command
        self.next_command = (seq + 1) & 65535
        body = struct.pack('<BBBH', 1, 3, len(payload), seq) + payload
        started = time.monotonic()
        self.send(b'\xa5\x5a' + body + struct.pack('<H', crc16(body)))
        end = started + .5
        while True:
            received, ack_seq, command, accepted, reason = self._get(self.acks, end, name + ' ACK')
            if received >= started and (ack_seq, command) == (seq, name):
                print('G3519 ACK seq=%d %s %s' % (seq, name, reason), flush=True)
                if not accepted:
                    raise RuntimeError('%s refused: %s' % (name, reason))
                return

    @staticmethod
    def _get(q, end, kind):
        left = end - time.monotonic()
        if left > 0:
            try:
                return q.get(timeout=left)
            except queue.Empty:
                pass
        raise TimeoutError(kind + ' timeout')

    def next_status(self, end):
        return self._get(self.status, end, '0x81')

    def next_debug(self, end):
        return self._get(self.debug, end, '0x82')

    def split_rx(self, data, received):
        """Called only by LinkJournal's RX thread. Forward complete 0x81 to its CSV."""
        buf = self.buffer
        buf.extend(data)
        legacy = bytearray()
        while True:
            i = buf.find(b'\xa5\x5a')
            if i < 0:
                del buf[:len(buf) - int(buf.endswith(b'\xa5'))]
                break
            del buf[:i]
            if len(buf) < 7:
                break
            version, kind, n, seq = struct.unpack_from('<BBBH', buf, 2)
            if version != 1 or kind not in (0x81, 0x82, 0x83) or n > 32:
                del buf[:1]
                continue
            if len(buf) < 9+n:
                break
            frame = bytes(buf[:9+n])
            if crc16(frame[2:-2]) != struct.unpack_from('<H', frame, 7+n)[0]:
                self.crc_errors += 1
                del buf[:1]
                continue
            del buf[:9+n]
            if kind == 0x83:
                self.acks.put((received,) + tuple(self.codec.ack(seq, frame[7:-2])))
            elif kind == 0x82 and n == 12:
                ack, ms, target, output, enabled, result = struct.unpack_from('<HIHHBB', frame, 7)
                self.debug.put(dict(received=received, command_seq=ack, board_timestamp=ms,
                                    target_pwm_ticks=target, output_pwm_ticks=output,
                                    remote_enabled=enabled, result=result))
            elif kind == 0x81 and n == 32:
                values = struct.unpack_from('<I5fHBBf', frame, 7)
                row = dict(zip(LinkJournal.fields[11:], values))
                flags = row['telemetry_flags']
                row.update(received=received, board_vision_valid=flags & 1,
                           board_auto_enabled=(flags >> 1) & 1, board_fault=(flags >> 2) & 1)
                self.status.put(row)
                legacy.extend(frame)
        return legacy

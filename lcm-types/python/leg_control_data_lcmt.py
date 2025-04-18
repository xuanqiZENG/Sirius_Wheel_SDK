"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

class leg_control_data_lcmt(object):

    __slots__ = ["q", "qd", "tauIq", "tauEst", "uq", "ud", "quat", "gyro", "accelerometer"]

    __typenames__ = ["float", "float", "float", "float", "float", "float", "float", "float", "float"]

    __dimensions__ = [[16], [16], [16], [16], [16], [16], [4], [3], [3]]

    def __init__(self):
        self.q = [ 0.0 for dim0 in range(16) ]
        """ LCM Type: float[16] """
        self.qd = [ 0.0 for dim0 in range(16) ]
        """ LCM Type: float[16] """
        self.tauIq = [ 0.0 for dim0 in range(16) ]
        """ LCM Type: float[16] """
        self.tauEst = [ 0.0 for dim0 in range(16) ]
        """ LCM Type: float[16] """
        self.uq = [ 0.0 for dim0 in range(16) ]
        """ LCM Type: float[16] """
        self.ud = [ 0.0 for dim0 in range(16) ]
        """ LCM Type: float[16] """
        self.quat = [ 0.0 for dim0 in range(4) ]
        """ LCM Type: float[4] """
        self.gyro = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: float[3] """
        self.accelerometer = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: float[3] """

    def encode(self):
        buf = BytesIO()
        buf.write(leg_control_data_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>16f', *self.q[:16]))
        buf.write(struct.pack('>16f', *self.qd[:16]))
        buf.write(struct.pack('>16f', *self.tauIq[:16]))
        buf.write(struct.pack('>16f', *self.tauEst[:16]))
        buf.write(struct.pack('>16f', *self.uq[:16]))
        buf.write(struct.pack('>16f', *self.ud[:16]))
        buf.write(struct.pack('>4f', *self.quat[:4]))
        buf.write(struct.pack('>3f', *self.gyro[:3]))
        buf.write(struct.pack('>3f', *self.accelerometer[:3]))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != leg_control_data_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return leg_control_data_lcmt._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = leg_control_data_lcmt()
        self.q = struct.unpack('>16f', buf.read(64))
        self.qd = struct.unpack('>16f', buf.read(64))
        self.tauIq = struct.unpack('>16f', buf.read(64))
        self.tauEst = struct.unpack('>16f', buf.read(64))
        self.uq = struct.unpack('>16f', buf.read(64))
        self.ud = struct.unpack('>16f', buf.read(64))
        self.quat = struct.unpack('>4f', buf.read(16))
        self.gyro = struct.unpack('>3f', buf.read(12))
        self.accelerometer = struct.unpack('>3f', buf.read(12))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if leg_control_data_lcmt in parents: return 0
        tmphash = (0x10b0e97b5e31a98a) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if leg_control_data_lcmt._packed_fingerprint is None:
            leg_control_data_lcmt._packed_fingerprint = struct.pack(">Q", leg_control_data_lcmt._get_hash_recursive([]))
        return leg_control_data_lcmt._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", leg_control_data_lcmt._get_packed_fingerprint())[0]


"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import markerlcm.marker_t

class marker_list_t(object):
    __slots__ = ["timestamp", "n", "markers"]

    def __init__(self):
        self.timestamp = 0
        self.n = 0
        self.markers = []

    def encode(self):
        buf = BytesIO()
        buf.write(marker_list_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qi", self.timestamp, self.n))
        for i0 in range(self.n):
            assert self.markers[i0]._get_packed_fingerprint() == markerlcm.marker_t._get_packed_fingerprint()
            self.markers[i0]._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != marker_list_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return marker_list_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = marker_list_t()
        self.timestamp, self.n = struct.unpack(">qi", buf.read(12))
        self.markers = []
        for i0 in range(self.n):
            self.markers.append(markerlcm.marker_t._decode_one(buf))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if marker_list_t in parents: return 0
        newparents = parents + [marker_list_t]
        tmphash = (0xa33efb33aed7090a+ markerlcm.marker_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if marker_list_t._packed_fingerprint is None:
            marker_list_t._packed_fingerprint = struct.pack(">Q", marker_list_t._get_hash_recursive([]))
        return marker_list_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

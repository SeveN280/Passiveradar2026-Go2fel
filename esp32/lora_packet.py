import os
import sys

from cryptography.hazmat.primitives.ciphers.aead import AESGCM

HDR = b"RD\x01"
NONCE, TAG, MAX_TEXT = 12, 16, 96


def seal(key: bytes, text: str, nonce: bytes = None) -> bytes:
    data = text.encode()
    if not 0 < len(data) <= MAX_TEXT:
        raise ValueError("text must be 1..96 bytes")
    nonce = nonce or os.urandom(NONCE)
    return HDR + nonce + AESGCM(key).encrypt(nonce, data, HDR)


def open_packet(key: bytes, pkt: bytes) -> str:
    if len(pkt) < len(HDR) + NONCE + 1 + TAG or pkt[:3] != HDR:
        raise ValueError("not a radar packet")
    nonce, body = pkt[3:3 + NONCE], pkt[3 + NONCE:]
    return AESGCM(key).decrypt(nonce, body, HDR).decode()


def _selftest():
    key = bytes(range(16))
    line = "PR,1762113600,DRONE?,1402,38.5,30.7,-"
    pkt = seal(key, line, nonce=bytes(12))
    assert open_packet(key, pkt) == line
    print(f"round trip OK, {len(line)} text bytes -> {len(pkt)} packet bytes")
    bad = bytearray(pkt); bad[-20] ^= 1
    for name, k, p in (("tampered", key, bytes(bad)), ("wrong key", bytes(16), pkt)):
        try:
            open_packet(k, p)
            raise SystemExit(f"FAIL: {name} packet accepted")
        except Exception as e:
            if isinstance(e, SystemExit):
                raise
            print(f"{name} packet rejected OK")
    print("test vector (key 00..0f, nonce zeros):", pkt.hex())


if __name__ == "__main__":
    _selftest() if len(sys.argv) == 1 else print(open_packet(bytes.fromhex(sys.argv[1]),
                                                             bytes.fromhex(sys.argv[2])))

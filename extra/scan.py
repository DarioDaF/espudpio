
import socket
from typing import Iterable

def readIpPort(s: str) -> tuple[str, int]:
    ip, port = s.split(':')
    return ip, int(port)

MAX_UDP_PACKER = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(3.0)
def sendRecv(msg: bytes, target: tuple[str, int], ignore_extra = False):
    global s, MAX_UDP_PACKER
    s.sendto(msg, target)
    while True:
        data, sender = s.recvfrom(MAX_UDP_PACKER)
        if sender == target:
            return data.decode().strip()
        elif not ignore_extra:
            raise KeyError(f'Received a spurious packet from {sender}')
def sendRecvMulti(msg: bytes, targets: Iterable[tuple[str, int]], ignore_extra = False):
    global s, MAX_UDP_PACKER
    missing = set(targets)
    for target in missing:
        s.sendto(msg, target)
    while len(missing) > 0:
        data, sender = s.recvfrom(MAX_UDP_PACKER)
        if sender in missing:
            missing.remove(sender)
            yield data.decode().strip(), sender
        elif not ignore_extra:
            raise KeyError(f'Received a spurious packet from {sender}')
def sendRecvMultiNoErr(msg: bytes, targets: Iterable[tuple[str, int]]):
    global s, MAX_UDP_PACKER
    missing = set(targets)
    for target in missing:
        s.sendto(msg, target)
    try:
        while len(missing) > 0:
            data, sender = s.recvfrom(MAX_UDP_PACKER)
            if sender in missing:
                missing.remove(sender)
                yield data.decode().strip(), sender
    except TimeoutError:
        yield from ((None, target) for target in missing)

DEFAULT_PORT = 5000

import argparse
ap = argparse.ArgumentParser()
ap.add_argument('--server', type=str, required=True)
ap.add_argument('--port', type=int, default=DEFAULT_PORT)
args = ap.parse_args()

SERVER = (args.server, args.port)

try:
    outputs = sendRecv(b'?O', SERVER)
except TimeoutError:
    outputs = 'Outputs: TIMEOUT'
print(outputs)

try:
    clients = sendRecv(b'?C', SERVER)
except TimeoutError:
    print('Could not read connected clients')
    clients = ''
conns = [ SERVER, *(readIpPort(client.strip()) for client in clients.split('\n')[1:]) ]

for data, sender in sendRecvMultiNoErr(b'?W', conns):
    if data is None:
        data = 'TIMEOUT'
    print(f'{sender} -> {data}')

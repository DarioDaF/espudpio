
from time import sleep 
import socket
from typing import Iterable

def readIpPort(s: str) -> tuple[str, int]:
    ip, port = s.split(':')
    return ip, int(port)

MAX_UDP_PACKER = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(4.0)
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
    print(outputs)
    clients = sendRecv(b'?C', SERVER)
except TimeoutError:
    print('ERROR: Server timed out')
    exit(1)

conns = [ SERVER, *(readIpPort(client.strip()) for client in clients.split('\n')[1:]) ]

rssis = { sender: data for data, sender in sendRecvMultiNoErr(b'?W', conns) }
infos = { sender: data for data, sender in sendRecvMultiNoErr(b'?I', conns) }
targets = {}

for sender, info in infos.items():
    target = None
    if info is not None:
        target = '#'
        for info_line in info.split('\n'):
            info_line = info_line.strip()
            TARGET_TEXT = 'Local Id: '
            if info_line.startswith(TARGET_TEXT):
                target = int(info_line[len(TARGET_TEXT):])
                break
    targets[sender] = target

for sender in conns:
    rssi = rssis[sender]
    target = targets[sender]
    if rssi is None:
        rssi = 'TIMEOUT'
    if target is None:
        target = 'TIMEOUT'
    print(f'{sender} [{target}] -> {rssi}')


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

DEFAULT_PORT = 5000

import argparse
ap = argparse.ArgumentParser()
ap.add_argument('command', type=str)
ap.add_argument('--ip', type=str, required=True)
ap.add_argument('--port', type=int, default=DEFAULT_PORT)
args = ap.parse_args()

TARGET = (args.ip, args.port)

try:
    print(sendRecv(b'?' + args.command.encode(), TARGET))
except TimeoutError:
    print('ERROR: TIMEOUT')
    exit(1)

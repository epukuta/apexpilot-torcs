"""
pyclient.py - UDP client that connects the ApexPilot driver to TORCS.

Usage:
    1. Start TORCS, enable the scr_server robot and load the Corkscrew
       track (the server listens on UDP port 3001 by default).
    2. Run:  python3 pyclient.py [--host localhost] [--port 3001]

Protocol: the client identifies itself with an init message carrying the
rangefinder angles, then answers every server state message with one
control message until the server sends ***shutdown*** or ***restart***.
"""

import argparse
import socket
import sys

from msgParser import msgParser
from driver import Driver, SENSOR_ANGLES

CLIENT_ID = 'SCR'


def build_init():
    angles = ' '.join(str(a) for a in SENSOR_ANGLES)
    return '%s(init %s)' % (CLIENT_ID, angles)


def main():
    ap = argparse.ArgumentParser(description='ApexPilot TORCS client')
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=3001)
    ap.add_argument('--max-steps', type=int, default=0,
                    help='0 = run until the server ends the race')
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    parser = msgParser()
    driver = Driver()
    driver.parser = parser

    # --- identification phase: retry until the server answers --------
    state = None
    for _ in range(100):
        sock.sendto(build_init().encode(), (args.host, args.port))
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue
        msg = data.decode('utf-8', 'ignore')
        if '***identified***' in msg:
            print('Identified by the TORCS server.')
            break
        state = parser.parse(msg)  # a state arrived directly; use it

    # --- racing loop ---------------------------------------------------
    steps = 0
    while True:
        if state is None:
            try:
                data, _ = sock.recvfrom(4096)
            except socket.timeout:
                continue
            msg = data.decode('utf-8', 'ignore')
            if '***shutdown***' in msg:
                print('Race finished (server shutdown).')
                break
            if '***restart***' in msg:
                print('Race restarted by the server.')
                state = None
                continue
            state = parser.parse(msg)

        actions = driver.drive(state)
        sock.sendto(parser.stringify(actions).encode(),
                    (args.host, args.port))
        state = None
        steps += 1
        if args.max_steps and steps >= args.max_steps:
            print('Step budget reached, disconnecting.')
            break

    sock.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())

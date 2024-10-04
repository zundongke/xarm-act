"""Network utilities.

References:
- https://stackoverflow.com/a/56950161
"""

import multiprocessing.dummy as mp
import socket


def connect(hostname, port=22):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socket.setdefaulttimeout(0.25)
    result = sock.connect_ex((hostname, port))
    sock.close()
    # print(hostname, port, result)
    return result == 0


def scan_xarm_ip_address(njobs=10):
    """Scan for the IP address of the xArm control box."""
    print("Scanning IP addresses... (can take a while)")
    offset = 2  # Start scanning from 192.168.1.2
    pool = mp.Pool(njobs)
    results = pool.map(connect, ["192.168.1." + str(i) for i in range(offset, 255)])
    pool.close()
    pool.join()
    connected_addresses = []
    for i in range(len(results)):
        if results[i]:
            addr = "192.168.1." + str(i + offset)
            print("Device found at: ", addr)
            connected_addresses.append(addr)
    return connected_addresses


def main():
    scan_xarm_ip_address()


if __name__ == "__main__":
    main()

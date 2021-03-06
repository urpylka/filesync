#!/usr/bin/env python2.7

from os import environ

try:
    from pyftpdlib.authorizers import DummyAuthorizer
    from pyftpdlib.handlers import FTPHandler
    from pyftpdlib.servers import FTPServer
except Exception as ex:
    print("Error! Please execute: 'pip install pyftpdlib' or 'sudo apt install pyftpdlib'")
    exit(1)

def main():
    """
    FTP_USER="test-1"
    FTP_PASS="passwd"
    FTP_ROOT="temp"
    FTP_PORT="21"
    """
    # Instantiate a dummy authorizer for managing 'virtual' users
    authorizer = DummyAuthorizer()

    # Define a new user having full r/w permissions and a read-only
    # anonymous user
    # authorizer.add_user(environ['FTP_USER'],
    #                     environ['FTP_PASS'],
    #                     environ['FTP_ROOT'], perm='elradfmwM')
    authorizer.add_user("test-1", "passwd", "temp", perm='elradfmwM')

    # Instantiate FTP handler class
    handler = FTPHandler
    handler.authorizer = authorizer

    # Define a customized banner (string returned when client connects)
    handler.banner = "pyftpdlib based ftpd ready."

    # Specify a masquerade address and the range of ports to use for
    # passive connections.  Decomment in case you're behind a NAT.
    # handler.masquerade_address = '151.25.42.11'
    # handler.passive_ports = range(60000, 65535)

    # Instantiate FTP server class and listen on 0.0.0.0:2121
    #address = ('', int(environ['FTP_PORT']))
    address = ('', 21)

    server = FTPServer(address, handler)

    # set a limit for connections
    server.max_cons = 256
    server.max_cons_per_ip = 5

    # start ftp server
    server.serve_forever()

if __name__ == '__main__':
    main()

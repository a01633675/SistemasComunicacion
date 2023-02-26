#myssn_client.py
# Based on the client from https://pymotw.com/3/socket/tcp.html

import myssn
import sys

#SERVER_ADDRESS = 'localhost'
SERVER_ADDRESS = "192.168.0.102"
indx = 0
try:
    client = myssn.client_connect(SERVER_ADDRESS)
    while True:
        try:
            print('TEST: {}'.format(indx))
            # Send data
            string_msg = input('$ ')
            if ':q' in string_msg:
                break

            print('======================== Tx DATA ========================')

            bytes_msg = string_msg.encode()
            myssn.send(client, bytes_msg)

            print('======================== Rx DATA ========================')

            # Look for the response
            dat = myssn.recv(client)
            if dat:
                print('DATA: {!r}'.format(dat))
            else:
                print('INFO: closing socket')
                myssn.close(sock)
                break
            indx = indx + 1
        except: 
            print('INFO: Client Exception') 
            break
        print('\r\n')       
except ConnectionRefusedError:
    print('INFO: The server is not available')
finally:
    print('INFO: Bye!')
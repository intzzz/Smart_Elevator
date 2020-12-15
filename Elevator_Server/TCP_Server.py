#!/usr/bin/env python
import string
import logging
import socket
from Crypto.Cipher import AES

# Encryption key, Data size, communication port, identifier
key = '0123456789010qwe'
BUF_SIZE = 70
PORT = 12345
HOST = ''
ID = "ID"
ID_LEN = len(ID)

FORMAT_CONS = '%(asctime)s \t%(message)s'

for handler in logging.root.handlers[:]:
    logging.root.removeHandler(handler)

# Logging
logging.basicConfig(filename='Server_receiving_log.log', filemode='a', level=logging.DEBUG, format=FORMAT_CONS)

decipher = AES.new(key, AES.MODE_ECB)

def main():
    print("Started main")
    for (data, client_addr) in tcp_server():
        decrypted_msg = decrypt(data)
        if (decrypted_msg != 0): # If decryption was successful
            if (decrypted_msg[:ID_LEN] == ID): # If data starts with ID
                logging.debug("%r\t%r" % (decrypted_msg, client_addr)) # Log the message

def udp_server(): # UDP server
    print("Started UDP")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    logging.info("Started listening on udp %s:%s" % (HOST, PORT))
    s.bind((HOST, PORT))
    while True:
        (data, addr) = s.recvfrom(BUF_SIZE)
        yield (data, addr)

def tcp_server(): # TCP server
    print("Started TCP")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    logging.info("Started listening on TCP %s:%s" % (HOST, PORT))
    s.bind((HOST, PORT))

    s.listen(1) # Allow 1 connection

    while True:
        connection, client_addr = s.accept()
        data = connection.recv(BUF_SIZE)
        yield (data, client_addr) # If data received go to main
        connection.close()

def decrypt(encrypted): # Divide message into blocks
    input_len = len(encrypted)
    if (input_len == 32):
        return decrypt_one_block(encrypted)
    elif (input_len == 64):
        encrypted1 = encrypted[:32]
        encrypted2 = encrypted[32:]
        decrypted1 = decrypt_one_block(encrypted1)
        decrypted2 = decrypt_one_block(encrypted2)
        return (decrypted1 + decrypted2)
    else: # Message not right size
        return 0

def decrypt_one_block(encrypted): # Decrypt one block
    try:
        decrypted = encrypted.decode("hex")
        decrypted = decipher.decrypt(decrypted)
        decrypted = ''.join(filter(lambda x: x in string.printable, decrypted))
        return decrypted
    except:
        return 0

if __name__ == "__main__":
    main()

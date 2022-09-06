import array
import socket
import sys
import subprocess
import time
from time import *
import xml.etree.ElementTree as ET
import struct
from flightgear_python.fg_if import FDMConnection
import io
from fg_structs import fgFDM
import numpy as np
from math import *
import matplotlib.pyplot as plt


def FG_UDPsend(phi, theta, psi, lat, lon, alt):
    init_latlon = np.array([lat, lon])  # in degrees - LOWI short final TFB
    my_fgFDM = fgFDM()
    my_fgFDM.set('latitude', init_latlon[0], units='degrees')
    my_fgFDM.set('longitude', init_latlon[1], units='degrees')
    my_fgFDM.set('altitude', alt, units='meters')
    my_fgFDM.set('num_engines', 1)
    my_fgFDM.set('num_tanks', 1)
    my_fgFDM.set('num_wheels', 3)
    my_fgFDM.set('eng_state', 3)
    my_fgFDM.set('rpm', 2200)
    my_fgFDM.set('phi', phi,units='degrees')
    my_fgFDM.set('theta', theta,units='degrees')
    my_fgFDM.set('psi', psi,units='degrees')
    return my_fgFDM


def unpack_netctrl(my_packR):
    # '>I 4x 3d 6f 11f 3f 2f I 4I 4f 4f 4f 4f 4f 4f 4f 4f 4f I 4f I 3I 3f 3f 3f I i f 10f'
    list_id = [1, 4, 3, 6, 11, 3, 2, 1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1, 4, 1, 3, 3, 3, 3, 1, 1, 1, 10]
    list_en = ['I', 'x', 'd', 'f', 'f', 'f', 'f', 'I', 'I', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'I', 'f', 'I',
               'I', 'f', 'f', 'f', 'I', 'i', 'f', 'f']
    type_i = ['I', 'i', 'x', 'd', 'f']
    type_l = [4, 4, 1, 8, 4]
    counter = 0
    net_ctrl = []
    for j in range(len(list_id)):  # range(len(list_id)):
        encoding_i = list_en[j]
        byte_num = list_id[j]
        byte_len = type_l[type_i.index(encoding_i)]
        counter1 = counter
        counter += byte_len * byte_num
        counter2 = counter
        encoding_sub = ''.join(['>', str(byte_num), encoding_i])
        items = struct.unpack(encoding_sub, my_packR[counter1:counter2])
        for k in range(len(items)):
            net_ctrl.append(items[k])
    return net_ctrl


UDP_IP = "127.0.0.1"
UDP_PORTS = 5502
UDP_PORTR = 5501

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORTR))

alt = 500
v_trim = 75
gamma_trim = 0.0
init_psi = 82.0
phi = 0
theta = 0
psi = 0
lat = 42.3656
lon = -71.0096

for i in range(361):
    my_fgFDM = FG_UDPsend(phi, theta, psi, lat, lon, alt)
    my_packS = my_fgFDM.pack()
    sock.sendto(my_packS, (UDP_IP, UDP_PORTS))
    my_packR, address = sock.recvfrom(1024)
    print(my_packR)
    sleep(1/120)
    net_ctrl = unpack_netctrl(my_packR) # received data from flightgear
    print(net_ctrl)
    theta += 1

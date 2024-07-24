#! /usr/bin/env python3
import rospy
import sensor_msgs.msg
from sharon_msgs.msg import GlassesData
from socket import *
from struct import unpack
from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes
import base64
import numpy as np
import sys
import os
import rospkg
import io
from PIL import Image, ImageDraw
from io import BytesIO
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point 

class CommsServer(object):
    def __init__(self):
        rospy.loginfo("Initalizing CommsServer...")
        self.server_ip = rospy.get_param('comms_server/server_ip')
        self.port = rospy.get_param('comms_server/port')

        self.wait_for_connection = True
        self.cipher_decrypt = None
        self.cipher_encrypt = None
        self.socket = None
        self.connection = None
        # get an instance of RosPack with the default search paths
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('proactive_assistance')+"/scripts"
        rospy.loginfo(str(self.path))
        # self.categories = np.load(self.path+'/categories.npy')
        # print(len(self.categories))



        # Publishers
        # self.pubGlassesImage = rospy.Publisher('comms_glasses_server/image', sensor_msgs.msg.Image,queue_size=20)
        # self.pubGlassesData = rospy.Publisher('comms_glasses_server/data', GlassesData,queue_size=20)
        # if self.mark_images:
        #     self.pubGlassesImageMarker = rospy.Publisher('comms_glasses_server/image_markers', sensor_msgs.msg.Image,queue_size=20)
        
        # rospy.loginfo("Set height: "+str(self.heightImage)+" width: "+str(self.widthImage))
        self.create_cipher_encrypt_decrypt()
        self.listen(self.server_ip, self.port)
        self.update()
        
    def create_cipher_encrypt_decrypt(self):
        with open(os.path.join(self.path,"keyEncryption.bin"), mode='rb') as file:  
            key = file.read()
        with open(os.path.join(self.path,"ivEncryption.bin"), mode='rb') as file:
            iv = file.read()
        
            
        self.cipher_encrypt = AES.new(key, AES.MODE_CFB, iv=iv)
        self.cipher_decrypt = AES.new(key, AES.MODE_CFB, iv=iv)
    
    def receive_and_decrypt_image_data(self, length_bytes):
        data = b''
        data_deciphered = b''

        while len(data_deciphered) < length_bytes:
            # receiving the image in batches
            to_read = length_bytes - len(data_deciphered)
            data = self.connection.recv(4096 if to_read > 4096 else to_read)
            data_deciphered += self.cipher_decrypt.decrypt(data)
        if(data_deciphered):
            return True, data_deciphered
        else:
            return False, data_deciphered
    
    def listen(self, server_ip, server_port):
        self.socket = socket(AF_INET, SOCK_STREAM)
        self.socket.setsockopt(SOL_SOCKET, SO_KEEPALIVE, 1)
        self.socket.bind((server_ip, server_port))
        self.socket.listen(1)
    
    def receive_and_decrypt_length_data(self, n_bytes):
        bs = self.connection.recv(n_bytes)
        if(len(bs) == n_bytes):
            deciphered_bytes = self.cipher_decrypt.decrypt(bs)
            (length,) = unpack('>Q', deciphered_bytes)
            return True, length
        else:
            return False, 0
    
    def receive_and_decrypt_array(self, length_bytes):
        data_deciphered = b''
        data = self.connection.recv(length_bytes)
        data_deciphered = self.cipher_decrypt.decrypt(data)
        
        data_decoded = base64.decodebytes(data_deciphered)                    
        data_array = np.frombuffer(data_decoded, dtype=np.float64)
        
        if data_array.size == 0:
            return False, np.array([])
        else:
            return True, data_array 

    
    def update(self):
        ''' 
        This function is called periodically every getPeriod() seconds
        '''
        print("Lets accept the connection")
        

        ok = False
        
        while not rospy.is_shutdown():
            if self.wait_for_connection:
                (self.connection, addr) = self.socket.accept()
                print("Connection accept")
                self.wait_for_connection = False
            else:
                try:

      
                  ok, length_data = self.receive_and_decrypt_length_data(8) # Receive and decipher the number of bytes that the fixation data occupy      
                  ok, data = self.receive_and_decrypt_array(length_data)

                        
                  

                  
                  if ok: # Send back ok
                      data_ok = b'\01'
                  else:
                      data_ok = b'\00'
                                                 
                  data_ok_cipher = self.cipher_encrypt.encrypt(data_ok)
                  try:
                      self.connection.sendall(data_ok_cipher)
                  except:
                      # self.close_connections()
                      self.create_cipher_encrypt_decrypt()
                      self.listen(self.server_ip, self.port)
                      self.wait_for_connection = True
                except KeyboardInterrupt:
                    break
    
     


if __name__ == '__main__':
	rospy.init_node('comms_server')
	commsGlassesServer = CommsServer()
	rospy.spin()
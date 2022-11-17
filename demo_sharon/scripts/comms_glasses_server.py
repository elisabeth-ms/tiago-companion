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
from PIL import Image, ImageDraw, UnidentifiedImageError
from io import BytesIO
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point 

class CommsGlassesServer(object):
    def __init__(self):
        rospy.loginfo("Initalizing CommsGlassesServer...")
        self.heightImage = rospy.get_param('comms_glasses_server/image/height')
        self.widthImage = rospy.get_param('comms_glasses_server/image/width')
        self.serverIp = rospy.get_param('comms_glasses_server/server_ip')
        self.port = rospy.get_param('comms_glasses_server/port')
        self.mark_images = rospy.get_param('comms_glasses_server/mark_images')

        self.wait_for_connection = True
        self.cipher_decrypt = None
        self.cipher_encrypt = None
        self.socket = None
        self.connection = None
        # get an instance of RosPack with the default search paths
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('demo_sharon')+"/scripts"
        rospy.loginfo(str(self.path))
        self.categories = np.load(self.path+'/categories.npy')
        print(len(self.categories))



        # Publishers
        self.pubGlassesImage = rospy.Publisher('comms_glasses_server/image', sensor_msgs.msg.Image,queue_size=20)
        self.pubGlassesData = rospy.Publisher('comms_glasses_server/data', GlassesData,queue_size=20)
        if self.mark_images:
            self.pubGlassesImageMarker = rospy.Publisher('comms_glasses_server/image_markers', sensor_msgs.msg.Image,queue_size=20)
        
        rospy.loginfo("Set height: "+str(self.heightImage)+" width: "+str(self.widthImage))
        self.create_cipher_encrypt_decrypt()
        self.listen(self.serverIp, self.port)
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
        

        print("Connection accept")
        ok = False
        
        while not rospy.is_shutdown():
            if self.wait_for_connection:
                (self.connection, addr) = self.socket.accept()
                
                self.wait_for_connection = False
            else:
                try:
                    ok, length_image_data = self.receive_and_decrypt_length_data(8)
                    ok, image_data = self.receive_and_decrypt_image_data(length_image_data)
                    if ok:
                        print("Image data received")
                        try:
                            image = Image.open(io.BytesIO(image_data))
                        except UnidentifiedImageError:
                            print("eoopoo")

              
                        # header = BytesIO(image_data)
                        # view = header.getbuffer()
                        # header.seek(0)
                        # image = Image.open(bytesio)
                        # image.verify()
                        # image.save(str("1.png"), "PNG")
                        #image = Image.open("ImagesReceivedSocket/"+str(self.file_num)+".png")
                        width, height = image.size
                        print("width:",width, "height: ", height)
                        

                        
                        ok, lengthFixationData = self.receive_and_decrypt_length_data(8) # Receive and decipher the number of bytes that the fixation data occupy
                        
                        ok, fixationData = self.receive_and_decrypt_array(lengthFixationData)
                        ok, lengthBoundingBox = self.receive_and_decrypt_length_data(8) # Receive and decipher the number of bytes that the probability vector data occupy
                        ok, boundingBox = self.receive_and_decrypt_array(lengthBoundingBox) # Receive and decipher the received probability vector
                        ok, lengthDecision = self.receive_and_decrypt_length_data(8) # Receive and decipher the number of bytes that the probability vector data occupy
                        ok, decision = self.receive_and_decrypt_array(lengthDecision) # Receive and decipher the received probability vector
                        glassesData = GlassesData()
                        glassesData.header.stamp = rospy.Time.now()
                        glassesData.fixation_point.append(fixationData[0])
                        glassesData.fixation_point.append(fixationData[1])
                        for i in range(4):
                            glassesData.bounding_box.append(int(boundingBox[i]))
                        glassesData.decision_vector = decision
                        
                        self.pubGlassesData.publish(glassesData)

                        msg = sensor_msgs.msg.Image()
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = "/base_footprint"

                        msg.height = height
                        msg.width = width
                        msg.encoding = "rgb8"
                        msg.is_bigendian = False
                        msg.step = 3 * width
                        msg.data = np.array(image).tobytes()
                        self.pubGlassesImage.publish(msg)
                        
                        
                        if self.mark_images:
                            img = image.copy()
                            img1 = ImageDraw.Draw(img)
                            shape = [(boundingBox[0], boundingBox[1]), (boundingBox[2], boundingBox[3])]
                            img1.rectangle(shape, outline ="red")
                        
                            msg = sensor_msgs.msg.Image()
                            msg.header.stamp = rospy.Time.now()
                            msg.header.frame_id = "/base_footprint"

                            msg.height = height
                            msg.width = width
                            msg.encoding = "rgb8"
                            msg.is_bigendian = False
                            msg.step = 3 * width
                            msg.data = np.array(img).tobytes()
                            self.pubGlassesImageMarker.publish(msg)
                        

                        
                    
                    if ok: # Send back ok
                        data_ok = b'\01'
                    else:
                        data_ok = b'\00'
                                                 
                    data_ok_cipher = self.cipher_encrypt.encrypt(data_ok)
                    try:
                        self.connection.sendall(data_ok_cipher)
                    except:
                        self.close_connections()
                        self.create_cipher_encrypt_decrypt()
                        self.listen(self.serverIp, self.port)
                        self.wait_for_connection = True
                except KeyboardInterrupt:
                    break
    
     


if __name__ == '__main__':
	rospy.init_node('comms_glasses_server')
	commsGlassesServer = CommsGlassesServer()
	rospy.spin()
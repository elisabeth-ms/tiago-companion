import sys
import pickle
import os
from socket import *
from struct import pack
import time
from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes
import numpy as np
import base64
from numpy.random import seed
from numpy.random import rand

# newKey = get_random_bytes(16)
# newFile = open("keyEncryptionOther.bin", "wb")
# newFile.write(newKey)
# newFile.close()


### ----------------- Just for the example ----------------------###


def get_frames_fixations_action_from_pkl(fixationsFileStr):
    with open(fixationsFileStr, 'rb') as f:
        data = pickle.load(f)
    print(data)
    frames = data['frames']
    fixations = data['fixations']
    print(fixations[0])
    
    labels = data['labels']
    # print(labels)
    print(fixations)
    action_level = data['action_labels']
    print(action_level)
    return frames, fixations, action_level, labels


### ----------------- Just for the example ----------------------##

def get_frames_fixations_from_pkl(fixationsFileStr):
    with open(fixationsFileStr, 'rb') as f:
        data = pickle.load(f)
    print(data)
    frames = data['frames']
    fixations = data['fixations']
    print(fixations[0])
    
    labels = data['labels']
    # print(labels)
    print(fixations)
    action_level = data['action_labels']
    print(action_level)
    return frames, fixations, action_level, labels

class ClientProtocol:

    def __init__(self):
        self.socket = None
        self.cipher_encrypt = None
        self.cipher_decrypt = None

    def create_cipher_encrypt_decrypt(self):
        with open(os.path.join(sys.path[0],"keyEncryption.bin"), mode='rb') as file:  
            key = file.read()
        with open(os.path.join(sys.path[0],"ivEncryption.bin"), mode='rb') as file:
            iv = file.read()
            
        self.cipher_encrypt = AES.new(key, AES.MODE_CFB, iv=iv)
        self.cipher_decrypt = AES.new(key, AES.MODE_CFB, iv=iv)

    def connect(self, server_ip, server_port):
        self.socket = socket(AF_INET, SOCK_STREAM)
        self.socket.connect((server_ip, server_port))

    def close(self):
        self.socket.shutdown(SHUT_WR)
        self.socket.close()
        self.socket = None


    def send_data(self, image_data, fixation, bbox, decision_vector):

        # use struct to make sure we have a consistent endianness on the length
        # Send the number of bytes in the image_data
        length = pack('>Q', len(image_data))
        ciphered_bytes = self.cipher_encrypt.encrypt(length)
        self.socket.sendall(ciphered_bytes)
        
         # Send the image data
        ciphered_bytes = self.cipher_encrypt.encrypt(image_data)
        self.socket.sendall(ciphered_bytes)
        
        # Send the number of bytes in the fixation data                 
        fixation_data_encoded = base64.b64encode(fixation)
        length_fixation_data = pack('>Q', len(fixation_data_encoded))
        ciphered_bytes = self.cipher_encrypt.encrypt(length_fixation_data)
        self.socket.sendall(ciphered_bytes)
        
        # Send the fixation data
        ciphered_bytes = self.cipher_encrypt.encrypt(fixation_data_encoded)
        self.socket.sendall(ciphered_bytes)
        
        bbox_data_encoded = base64.b64encode(bbox)
        length_bbox_data = pack('>Q', len(bbox_data_encoded))
        ciphered_bytes = self.cipher_encrypt.encrypt(length_bbox_data)
        self.socket.sendall(ciphered_bytes)
        
        ciphered_bytes = self.cipher_encrypt.encrypt(bbox_data_encoded)
        self.socket.sendall(ciphered_bytes)
        
        
        # Send the number of bytes in the probability vector
        decision_vector_data_encoded = base64.b64encode(decision_vector)
        length_decision_vector_data = pack('>Q', len(decision_vector_data_encoded))
        ciphered_bytes = self.cipher_encrypt.encrypt(length_decision_vector_data)
        self.socket.sendall(ciphered_bytes)
        
        # Send the probability vector
        ciphered_bytes = self.cipher_encrypt.encrypt(decision_vector_data_encoded)
        self.socket.sendall(ciphered_bytes)
        
        
        ok_ciphered = self.socket.recv(1)
        ok = self.cipher_decrypt.decrypt(ok_ciphered)
        print(ok)
        

if __name__ == '__main__':
    
    if len(sys.argv) == 4:
        print(sys.argv[1])
        path_images = sys.argv[1]
        from_frame = sys.argv[2]
        number_frames = sys.argv[3]
        


        categories = np.load(sys.path[0]+'/../demoSharon/'+'categories.npy')
        print(categories)
        
        fixations_file_str = path_images+'/fixations.pkl'
        frames, fixations, action_level, labels = get_frames_fixations_action_from_pkl(fixations_file_str)
        for i in range(len(frames)):
            print(frames[i], action_level[i])

        #Lets check the category of the graspped object in the video
        selected_index_category = None
        selected_category = None
        for index_category, category in enumerate(categories):
            if category in path_images:
                selected_category = category
                selected_index_category = index_category
                break
        print('Selected category:', selected_category, selected_index_category)

        
        frame_number = int(from_frame)
    
        cp = ClientProtocol()
        cp.create_cipher_encrypt_decrypt()
        image_data = None
        remote_ip =  '2.2.2.109' #163.117.150.74' #'163.117.150.88' 
        print("Lets connect")
        cp.connect(remote_ip, 55555)
        print("connection done")
        
        width = 725
        height = 720
        while frame_number<int(number_frames):
            try:
                with open(path_images+"/Frames/"+frames[frame_number]+".png", 'rb') as fp:
                #with open('1.png', 'rb') as fp:
                    image_data = fp.read()
                    print("Lets send the new image")
                    
                    #print(rand(categories.size))
                    
                    decision_vector = np.zeros(categories.size)
                    

                    if action_level[frame_number] ==  1:
                        print(frames[frame_number], action_level[frame_number] )
                        decision_vector[labels[frame_number]] = 1
                        #decision_vector[3] = 1
                    #print(probability_vector)
                    #print(np.array([fixations[frame_number][0], fixations[frame_number][1]]))
                    x = float(fixations[frame_number][0])
                    y = float(fixations[frame_number][1])
                    bbox_width = 150
                    bbox_height = 150
                    tlx = x - bbox_width/2.0
                    if tlx < 0:
                        tlx = 0
                    tly = y - bbox_height/2.0
                    if tly < 0:
                        tly = 0
                    brx = x + bbox_width/2.0
                    if brx > width:
                        brx = width
                    bry = y + bbox_height/2.0
                    if bry > height:
                        bry = height
                    bbox = np.array([tlx, tly, brx, bry])
                    cp.send_data(image_data, np.array([x,y]),bbox, decision_vector)
                    print("Image, fixation point and probability vector sent")
                frame_number=frame_number+1
                time.sleep(0.1)
            except KeyboardInterrupt:
                break
        cp.close()
        sys.exit()
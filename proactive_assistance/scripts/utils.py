import argparse
import importlib
import socket
import pickle
#import cv2
import threading
# import simpleaudio as sa
import time

class Frame():

    def __init__(self, index, pixels, timestamp):

        self.index = index
        self.bgr_pixels = pixels
        self.datetime = timestamp
        return

class Gaze():

    def __init__(self, index, x, y, timestamp):

        self.index = index
        self.x = x
        self.y = y
        self.datetime = timestamp
        return

def send_UDP(UDP_IP, UDP_PORT, messageObj):

    # Create a socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # Serialize the Message object
        data = pickle.dumps(messageObj)

        # Send the serialized data via UDP
        sock.sendto(data, (UDP_IP, UDP_PORT))
        print("Message sent successfully!")

    except Exception as e:
        print("Error:", e)

    finally:
        # Close the socket
        sock.close()

    return

def retriveConfig(description=""):

    parser = argparse.ArgumentParser(description=description)

    # add all command line arguments desired to the parser
    parser.add_argument("--config_num", default=0, help="index of config file (default: 0)") 

    # parse the args passed
    args = parser.parse_args()
    config_num = int(args.config_num)
    
    print("Using config file 'config%d.py'" %config_num)
    cfg = importlib.import_module('configs.config%d'%config_num)
    return cfg

def resizeFrame(bgrPixels, gpuImage, newSize):

    """
    This function uses opencv cuda to convert a bgr colored image
    into a smaller gray scaled version. Used to log the frames.
    """
    gpuImage.upload(bgrPixels)
    gpuGray = cv2.cuda.cvtColor(gpuImage, cv2.COLOR_BGR2GRAY)
    gpuSmall = cv2.cuda.resize(gpuGray, newSize)
    cpuSmall = gpuSmall.download()
    return cpuSmall

def playAudio(filePath):

    def play():
        waveObj = sa.WaveObject.from_wave_file(filePath)
        playObj = waveObj.play()
        playObj.wait_done()
    
    # Create and start a new thread for playing the audio
    thread = threading.Thread(target=play)
    thread.start()

if __name__ == '__main__':

    time.sleep(3)
    audioPath = './ASR/asr_success.wav'
    playAudio(audioPath)

    time.sleep(3)
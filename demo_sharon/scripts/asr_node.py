#! /usr/bin/env python3

import speech_recognition as sr
import os
import datetime
import numpy 
from jellyfish import soundex, metaphone, nysiis
import rospy
from std_msgs.msg import String, Empty, Bool
from sharon_msgs.srv import ActivateASR


word_dict = ['sugar',
             'cereals', 
             'coffee',
             'bread',
             'milk',
             'water',
             'butter',
             'nesquick', 
             'please',
             ]

# word_dict = ['plate', 
#              'bowl', 
#              'cup', 
#              'fork', 
#              'spoon', 
#              'knife', 
#              'cutting board', 
#              'microwave', 
#              'toaster',
#              'fridge',
#              'sugar',
#              'cereals', 
#              'olive oil', 
#              'nesquick',
#              'coffee',
#              'sliced bread',
#              'tomato sauce', 
#              'jam', 
#              'nutella',
#              'butter',
#              'milk',
#              'water', 
#     	     'background', 
#              'start',
#              'end',
#              'training',
#              'grasping'
#              ]



class ASR(object):
    def __init__(self):
        
        rospy.loginfo("Initalizing asr_node...")
        self.pubAsr = rospy.Publisher('asr_node/data', String,queue_size=20)
        self.pubAsrDetectsSound = rospy.Publisher('asr_node/detects_sound', Bool ,queue_size=20)
        self.device_id = None
        self.serviceActiveASR = rospy.Service('/asr_node/activate_asr', ActivateASR, self.activate_asr)
        self.activated = False
        self.r = sr.Recognizer()
        self.r.pause_threshold = 0.6
        self.r.non_speaking_duration = 0.18
        mic_list = sr.Microphone.list_microphone_names()
        #mic_name = "Andrea Comm USB-SA Headset: Audio (hw:1,0)"
        mic_name = "sof-hda-dsp: - (hw:1,7)"
        print(mic_list)
        for i, microphone_name in enumerate(mic_list):
            if microphone_name == mic_name:
                self.device_id = i
    
        # Creacion de la biblioteca de sonidos
        self.sounds = []
        for val  in (word_dict): 
            print(val)
            self.sounds.append(nysiis(val))
        
        print(self.sounds)
        self.sound_dict = dict(zip(word_dict, self.sounds))
        
        self.update()

    def activate_asr(self, req):
        self.activated = req.activate
        return True
        
    
    def get_key(self,val):
        for key, value in self.sound_dict.items():
            if val == value:
                return key

    def levenshteinDistanceDP(self, token1, token2):
        '''
        Funcion que calcula la distancia Levenshtein, es una medida de similitud entre dos cadenas. 
        Se define como el número mínimo de cambios necesarios para convertir la cadena a en la cadena b 
        Cuanto menor sea la distancia Levenshtein, más parecidas son las cadenas. 
        
        Parameters
        ----------
        token1 : string
            Palabra a comparar 1
        token2 : string
            Palabra a comparar 2

        Returns
        -------
        Devuelve la distancia entre las dos palabras.

        '''
        distances = numpy.zeros((len(token1) + 1, len(token2) + 1))

        for t1 in range(len(token1) + 1):
            distances[t1][0] = t1

        for t2 in range(len(token2) + 1):
            distances[0][t2] = t2
            
        a = 0
        b = 0
        c = 0
        
        for t1 in range(1, len(token1) + 1):
            for t2 in range(1, len(token2) + 1):
                if (token1[t1-1] == token2[t2-1]):
                    distances[t1][t2] = distances[t1 - 1][t2 - 1]
                else:
                    a = distances[t1][t2 - 1]
                    b = distances[t1 - 1][t2]
                    c = distances[t1 - 1][t2 - 1]
                    
                    if (a <= b and a <= c):
                        distances[t1][t2] = a + 1
                    elif (b <= a and b <= c):
                        distances[t1][t2] = b + 1
                    else:
                        distances[t1][t2] = c + 1


        return distances[len(token1)][len(token2)]

    def calcDictDistance(self, word, numWords, dictionary):
        '''
        Funcion que devuelve las palabras  de un diccionario que más se asemejan a una dada.

        Parameters
        ----------
        word : string
            la palabra a comparar.
        numWords : int
            numero total de palabras similares que devuelve la funcion.
        dictionary : dict
            diccionario de palabras seleccionado. Disponibles: "sounds", "word_dict"
            
        Ahora mismo el diccionario que se pasa es el de sonidos, pero si se quisiese hacer una comparativa
        por semejanza de palabras se pasaría "word_dict", en ese caso la palabra se pasaría tal cual, sin llamar
        a una funcion de sonidos "oundex, metaphone o nysiis"
        
        Returns
        -------
        closestWords : list
            devuelve las palabras del diccionario que mas se asemejan a la palabra dada.

        '''

        dictWordDist = []
        wordIdx = 0
        
        for line in dictionary: 

            wordDistance = self.levenshteinDistanceDP(word, line.strip())
            if wordDistance >= 10:
                wordDistance = 9
            dictWordDist.append(str(int(wordDistance)) + "-" + line.strip())
            wordIdx = wordIdx + 1

        closestWords = []
        wordDetails = []
        currWordDist = 0
        dictWordDist.sort()

        for i in range(numWords):
            currWordDist = dictWordDist[i]
            wordDetails = currWordDist.split("-")
            closestWords.append(wordDetails[1])
        return closestWords
    
    def update(self):

        with sr.Microphone(device_index = self.device_id) as source:
            print("Adjusting ambient noise...")
            self.r.adjust_for_ambient_noise(source, duration = 5.5)
        self.r.energy_threshold = self.r.energy_threshold*1.2
        
        self.r.dynamic_energy_threshold = False
        print(self.r.energy_threshold)
        
        while not rospy.is_shutdown():
            
            if self.activated:

                with sr.Microphone(device_index = self.device_id) as source:
                    print("Talk:")
                    audio_data = self.r.listen(source)
                    print(audio_data)
                    print("Stop - Recognizing...\n")
                    
                    msgDetectedSound = Bool()
                    msgDetectedSound.data = True
                    self.pubAsrDetectsSound.publish(msgDetectedSound)
                    
                    try:
                        # esta pensado que se hable en inglés, si se quiere hablar en español r.recognize_google(audio_data,language="es-ES")
                        text = self.r.recognize_google(audio_data, language="en-us")
                        print("What you said: " + text)
                        words = text.split()
                
                        text_list = []
                        
                        if words[-1] == 'please':
                            word_dictionary_sound = self.calcDictDistance(nysiis(words[-2]), 1, self.sounds)
                        else:
                            word_dictionary_sound = self.calcDictDistance(nysiis(words[-1]), 1, self.sounds)

                    
                        #     # si se quiere hacer por similitud de palabras y no por similitud de sonido
                        #     # word_dictionary = calcDictDistance(word, 1, word_dict)
                        print("Similar word: ", word_dictionary_sound[0])
                        
                        if  self.get_key(word_dictionary_sound[0]) == 'bread':
                            text_list.append("sliced bread")
                        else:
                            text_list.append(self.get_key(word_dictionary_sound[0]))
                    
                        #     # las palabras compuestas son casos especiales
                        #     if self.get_key(word_dictionary_sound[0]) == 'cutting board':
                        #         break;
                        #     elif self.get_key(word_dictionary_sound[0]) == 'tomato sauce':
                        #         break;
                        #     elif self.get_key(word_dictionary_sound[0]) == 'sliced bread':
                        #         break;
                        #     elif self.get_key(word_dictionary_sound[0]) == 'olive oil':
                        #         break;
                        
                        
                        
                        # for word in words:
                        #     print(word)
                        #     word_dictionary_sound = self.calcDictDistance(nysiis(word), 1, self.sounds)
                    
                        #     # si se quiere hacer por similitud de palabras y no por similitud de sonido
                        #     # word_dictionary = calcDictDistance(word, 1, word_dict)
                        #     # print("Similar word: ", word_dictionary[0])
                    
                        #     text_list.append(self.get_key(word_dictionary_sound[0]))
                    
                        #     # las palabras compuestas son casos especiales
                        #     if self.get_key(word_dictionary_sound[0]) == 'cutting board':
                        #         break;
                        #     elif self.get_key(word_dictionary_sound[0]) == 'tomato sauce':
                        #         break;
                        #     elif self.get_key(word_dictionary_sound[0]) == 'sliced bread':
                        #         break;
                        #     elif self.get_key(word_dictionary_sound[0]) == 'olive oil':
                        #         break;
                    
                        save_text = ' '.join(text_list)
                        print("Autocorrection: ",save_text)
                        print("---------------------------------")
                        
                        words_after_correction = save_text.split()
                        msgText = String()
                        msgText.data = words_after_correction[-1].lower()
                        self.pubAsr.publish(msgText)
                        
                    except IndexError: # the API key didn't work
                        print("No internet connection")
                    except KeyError:   # the API key didn't work
                        print("Invalid API key or quota maxed out")
                    except LookupError: # speech is unintelligible
                        print("Could not understand audio")
                    except sr.UnknownValueError:
                        print("Google Speech Recognition could not understand audio")
                    except sr.RequestError as e:
                        print("Could not request results from Google Speech Recognition service; {0}".format(e))
                    except:                        
                        print('Sorry.. run again...')
            





if __name__ == '__main__':
	rospy.init_node('asr_node')
	asr = ASR()
	rospy.spin()



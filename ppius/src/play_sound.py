#!/usr/bin/env python
import alsaaudio
import wave
import rospy
from ppius_msg.msg import sound
import struct

class sound_player_node():
	def __init__(self):
	# Postavke kanala
		output_format = alsaaudio.PCM_FORMAT_FLOAT_LE	#format ispisa
		channl_num = 2; 							#broj kanala
		fs = 44100									#frekvencija uzorkovanja
		frame_buffer = 1024							#velicina buffera
		device = 'default'
		# otvranje audio stream-a za pisanje i postavljanje postavka 
		self.audio_out = alsaaudio.PCM(alsaaudio.PCM_PLAYBACK, device=device)
		self.audio_out.setchannels(channl_num)
		self.audio_out.setrate(fs)
		self.audio_out.setformat(output_format)
		self.audio_out.setperiodsize(frame_buffer)
		#self.rate = rospy.Rate(10.76426)
		rospy.Subscriber("sound", sound, self.sound_callback)
	
	def sound_callback(self, data):

		#data.size
		#data.sound			 										# zvuk u listi u float formatu
		values =  [struct.pack('f',s) for s in data.sound ] 		# pretvaranje flaot formata u byte_list
		print("-------------------------------------------------------")
		print(len(data.sound))
		output =  [ b"".join([value, value]) for value in values] 	# odkomentirati ako je samo jedan           															# kanal dostupan, ovo ga podupla
		
		byte_stream_sound = b"".join(values)						# spajnje liste bajtova u niz 
		self.audio_out.write(byte_stream_sound)						# sviranje
		#self.rate.sleep()





if __name__ == '__main__':
	rospy.init_node('pyclass')
	
	try:
		player = sound_player_node()
	except rospy.ROSInterruptException: pass
	rospy.spin()

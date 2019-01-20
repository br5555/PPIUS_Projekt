import wave
import rospy
from ppius.msgs import ppius

class wav_reader():
frame_buffer = 1024

	#wav objekt za peristupanje wav file-u
	def __init__(self):
		self.wav_object = wave.open('my_sound.wav', 'r')
		self.number_of_frames = wav_object.getnframes()
		self.pub = rospy.Publisher('sound_topic', ppius, queue_size=10)
        self.sound_msg = ppius
	def run(self):
		while(self.number_of_frames - self.wav_object.tell() > 0):
			if number_of_frames - wav_object.tell() > frame_buffer: #if end of file is reached
			n = frame_buffer - number_of_frames - wav_object.tell()
			frame = wav_object.readframes(frame_buffer-n)
			self.sound_msg.dbl_vec = frame
			self.pub.publish(self.sound_msg)
			time.sleep(frame_buffer/44100*0.75)
			

if __name__ == '__main__':
	rospy.init_node('pyclass')
	
	try:
		node = wav_reader())
		node.run()
	except rospy.ROSInterruptException: pass

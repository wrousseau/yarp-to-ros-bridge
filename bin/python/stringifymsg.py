import rospy, os, sys, rostopic
from importlib import import_module
from rostopic import get_topic_type

class Stringifier:
	def __init__(self):
		self.data = []

	def processDetailed(self,string):
		result = ""
		lst = string.split('\n')
		if (len(lst) > 1):
			exclude = [] # List containing elements of the list we don't need
			for i in range(len(lst)):
				lst[i] = ' '.join(lst[i].split())
				tmp = lst[i].split(": ")
				if (len(tmp)>1):
					lst[i]=tmp[1]
				else:
					exclude.append(tmp[0])
			for i in range(len(exclude)):
				lst.remove(exclude[i])
			for i in range(len(lst)):
				result += lst[i] + " "
		else:
			result = lst[0]
		return result

	def processField(self,attr):
		if isinstance(attr, basestring): # If it is a string, we don't want to split by character
			string = str(attr)
			string = self.processDetailed(string)
			if ' ' in string:
				string = '""' + string + '""' # If the string contains spaces, we mark it with ""
			return string
		try:
			count = len(attr) # Will work if it is a list
			string = ' '.join(attr)
			string = '"v"' + str(count) + " " + string + '"v"' # We mark it as a vector with "v" for YARP and add the size
		except TypeError:
			string = str(attr) # If not a list, we just grab it as a string
		return self.processDetailed(string)

	def callback(self,msg):
		msgStr = "";
		for slot in msg.__slots__:
			string = self.processField(getattr(msg,slot))
			msgStr = msgStr + string + " "
		msgStr = ' '.join(msgStr.split())
		if msgStr:
			self.data[0].write(msgStr)
			self.data[0].flush()

	def listener(self,fd,topicName):
		fo = os.fdopen(int(fd), "w", 0)
		self.data.append(fo)
		rosmsg = get_topic_type(topicName)[0]

		rosmsg = rosmsg.split("/")
		module = import_module(rosmsg[0]+".msg")

		rospy.init_node('listener', anonymous=True)
		rospy.Subscriber(topicName, getattr(module,rosmsg[1]), self.callback)
		rospy.spin()


def stringify(fd,topicName):
	sf = Stringifier()
	sf.listener(fd,topicName)
	return 0
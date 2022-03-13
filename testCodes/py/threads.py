import threading 

"""
Class for multi-thread
Highly inspired of "#https://www.tutorialspoint.com/
        python/python_multithreading.htm"
"""
class myThread (threading.Thread):
   def __init__(self, func, arg1, threadID):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.status = 0
      self.arg = arg1 #if need more than 1 arg, make a list
      self.func = func

   def run(self):
      print("Starting : threadID = {}\n".format(self.threadID))
      self.func(self.arg)
      print("Exiting  : threadID = {}\n".format(self.threadID))
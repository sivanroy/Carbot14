

"""
Implement a stack (LIFO)
push -> add a new value at the head of the stack
pop  -> remove the head of the stack
"""
class Stack(object):
    def __init__(self):
        self.array = []

    def push(self,new):
        self.array.append(new)
            
    def pop(self):
        if len(self.array) > 0:
            self.array = self.array[:-1]

    def value(self):
    	return self.array[-1]
            
    def __str__(self):
        return "{}".format(self.array)
import math

def calcDistExtend(angle):
    return(math.sqrt(6044 - 6192 * math.cos(angle + 0.519)))

def calcDistReturn(angle):
    return(math.sqrt(12956 - 12384 * math.cos(angle + 0.519)))
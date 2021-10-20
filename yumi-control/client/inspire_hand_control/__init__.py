import os, inspect, sys
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,currentdir)

from inspire_hand import InspireHand
from inspire_hand_execute import HandControlThread
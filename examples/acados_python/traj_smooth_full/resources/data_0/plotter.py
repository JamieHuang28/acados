import json
import math
from easydict import EasyDict
import matplotlib 
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt

CAR_PARAMS={
    'length':5.098,
    'width':2.116,
    'front_edge_to_rear_real':4.015,
    'wheel_base': 3.100
}

def smoothTheta(thetas):
  index = len(thetas) - 1
  
  for i in range(len(thetas) - 1):
    change = 0
    if(math.fabs(thetas[i+1] - thetas[i]) > math.pi):
      change = -1 if thetas[i+1] - thetas[i] > 0 else 1
      index = i+1
      break
  
  for i in range(index, len(thetas)):
    thetas[i] = thetas[i] + 2 * change * math.pi
  
def loadData():
  data_dir = "./"
  data_path = data_dir + "data_0_0.json"
  with open(data_path, "r") as infile: # open file for reading
    j = json.load(infile) # load the JSON object from the file
    
    x = j["x"] # get the double list from the JSON object
    y = j["y"]
    phi = j["phi"]
    delta = j["delta"]
    v = j["v"]
    left_bound = j["left_bound"]
    right_bound = j["right_bound"]
    front_bound = j["front_bound"]
    back_bound = j["back_bound"]
  smoothTheta(phi)
  
  return (x, y, phi, delta, v, left_bound, right_bound, front_bound, back_bound)

def plotTrajAndBound(data):
  x, y, phi, delta, v, left_bound, right_bound, front_bound, back_bound = data
  indexes = list(range(1, len(x) + 1))
  
  fig = plt.figure()
  ax1 = fig.add_subplot(2,4,1)
  ax1.plot(x, y)
  ax1.set_title("xy")
  ax2 = fig.add_subplot(2,4,2)
  ax2.plot(indexes, phi)
  ax2.set_title("phi")
  ax0 = fig.add_subplot(2,4,3)
  ax0.plot(indexes, delta)
  ax0.set_title("delta")
  ax3 = fig.add_subplot(2,4,4)
  ax3.plot(indexes, v)
  ax3.set_title("v")
  
  ax4 = fig.add_subplot(2,4,5)
  ax4.plot(indexes, left_bound)
  ax4.set_title("left_bound")
  ax5 = fig.add_subplot(2,4,6)
  ax5.plot(indexes, right_bound)
  ax5.set_title("right_bound")
  ax6 = fig.add_subplot(2,4,7)
  ax6.plot(indexes, front_bound)
  ax6.set_title("front_bound")
  ax7 = fig.add_subplot(2,4,8)
  ax7.plot(indexes, back_bound)
  ax7.set_title("back_bound")
  
  plt.show()
  return 0

if __name__ == "__main__":
  data = loadData()
  plotTrajAndBound(data)
  
  print("done")

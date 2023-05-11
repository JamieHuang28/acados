import json
import math

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
  
def loadData(data_path):
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
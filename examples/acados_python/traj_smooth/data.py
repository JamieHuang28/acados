import json
import numpy as np

def load_data():
    data_dir = "./data_0/"
    data_path = data_dir + "data_0_0.json"
    j = None
    with open(data_path, "r") as infile: # open file for reading
        j = json.load(infile) # load the JSON object from the file
    return j
    # vel = 2.0
    # T = 10
    # j = {}
    # j['x'] = np.linspace(0, T * vel, 101)
    # j['y'] = np.zeros(101)
    # j['phi'] = np.zeros(101)
    # j['v'] = np.ones(101)
    # return j

if __name__ == "__main__":
    j = load_data()
    print(j)
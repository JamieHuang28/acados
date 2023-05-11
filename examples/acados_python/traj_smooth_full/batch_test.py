import sys, os
# import closed_loop_simulation from main.py and call it
from main import closed_loop_simulation

# import the global variable data_path from json_parser.py and print it
from json_parser import data_path
import json_parser

if __name__ == "__main__":
    # parse the first argument as the root directory of the data
    data_dir = sys.argv[1]
    start_idx = sys.argv[2]

    # travers all files in data_dir and print them
    files = []
    for file in os.listdir(data_dir):
        files.append(file)
    
    # ignore files starting with .
    files = filter(lambda file: not file.startswith("."), files)

    # keep files ending with .json
    files = filter(lambda file: file.endswith(".json"), files)
    
    # transform the files into its full path
    files = map(lambda file: data_dir + "/" + file, files)

    
    # # print the sorted files
    # for file in files:
    #     print(file)

    index = 0
    for file in files:
        index += 1
        if index < int(start_idx):
            continue
        json_parser.data_path = file
        print("data_path: ", json_parser.data_path)
        closed_loop_simulation()
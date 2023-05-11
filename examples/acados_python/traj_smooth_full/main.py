from data import loadData
from collision_coeff import GetCollisionCoeff
from acados_main import AcadosParams, create_ocp_solver_description, closed_loop_simulation
from utils import plot_xy, plot_apa
import matplotlib.pyplot as plt

ocp = create_ocp_solver_description()

def plan(file_name):
    # load data
    x, y, phi, delta, v, left_bound, right_bound, front_bound, back_bound = loadData(file_name)
    params = AcadosParams()

    collisionCoeff = GetCollisionCoeff(params, x, y, phi, left_bound, right_bound, front_bound, back_bound)
    # run closed loop simulation
    simX, simU = closed_loop_simulation(params, ocp, x, y, phi, delta, v, collisionCoeff)

    # plot results
    plot_xy(x, y, simX[:, 0], simX[:, 1], plt_show=False)
    # plot_apa(np.linspace(0, T_horizon, N_horizon+1), 1.0, simU, simX, latexify=False, plt_show=False)
    plt.show()

if __name__ == "__main__":
    import sys, os
    if len(sys.argv) != 3:
        print("Usage: python main.py <data_dir> <start_index>")
        exit(1)
    
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
        plan(file)

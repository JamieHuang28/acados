from data import loadData
from acados_main import AcadosParams, closed_loop_simulation
from utils import plot_xy, plot_apa
import matplotlib.pyplot as plt

def plan(file_name):
    # load data
    x, y, phi, delta, v, left_bound, right_bound, front_bound, back_bound = loadData(data_path)
    params = AcadosParams()

    # run closed loop simulation
    simX, simU = closed_loop_simulation(params, x, y, phi, delta, v, left_bound, right_bound, front_bound, back_bound)

    # plot results
    plot_xy(x, y, simX[:, 0], simX[:, 1], plt_show=False)
    # plot_apa(np.linspace(0, T_horizon, N_horizon+1), 1.0, simU, simX, latexify=False, plt_show=False)
    plt.show()

if __name__ == "__main__":
    import sys
    # check the count of argv
    if len(sys.argv) != 2:
        print("usage: python main.py <data_path>")
        sys.exit(1)
    data_path = sys.argv[1]
    plan(data_path)

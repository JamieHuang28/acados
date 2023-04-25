from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from vehicle_model import export_vehicle_model
import numpy as np
import scipy.linalg
from utils import plot_xy, plot_apa
from json_parser import GetCollisionCoeff, loadData
import matplotlib.pyplot as plt
import time

N_horizon = 100   # Define the number of discretization steps
T_horizon = 10.0  # Define the prediction horizon

NUM_MIN = -1e10
NUM_MAX = 1e10

delta_max = 0.61  
v_max = 10.0
acceleration_max = 6.0
wheelAngleRate_max = 0.52
slackLR_max = 0.05
slackFR_max = 0.05
# wheelBase = 3.100

def create_ocp_solver_description() -> AcadosOcp:
    ocp = AcadosOcp()

    model = export_vehicle_model()
    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ocp.dims.N = N_horizon

    wheelBase = np.array([3.1])
    ocp.parameter_values = wheelBase
 
    Q_mat = 2 * np.diag([0.0, 0.0, 0.0, 2000, 100])       
    R_mat = 2 * np.diag([100, 2000, 30.0, 30.0])         

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

    Vx = np.zeros((ny, nx))
    Vx[0:nx, 0:nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[nx : ny, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.yref = np.zeros((ny,))
    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    ocp.constraints.lbx = np.array([NUM_MIN, NUM_MIN])
    ocp.constraints.ubx = np.array([NUM_MAX, NUM_MAX])
    ocp.constraints.idxbx = np.array([3, 4])               

    ocp.constraints.lbu = np.array([NUM_MIN, NUM_MIN, 0.0, 0.0])
    ocp.constraints.ubu = np.array([NUM_MAX, NUM_MAX, NUM_MAX, NUM_MAX])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])        
    ocp.constraints.lbx_e = np.array([NUM_MIN, NUM_MIN, NUM_MIN, NUM_MIN, NUM_MIN])   
    ocp.constraints.ubx_e = np.array([NUM_MAX, NUM_MAX, NUM_MAX, NUM_MAX, NUM_MAX])
    ocp.constraints.idxbx_e = np.array([0, 1, 2, 3, 4])     
    ocp.constraints.C   = np.zeros((8, nx))
    ocp.constraints.D   = np.zeros((8, nu))
    ocp.constraints.lg  = np.array([NUM_MIN, NUM_MIN, NUM_MIN, NUM_MIN, NUM_MIN, NUM_MIN, NUM_MIN, NUM_MIN])
    ocp.constraints.ug  = np.array([NUM_MAX, NUM_MAX, NUM_MAX, NUM_MAX, NUM_MAX, NUM_MAX, NUM_MAX, NUM_MAX])

    # set options
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "IRK"
    ocp.solver_options.nlp_solver_type = "SQP"  # SQP_RTI, SQP
    ocp.solver_options.nlp_solver_max_iter = 100

    # set prediction horizon
    ocp.solver_options.tf = T_horizon

    return ocp


def closed_loop_simulation():

    x, y, phi, delta, v, left_bound, right_bound, front_bound, back_bound = loadData()
    collisionCoeff = GetCollisionCoeff()


    ocp = create_ocp_solver_description()
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json")

    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]


    simX = np.ndarray((N_horizon + 1, nx))
    simU = np.ndarray((N_horizon, nu))

    for stage in range(N_horizon + 1):
        xInit = np.array([x[stage], y[stage], phi[stage], delta[stage], v[stage]])
        acados_ocp_solver.set(stage, "x", xInit)
    for stage in range(N_horizon):
        uInit = np.array([0.0, 0.0, slackLR_max, slackFR_max])
        acados_ocp_solver.set(stage, "u", uInit)
    
    for i in range(N_horizon):
        wheelBase = np.array([3.1])
        acados_ocp_solver.set(i, "p", wheelBase)

        if i == 0:
            x0 = np.array([x[0], y[0], phi[0], 0.0, 0.0])
            acados_ocp_solver.set(0, "lbx", x0)
            acados_ocp_solver.set(0, "ubx", x0)
        else:
            if (v[i] > 0.5): 
                lbx = np.array([-delta_max, 0.0])
                ubx = np.array([delta_max, v_max])
                acados_ocp_solver.set(i, "lbx", lbx)
                acados_ocp_solver.set(i, "ubx", ubx)
            elif (v[i] < -0.5):  
                lbx = np.array([-delta_max, -v_max])
                ubx = np.array([delta_max, 0.0])
                acados_ocp_solver.set(i, "lbx", lbx)
                acados_ocp_solver.set(i, "ubx", ubx)
            else:  
                lbx = np.array([0.0, -0.1])
                ubx = np.array([0.0, 0.1])
                acados_ocp_solver.set(i, "lbx", lbx)
                acados_ocp_solver.set(i, "ubx", ubx)


        C = np.array(collisionCoeff[(8 * i + 0):(8 * (i + 1) + 0), 0:5])
        D = collisionCoeff[(8 * i + 0):(8 * (i + 1) + 0), 5:9]
        lg = np.array([NUM_MIN, NUM_MIN, NUM_MIN, NUM_MIN, NUM_MIN, NUM_MIN, NUM_MIN, NUM_MIN])
        ug = collisionCoeff[(8 * i + 0):(8 * (i + 1) + 0), 9]

        acados_ocp_solver.constraints_set(i, "C", C, api='new')
        acados_ocp_solver.constraints_set(i, "D", D, api='new')
        acados_ocp_solver.constraints_set(i, "lg", lg)
        acados_ocp_solver.constraints_set(i, "ug", ug)

        lbu = np.array([-acceleration_max, -wheelAngleRate_max, 0.0, 0.0])
        ubu = np.array([acceleration_max, wheelAngleRate_max, slackLR_max, slackFR_max])
        acados_ocp_solver.set(i, "lbu", lbu)
        acados_ocp_solver.set(i, "ubu", ubu)

        acados_ocp_solver.cost_set(i, 'Vx', ocp.cost.Vx, api='new')
        acados_ocp_solver.cost_set(i, 'Vu', ocp.cost.Vu, api='new')
        acados_ocp_solver.cost_set(i, 'W', ocp.cost.W, api='new')
        acados_ocp_solver.cost_set(i, 'yref', ocp.cost.yref, api='new')

    lbx_e = np.array([x[N_horizon], y[N_horizon], phi[N_horizon], 0.0, 0.0])   
    ubx_e = np.array([x[N_horizon], y[N_horizon], phi[N_horizon], 0.0, 0.0])
    acados_ocp_solver.set(N_horizon, "lbx", lbx_e)
    acados_ocp_solver.set(N_horizon, "ubx", ubx_e)

    wheelBase = np.array([3.1])
    acados_ocp_solver.set(N_horizon, "p", wheelBase)

    # solve ocp
    start_time = time.time()
    status = acados_ocp_solver.solve()
    end_time = time.time()
    print("elapsed_time = ", end_time - start_time)
    # if status != 0:
    #     raise Exception(f'acados returned status {status}.')
    print(f'acados returned status {status}.')

    for stage in range(N_horizon + 1):
        simX[stage, :] = acados_ocp_solver.get(stage, "x")
    for stage in range(N_horizon):
        simU[stage, :] = acados_ocp_solver.get(stage, "u")
    
    plot_xy(x, y, simX[:, 0], simX[:, 1], plt_show=False)
    plot_apa(np.linspace(0, T_horizon, N_horizon+1), 1.0, simU, simX, latexify=False, plt_show=False)
    plt.show()

if __name__ == "__main__":
    closed_loop_simulation()

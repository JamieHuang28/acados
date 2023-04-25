#
# Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
# Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
# Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
# Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

from acados_template import AcadosOcp, AcadosOcpSolver
from vehicle_model import export_pendulum_ode_model
import numpy as np
from utils import plot_apa, plot_xy
from data import load_data

def main():
    traj_ref = load_data()
    for i in range(len(traj_ref['phi'])):
        if traj_ref['phi'][i] < 0:
            traj_ref['phi'][i] += 2 * np.pi

    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # set model
    model = export_pendulum_ode_model()
    ocp.model = model

    Tf = 10.0
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    N = 100

    # set dimensions
    ocp.dims.N = N

    # set cost
    Wus = np.ones(8)
    Ws = np.append([1, 100, 1, 100], Wus)
    Q_mat = 2*np.diag([0.0, 0.0, 0.0, Ws[0], Ws[2]])
    R_mat = 2*np.diag([Ws[1], Ws[3], Ws[4], Ws[5], Ws[6], Ws[7], Ws[8], Ws[9], Ws[10], Ws[11]])

    # the 'EXTERNAL' cost type can be used to define general cost terms
    # NOTE: This leads to additional (exact) hessian contributions when using GAUSS_NEWTON hessian.
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost = model.x.T @ Q_mat @ model.x + model.u.T @ R_mat @ model.u
    ocp.model.cost_expr_ext_cost_e = model.x.T @ Q_mat @ model.x

    # set constraints
    omega_max = 1.0
    a_max = 5.0
    u_slack_min = np.zeros(8)
    u_slack_max = np.ones(8) * 0.1
    ocp.constraints.lbu = np.append([-omega_max, -a_max], u_slack_min)
    ocp.constraints.ubu = np.append([+omega_max, +a_max], u_slack_max)
    ocp.constraints.idxbu = np.arange(0, 10)

    x0 = np.array([traj_ref['x'][0], traj_ref['y'][0], traj_ref['phi'][0]])
    ocp.constraints.x0 = x0

    delta_max = 0.6
    v_max = 3.0
    ocp.constraints.lbx = np.array([-delta_max, -v_max])
    ocp.constraints.ubx = np.array([+delta_max, +v_max])
    ocp.constraints.idxbx = np.abs([3, 4])

    ocp.constraints.lbx_0 = x0
    ocp.constraints.ubx_0 = x0
    ocp.constraints.idxbx_0 = np.array([0, 1, 2])

    xf = np.array([traj_ref['x'][N], traj_ref['y'][N], traj_ref['phi'][N], traj_ref['delta'][N], traj_ref['v'][N]])
    # print("xf:", xf)
    ocp.constraints.lbx_e = xf
    ocp.constraints.ubx_e = xf
    ocp.constraints.idxbx_e = np.arange(0, 5)

    # set options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
    # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
    # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = 'IRK'
    # ocp.solver_options.print_level = 1
    ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI, SQP
    ocp.solver_options.qp_solver_warm_start = True
    ocp.solver_options.nlp_solver_max_iter = 500

    # set prediction horizon
    ocp.solver_options.tf = Tf

    ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')

    simX = np.ndarray((N+1, nx))
    simU = np.ndarray((N, nu))

    # set initial value
    for i in range(N + 1):
        ocp_solver.set(i, "x", np.array(\
            [traj_ref['x'][i], traj_ref['y'][i], traj_ref['phi'][i], \
             traj_ref['delta'][i], traj_ref['v'][i]]))
    status = ocp_solver.solve()
    ocp_solver.print_statistics() # encapsulates: stat = ocp_solver.get_stats("statistics")

    if status != 0:
        raise Exception(f'acados returned status {status}.')

    # get solution
    for i in range(N):
        simX[i,:] = ocp_solver.get(i, "x")
        # print(simX[i,:])
        simU[i,:] = ocp_solver.get(i, "u")
    simX[N,:] = ocp_solver.get(N, "x")

    # plot_apa(np.linspace(0, Tf, N+1), omega_max, simU, simX, latexify=False)
    # plot_xy(simX[:, 0], simX[:, 1])
    plot_xy(traj_ref['x'], traj_ref['y'], simX[:, 0], simX[:, 1])


if __name__ == '__main__':
    main()
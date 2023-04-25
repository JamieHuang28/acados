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

from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, tangent

def export_pendulum_ode_model() -> AcadosModel:

    model_name = 'vehicle_ode'

    # constants
    L = 3.1

    # set up states & controls
    x       = SX.sym('x')
    y       = SX.sym('y')
    phi     = SX.sym('phi')
    delta   = SX.sym('delta')
    v       = SX.sym('v')

    x = vertcat(x, y, phi, delta, v)

    omega = SX.sym('omega')
    a = SX.sym('a')
    u_slack1 = SX.sym('u_slack1')
    u_slack2 = SX.sym('u_slack2')
    u_slack3 = SX.sym('u_slack3')
    u_slack4 = SX.sym('u_slack4')
    u_slack5 = SX.sym('u_slack5')
    u_slack6 = SX.sym('u_slack6')
    u_slack7 = SX.sym('u_slack7')
    u_slack8 = SX.sym('u_slack8')
    u = vertcat(omega, a, u_slack1, u_slack2, u_slack3, u_slack4, \
                u_slack5, u_slack6, u_slack7, u_slack8)

    # xdot
    x_dot       = SX.sym('x_dot')
    y_dot       = SX.sym('y_dot')
    phi_dot     = SX.sym('phi_dot')
    delta_dot   = SX.sym('delta_dot')
    v_dot       = SX.sym('v_dot')

    xdot = vertcat(x_dot, y_dot, phi_dot, delta_dot, v_dot)

    # dynamics
    cos_phi = cos(phi)
    sin_phi = sin(phi)
    tan_delta = sin(delta) / cos(delta)
    f_expl = vertcat(v*cos_phi,
                     v*sin_phi,
                     v*tan_delta/L,
                     omega,
                     a + u_slack1 - u_slack1 + u_slack2 - u_slack2 \
                        + u_slack3 -u_slack3 +u_slack4 - u_slack4 \
                        + u_slack5 - u_slack5 + u_slack6 - u_slack6 \
                        + u_slack7 - u_slack7 + u_slack8 - u_slack8)

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name

    return model


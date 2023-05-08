from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, tan


def export_vehicle_model() -> AcadosModel:
    model_name = "path_smoother"

    # set up states & controls
    x = SX.sym("x")
    y = SX.sym("y")
    phi = SX.sym("phi")
    delta = SX.sym("delta")
    v = SX.sym("v")
    stateVec = vertcat(x, y, phi, delta, v)

    acceleration = SX.sym("acceleration")
    wheelAngleRate = SX.sym("wheelAngleRate")
    slackLR = SX.sym("slackLR")
    slackFR = SX.sym("slackFR")
    controlVec = vertcat(acceleration, wheelAngleRate, slackLR, slackFR)

    # xdot
    x_dot = SX.sym("x_dot")
    y_dot = SX.sym("y_dot")
    phi_dot = SX.sym("phi_dot")
    delta_dot = SX.sym("delta_dot")
    v_dot = SX.sym("v_dot")
    stateDotVec = vertcat(x_dot, y_dot, phi_dot, delta_dot, v_dot)

    # parameters
    wheelBase = SX.sym("wheelBase")
    p = vertcat(wheelBase)

    # dynamics 
    f_expl = vertcat(v * cos(phi), 
                     v * sin(phi), 
                     v * tan(delta) / wheelBase,
                     wheelAngleRate, 
                     acceleration + slackLR - slackLR + slackFR - slackFR)

    f_impl = stateDotVec - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = stateVec
    model.xdot = stateDotVec
    model.u = controlVec
    model.p = p
    model.name = model_name

    return model

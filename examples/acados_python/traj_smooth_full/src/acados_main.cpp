#include "acados_main.h"

namespace acados_main {

path_smoother_solver_capsule * create_ocp_solver_description() {
  path_smoother_solver_capsule *acados_ocp_capsule = path_smoother_acados_create_capsule();
  /* set weights and so on by call
   * path_smoother_acados_create_with_discretization. This function does almost
   * everything needed for problem shaping */
  // there is an opportunity to change the number of shooting intervals in C
  // without new code generation
  int N = PATH_SMOOTHER_N;
  // allocate the array and fill it accordingly
  double *new_time_steps = NULL;
  int status = path_smoother_acados_create_with_discretization(
      acados_ocp_capsule, N, new_time_steps);

  if (status) {
    printf("path_smoother_acados_create() returned status %d. Exiting.\n",
             status);
    exit(1);
  }
  return acados_ocp_capsule;
}

Eigen::MatrixXd closed_loop_simulation(const AcadosParams &params,
    path_smoother_solver_capsule *acados_ocp_capsule,
    const Eigen::VectorXd &x,
    const Eigen::VectorXd &y, const Eigen::VectorXd &phi,
    const Eigen::VectorXd &delta, const Eigen::VectorXd &v,
    const Eigen::MatrixXd &collision_coeff, std::string &debug_str) {
    std::vector<std::vector<double>> collisionCoeff;
    std::vector<double> tmp_row(10);
    for (int i = 0; i < collision_coeff.rows(); i++) {
        for (int j = 0; j < collision_coeff.cols(); j++) {
            tmp_row.at(j) = collision_coeff(i, j);
        }
        collisionCoeff.push_back(tmp_row);
    }
    
    int N = PATH_SMOOTHER_N;    // 100
    int status = 0;

    // 同上，先创建容器，后续填入数据
    ocp_nlp_config *nlp_config = path_smoother_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = path_smoother_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = path_smoother_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = path_smoother_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = path_smoother_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = path_smoother_acados_get_nlp_opts(acados_ocp_capsule);

    double delta_max = params.delta_max;
    double v_max = params.v_max;
    double slackLR_max = params.slackLR_max;
    double slackFR_max = params.slackFR_max;

    // initial condition
    double lbx0[NBX0];
    double ubx0[NBX0];
    lbx0[0] = x[0];
    ubx0[0] = x[0];
    lbx0[1] = y[0];
    ubx0[1] = y[0];
    lbx0[2] = phi[0];
    ubx0[2] = phi[0];
    lbx0[3] = 0;
    ubx0[3] = 0;
    lbx0[4] = 0;
    ubx0[4] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // initialization for state values
    double x_init[NX];
    x_init[0] = x[0];
    x_init[1] = y[0];
    x_init[2] = phi[0];
    x_init[3] = 0.0;
    x_init[4] = 0.0;    //X0

    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = slackLR_max;    // slackLR_max
    u0[3] = slackFR_max;    // slackFR_max
    // set parameters
    double p[NP];
    p[0] = params.wheelBase;

    for (int ii = 0; ii <= N; ii++)
    {
        path_smoother_acados_update_params(acados_ocp_capsule, ii, p, NP);
    }
  

    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)];
    double utraj[NU * N];


    // solve ocp in loop
    int rti_phase = 0;
    double lubx[NX];

    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        for (int i = 0; i < N; i++)
        {
            x_init[0] = x[i];
            x_init[1] = y[i];
            x_init[2] = phi[i];
            x_init[3] = delta[i];
            x_init[4] = v[i];                                                  
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);     // set X[i]
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);         // set U[i]
        }
        x_init[0] = x[N];
        x_init[1] = y[N];
        x_init[2] = phi[N];
        x_init[3] = delta[N];
        x_init[4] = v[N];
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);

        /**** Constraints ****/

        // bounds for initial stage
        // x0
        double* lubx0 = static_cast<double *>(calloc(2 * NBX0, sizeof(double)));
        double* lbx0 = lubx0;
        lbx0[0] = x[0];
        lbx0[1] = y[0];
        lbx0[2] = phi[0];
        double* ubx0 = lubx0 + NBX0;
        ubx0[0] = x[0];
        ubx0[1] = y[0];
        ubx0[2] = phi[0];
        // change only the non-zero elements:
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
        free(lubx0);

        // x
        double* lubx = static_cast<double *>(calloc(2 * NBX, sizeof(double)));
        double* lbx = lubx;
        double* ubx = lubx + NBX;

        for (int i = 1; i < N; i++)
        {   
            if(v[i] > 0.5) {
                lbx[0] = -delta_max;
                ubx[0] = delta_max;
                lbx[1] = 0.0;
                ubx[1] = v_max;
            } else if(v[i] < -0.5) {
                lbx[0] = -delta_max;
                ubx[0] = delta_max;
                lbx[1] = -v_max;
                ubx[1] = 0.0;
            } else {
                lbx[0] = 0.0;
                ubx[0] = 0.0;
                lbx[1] = -0.1;
                ubx[1] = 0.1;
            }
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
        }
        free(lubx);

        /* constraints that are the same for initial and intermediate */
        // u
        double* lubu = static_cast<double *>(calloc(2 * NBU, sizeof(double)));
        double* lbu = lubu;
        double* ubu = lubu + NBU;

        const double acceleration_max = params.acceleration_max;
        const double wheelAngleRate_max = params.wheelAngleRate_max;
        lbu[0] = -acceleration_max;
        ubu[0] = acceleration_max;
        lbu[1] = -wheelAngleRate_max;
        ubu[1] = wheelAngleRate_max;
        ubu[2] = slackLR_max;
        ubu[3] = slackFR_max;

        for (int i = 0; i < N; i++)
        {
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
        }
        free(lubu);

        // set up general constraints for stage 0 to N-1
        double* D = static_cast<double *>(calloc(NG * NU, sizeof(double)));
        double* C = static_cast<double *>(calloc(NG * NX, sizeof(double)));
        double* lug = static_cast<double *>(calloc(2 * NG, sizeof(double)));
        double* lg = lug;
        double* ug = lug + NG;


        const double NUM_MIN = -1e10;
        lg[0] = NUM_MIN;
        lg[1] = NUM_MIN;
        lg[2] = NUM_MIN;
        lg[3] = NUM_MIN;
        lg[4] = NUM_MIN;
        lg[5] = NUM_MIN;
        lg[6] = NUM_MIN;
        lg[7] = NUM_MIN;

        for (int i = 0; i < N; i++)
        {   
            D[0] = collisionCoeff[8*i + 0][5];
            D[1] = collisionCoeff[8*i + 1][5];
            D[2] = collisionCoeff[8*i + 2][5];
            D[3] = collisionCoeff[8*i + 3][5];
            D[4] = collisionCoeff[8*i + 4][5];
            D[5] = collisionCoeff[8*i + 5][5];
            D[6] = collisionCoeff[8*i + 6][5];
            D[7] = collisionCoeff[8*i + 7][5];
            D[8] = collisionCoeff[8*i + 0][6];
            D[9] = collisionCoeff[8*i + 1][6];
            D[10] = collisionCoeff[8*i + 2][6];
            D[11] = collisionCoeff[8*i + 3][6];
            D[12] = collisionCoeff[8*i + 4][6];
            D[13] = collisionCoeff[8*i + 5][6];
            D[14] = collisionCoeff[8*i + 6][6];
            D[15] = collisionCoeff[8*i + 7][6];
            D[16] = collisionCoeff[8*i + 0][7];
            D[17] = collisionCoeff[8*i + 1][7];
            D[18] = collisionCoeff[8*i + 2][7];
            D[19] = collisionCoeff[8*i + 3][7];
            D[20] = collisionCoeff[8*i + 4][7];
            D[21] = collisionCoeff[8*i + 5][7];
            D[22] = collisionCoeff[8*i + 6][7];
            D[23] = collisionCoeff[8*i + 7][7];
            D[24] = collisionCoeff[8*i + 0][8];
            D[25] = collisionCoeff[8*i + 1][8];
            D[26] = collisionCoeff[8*i + 2][8];
            D[27] = collisionCoeff[8*i + 3][8];
            D[28] = collisionCoeff[8*i + 4][8];
            D[29] = collisionCoeff[8*i + 5][8];
            D[30] = collisionCoeff[8*i + 6][8];
            D[31] = collisionCoeff[8*i + 7][8];
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "D", D);

            C[0] = collisionCoeff[8*i + 0][0];
            C[1] = collisionCoeff[8*i + 1][0];
            C[2] = collisionCoeff[8*i + 2][0];
            C[3] = collisionCoeff[8*i + 3][0];
            C[4] = collisionCoeff[8*i + 4][0];
            C[5] = collisionCoeff[8*i + 5][0];
            C[6] = collisionCoeff[8*i + 6][0];
            C[7] = collisionCoeff[8*i + 7][0];
            C[8] = collisionCoeff[8*i + 0][1];
            C[9] = collisionCoeff[8*i + 1][1];
            C[10] = collisionCoeff[8*i + 2][1];
            C[11] = collisionCoeff[8*i + 3][1];
            C[12] = collisionCoeff[8*i + 4][1];
            C[13] = collisionCoeff[8*i + 5][1];
            C[14] = collisionCoeff[8*i + 6][1];
            C[15] = collisionCoeff[8*i + 7][1];
            C[16] = collisionCoeff[8*i + 0][2];
            C[17] = collisionCoeff[8*i + 1][2];
            C[18] = collisionCoeff[8*i + 2][2];
            C[19] = collisionCoeff[8*i + 3][2];
            C[20] = collisionCoeff[8*i + 4][2];
            C[21] = collisionCoeff[8*i + 5][2];
            C[22] = collisionCoeff[8*i + 6][2];
            C[23] = collisionCoeff[8*i + 7][2];
            C[24] = collisionCoeff[8*i + 0][3];
            C[25] = collisionCoeff[8*i + 1][3];
            C[26] = collisionCoeff[8*i + 2][3];
            C[27] = collisionCoeff[8*i + 3][3];
            C[28] = collisionCoeff[8*i + 4][3];
            C[29] = collisionCoeff[8*i + 5][3];
            C[30] = collisionCoeff[8*i + 6][3];
            C[31] = collisionCoeff[8*i + 7][3];
            C[32] = collisionCoeff[8*i + 0][4];
            C[33] = collisionCoeff[8*i + 1][4];
            C[34] = collisionCoeff[8*i + 2][4];
            C[35] = collisionCoeff[8*i + 3][4];
            C[36] = collisionCoeff[8*i + 4][4];
            C[37] = collisionCoeff[8*i + 5][4];
            C[38] = collisionCoeff[8*i + 6][4];
            C[39] = collisionCoeff[8*i + 7][4];
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "C", C);

            ug[0] = collisionCoeff[8*i + 0][9];
            ug[1] = collisionCoeff[8*i + 1][9];
            ug[2] = collisionCoeff[8*i + 2][9];
            ug[3] = collisionCoeff[8*i + 3][9];
            ug[4] = collisionCoeff[8*i + 4][9];
            ug[5] = collisionCoeff[8*i + 5][9];
            ug[6] = collisionCoeff[8*i + 6][9];
            ug[7] = collisionCoeff[8*i + 7][9];
            
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lg", lg);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ug", ug);
        }

        /* 5. terminal constraints */
        double lbx_e[NBX0];
        double ubx_e[NBX0];
        lbx_e[0] = x(PATH_SMOOTHER_N);
        ubx_e[0] = x(PATH_SMOOTHER_N);
        lbx_e[1] = y(PATH_SMOOTHER_N);
        ubx_e[1] = y(PATH_SMOOTHER_N);
        lbx_e[2] = phi(PATH_SMOOTHER_N);
        ubx_e[2] = phi(PATH_SMOOTHER_N);
        lbx_e[3] = 0.0;
        ubx_e[3] = 0.0;
        lbx_e[4] = 0.0;
        ubx_e[4] = 0.0;

        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, PATH_SMOOTHER_N,
                                        "lbx", lbx_e);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, PATH_SMOOTHER_N,
                                        "ubx", ubx_e);

        // ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = path_smoother_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }
    debug_str = "elapsed time: ";
    debug_str += std::to_string(min_time);
    debug_str += "\n";
    debug_str += "acados returned status " + std::to_string(status) + "\n";

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( NX, N+1, xtraj, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( NU, N, utraj, NU );
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);
    Eigen::MatrixXd traj_x(NX + NU, N + 1);
    // transform xtraj to traj_x
    for (int i = 0; i < NX; ++i) {
        for (int j = 0; j < N + 1; ++j) {
            traj_x(i, j) = xtraj[i + j * NX];
        }
    }
    // transform utraj to traj_x
    for (int i = 0; i < NU; ++i) {
        for (int j = 0; j < N; ++j) {
            traj_x(i + NX, j) = utraj[i + j * NU];
        }
    }

    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("path_smoother_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("path_smoother_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    path_smoother_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

    // free solver
    status = path_smoother_acados_free(acados_ocp_capsule);
    if (status) {
        printf("path_smoother_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = path_smoother_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("path_smoother_acados_free_capsule() returned status %d. \n", status);
    }

    return traj_x;
}

} // namespace acados_main
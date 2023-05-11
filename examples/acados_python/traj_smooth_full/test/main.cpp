#include "acados_main.h"
#include "data.h"
#include "collision_coeff.h"
#include <fstream>
#include <iostream>
#include <dirent.h>

using namespace acados_main;
using json = nlohmann::json;

AcadosParams acados_params;

path_smoother_solver_capsule *acados_ocp_capsule = create_ocp_solver_description();

void RecordToFile(std::string file_name, const Eigen::VectorXd &x_ref, const Eigen::VectorXd &y_ref) {
    std::ofstream ofs(file_name);
    size_t min_size = std::min(x_ref.size(), y_ref.size());
    for (size_t i = 0; i < min_size; ++i) {
        ofs << x_ref(i) << " " << y_ref(i) << std::endl;
    }
    ofs.close();
}

int plan(const std::string &file_name)
{
    // preprare input stats
    Eigen::VectorXd x, y, phi, delta, v;
    std::vector<double> left_bound, right_bound, front_bound, back_bound;

    TrajectoryData data = loadData(file_name);
    
    // transfrom vector to Eigen::VectorXd
    x = Eigen::Map<Eigen::VectorXd>(data.x.data(), data.x.size());
    y = Eigen::Map<Eigen::VectorXd>(data.y.data(), data.y.size());
    phi = Eigen::Map<Eigen::VectorXd>(data.phi.data(), data.phi.size());
    delta = Eigen::Map<Eigen::VectorXd>(data.delta.data(), data.delta.size());
    v = Eigen::Map<Eigen::VectorXd>(data.v.data(), data.v.size());
    left_bound = data.left_bound;
    right_bound = data.right_bound;
    front_bound = data.front_bound;
    back_bound = data.back_bound;

    Eigen::MatrixXd collision_coeff_mat = GetCollisionCoeff(
        acados_params, x, y, phi, v, left_bound, right_bound, front_bound, back_bound);
    std::string debug_str;
    Eigen::MatrixXd traj_x = closed_loop_simulation(acados_params, acados_ocp_capsule, x, y, phi, delta, v, collision_coeff_mat, debug_str);

    // dump infomation to file
    RecordToFile(file_name + std::string("_ref.data"), x, y);
    RecordToFile(file_name + std::string(".data"), traj_x.row(0), traj_x.row(1));
    // print the two lines above into file
    std::ofstream ofs;
    ofs.open(file_name + std::string(".txt"), std::ios::out);
    ofs << debug_str;
    ofs.close();
}

void getFiles(const std::string &directory, std::vector<std::string> &files) {
    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir = opendir(directory.c_str())) == NULL) {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL) {
        if (strcmp(ptr->d_name, ".") == 0 ||
            strcmp(ptr->d_name, "..") == 0) {    // current dir OR parrent dir
            continue;
        } else if (ptr->d_type == 8) {    // file
            sprintf(base, "%s/%s", directory.c_str(), ptr->d_name);
            files.push_back(base);
        } else if (ptr->d_type == 10) {    // link file
            continue;
        } else if (ptr->d_type == 4) {    // dir
            continue;
        }
    }
    closedir(dir);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <path_to_trajectory_data>" << std::endl;
        return -1;
    }

    std::string directory = argv[1];
    // traverse all files in the directory
    std::vector<std::string> files;
    getFiles(directory, files);
    // keep only .json files
    std::vector<std::string> json_files;
    for (auto file : files) {
        if (file.find(".json") != std::string::npos) {
            json_files.push_back(file);
        }
    }
    files = json_files;
    
    // plan for each file
    for (auto file : files) {
        std::cout << "Planning for file: " << file << std::endl;
        if (plan(file) != 0) {
            std::cout << "Failed to plan for file: " << file << std::endl;
            return -1;
        }
    }
    return 0;
}
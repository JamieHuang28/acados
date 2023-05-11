#include "data.h"
#include <fstream>
#include <iostream>

namespace acados_main
{

void smoothTheta(std::vector<double> &thetas) {
  if(thetas.size() < 2) {
    return;
  }
  size_t index = thetas.size() - 1;
  double change = 0;

  for (size_t i = 0; i < thetas.size() - 1; ++i) {
    change = 0;
    if(std::fabs(thetas[i+1] - thetas[i]) > M_PI) {
      change = thetas[i+1] - thetas[i] > 0 ? -1 : 1;
      index = i+1;
      break;
    }
  }
  for (size_t i = index; i < thetas.size(); ++i) {
    thetas[i] = thetas[i] + 2 * change * M_PI;
  }
}

TrajectoryData loadData(const std::string &file_name) {
  TrajectoryData data;
  std::ifstream fs;
  fs.open(file_name);
  if (!fs.is_open()) {
      std::cout << "Failed to open file: " << file_name << std::endl;
  }
  nlohmann::json j;
  fs >> j;
  fs.close();
  data = j;
  smoothTheta(data.phi);
  return data;
}

} // namespace acados_main

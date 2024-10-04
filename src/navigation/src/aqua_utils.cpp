#include "aqua_utils.h"

std::vector<double>
calculateFreeAcceleration(const std::vector<double> &accImu,
                          const tf2::Quaternion &q)  {
   
   tf2::Matrix3x3 rotation_matrix(q);
  // Vecteur de gravité dans le repère monde (0, 0, -g)
  double gravity_world[3] = {0, 0, 9.81};

  // Appliquer la rotation inverse (rotation_matrix transpose) pour avoir la
  // gravité dans le repère IMU
  double gravity_imu[3];
  for (int i = 0; i < 3; ++i) {
    gravity_imu[i] = 0;
    for (int j = 0; j < 3; ++j) {
      gravity_imu[i] += rotation_matrix[j][i] * gravity_world[j];
    }
  }

  // Calculer la "free acceleration" en soustrayant la gravité de
  // l'accélération mesurée
  std::vector<double> freeAcceleration(3, 0.0);
  for (int i = 0; i < 3; i++) {
    freeAcceleration[i] = accImu[i] - gravity_world[i];
  }

  return freeAcceleration;
}

std::vector<double> Quaternion2RPY(const tf2::Quaternion &q) {
  double roll, pitch, yaw;
  tf2::Matrix3x3 rotation_matrix(q);
  rotation_matrix.getRPY(roll, pitch, yaw);
  std::vector<double> euler{roll, pitch, yaw};
  return euler;
}

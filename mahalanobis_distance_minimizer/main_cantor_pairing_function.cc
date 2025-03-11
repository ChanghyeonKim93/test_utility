
uint64_t ComputeCantorPairingNumber(const double x, const double y,
                                    const double z, const double resolution) {
  CHECK_FUNCTION_TIME_FROM_THIS
  const int x_index = std::round(x / resolution);
  const int y_index = std::round(y / resolution);
  const int z_index = std::round(z / resolution);
  const auto x_key =
      static_cast<uint32_t>(x_index > 0 ? x_index * 2 : -(x_index * 2) + 1);
  const auto y_key =
      static_cast<uint32_t>(x_index > 0 ? y_index * 2 : -(y_index * 2) + 1);
  const auto z_key =
      static_cast<uint32_t>(x_index > 0 ? z_index * 2 : -(z_index * 2) + 1);
  const uint64_t xy_key = (x_key + y_key) * (x_key + y_key + 1) / 2 + y_key;
  const uint64_t xyz_key = (xy_key + z_key) * (xy_key + z_key + 1) / 2 + z_key;
  return xyz_key;
}

uint64_t ComputeCantorPairingNumber4(const Eigen::Vector3d& p,
                                     const double resolution) {
  CHECK_FUNCTION_TIME_FROM_THIS
  const Eigen::Vector3i pi = Eigen::Vector3d(p / resolution).cast<int>();

  const int x_index = pi.x();
  const int y_index = pi.y();
  const int z_index = pi.z();
  const auto x_key =
      static_cast<uint32_t>(x_index > 0 ? x_index * 2 : -(x_index * 2) + 1);
  const auto y_key =
      static_cast<uint32_t>(x_index > 0 ? y_index * 2 : -(y_index * 2) + 1);
  const auto z_key =
      static_cast<uint32_t>(x_index > 0 ? z_index * 2 : -(z_index * 2) + 1);
  const uint64_t xy_key = (x_key + y_key) * (x_key + y_key + 1) / 2 + y_key;
  const uint64_t xyz_key = (xy_key + z_key) * (xy_key + z_key + 1) / 2 + z_key;
  return xyz_key;
}

uint64_t ComputeCantorPairingNumber2(const double x, const double y,
                                     const double z) {
  static const double inv_resolution = 1.0 / 0.5;
  CHECK_FUNCTION_TIME_FROM_THIS
  const int x_index = static_cast<int>(x * inv_resolution);
  const int y_index = static_cast<int>(y * inv_resolution);
  const int z_index = static_cast<int>(z * inv_resolution);
  const auto x_key =
      static_cast<uint32_t>(x_index > 0 ? x_index << 1 : -(x_index * 2) + 1);
  const auto y_key =
      static_cast<uint32_t>(x_index > 0 ? y_index << 1 : -(y_index * 2) + 1);
  const auto z_key =
      static_cast<uint32_t>(x_index > 0 ? z_index << 1 : -(z_index * 2) + 1);
  const uint64_t xy_key = (x_key + y_key) * (x_key + y_key + 1) / 2 + y_key;
  const uint64_t xyz_key = (xy_key + z_key) * (xy_key + z_key + 1) / 2 + z_key;
  return xyz_key;
}
uint64_t ComputeCantorPairingNumber3(const double x, const double y,
                                     const double z) {
  static const double inv_resolution = 1.0 / 0.5;

  CHECK_FUNCTION_TIME_FROM_THIS
  const int x_index = static_cast<int>(x * inv_resolution);
  const int y_index = static_cast<int>(y * inv_resolution);
  const int z_index = static_cast<int>(z * inv_resolution);
  int x_sign = x_index & 0x80000000;
  int y_sign = x_index & 0x80000000;
  int z_sign = x_index & 0x80000000;
  const auto x_key =
      static_cast<uint32_t>((x_index << 1) | x_sign + x_sign >> 31);
  const auto y_key =
      static_cast<uint32_t>((y_index << 1) | y_sign + y_sign >> 31);
  const auto z_key =
      static_cast<uint32_t>((z_index << 1) | z_sign + z_sign >> 31);
  const uint64_t xy_key = ((x_key + y_key) * (x_key + y_key + 1)) >> 1 + y_key;
  const uint64_t xyz_key =
      ((xy_key + z_key) * (xy_key + z_key + 1)) >> 1 + z_key;
  return xyz_key;
}

#define STEERING_LUT_SIZE 21

static const float steering_wheel_deg[STEERING_LUT_SIZE] = {
  0.000, 5.250, 10.500, 15.750, 21.000,
  26.250, 31.500, 36.750, 42.000, 47.250,
  52.500, 57.750, 63.000, 68.250, 73.500,
  78.750, 84.000, 89.250, 94.500, 99.750,
  105.000
};

static const float steer_fl_deg_pos[STEERING_LUT_SIZE] = {
  0.0000, 0.9295, 1.8680, 2.8180, 3.7800,
  4.7540, 5.7410, 6.7420, 7.7590, 8.7920,
  9.8430, 10.9130, 12.0050, 13.1190, 14.2580,
  15.4250, 16.6220, 17.8540, 19.1230, 20.4360,
  21.7970
};

static const float steer_fr_deg_pos[STEERING_LUT_SIZE] = {
  0.0000, 0.9210, 1.8300, 2.7320, 3.6270,
  4.5140, 5.3940, 6.2670, 7.1340, 7.9950,
  8.8510, 9.7020, 10.5470, 11.3880, 12.2250,
  13.0580, 13.8870, 14.7120, 15.5340, 16.3530,
  17.1680
};

int find_segment(float x, const float *x_table, int size) {
  int low = 0;
  int high = size - 1;

  while (high - low > 1) {
    int mid = (low + high) >> 1;
    if (x_table[mid] > x)
      high = mid;
    else
      low = mid;
  }
  return low;  // segment [low, low+1]
}

float lut_1D_interp(float x, const float *x_table, const float *y_table, int size) {
  // ---- Below range ----
  if (x <= x_table[0]) {
    float dx = x_table[1] - x_table[0];
    float dy = y_table[1] - y_table[0];
    return y_table[0] + (x - x_table[0]) * (dy / dx);
  }

  // ---- Above range ----
  if (x >= x_table[size - 1]) {
    float dx = x_table[size - 1] - x_table[size - 2];
    float dy = y_table[size - 1] - y_table[size - 2];
    return y_table[size - 1] + (x - x_table[size - 1]) * (dy / dx);
  }

  // ---- Interpolation ----
  int i = find_segment(x, x_table, size);

  float x0 = x_table[i];
  float x1 = x_table[i + 1];
  float y0 = y_table[i];
  float y1 = y_table[i + 1];

  float t = (x - x0) / (x1 - x0);

  return y0 + t * (y1 - y0);
}

void steering_wheel_to_front_angles(float steering_wheel_deg_in,
                                    float *fl_deg,
                                    float *fr_deg) {
  float abs_sw = fabsf(steering_wheel_deg_in);

  float fl_pos = lut_1D_interp(abs_sw,
                               steering_wheel_deg,
                               steer_fl_deg_pos,
                               STEERING_LUT_SIZE);

  float fr_pos = lut_1D_interp(abs_sw,
                               steering_wheel_deg,
                               steer_fr_deg_pos,
                               STEERING_LUT_SIZE);

  if (steering_wheel_deg_in >= 0.0f) {
    // Positive CAN steering is a left turn: FL is inside and turns more.
    *fl_deg = fl_pos;
    *fr_deg = fr_pos;
  } else {
    // Negative CAN steering is a right turn: FR is inside and turns more.
    *fl_deg = -fr_pos;
    *fr_deg = -fl_pos;
  }
}

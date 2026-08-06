#define STEERING_LUT_SIZE 21

static const float steering_wheel_deg[STEERING_LUT_SIZE] = {
  0.000, 5.250, 10.500, 15.750, 21.000,
  26.250, 31.500, 36.750, 42.000, 47.250,
  52.500, 57.750, 63.000, 68.250, 73.500,
  78.750, 84.000, 89.250, 94.500, 99.750,
  105.000
};

static const float steer_fl_deg_pos[STEERING_LUT_SIZE] = {
  0.000, 0.901, 1.795, 2.683, 3.566,
  4.443, 5.315, 6.183, 7.047, 7.906,
  8.762, 9.614, 10.463, 11.309, 12.152,
  12.992, 13.830, 14.666, 15.500, 16.332,
  17.162
};

static const float steer_fr_deg_pos[STEERING_LUT_SIZE] = {
  0.000, 0.907, 1.822, 2.744, 3.674,
  4.613, 5.562, 6.520, 7.489, 8.469,
  9.461, 10.466, 11.485, 12.519, 13.569,
  14.637, 15.724, 16.831, 17.961, 19.115,
  20.296
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
    // Positive steering direction, table as-given
    *fl_deg = fl_pos;
    *fr_deg = fr_pos;
  } else {
    // Mirror for opposite steering direction
    *fl_deg = -fr_pos;
    *fr_deg = -fl_pos;
  }
}
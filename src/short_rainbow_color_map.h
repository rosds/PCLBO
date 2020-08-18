#pragma once

/** \brief Color code of a value with a short-raibow pallete
 *  \param[in] value The value to encode
 *  \param[in] min Minimum value in rangle which will be blue.
 *  \param[in] max Maximum value in rage which will be red.
 *  \return The RGB code inside a double
 */
float shortRainbowColorMap(const double value, const double min,
                           const double max) {
  uint8_t r, g, b;

  if (isnan(value)) {
    r = 255;
    g = 0;
    b = 0;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    return *reinterpret_cast<float *>(&rgb);
  }

  // Normalize value to [0, 1]
  double value_normalized = (value - min) / (max - min);

  double a = (1.0f - value_normalized) / 0.25f;
  int X = static_cast<int>(floor(a));
  int Y = static_cast<int>(floor(255.0f * (a - X)));

  switch (X) {
    case 0:
      r = 255;
      g = Y;
      b = 0;
      break;
    case 1:
      r = 255 - Y;
      g = 255;
      b = 0;
      break;
    case 2:
      r = 0;
      g = 255;
      b = Y;
      break;
    case 3:
      r = 0;
      g = 255 - Y;
      b = 255;
      break;
    case 4:
      r = 0;
      g = 0;
      b = 255;
      break;
  }

  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  return *reinterpret_cast<float *>(&rgb);
}

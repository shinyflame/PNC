
#include "../math/angle.h"
#include "../math/sin_table.h"

namespace math {

float sin(Angle16 a) {
  auto idx = a.raw();
  if (idx < -Angle16::RAW_PI_2) {
    idx += Angle16::RAW_PI;
    auto bb = Angle16::RAW_PI;
    return -SIN_TABLE[idx % SIN_TABLE_SIZE];
  }
  if (idx < 0) {
    return -SIN_TABLE[(-idx) % SIN_TABLE_SIZE];
  }
  if (idx < Angle16::RAW_PI_2) {
    return SIN_TABLE[idx % SIN_TABLE_SIZE];
  }
  idx = Angle16::RAW_PI - idx;
  return SIN_TABLE[idx % SIN_TABLE_SIZE];
}

float cos(Angle16 a) {
  Angle16 b(Angle16::RAW_PI_2 - a.raw());
  return sin(b);
}

float tan(Angle16 a) { return sin(a) / cos(a); }

float sin(Angle8 a) {
  Angle16 b(a.raw() << 8);
  return sin(b);
}

float cos(Angle8 a) {
  Angle16 b(a.raw() << 8);
  return cos(b);
}

float tan(Angle8 a) {
  Angle16 b(a.raw() << 8);
  return tan(b);
}

}  // namespace math

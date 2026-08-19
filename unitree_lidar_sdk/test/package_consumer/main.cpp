#include <unitree_lidar_sdk.h>

int main()
{
  // Linking this factory verifies that the selected architecture archive and
  // its transitive thread dependency came from the installed CMake package.
  return unilidar_sdk2::createUnitreeLidarReader() == nullptr ? 1 : 0;
}

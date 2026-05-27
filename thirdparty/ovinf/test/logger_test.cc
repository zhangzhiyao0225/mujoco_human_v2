#include "utils/csv_logger.hpp"

int main() {
  ovinf::CsvLogger logger("test.csv", {"a", "b", "c"});
  logger.Write({1.0, 2.0, 3.0});
  logger.Write({4.0, 5.0, 6.0});

  return 0;
}

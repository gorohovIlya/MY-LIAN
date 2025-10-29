#include <iostream>
using namespace std;

#include "LIAN.h"

int main()
{
  Map* map = new Map("testLIAN-3.txt");
  Point* start = new Point(1, 1);
  Point* end = new Point(25, 1);
  LIAN lian(30, 4, map, start, end, ANGLE_IN_DEGREES);
  bool pathFound = lian.findOptimalWay();
  if  (pathFound) {
    lian.drawBestWay();
  }
  cout << *map;
  delete start;
  delete end;
  delete map;
  return 0;
}
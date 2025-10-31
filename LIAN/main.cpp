#include <iostream>
using namespace std;

#include "LIAN.h"

int main(int argc, char** argv)
{
  if (argc < 3) {
    cout << "Launch ./program configFile.txt mapFile.txt" << endl;
    return -1;
  }
  LIAN lian(argv[1], argv[2]);
  
  bool pathFound = lian.findOptimalWay();

  if (pathFound) {
    char answer;
    cout << "Optimal way is found. Draw sections completely?" << endl << "[y/n] ";
    cin >> answer;
    if (answer == 'y') {
      lian.drawBestWay(DRAW_POINTS_BETWEEN);
    }
    else {
      lian.drawBestWay();
    }
  }
  else {
    cout << "Optimal way is not found. Here are results of optimal way searching:";
  }
  Map* LIANmap = lian.getMap();
  cout << endl << *LIANmap;
  // Map* map = new Map(argv[2]);
  // Point* start = new Point(1,10);
  // Point* end = new Point(34,15);
  // LIAN lian(30, 3, map, start, end, ANGLE_IN_DEGREES);
  // bool pathFound = lian.findOptimalWay();
  // if  (pathFound) {
  //   lian.drawBestWay(DRAW_POINTS_BETWEEN);
  //   map->clearTrash();
  // }
  // map->setElByPos(start->x, start->y, 'A');
  // map->setElByPos(end->x, end->y, 'B');
  // cout << *map;
  // delete start;
  // delete end;
  // delete map;
  return 0;
}
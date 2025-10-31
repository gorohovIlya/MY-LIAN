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
  return 0;
}

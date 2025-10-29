#ifndef _LIAN_H
#define _LIAN_H

#include <vector>
#include <set>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <cfloat>

const bool ANGLE_IN_DEGREES = true;
const int IS_LEFT_DIRECTION = -1;

class Map {
    char** data;
    int length;
    int width;
public:
    Map(const char* filename);
    ~Map();
    int getLength() const;
    int getWidth() const;
    char getElByPos(int x, int y) const;
    // Отладочная функция
    void setElByPos(int x, int y, char chr);
};


struct Point {

    int x;
    int y;
    double pWay;
    double euclid;
    Point* previous;

    Point(int xCoord, int yCoord, double euc = 0.0, Point* pt = nullptr);
            // x(xCoord), y(yCoord), pWay(0.0), euclid(euc), previous(pt) {};
    double distance(Point* other) const;
    // void setPassedWay();
    bool isAllowedByAngle(Point* next, double angle, bool inDegrees = false) const;
    bool isNotObstacle(Point* next, Map* map) const;
    std::vector<Point*> reachablePoints(int radius, int borderX, int borderY, Point* end);
};

struct PointComparator {
    bool operator()(Point* a, Point* b) const;
};

std::ostream& operator<<(std::ostream& os, const Map& obj);

struct Way {
    std::vector<Point>* way;
    double angleSum;
};

class LIAN {

    std::priority_queue<Point*, std::vector<Point*>, PointComparator> open;
    std::vector<Point*> closed;
    double maxAngle;
    bool isAngleInDegrees;
    int radius;
    Map* map;
    Point* start;
    Point* end;
    std::vector<Point*> bestWay;
    double bestWayLen = 1000000000;
public:
    LIAN(double maxAngle, int radius, Map* map, Point* start, Point* end, bool inDegrees = false);
    // void clearOpen();
    // void clearClosed();
    // void clearBestWay();
    ~LIAN();
    void addPointsToOpenIfPossible(Point* point);
    bool findOptimalWay();
    std::priority_queue<Point*, std::vector<Point*>, PointComparator> getOpen() const;
    void drawBestWay();
};

std::ostream& operator<<(std::ostream& os, const std::priority_queue<Point*, std::vector<Point*>, PointComparator>& pq);

#endif
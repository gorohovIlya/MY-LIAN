#ifndef _LIAN_H
#define _LIAN_H

#include <vector>
#include <set>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>

const bool ANGLE_IN_DEGREES = true;
const int IS_LEFT_DIRECTION = -1;
const bool DRAW_POINTS_BETWEEN = true;

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
    void setElByPos(int x, int y, char chr);
    void clearTrash();
};


struct Point {

    int x;
    int y;
    double pWay;
    double euclid;
    Point* previous;

    Point(int xCoord, int yCoord, double euc = 0.0, Point* pt = nullptr);
    double distance(Point* other) const;
    bool isAllowedByAngle(Point* next, double angle, bool inDegrees = false) const;
    bool isNotObstacle(Point* next, Map* map) const;
    bool isSectionAllowed(Point* next, Map* map) const;
    std::vector<Point*> reachablePoints(int radius, int borderX, int borderY, Point* end);
};

struct PointComparator {
    bool operator()(Point* a, Point* b) const;
};

std::ostream& operator<<(std::ostream& os, const Map& obj);

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
    double bestWayLen = 0;
    bool isEndPointReachable(Point* pt);
    void setBestWayByPoint(Point* pt);
public:
    LIAN(double maxAngle, int radius, Map* map, Point* start, Point* end, bool inDegrees = false);
    LIAN(const char* configurationFileName, const char* mapFileName);
    ~LIAN();
    void addPointsToOpenIfPossible(Point* point);
    bool findOptimalWay();
    std::priority_queue<Point*, std::vector<Point*>, PointComparator> getOpen() const;
    Map* getMap();
    void drawLineBetween(Point* first, Point* second);
    void drawBestWay(bool drawPointsBetween = false);
};

std::ostream& operator<<(std::ostream& os, const std::priority_queue<Point*, std::vector<Point*>, PointComparator>& pq);

#endif
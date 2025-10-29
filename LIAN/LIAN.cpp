#include "LIAN.h"


Map::Map(const char* filename) {
    std::ifstream in(filename);
    in >> length;
    in >> width;

    data = new char*[width];
    for (int i = 0; i < width; i++) {
        data[i] = new char[length];
    }
    
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < length; j++) {
            in >> data[i][j];
        }
    }
}

Map::~Map() {
    for (int i = 0; i < width; i++) {
        delete[] data[i];
    }
    delete[] data;
}

int Map::getWidth() const {
    return width;
}

int Map::getLength() const {
    return length;
}

char Map::getElByPos(int x, int y) const {
    return data[y][x];
}

void Map::setElByPos(int x, int y, char chr) {
    data[y][x] = chr;
}

std::ostream& operator<<(std::ostream& os, const Map& obj) {
    for (int i = 0; i < obj.getWidth(); i++) {
        for (int j = 0; j < obj.getLength(); j++) {
            os << obj.getElByPos(j, i);
        }
        os << std::endl;
    }
    return os;
}

Point::Point(int xCoord, int yCoord, double euc, Point* pt) {
    x = xCoord;
    y = yCoord;
    euclid = euc;
    previous = pt;
    pWay = 0;
}

double Point::distance(Point* other) const {
    int xx = other->x - this->x;
    int yy = other->y - this->y;
    double res = sqrt(xx * xx + yy * yy);
    return res;
}

bool Point::isAllowedByAngle(Point* next, double angle, bool inDegrees) const {

    Point* prev = this->previous;

    if (prev == nullptr) {return true;}

    double mod1 = distance(prev);
    if (!mod1) { return true; }
    double mod2 = distance(next);

    int xCoord1 = this->x - prev->x;
    int xCoord2 = next->x - this->x;
    int yCoord1 = this->y - prev->y;
    int yCoord2 = next->y - this->y;

    double cosa = (xCoord1 * xCoord2 + yCoord1 * yCoord2) / (mod1 * mod2);

    double ang = acos(cosa);

    if (inDegrees) {
        ang *= 180 / M_PI;
    }

    if (angle >= ang) {
        return true;
    } else {
        return false;
    }
}

bool Point::isNotObstacle(Point* next, Map* map) const {
    if (map->getElByPos(next->x, next->y) == '1') {
        return false;
    }
    int x0 = this->x;
    int y0 = this->y;
    int x1 = next->x;
    int y1 = next->y;
    
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        if (x0 != this->x || y0 != this->y) {
            if (map->getElByPos(x0, y0) == '1') {
                return false;
            }
        }
        
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    
    return true;
}

std::vector<Point*> Point::reachablePoints(int radius, int borderX, int borderY, Point* end) {
    std::vector<Point*> res;

    int xc = this->x;
    int yc = this->y;
    int m = borderX;
    int n = borderY;

    // double passedWay = 0.0;
    // Point* currPoint = this;
    // while (currPoint->previous != nullptr) {
    //     passedWay += currPoint->distance(currPoint->previous);
    //     currPoint = currPoint->previous;
    // }
    int x = 0; 
    int y = radius;
    int d = 3 - 2 * radius;

    while (x <= y) {
        int points[8][2] = {
            {xc + x, yc + y}, {xc - x, yc + y},
            {xc + x, yc - y}, {xc - x, yc - y},
            {xc + y, yc + x}, {xc - y, yc + x},
            {xc + y, yc - x}, {xc - y, yc - x}
        };

        for (int i = 0; i < 8; i++) {
            int px = points[i][0];
            int py = points[i][1];
            
            if (px >= 0 && px < m && py >= 0 && py < n) {
                Point* pt = new Point(px, py, 0.0, this);
                res.push_back(pt);
            }
        }
        x++;
        if (d < 0) {
            d = d + 4 * x + 6;
        } else {
            y--;
            d = d + 4 * (x - y) + 10;
        }
    }

    return res;
}

bool PointComparator::operator()(Point* a, Point* b) const {
        return (a->pWay + a->euclid) > (b->pWay + b->euclid);
    }

LIAN::LIAN(double maxAngle, int radius, Map* map, Point* start, Point* end, bool inDegrees) {
    this->maxAngle = maxAngle;
    this->radius = radius;
    this->start = start;
    this->end = end;
    this->map = map;
    this->isAngleInDegrees = inDegrees;
}

LIAN::~LIAN() {
        std::set<Point*> allPoints;
        while (!open.empty()) {
            allPoints.insert(open.top());
            open.pop();
        }
        for (Point* pt : closed) {
            allPoints.insert(pt);
        }
        for (Point* pt : bestWay) {
            allPoints.insert(pt);
        }
        for (Point* pt : allPoints) {
            if (pt != start && pt != end) {
                delete pt;
            }
        }
    }

std::priority_queue<Point*, std::vector<Point*>, PointComparator> LIAN::getOpen() const {
    return open;
}

void LIAN::addPointsToOpenIfPossible(Point* point) {

    map->setElByPos(point->x, point->y, '3');

    std::vector<Point*> reachablePts = point->reachablePoints(radius, map->getLength(), map->getWidth(), end);

    for (Point* pt : reachablePts) {

        bool alreadyInClosed = false;
        for (Point* closedPt : closed) {
            if (closedPt->x == pt->x && closedPt->y == pt->y) {
            alreadyInClosed = true;
            break;
            }
        }
        
        if (alreadyInClosed) {
            delete pt;
            continue;
        }

        char mapChar = map->getElByPos(pt->x, pt->y);

        if (mapChar == '2' || mapChar == '3') {
            delete pt;
            continue;
        }

        if (point->isAllowedByAngle(pt, maxAngle, isAngleInDegrees) && point->isNotObstacle(pt, map)) {
            pt->euclid = pt->distance(end);
            if (pt->previous != nullptr) {
                pt->pWay = point->pWay + pt->distance(point);
            }
            open.push(pt);
            map->setElByPos(pt->x, pt->y, '2');
        }
        else {
            delete pt; 
        }
    }
}

bool LIAN::findOptimalWay() {
    start->euclid = start->distance(end);
    open.push(start);
    
    while (!open.empty()) {
        Point* a = open.top();
        open.pop();
        
        if (a->x == end->x && a->y == end->y) {
            Point* current = a;
            while (current != nullptr) {
                bestWay.push_back(current);
                current = current->previous;
            }
            bestWayLen = a->pWay;
            // clearOpen();
            // clearClosed();
            return true;
        }
        
        closed.push_back(a);
        addPointsToOpenIfPossible(a);
    }
    return false;
}

// void LIAN::clearOpen() {
//     while (!open.empty()) {
//         if (open.top() != start && open.top() != end) {
//             delete open.top();
//         }
//         open.pop();
//     }
// }

// void LIAN::clearClosed() {
//     for (Point* pt : closed) {
//         if (pt != start && pt != end) {
//             delete pt;
//         }
//     }
// }

// void LIAN::clearBestWay() {
//     for (Point* pt : bestWay) {
//         if (pt != start && pt != end) {
//             delete pt;
//         }
//     }
// }

void LIAN::drawBestWay() {
    for (Point* pt : bestWay) {
        map->setElByPos(pt->x, pt->y, '*');
    }
}

std::ostream& operator<<(std::ostream& os, const std::priority_queue<Point*, std::vector<Point*>, PointComparator>& pq) {
    std::priority_queue<Point*, std::vector<Point*>, PointComparator> open = pq;
    while (!open.empty()) {
    Point* curr = open.top();
    double e = curr->euclid;
    double p = curr->pWay;
    double priority = e + p;
    os << "Point (" << curr->x << ", " << curr->y << "), Euclid: " << e 
                      << ", Passed way: " << p << ", Priority: " << priority << std::endl;
    open.pop();
  }
  return os;
}


#ifndef EDGE_H
#define EDGE_H
#include "point.h"

struct Edge
{
private:
    static int n;
public:
    Point p1;
    Point p2;
    int tag;

    //Edge(float p1x, float p1y, float p2x, float p2y);

    Edge(Point pp1, Point pp2);
    //Edge(Point, Point, int);

    bool operator==(const Edge& e) const;
    bool operator<(const Edge& e) const;

};

#endif // EDGE_H

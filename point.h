#ifndef POINT_H
#define POINT_H


struct Point
{
    float x;
    float y;

    Point(float fx, float fy);


    bool operator==(const Point& p) const;
    bool operator<(const Point& p) const;
};

#endif // POINT_H

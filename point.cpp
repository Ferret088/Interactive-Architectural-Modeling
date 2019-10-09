#include "point.h"


Point::Point(float fx, float fy)
    :x(fx), y(fy){}



bool Point::operator==(const Point& p) const
{
    return x == p.x && y == p.y;
}

bool Point::operator<(const Point& p) const
{
    if(x != p.x)
    {
        return x < p.x;
    } else
    {
        return y < p.y;
    }
}

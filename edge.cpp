#include "edge.h"

/*Edge::Edge(float p1x, float p1y, float p2x, float p2y)
    :p1(p1x,p1y), p2(p2x,p2y), tag(0) {}
*/
Edge::Edge(Point pp1, Point pp2)
    :p1(pp1), p2(pp2), tag(0){}


bool Edge::operator==(const Edge& e) const
{
    return p1==e.p1 && p2 ==e.p2;
}

bool Edge::operator<(const Edge& e) const
{
    if(!(p1 == e.p1))
    {
        return p1 < e.p1;
    } else
    {
        return p2 < e.p2;
    }
}

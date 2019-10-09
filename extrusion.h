#ifndef EXTRUSION_H
#define EXTRUSION_H


#include "plandata.h"

namespace Extrsn {
struct Event
{
    int type;
    Point_3 eventPoint;
    //for intersection event
    PlanData::Edge_handle cornerPrevEdge, cornerNextEdge, edge;

    //for direction event
    double newAngle;
    int profileTag;
    //needed by horizontal direction event
    double distance;
    bool towardInside;

    //for profile offset events
    double dist_inside, dist_outside;
    std::vector<Point> profile_inside, profile_outside;
/*
    bool operator<(const Event& evnt) const
    {
        return eventPoint.z() > evnt.eventPoint.z();
    }
*/
};



}


extern void debug(PlanData::Edge_handle e);

//Handles building model generation
class Extrusion
{
private:
#define EVENT_TYPE_GENERALIZED_INTERSECTION 1
#define EVENT_TYPE_EDGE_DIRECTION_NON_HORIZONTAL 2
#define EVENT_TYPE_PROFILE_OFFSET 3
#define EVENT_TYPE_ANCHOR 4



public:
    Extrusion();
    std::vector<Polygon_2> triangulate(std::vector<Polygon_2> pgns);
    void build_polyhedron(Editor& floor, Editor& profile, MyPolyhedron&);
};

class compare_event
{
public:
    bool operator()(const Event &e1,const Event &e2)
    {
        /*
        if(fabs(e1.eventPoint.z() -e2.eventPoint.z()) > 1e-4)
        {
            return e1.eventPoint.z() > e2.eventPoint.z();
        }
        else
            return e1.type > e2.type;*/
        if( e1.eventPoint.z() !=  e2.eventPoint.z())
        {
            return e1.eventPoint.z() > e2.eventPoint.z();
        }else
            return e1.type > e2.type;
    }
};

class compare_event_intersect
{
private:
    bool my_cmp(const Plane_3 &p1, const Plane_3 &p2)
    {
        if(p1.a() != p2.a())
        {
            return p1.a() < p2.a();
        }else if(p1.b() != p2.b())
        {
            return p1.b() < p2.b();
        }else if(p1.c() != p2.c())
        {
            return p1.c() < p2.c();
        }else
        {
            return p1.d() < p2.d();
        }
    }
    void my_sort(Plane_3 a[3])
    {
        for(int i=0; i<3; ++i)
        {
            for(int j=i+1; j<3; ++j)
            {
                if(!my_cmp(a[i], a[j]))
                {
                    Plane_3 tmp = a[i];
                    a[i] = a[j];
                    a[j] = tmp;
                }

            }

        }
    }

public:

    bool operator()(const Event &e1,const Event &e2)
    {


        assert(e1.type == EVENT_TYPE_GENERALIZED_INTERSECTION);
        assert(e2.type == EVENT_TYPE_GENERALIZED_INTERSECTION);

        Plane_3 e1p [] = {e1.cornerPrevEdge->directionPlane, e1.cornerNextEdge->directionPlane, e1.edge->directionPlane};
        Plane_3 e2p [] = {e2.cornerPrevEdge->directionPlane, e2.cornerNextEdge->directionPlane, e2.edge->directionPlane};

        my_sort(e1p);
        my_sort(e2p);

        for(int i=0; i<3; ++i)
        {
            if(my_cmp(e1p[i], e2p[i]) || my_cmp(e2p[i], e1p[i]))
            {
                return my_cmp(e1p[i], e2p[i]);
            }
        }
        return false;

        /*int i,j;
        bool allmatched = true;
        if(e1.eventPoint == e2.eventPoint)
        {
            printf("%%%%%%%%\n");
            debug(e1.cornerNextEdge);
            debug(e1.cornerPrevEdge);
            debug(e1.edge);
            printf("|||||||||||\n");
            debug(e2.cornerNextEdge);
            debug(e2.cornerPrevEdge);
            debug(e2.edge);
            printf("%%%%%%%%\n");
        }
        for(i=0; i<3; ++i)
        {
            bool matched = false;
            for(j=0; j<3; ++j)
            {
                if(e1p[i] == e2p[j])
                {
                    matched = true;
                    break;
                }
            }
            if(!matched)
            {
                allmatched = false;
                break;
            }
        }
        if(allmatched)
        {
            return false;
        }


        if(e1.cornerPrevEdge->directionPlane != e2.cornerPrevEdge->directionPlane)
        {
            return mycmp(e1.cornerPrevEdge->directionPlane, e2.cornerPrevEdge->directionPlane);
        }else if(e1.cornerNextEdge->directionPlane != e2.cornerNextEdge->directionPlane)
        {
            return mycmp(e1.cornerNextEdge->directionPlane, e2.cornerNextEdge->directionPlane);
        }else
        {
            return mycmp(e1.edge->directionPlane, e2.edge->directionPlane);
        }*/
    }
};

extern Extrusion extrusion;
#endif

#include "plandata.h"
#include <math.h>
#include "GLUT/glut.h"
#include <CGAL/Aff_transformation_3.h>

// -----------------------------------
//             PlanEdge
// -----------------------------------
namespace Extrsn {
    int PlanEdge::n = 0;
    int Vertex::n = 0;
}

using namespace std;
// -----------------------------------
//             PlanData
// -----------------------------------
#define PI 3.14159265
double get_angle_from_profile_point(Point p1, Point p2)
{
    return atan2(p2.x-p1.x, p2.y-p1.y) * 180/PI;
}

//use OPENGL to caculate the matrix and then pass the matrix to CGAL to get the rotated plane
Plane_3 get_plane_from_edge_and_angle(Point_3 p1, Point_3 p2, double angle)
{
    Vector_3 v = p2 - p1;
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();


    glTranslatef(p1.x(), p1.y(), p1.z());
    glRotatef(90-angle, v.x(), v.y(), v.z() );
    glTranslatef(-p1.x(), -p1.y(), -p1.z());

    float m[16];
    glGetFloatv (GL_MODELVIEW_MATRIX, m);
    glPopMatrix();


    CGAL::Aff_transformation_3<K> rotate(m[0], m[4], m[8], m[12], m[1], m[5], m[9], m[13],
            m[2], m[6], m[10], m[14], m[15]);
    //CGAL::Aff_transformation_3<K> rotate(1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1);
    Vector_3 v1 = Vector_3(-v.y(), v.x(), v.z());
    Point_3 myPoint = p2 + v1;
    //printf("\nmyPoint: %lf, %lf, %lf\n", UNFOLD(myPoint));
    myPoint = rotate(myPoint);

    //printf("p1:%lf, %lf, %lf; p2:%lf, %lf, %lf; myPoint:%lf, %lf, %lf\n\n", UNFOLD(p1), UNFOLD(p2), UNFOLD(myPoint));
    return Plane_3(p1, p2, myPoint);
}

void add_prev_and_next_planes(PlanData &pd)
{
    for(PlanData::SLAV_iterator SLAVIter = pd.SLAV_begin(); SLAVIter != pd.SLAV_end(); ++SLAVIter)
    {
        for(list<Vertex>::iterator vIter = SLAVIter->begin(); vIter != SLAVIter->end(); ++vIter)
        {
            vIter->nextEdgeHndl->prevDirectionPlane = vIter->prevEdgeHndl->directionPlane;
            vIter->nextEdgeHndl->nextDirectionPlane = PlanData::next(SLAVIter, vIter)->nextEdgeHndl->directionPlane;
        }
    }
}

void PlanData::do_construction(Editor &floor, Editor &profile, double startHeight)
{
    std::vector<Polygon_2> pgns = floor.get_polygons();
    for(std::vector<Polygon_2>::iterator pIter = pgns.begin(); pIter != pgns.end(); ++pIter)
    {
        vertices.push_back(std::list<Vertex>());
        SLAV_iterator pLAV = vertices.end();
        --pLAV;
        Vertex_handle lastVh, currVh, firstVh;
        for(Polygon_2::Vertex_iterator vIter = pIter->vertices_begin(); vIter != pIter->vertices_end(); ++vIter)
        {
            Vertex v;
            v.point = Point_3(vIter->x(), vIter->y(), startHeight);

            pLAV->push_back(v);
            currVh = pLAV->end();
            --currVh;


            if(vIter != pIter->vertices_begin())
            {
                PlanEdge planEdge;
                planEdge.p1 = lastVh->point;
                planEdge.p2 = currVh->point;

                planEdge.vh1 = lastVh;
                planEdge.vh2 = currVh;
                bind_profile(planEdge, floor, profile);
                edges.push_back(planEdge);

                lastVh->nextEdgeHndl = edges.end();
                --lastVh->nextEdgeHndl;

                currVh->prevEdgeHndl = edges.end();
                --currVh->prevEdgeHndl;


            } else
            {
                firstVh = currVh;
            }
            lastVh = currVh;

        }


        PlanEdge planEdge;
        planEdge.p1 = lastVh->point;
        planEdge.p2 = firstVh->point;
        planEdge.vh1 = lastVh;
        planEdge.vh2 = firstVh;
        bind_profile(planEdge, floor, profile);
        edges.push_back(planEdge);
        lastVh->nextEdgeHndl = edges.end();
        --lastVh->nextEdgeHndl;
        firstVh->prevEdgeHndl = edges.end();
        --firstVh->prevEdgeHndl;

    }
    {//add aditional information for removing invalid event in extrusion algorithm
        add_prev_and_next_planes(*this);
    }

}

//construct the initial active plan
PlanData::PlanData(Editor& floor, Editor& profile): floor(floor), profile(profile)
{
    //this->floor = floor;
    //this->profile = profile;
    double startHeight = profile.get_start_height();
    if(profile.get_profiles().size() > 0)
    {
        do_construction(floor, profile, startHeight);
    }

}

PlanData::Edge_handle PlanData::duplicateEdge(PlanData::Edge_handle e)
{
    Extrsn::PlanEdge newE(*e);
    newE.set_new_id();
    this->edges.push_back(newE);

    return --this->edges.end();
}

void PlanData::bind_profile(PlanEdge& pe, Editor &floor, Editor &profiles)
{
    //pe.profileTag = 0;
    pe.profileTag = floor.get_tag(Point(pe.p1.x(), pe.p1.y()), Point(pe.p2.x(), pe.p2.y()));
    pe.profileIndex = 0; //It's right for the initial one
    std::vector<Point> profile = profiles.get_profile(pe.profileTag);
    //printf("profile_tag = %d\n", pe.profileTag);

    float height = profile[pe.profileIndex].y;
    pe.p1 = Point_3(pe.p1.x(), pe.p1.y(), height);
    pe.p2 = Point_3(pe.p2.x(), pe.p2.y(), height);
    pe.vh1->point = pe.p1;
    pe.vh2->point = pe.p2;
    assert(profile.size() >= 2);

    double angle = get_angle_from_profile_point(profile[0], profile[1]);

    pe.directionPlane = get_plane_from_edge_and_angle(pe.p1, pe.p2, angle);

}

PlanData::SLAV_iterator PlanData::SLAV_begin()
{
    return vertices.begin();
}

PlanData::SLAV_iterator PlanData::SLAV_end()
{
    return vertices.end();
}

PlanData::Vertex_iterator PlanData::next(SLAV_iterator slavIter, Vertex_iterator vIter)
{
    ++vIter;
    if(vIter == slavIter->end())
        return slavIter->begin();
    else
        return vIter;
}

PlanData::Vertex_iterator PlanData::prev(SLAV_iterator slavIter, Vertex_iterator vIter)
{
    if(vIter == slavIter->begin())
        return --slavIter->end();
    else
        return --vIter;
}


PlanData::Vertex_Map PlanData::get_vertex_map()
{
    Vertex_Map map;
    for(PlanData::SLAV_iterator i = this->SLAV_begin(); i != this->SLAV_end(); ++i)
    {
        for(PlanData::Vertex_iterator j = i->begin(); j != i->end(); ++j)
        {
            map[j] = make_pair(i,j);
        }
    }
    return map;

}

/*
void PlanData::update_plane_to_edge_map()
{
    this->planeToEdgeMap.clear();
    map<pair<Point_3, Point_3>, list<PlanData::Edge_handle> > &map = this->planeToEdgeMap;

    for(PlanData::SLAV_iterator i = this->SLAV_begin(); i != this->SLAV_end(); ++i)
    {
        for(list<Vertex>::iterator j = i->begin(); j != i->end(); ++j)
        {
            pair<Point_3, Point_3> pair = make_pair(j->nextEdgeHndl->p1, j->nextEdgeHndl->p2);
            if(map.find(pair) != map.end())
            {
                map[pair].push_back(j->nextEdgeHndl);
                //assert(false);
                //myprintf("WARNING: remove_invalid_intersect_events, map.find(pair) != map.end()\n");
            }
            else
            {
                map[pair] = list<PlanData::Edge_handle>();
                map[pair].push_back(j->nextEdgeHndl);
            }
        }
    }
}*/



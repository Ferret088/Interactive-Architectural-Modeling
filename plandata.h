#ifndef PLANDATA_H
#define PLANDATA_H
#include <list>
#include <vector>
#include <map>
#include <iostream>
#include <string>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Plane_3.h>

#include "edit.h"
#define UNFOLD(p) (p).x(),(p).y(),(p).z()

//Library type define
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K>                     Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K>           Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>              TDS;
typedef CGAL::Exact_predicates_tag                               Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;
typedef CDT::Vertex_handle Vertex_handle;

typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Triangle_3<K> Triangle_3;

typedef K::Point_3 Point_3;
typedef K::Vector_3 Vector_3;
typedef std::vector<Triangle_3> MyPolyhedron;
typedef K::Plane_3 Plane_3;

//Encodes the active plan

namespace Extrsn {
struct Vertex;
struct PlanEdge
{
    //Endpoint vertices in the active plan
    std::list<Vertex>::iterator vh1, vh2;
    //Defines the geometry plane
    Point_3 p1, p2;
    Plane_3 directionPlane, prevDirectionPlane, nextDirectionPlane;

    //associated profile
    int profileTag;
    int profileIndex;


    PlanEdge(): id(++n) {}
    bool operator <(const PlanEdge& e) const
    {
        return this->id < e.id;
    }
    bool operator ==(const PlanEdge& e) const
    {
        return this->id < e.id;
    }

    int set_new_id()
    {
        id = ++n;
        return id;
    }
private:
    static int n;
    int id;
};


struct Vertex
{
    Point_3 point;
    std::list<PlanEdge>::iterator prevEdgeHndl, nextEdgeHndl;
    bool operator <(const Vertex& v) const
    {
        if(this->id == v.id) assert(this->point == v.point);

        if(this->point != v.point)
            return this->point < v.point;
        else
            return this->id < v.id;
    }
    bool operator ==(const Vertex &v) const
    {
        if(this->id == v.id) assert(this->point == v.point);


        return this->id == v.id;
        //return this->point == v.point;
    }
    Vertex(): id(++n) {}

private:
    static int n;
    int id;
};


class compare_vertex_handle {
   public:
      bool operator()(std::list<Vertex>::iterator i, std::list<Vertex>::iterator j)
      {
          return *i < *j;
      }
};

class compare_edge_handl
{
public:
   bool operator()(std::list<PlanEdge>::iterator i, std::list<PlanEdge>::iterator j)
   {
       return *i < *j;
   }
};


}


using namespace Extrsn;

#define PI 3.14159265
double get_angle_from_profile_point(Point p1, Point p2);
Plane_3 get_plane_from_edge_and_angle(Point_3 p1, Point_3 p2, double angle);

class PlanData
{
private:
    void bind_profile(PlanEdge&, Editor&, Editor&);
public:
    std::list<PlanEdge> edges;

    Editor &floor, &profile;

    std::list<std::list<Vertex> >  vertices, newVertices;
    std::list<Vertex> tmpVetices;
    typedef std::list<std::list<Vertex> >::iterator SLAV_iterator;
    typedef std::list<Vertex>::iterator Vertex_iterator;
    typedef Vertex_iterator                       Vertex_handle;
    typedef std::list<PlanEdge>::iterator    Edge_handle;

    typedef std::pair<PlanData::SLAV_iterator, PlanData::Vertex_iterator> SLAV_pair;
    typedef std::map<PlanData::Vertex_handle, SLAV_pair,compare_vertex_handle> Vertex_Map;

    PlanData( Editor& floor,  Editor& profile);

    SLAV_iterator SLAV_begin();
    SLAV_iterator SLAV_end();
    static Vertex_iterator next(SLAV_iterator, Vertex_iterator);
    static Vertex_iterator prev(SLAV_iterator, Vertex_iterator);
    Vertex_Map get_vertex_map();

    Edge_handle duplicateEdge(Edge_handle e);
    void do_construction(Editor &floor, Editor &profile, double);
    //std::map<std::pair<Point_3, Point_3>, std::list<PlanData::Edge_handle> > planeToEdgeMap;
    //void update_plane_to_edge_map();
};

#endif // PLANDATA_H

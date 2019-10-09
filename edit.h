
//
//  Edit.h
//  Extrusion
//
//  Created by Yubin Lai on 10/18/13.
//  Copyright (c) 2013 Yubin Lai. All rights reserved.
//

#ifndef __Extrusion__Edit__
#define __Extrusion__Edit__
#include <set>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include "point.h"
#include "edge.h"

//Library type define
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;

//stores the closest point and the distance
struct Magnet
{
    Point p;
    double dist;
    Magnet(double ddist);
};

// Stores the user input data
class Editor
{
private:
    std::set<Edge> edges;
    Point adjust_overhanging_starting_point(Point p1, Point p2, float y);

public:
    Editor();

    void add_edge(Point p1, Point p2, bool previewFlag=true);
    void add_edge(Point p1, Point p2, int tag, bool previewFlag=true);

    Magnet magnet(Point p) const;

    std::set<Edge> get_edges() const;

    std::vector<Polygon_2 > get_polygons() const;

    std::vector<std::vector<Point> > get_profiles() const;

    void remove(Point p);

    int get_tag(Point p1, Point p2) const;

    std::vector<Point>  get_profile(int tag) const;
    std::vector<std::vector<Point> > get_overhanging_profiles(int tag);
    double get_start_height() const;

    int get_overhanging_inside_tag(int tag) const;
    int get_overhanging_outside_tag(int tag) const;
};


extern int no_profile_offset_event;

#endif /* defined(__Extrusion__Edit__) */

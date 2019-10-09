#include "edit.h"

Magnet::Magnet(double ddist): p(Point(0,0)), dist(ddist) {}
// -----------------------------------
//  auxiliary functions and data structs
// -----------------------------------

//caculate the distance of two points
double dist_square( Point a, Point b)
{
    double deltax = a.x - b.x;
    double deltay = a.y - b.y;
    return deltax*deltax + deltay*deltay;
}

//stores a pair of edges
struct Edge_pair
{
    int ne; //number of edges stored
    Edge e1;
    Edge e2;
    Edge_pair():ne(0), e1(Point(0,0), Point(0,0)), e2(Point(0,0), Point(0,0)){}
};

//map a point to a pair of edges
void add_to_map(std::map<Point, Edge_pair > &map, Point p, Edge e)
{
    if (map.find(p) == map.end()) {
        map[p] = Edge_pair();
        map[p].e1 = e;
        map[p].ne = map[p].ne + 1;
    } else {
        map[p].e2 = e;
        map[p].ne = map[p].ne + 1;
    }
}

//convert data
Polygon_2 convert_to_CGAL(std::vector<Point> poly)
{
    Polygon_2 pgn;
    for(std::vector<Point>::iterator it = poly.begin(); it != poly.end(); ++it)
    {
        Point_2 p2(it->x, it->y);
        pgn.push_back(p2);
    }
    return pgn;
}

//check if polygon is simple
bool is_simple(std::vector<Point> poly)
{
    Polygon_2 pgn = convert_to_CGAL(poly);
    return pgn.is_simple();
}

//orient polygons so that they are counterclockwise unless they are holes, in which case it will be clockwise
std::vector<Polygon_2> orient(std::vector<std::vector<Point> > polys)
{
    std::vector<Polygon_2> pgns, pgnsOriented;
    for(std::vector<std::vector<Point> >::iterator it = polys.begin(); it!= polys.end(); ++it)
    {
        Polygon_2 pgn = convert_to_CGAL(*it);
        pgns.push_back(pgn);
    }
    for(std::vector<Polygon_2>::iterator it = pgns.begin(); it != pgns.end(); ++it)
    {
        Polygon_2 ply = *it;
        Point_2 p = it->vertex(0);
        int cnt = 0;
        for(std::vector<Polygon_2>::iterator it1 = pgns.begin(); it1 != pgns.end(); ++it1)
        {
            if (*it != *it1)
            {
                if (it1->bounded_side(p) == CGAL::ON_BOUNDED_SIDE) ++cnt;
            }
        }
        if ((cnt % 2 && ply.is_counterclockwise_oriented()) || (!(cnt%2) && ply.is_clockwise_oriented() ))
        {
            ply.reverse_orientation();
        }
        pgnsOriented.push_back(ply);

    }
    return pgnsOriented;
}

Editor::Editor(){}

extern void preview();
//add one edge
void Editor::add_edge(Point p1, Point p2, bool previewFlag)
{
    if (p1 == p2)
    {
        return;
    }
    edges.insert(Edge(p1, p2));
    if(previewFlag) preview();
}

void Editor::add_edge(Point p1, Point p2, int tag, bool previewFlag)
{
    if(p1 == p2)
    {
        return;
    }
    Edge e(p1, p2);
    e.tag = tag;
    edges.insert(e);
    if(previewFlag) preview();
}

//Get the closest point in the plan
Magnet Editor::magnet(Point p) const
{
    Magnet magnet(-1);
    double d;
    for (std::set<Edge>::iterator it = edges.begin(); it != edges.end(); ++it)
    {
        d = dist_square(it->p1, p);
        if (magnet.dist == -1 || d < magnet.dist)
        {
            magnet.dist = d;
            magnet.p = it->p1;
        }
        d = dist_square(it->p2, p);
        if (magnet.dist == -1 || d < magnet.dist)
        {
            magnet.dist = d;
            magnet.p = it->p2;
        }
    }
    return magnet;
}

//Get all the edges
std::set<Edge> Editor::get_edges() const
{
    return edges;
}


//Get all polygons
std::vector<Polygon_2 > Editor::get_polygons() const
{
    std::vector<std::vector<Point> > polygons;
    std::map<Point, Edge_pair > p2e;
    std::map<Edge, int> flag;
    int nFlag = 0;
    int i;
    //map points to edges
    for (std::set<Edge>::iterator it = edges.begin(); it != edges.end(); ++it)
    {
        flag[(*it)] = 0;
        add_to_map(p2e, it->p1, (*it));
        add_to_map(p2e, it->p2, (*it));
    }
    //pick polygon one at a time
    for (std::set<Edge>::iterator it = edges.begin(); it != edges.end(); ++it) if(!flag[*it])
    {
        std::vector<Point> poly;
        Edge currEdge = (*it);
        Edge starEdge = (*it);
        bool valid = false;
        int nLines = 0;
        nFlag = nFlag + 1;
        while(1)
        {
            nLines = nLines + 1;
            flag[currEdge] = nFlag;
            Point p1 = currEdge.p1;
            Point p2 = currEdge.p2;
            if(p2e[p1].ne > 2 || p2e[p2].ne > 2) break;
            Edge e[4] = {p2e[p1].e1, p2e[p1].e2, p2e[p2].e1, p2e[p2].e2};
            if (nLines >= 3)
            {
                for (i=0; i<4; ++i)
                {
                    if (e[i] == starEdge)
                    {
                        valid = true;
                        poly.push_back(i<2?p1:p2);
                        break;
                    }
                }
                if (valid) break;
            }
            for (i=0; i<4; ++i)
            {
                if (!flag[e[i]])
                {
                    currEdge = e[i];
                    poly.push_back(i<2?p1: p2);
                    break;
                }
            }
            if (i==4) break;

        }//end of while
        if (valid)
        {
            if(is_simple(poly))
                polygons.push_back(poly);
        }
    }//end of for
    return orient(polygons);

}//end of getPolygons


std::vector<std::vector<Point> > Editor::get_profiles() const
{
    std::set<Edge> edgesYX;
    std::vector<std::vector<Point> > lineStrips, profiles;
    //Sort edges by Y
    for(std::set<Edge>::iterator it = edges.begin(); it != edges.end(); ++it)
    {
        //printf("%f, %f, %f, %f\n", it->p1.x, it->p1.y, it->p2.x, it->p2.y);
        Point p1 = Point(it->p1.y, it->p1.x);
        Point p2 = Point(it->p2.y, it->p2.x);
        if(it->p1.y <= it->p2.y)
            edgesYX.insert(Edge(p1, p2));
        else
            edgesYX.insert(Edge(p2, p1));
    }
    //connect edge strips
    lineStrips.push_back(std::vector<Point>());
    while(!edgesYX.empty())
    {
        bool processed = false;
        std::set<Edge>::iterator tmpiter;

        for(std::set<Edge>::iterator it = edgesYX.begin(); it != edgesYX.end(); )
        {
            std::vector<std::vector<Point> >::iterator it1;
            for(it1 = lineStrips.begin(); it1 != lineStrips.end(); ++it1)
            {
                Point testEqual(it->p1.y, it->p1.x);
                if(it1->empty())
                {
                    it1->push_back(Point(it->p1.y, it->p1.x));
                    it1->push_back(Point(it->p2.y, it->p2.x));
                    tmpiter = it;
                    ++tmpiter;
                    edgesYX.erase(it);
                    it = tmpiter;
                    processed = true;
                    break;
                }
                if(it1->back() == testEqual)
                {
                    it1->push_back(Point(it->p2.y, it->p2.x));
                    tmpiter = it;
                    ++tmpiter;
                    edgesYX.erase(it);
                    it = tmpiter;
                    processed = true;
                    break;
                }

                if(it->p1.y == it->p2.y)
                {
                    testEqual = Point(it->p2.y, it->p2.x);
                    if(it1->back() == testEqual)
                    {
                        it1->push_back(Point(it->p1.y, it->p1.x));
                        tmpiter = it;
                        ++tmpiter;
                        edgesYX.erase(it);
                        it = tmpiter;
                        processed = true;
                        break;
                    }
                }
            }
            if(it1 == lineStrips.end())
            {
                ++it;
            }
        }
        if(!processed)
        {
            std::vector<Point> tmp;
            lineStrips.push_back(tmp);
        }
    }
    //valid if there is no loops
    std::set<Point> ps;
    for(std::vector<std::vector<Point> >::iterator it = lineStrips.begin(); it != lineStrips.end(); ++it)
    {

        bool valid = true;
        for(std::vector<Point>::iterator it1 = it->begin(); it1 != it->end(); ++it1)
        {
            if(ps.find(*it1) != ps.end())
            {
                valid = false;
                break;
            }
            else
            {
                ps.insert(*it1);
            }
        }

        if(valid)
            profiles.push_back(*it);
    }
    return profiles;
}

double Editor::get_start_height() const
{
    //TBD:...
    if(get_profiles().size() == 0)
        return 0;
    else
        if(get_profile(0).empty()) return 0;
        return get_profile(0)[0].y;
}


std::vector<Point> Editor::get_profile(int tag) const
{
    //printf("tag=%d\n", tag);
    std::vector<std::vector<Point> > profiles = get_profiles();
    for(std::vector<std::vector<Point> >::iterator i = profiles.begin(); i != profiles.end(); ++i)
    {
        if(i->size() >= 2) {
            if(get_tag(*(i->begin()), *(++i->begin())) == tag)
            {
                return *i;
            }
        }
    }
    printf("couldn't find profile with tag=%d\n", tag);
    return get_profiles()[0];
}
int no_profile_offset_event=1;

Point Editor::adjust_overhanging_starting_point(Point p1, Point p2, float y)
{
    float dx = p2.x - p1.x, dy = p2.y - p1.y;
    float dy_new = p2.y - y;
    float dx_new = dy_new * dx/dy;
    float new_x = p2.x - dx_new;
    Point newP = Point(new_x, y);
    if(no_profile_offset_event)
        return newP;
    if(this->edges.find(Edge(p1, p2)) != this->edges.end())
    {
        this->edges.erase(Edge(p1, p2));
    }
    else
    {
        printf("WARNING, adjust_overhanging_starting_point: edge not found\n");
    }
    this->edges.insert(Edge(newP, p2));

    return newP;
}

double get_x_in_active_profile(const std::vector<Point> &profile, double y);

std::vector<std::vector <Point> > Editor::get_overhanging_profiles(int tag)
{
    std::vector<std::vector <Point> > ret;
    std::vector<Point> first, second;
    std::vector<std::vector<Point> > profiles = get_profiles();

    //choose the profiles
    int cnt = 0;
    for(std::vector<std::vector<Point> >::iterator i = profiles.begin(); i != profiles.end(); ++i)
    {
        if(i->size() >= 2) {
            if(get_tag(*(i->begin()), *(++i->begin())) == tag)
            {
                if(cnt == 1)
                {
                    first = *i;
                }else if(cnt == 2)
                {
                    second = *i;
                    assert(first.begin()->y <= second.begin()->y);
                    Point newStart = adjust_overhanging_starting_point(*second.begin(), *(++second.begin()), first.begin()->y);
                    *(second.begin()) = newStart;
                    //assert(first.begin()->y == second.begin()->y);
                    if(second.begin()->x < first.begin()->x)
                    {
                        ret.push_back(first);
                        ret.push_back(second);
                    }else
                    {
                        ret.push_back(second);
                        ret.push_back(first);
                    }
                }
                ++cnt;
            }
        }
    }

    //compare with the non-overhanging profile
    if(ret.size() >=2)
    {
        std::vector<Point> activeProfile = get_profile(tag);
        double cmpX = get_x_in_active_profile(activeProfile, ret[0][0].y);
        if(cmpX < ret[0][0].x)
        {
            std::vector<std::vector <Point> > reversed_ret;
            reversed_ret.push_back(ret.back());
            reversed_ret.push_back(ret.front());
            ret = reversed_ret;
        }
        if((cmpX - ret[0][0].x) * (cmpX - ret[1][0].x) < 0)
        {
            ret.clear();
            printf("overhanging profile not valid, shoulb be both inside or outside the wall profile");
            return ret;
        }
    }
    return ret;

}
void Editor::remove(Point p)
{
    std::set<Edge> edgesNew;
    for (std::set<Edge>::iterator it = edges.begin(); it != edges.end(); ++it)
    {
        if (!(it->p1 == p || it->p2 == p))
        {
            edgesNew.insert(*it);
        }
    }
    this->edges = edgesNew;
    preview();
}

int Editor::get_tag(Point p1, Point p2) const
{
    Point myp1(p1.x, p1.y);
    Point myp2(p2.x, p2.y);
    Edge e1 = Edge(myp1, myp2);
    Edge e2 = Edge(myp2, myp1);
    std::set<Edge>::iterator it = edges.find(e1);
    if(it != edges.end())
    {
        return it->tag;
    }
    it = edges.find(e2);
    if(it != edges.end())
    {
        return it->tag;
    }
    assert(false);
    return 0;
}

int Editor::get_overhanging_inside_tag(int tag) const
{
    return -(tag*2)-1;
}

int Editor::get_overhanging_outside_tag(int tag) const
{
    return -(tag*2)-2;
}


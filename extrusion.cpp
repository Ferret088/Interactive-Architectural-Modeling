#include "extrusion.h"
Extrusion extrusion;
#include <queue>
#include <map>
using namespace Extrsn;
using namespace std;
#include <GLUT/glut.h>
//warp the printf
void my_printf(char* fmt, ...)
{
    va_list args;
    va_start(args,fmt);
    //vprintf(fmt,args);
    va_end(args);
}

//debug functions
void debug1(Plane_3 plane)
{

    my_printf("    Debug1:a=%lf, b=%lf, c=%lf, d=%lf\n", plane.a(), plane.b(), plane.c(), plane.d());
}

void debug(PlanData::Edge_handle e)
{
    my_printf("    Debug:vh1(%lf, %lf, %lf), vh2(%lf,%lf,%lf), \n          p1(%lf, %lf,%lf), p2(%lf,%lf,%lf)\n", UNFOLD(e->vh1->point), UNFOLD(e->vh2->point),
              UNFOLD(e->p1), UNFOLD(e->p2));
    //my_printf("    a=%lf, b=%lf, c=%lf, d=%lf\n", plane.a(), plane.b(), plane.c(), plane.d());
    debug1(e->directionPlane);
}

void debug(set<Event, compare_event_intersect> eventSet)
{
    my_printf("&&&&&size=%d\n", eventSet.size());
    for(set<Event, compare_event_intersect>::iterator it = eventSet.begin(); it != eventSet.end(); ++it)
    {
        my_printf("%lf, %lf, %lf\n", UNFOLD(it->eventPoint));
    }
    my_printf("#####\n");

}

//check if parallel, same edge return false
bool parallel(PlanData::Edge_handle e1, PlanData::Edge_handle e2)
{
    //if(e1->p1 == e2->p1 && e1->p2 == e2->p2) return false;
    //since p1 p2 is always parallel to z plane, z is ignored.
    Vector_3 v1 = e1->p2 - e1->p1, v2 = e2->p2 - e2->p1;
    return fabs(v1.x() * v2.y() - v1.y() * v2.x()) < 1e-6;
}

//calculate the intersection of 3 planes if the intersection is a point
bool my_intersect(const Plane_3 &plane1, const Plane_3 &plane2, const Plane_3 &plane3, Point_3& outPoint)
{
    auto result = CGAL::intersection(plane1, plane2, plane3);
    if(CGAL::assign(outPoint, result))
    {
        return true;
    }
    debug1(plane1);
    debug1(plane2);
    debug1(plane3);
    return false;
}




//insert all meaningful intersection events to the eventQueue
void insert_intersection_events(priority_queue<Event, std::vector<Event>, compare_event> &eventQueue,
                                priority_queue<Event, std::vector<Event>, compare_event> &historyEvents,
                                PlanData &pd, double startHeight)
{
    std::priority_queue<Event, std::vector<Event>, compare_event> tmpQueue(eventQueue);
    set<Event, compare_event_intersect> eventSet;
    while(!tmpQueue.empty())
    {
        Event e = tmpQueue.top();
        if(e.type == EVENT_TYPE_GENERALIZED_INTERSECTION)
        {
            eventSet.insert(e);
        }
        tmpQueue.pop();
    }
    tmpQueue = historyEvents;
    while(!tmpQueue.empty())
    {
        Event e = tmpQueue.top();
        if(e.type == EVENT_TYPE_GENERALIZED_INTERSECTION)
        {
            eventSet.insert(e);
        }
        tmpQueue.pop();
    }

    for(PlanData::SLAV_iterator SLAVIter = pd.SLAV_begin(); SLAVIter != pd.SLAV_end(); ++SLAVIter)
    {
        for(list<Vertex>::iterator vIter = SLAVIter->begin(); vIter != SLAVIter->end(); ++vIter)
        {
            for(PlanData::SLAV_iterator j = pd.SLAV_begin(); j != pd.SLAV_end(); ++j)
            {
                for(list<Vertex>::iterator vj = j->begin(); vj != j->end(); ++vj) if(vIter->prevEdgeHndl->directionPlane != vj->nextEdgeHndl->directionPlane
                                                                                     &&vIter->nextEdgeHndl->directionPlane != vj->nextEdgeHndl->directionPlane
                                                                                     &&vIter->prevEdgeHndl->directionPlane != vIter->nextEdgeHndl->directionPlane){
                    Event event;
                    const PlanEdge &prevE = (*vIter->prevEdgeHndl), &nextE = (*vIter->nextEdgeHndl), &ej = (*vj->nextEdgeHndl);

                    event.type = EVENT_TYPE_GENERALIZED_INTERSECTION;
                    event.cornerPrevEdge = vIter->prevEdgeHndl;
                    event.cornerNextEdge = vIter->nextEdgeHndl;
                    event.edge = vj->nextEdgeHndl;


                    if (my_intersect(prevE.directionPlane, nextE.directionPlane, ej.directionPlane, event.eventPoint))
                    {


                        if(event.eventPoint.z() > startHeight - 1e-4)
                        {
                            if(eventSet.find(event) == eventSet.end())
                            {

                                eventQueue.push(event);
                                eventSet.insert(event);
                                my_printf("(((add intersection:%lf, %lf, %lf\n", event.eventPoint.x(), event.eventPoint.y(), event.eventPoint.z());
                                debug(event.cornerNextEdge);
                                debug(event.cornerPrevEdge);
                                debug(event.edge);
                                my_printf(")))\n");
                                debug(eventSet);
                            }
                         }
                        else
                        {
                            while(0){}
                        }
                    }
                }
            }
        }
    }
}

//helper function for inserting edge direction events
void do_directon_event_insertion(priority_queue<Event, std::vector<Event>, compare_event> &eventQueue, vector<Point> &profile, int tag)
{
    vector<Point>::iterator i = profile.begin();
    for(vector<Point>::iterator j = profile.begin(); j != profile.end(); ++j)
    {
        if(i != profile.begin())
        {
            double angle = get_angle_from_profile_point(*i, *j);
            //my_printf("i(%lf, %lf), j(%lf, %lf)\n", i->x, i->y, j->x, j->y);
            my_printf("angle=%lf\n",angle);

            if(fabs(fabs(angle) - 90) > 1e-4)
            {
                //normal change
                my_printf("normal change\n");
                Event e;
                e.type = EVENT_TYPE_EDGE_DIRECTION_NON_HORIZONTAL;
                e.eventPoint = Point_3(0, 0, i->y);
                e.newAngle = angle;
                e.profileTag = tag;
                eventQueue.push(e);
            }
            else
            {
                //Horizontal
                my_printf("Horizontal!\n");
            }
        }
        i = j;
    }
}

// insert edge direction events
void insert_direction_events(priority_queue<Event, std::vector<Event>, compare_event> &eventQueue, PlanData &pd)
{
    int tag = 0;
    vector<Point> profile= pd.profile.get_profile(tag);
    do_directon_event_insertion(eventQueue, profile, tag);
}

//helper function
double get_x_in_active_profile(const vector<Point> &profile, double y)
{
    assert(profile.size() >= 2);
    vector<Point>::const_iterator i;
    for(i = profile.begin(); i != profile.end(); ++i)
    {
        if(i != profile.begin())
        {
            if(i->y > y)
            {
                break;
            }
        }
    }
    if( i == profile.end())
    {
        --i;
    }
    Point p2 = *i;
    Point p1 = *(--i);

    float dx = p2.x - p1.x, dy = p2.y - p1.y;
    float dy_new = p2.y - y;
    float dx_new = dy_new * dx/dy;
    float new_x = p2.x - dx_new;
    return new_x;
}


// insert profile offset events (for overhanging roofs)
void insert_profile_offset_events(priority_queue<Event, std::vector<Event>, compare_event> &eventQueue, PlanData &pd)
{
    int tag = 0;
    vector<vector<Point> > offsetProfiles = pd.profile.get_overhanging_profiles(tag);
    if(offsetProfiles.size() < 2)
    {
        return;
    }
    vector<Point> inside = offsetProfiles[0];
    vector<Point> outside = offsetProfiles[1];

    double y = inside.front().y;
    double inside_x = inside.front().x;
    double outside_x = outside.front().x;

    vector<Point> activeProfile = pd.profile.get_profile(tag);
    double cmpX = get_x_in_active_profile(activeProfile, y);


    Event e;
    e.type = EVENT_TYPE_PROFILE_OFFSET;
    e.eventPoint = Point_3(0, 0, y);
    e.dist_inside = inside_x - cmpX;
    e.dist_outside = outside_x - cmpX;
    e.profileTag = tag;
    e.profile_inside = inside;
    e.profile_outside = outside;

    eventQueue.push(e);
    do_directon_event_insertion(eventQueue, inside, pd.profile.get_overhanging_inside_tag(tag));
    do_directon_event_insertion(eventQueue, outside, pd.profile.get_overhanging_outside_tag(tag));

    my_printf("y=%lf, cmpX = %lf, inside_x=%lf, outside_x=%lf, dist_inside%lf, dist_outside=%lf\n", y, cmpX, inside_x, outside_x, e.dist_inside, e.dist_outside);

}

//for clustering events
bool within_z(double z1, double z2)
{
    return fabs(z1-z2) < 1e-4;
}

//for clustering events
bool within_radius(Point_3 p1, Point_3 p2)
{
    Vector_3 v = p2 - p1;
    return sqrt(v.x() * v.x() + v.y() * v.y()) < 1e-6;
}

//cluster events
list<Event> cluster_intersect_events(priority_queue<Event, std::vector<Event>, compare_event> &eventQueue, double sweepZ, Point_3 epoint)
{
     //pending events are not clustered, and will be put back to eventQueue
    list<Event> ret, pending;

    while(!eventQueue.empty())
    {
        Event event = eventQueue.top();
        eventQueue.pop();
        if(event.type == EVENT_TYPE_GENERALIZED_INTERSECTION)
        {
            if(!within_z(event.eventPoint.z(), sweepZ))
            {
                pending.push_back(event);
                break; //Important
            } else
            {
                if(within_radius(event.eventPoint, epoint))
                    ret.push_back(event);
                else
                    pending.push_back(event);
            }
        }
        else
        {
            pending.push_back(event);
        }

    }
    for(list<Event>::iterator iter = pending.begin(); iter != pending.end(); ++iter)
    {
        eventQueue.push(*iter);
    }
    return ret;
}

extern list<list<PlanData::Vertex_handle> >output;
extern void display_test_sw3();
//Consturct a set of chains for the clustered events
list<list<PlanData::Vertex_handle> > get_chains(/*list<Event> &clustered*/ set<PlanData::Edge_handle, Extrsn::compare_edge_handl> &intersect_edges_from_event, PlanData &pd)
{
     /*set<pair<Point_3, Point_3> > intersect_edges_from_event;*/
     list<list<PlanData::Vertex_handle> > chains;
     /*for(list<Event>::iterator i = clustered.begin(); i != clustered.end(); ++i)
     {
         intersect_edges_from_event.insert(make_pair(i->cornerPrevEdge->p1, i->cornerPrevEdge->p2));
         intersect_edges_from_event.insert(make_pair(i->cornerNextEdge->p1, i->cornerNextEdge->p2));
         intersect_edges_from_event.insert(make_pair(i->edge->p1, i->edge->p2));
     }*/
     PlanData::Vertex_handle lastV;
     for(PlanData::SLAV_iterator i = pd.SLAV_begin(); i != pd.SLAV_end(); ++i)
     {
         list<PlanData::Vertex_handle> chain;
         for(list<Vertex>::iterator j = i->begin(); j != i->end(); ++j)
         {
             //pair<Point_3, Point_3> pair = make_pair(j->nextEdgeHndl->p1, j->nextEdgeHndl->p2);
             if (intersect_edges_from_event.find(/*pair*/ j->nextEdgeHndl) != intersect_edges_from_event.end())
             {
                 if(!chain.empty())
                 {
                     if (*chain.rbegin() != j)
                     {
                         //a new chain
                         chains.push_back(chain);
                         chain = list<PlanData::Vertex_handle>();
                         chain.push_back(j);
                     }
                     chain.push_back(PlanData::next(i, j));
                 }
                 else
                 {
                     chain.push_back(j);
                     chain.push_back(PlanData::next(i, j));
                 }

             }

         }
         if (!chain.empty())
           chains.push_back(chain);
     }

     for(list<list<PlanData::Vertex_handle> >::iterator i = chains.begin(); i != chains.end(); ++i)
     {
         for(list<list<PlanData::Vertex_handle> >::iterator j = chains.begin(); j != chains.end(); ++j)
         {
             if(i != j && i->size() != 0 && j->size() != 0)
             {
                 if(*(i->rbegin()) == (*(j->begin())))
                 {
                     i->pop_back();
                     i->splice(i->end(), *j);
                     chains.erase(j);
                     break;
                 }
             }
         }
     }

     return chains;
}

//Consturct a set of chains for the clustered events (for debug only)
list<list<PlanData::Vertex_handle> > get_chains_old(list<Event> &clustered, PlanData &pd)
{
     set<pair<Point_3, Point_3> > intersect_edges_from_event;
     list<list<PlanData::Vertex_handle> > chains;
     for(list<Event>::iterator i = clustered.begin(); i != clustered.end(); ++i)
     {
         intersect_edges_from_event.insert(make_pair(i->cornerPrevEdge->p1, i->cornerPrevEdge->p2));
         intersect_edges_from_event.insert(make_pair(i->cornerNextEdge->p1, i->cornerNextEdge->p2));
         intersect_edges_from_event.insert(make_pair(i->edge->p1, i->edge->p2));
     }
     PlanData::Vertex_handle lastV;
     for(PlanData::SLAV_iterator i = pd.SLAV_begin(); i != pd.SLAV_end(); ++i)
     {
         list<PlanData::Vertex_handle> chain;
         for(list<Vertex>::iterator j = i->begin(); j != i->end(); ++j)
         {
             pair<Point_3, Point_3> pair = make_pair(j->nextEdgeHndl->p1, j->nextEdgeHndl->p2);
             if (intersect_edges_from_event.find(pair) != intersect_edges_from_event.end())
             {
                 if(!chain.empty())
                 {
                     if (*chain.rbegin() != j)
                     {
                         //a new chain
                         chains.push_back(chain);
                         chain = list<PlanData::Vertex_handle>();
                         chain.push_back(j);
                     }
                     chain.push_back(PlanData::next(i, j));
                 }
                 else
                 {
                     chain.push_back(j);
                     chain.push_back(PlanData::next(i, j));
                 }

             }

         }
         if (!chain.empty())
           chains.push_back(chain);
     }

     for(list<list<PlanData::Vertex_handle> >::iterator i = chains.begin(); i != chains.end(); ++i)
     {
         for(list<list<PlanData::Vertex_handle> >::iterator j = chains.begin(); j != chains.end(); ++j)
         {
             if(i != j && i->size() != 0 && j->size() != 0)
             {
                 if(*(i->rbegin()) == (*(j->begin())))
                 {
                     i->pop_back();
                     i->splice(i->end(), *j);
                     chains.erase(j);
                     break;
                 }
             }
         }
     }

     return chains;
}

double get_angle_radian(Point_3 p1, Point_3 p2)
{
    return atan2(p2.y()-p1.y(), p2.x()-p1.x());
}

bool cmp_chains(const list<PlanData::Vertex_handle> &a, const list<PlanData::Vertex_handle>  &b)
{
    assert(a.size() >= 2);
    assert(b.size() >= 2);
    PlanData::Vertex_handle v1 = *(a.begin()), v2 = *(++a.begin());
    double angleA = get_angle_radian(v1->point, v2->point);
    v1 = *(b.begin());
    v2 = *(++b.begin());
    double angleB = get_angle_radian(v1->point, v2->point);
    return angleA < angleB;
}

void sort_chains(list<list<PlanData::Vertex_handle> > &chains)
{
    chains.sort(cmp_chains);
}

bool point_is_in_between_edge(Point_3 p, Point_3 ep1, Point_3 ep2)
{
    double minX = min(ep1.x(), ep2.x());
    double maxX = max(ep1.x(), ep2.x());
    double minY = min(ep1.y(), ep2.y());
    double maxY = max(ep1.y(), ep2.y());
    double esp = 1e-5;
    return (p.x() < maxX + esp && p.x() > minX - esp && p.y() < maxY + esp && p.y() > minY - esp );
}



//An edge is valid if it exsits in the current active plan and the intersection is on it
bool edge_is_valid(PlanData::Edge_handle edge, Point_3 eventPoint, map<pair<Point_3, Point_3>, list<PlanData::Edge_handle> > &map, PlanData &pd,
                   list<PlanData::Edge_handle> &edgeInvolvedInEvent)
{
    pair<Point_3, Point_3> pair = make_pair(edge->p1, edge->p2);
    if(map.find(pair) == map.end())
       return false;

    const list<PlanData::Edge_handle> &edgeInCurrentPlan = map[pair];
    /*if (edgeInCurrentPlan->vh1->prevEdgeHndl->p2 != edge->p1)
        return false;
    if (edgeInCurrentPlan->vh2->nextEdgeHndl->p1 != edge->p2)
        return false;
    */
    bool ret = false;
    for(list<PlanData::Edge_handle>::const_iterator i = edgeInCurrentPlan.begin(); i != edgeInCurrentPlan.end(); ++i)
    {
        double z = eventPoint.z();
        Plane_3 sweepPlane(Point_3(0,0,z), Point_3(0,1,z), Point_3(1, 0, z));
        Point_3 p1, p2;



        if(!my_intersect((*i)->directionPlane,
                                  (*i)->vh1->prevEdgeHndl->directionPlane,
                                  sweepPlane, p1))
        {
            if(parallel(*i, (*i)->vh1->prevEdgeHndl))
            {
                p1 = (*i)->vh1->point;
                //if(!my_intersect((*i)->directionPlane,
                                      //(*i)->prevDirectionPlane,
                                      //sweepPlane, p1)) continue;
            }else
                continue;
        }


        if(!my_intersect((*i)->directionPlane,
                                  (*i)->vh2->nextEdgeHndl->directionPlane,
                                  sweepPlane, p2))
        {


            if(parallel(*i, (*i)->vh2->nextEdgeHndl))
            {
                p2 = (*i)->vh2->point;

                //if(!my_intersect((*i)->directionPlane,
                 //                     (*i)->nextDirectionPlane,
                  //                    sweepPlane, p2)) continue;
            }else
                continue;

        }




        if( point_is_in_between_edge(eventPoint, p1, p2) )
        {
            ret = true;
            edgeInvolvedInEvent.push_back(*i);
        }
    }
    return ret;
}


 //Filter out invalid events
void remove_invalid_intersect_events(list<Event> &clustered, set<PlanData::Edge_handle, Extrsn::compare_edge_handl> &intersect_edges_from_event, PlanData &pd)
{
    map<pair<Point_3, Point_3>, list<PlanData::Edge_handle> > map;
    list<Event> validEvents;

    for(PlanData::SLAV_iterator i = pd.SLAV_begin(); i != pd.SLAV_end(); ++i)
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
    for(list<Event>::iterator i = clustered.begin(); i != clustered.end(); ++i)
    {
        list<PlanData::Edge_handle> edgeInvolvedInEvent;
        if(!edge_is_valid(i->cornerPrevEdge, i->eventPoint, map, pd, edgeInvolvedInEvent))
            continue;
        if(!edge_is_valid(i->cornerNextEdge, i->eventPoint, map, pd, edgeInvolvedInEvent))
            continue;
        if(!edge_is_valid(i->edge, i->eventPoint, map, pd, edgeInvolvedInEvent))
            continue;
        for(list<PlanData::Edge_handle>::iterator j = edgeInvolvedInEvent.begin(); j != edgeInvolvedInEvent.end(); ++j)
        {
            intersect_edges_from_event.insert(*j);
        }
        validEvents.push_back(*i);
    }
    clustered = validEvents;
}

void SLAV_sanity_check( PlanData &pd)
{
    for(PlanData::SLAV_iterator j = pd.SLAV_begin(); j != pd.SLAV_end(); ++j)
    {

        PlanData::Vertex_iterator tmpK;
        assert(j->size() > 2);
        for(PlanData::Vertex_iterator k = j->begin(); k != j->end(); ++k)
        {
            //check if SLAV is valid
            if( k != --j->end())
            {
                tmpK = k;
                ++tmpK;
                if(!(k->nextEdgeHndl->vh2 == tmpK))
                {
                    my_printf("k(%lf, %lf, %lf), vh2(%lf,%lf, %lf), tmpK(%lf, %lf,%lf)",
                           UNFOLD(k->point), UNFOLD(k->nextEdgeHndl->vh2->point), UNFOLD(tmpK->point));
                    assert(k->nextEdgeHndl->vh2 == tmpK);
                }

            } else
            {
                if(!(k->nextEdgeHndl->vh2 == j->begin()))
                {
                    my_printf("k(%lf, %lf, %lf), vh2(%lf,%lf, %lf), jbegin(%lf, %lf,%lf)",
                           UNFOLD(k->point), UNFOLD(k->nextEdgeHndl->vh2->point), UNFOLD(j->begin()->point));
                    assert(k->nextEdgeHndl->vh2 == j->begin());
                }
            }
            if( k != j->begin())
            {
                tmpK = k;
                --tmpK;
                if(!(k->prevEdgeHndl->vh1 == tmpK))
                {
                    my_printf("k(%lf, %lf, %lf), vh1(%lf,%lf, %lf), tmpK(%lf, %lf,%lf)",
                           UNFOLD(k->point), UNFOLD(k->prevEdgeHndl->vh1->point), UNFOLD(tmpK->point));
                    assert(k->prevEdgeHndl->vh1 == tmpK);
                }
            }else
            {
                if(!(k->prevEdgeHndl->vh1 == --j->end()))
                {
                    my_printf("k(%lf, %lf, %lf), vh1(%lf,%lf, %lf), --jend(%lf, %lf,%lf)",
                           UNFOLD(k->point), UNFOLD(k->prevEdgeHndl->vh1->point), UNFOLD((--j->end())->point));
                    assert(k->prevEdgeHndl->vh1 == --j->end());

                }
            }
        }
    }
}

//not doing anything, just to avoid bugs
void chains_corresponds_SLAV(list<list<PlanData::Vertex_handle> > &chains,PlanData& pd)
{
    for(list<list<PlanData::Vertex_handle> >::iterator i = chains.begin(); i != chains.end(); ++i)
    {
        bool have_corresponding_LAV = false;
        for(PlanData::SLAV_iterator j = pd.SLAV_begin(); j != pd.SLAV_end(); ++j)
        {
            PlanData::Vertex_iterator circulator;
            PlanData::Vertex_iterator tmpK;
            assert(j->size() > 2);
            for(PlanData::Vertex_iterator k = j->begin(); k != j->end(); ++k)
            {
                //check if SLAV is valid
                if( k != --j->end())
                {
                    tmpK = k;
                    ++tmpK;
                    if(!(k->nextEdgeHndl->vh2 == tmpK))
                    {
                        my_printf("k(%lf, %lf, %lf), vh2(%lf,%lf, %lf), tmpK(%lf, %lf,%lf)",
                               UNFOLD(k->point), UNFOLD(k->nextEdgeHndl->vh2->point), UNFOLD(tmpK->point));
                        assert(k->nextEdgeHndl->vh2 == tmpK);
                    }

                } else
                {
                    if(!(k->nextEdgeHndl->vh2 == j->begin()))
                    {
                        my_printf("k(%lf, %lf, %lf), vh2(%lf,%lf, %lf), jbegin(%lf, %lf,%lf)",
                               UNFOLD(k->point), UNFOLD(k->nextEdgeHndl->vh2->point), UNFOLD(j->begin()->point));
                        assert(k->nextEdgeHndl->vh2 == j->begin());
                    }
                }
                if( k != j->begin())
                {
                    tmpK = k;
                    --tmpK;
                    if(!(k->prevEdgeHndl->vh1 == tmpK))
                    {
                        my_printf("k(%lf, %lf, %lf), vh1(%lf,%lf, %lf), tmpK(%lf, %lf,%lf)",
                               UNFOLD(k->point), UNFOLD(k->prevEdgeHndl->vh1->point), UNFOLD(tmpK->point));
                        assert(k->prevEdgeHndl->vh1 == tmpK);
                    }
                }else
                {
                    if(!(k->prevEdgeHndl->vh1 == --j->end()))
                    {
                        my_printf("k(%lf, %lf, %lf), vh1(%lf,%lf, %lf), --jend(%lf, %lf,%lf)",
                               UNFOLD(k->point), UNFOLD(k->prevEdgeHndl->vh1->point), UNFOLD((--j->end())->point));
                        assert(k->prevEdgeHndl->vh1 == --j->end());

                    }
                }

                //check SLAV corresponds chains
                if(*i->begin() == k)
                {
                    have_corresponding_LAV = true;
                    circulator = k;

                    for(list<PlanData::Vertex_handle>::iterator m = i->begin(); m != i->end(); ++m)
                    {
                        assert(*m == circulator);
                        circulator = PlanData::next(j, circulator);
                    }

                    break;
                }
            }
        }
        assert(have_corresponding_LAV);
    }
}


//items to delete were only marked before, here they get actually deleted
void delete_with_flags(list<list<PlanData::Vertex_handle> > &chains, set<PlanData::Vertex_handle, compare_vertex_handle> &deleteFlag, PlanData::Vertex_Map &vmap)
{
    for(set<PlanData::Vertex_handle>::iterator i = deleteFlag.begin(); i != deleteFlag.end(); ++i)
    {
        vmap[*i].first->erase(*i);
        for(list<list<PlanData::Vertex_handle> >::iterator k = chains.begin();  k != chains.end(); ++k)
        {
            for(list<PlanData::Vertex_handle>::iterator j= k->begin(); j!= k->end(); )
            {
                if(*j == *i)
                {
                    list<PlanData::Vertex_handle>::iterator jNext = j;
                    ++jNext;
                    k->erase(j);
                    j = jNext;
                }else
                    ++j;
            }
        }
    }
    deleteFlag.clear();
}

//output the cross section
void output_SLAV(list<list<Vertex> > vertices)
{
    //static map<Vertex, int> idMapForDebug;
    //static int id = 1;
    my_printf("**output_SLAV:\n");
    for(list<list<Vertex> >::iterator i = vertices.begin(); i != vertices.end(); ++i)
    {
        my_printf("{{");
        for(list<Vertex>::iterator j = i->begin(); j != i->end(); ++j)
        {
            /*if(idMapForDebug.find(*j) == idMapForDebug.end())
            {
                idMapForDebug[*j] = ++id;
            }*/
            my_printf("%lf,%lf,%lf\n", UNFOLD(j->point));
            debug(j->nextEdgeHndl);
        }
        my_printf("}}\n");

    }
}

//called by interchain_process
void construct_LAV(std::list<std::list<Vertex> >::iterator pLAV, PlanData::Vertex_Map &vmap, map<PlanData::Vertex_handle, PlanData::Vertex_handle, compare_vertex_handle> &next, const Event &event, PlanData &pd, list<list<PlanData::Vertex_handle> >&chains)
{
    list<Vertex> &LAV = *pLAV;
    list<list<Vertex> >::iterator SLAV_it = pd.vertices.begin();
    list<Vertex>::iterator i = SLAV_it->begin(), j, jtmp;
    vmap = pd.get_vertex_map();

    bool end = false;
    //construct one new LAV
    int debugCnt = 1;
    while (!end)
    {

        my_printf("constructLAV: count=%d:\n", debugCnt++);
        if(debugCnt > 10)
        {
            my_printf("!!!!!\n");
        }
        /*myprintf("pd.vertices:\n");
        output_SLAV(pd.vertices);
        myprintf("LAV:\n[");
        for(list<Vertex>::iterator kkk = LAV.begin(); kkk != LAV.end(); ++kkk)
        {
            myprintf("%lf, %lf, %lf\n",UNFOLD(kkk->point));
        }
        myprintf("]\n");
        */

        for(j = i; j != SLAV_it->end(); ++j)
        {
            //myprintf("+j{%lf %lf %lf}-", UNFOLD(j->point));


            /*if(j->point.z() < 1)
            {
                myprintf("caught you\n");
                vmap = pd.get_vertex_map();
                myprintf("j==vmap[i].first->end:%d\n", j == (vmap[i].first)->end());
                myprintf("vmap[i].first==SLAV_it:%d\n", SLAV_it->end() == (vmap[i].first)->end());
                myprintf("j==LAV.end:%d\n", j == (LAV.end()));
            }*/
            if(next.find(j) != next.end())
            {
                //if(debugCnt > 10)
                {
                    my_printf("j(%lf, %lf, %lf), next[j](%lf, %lf, %lf)\n",
                           UNFOLD(j->point),
                           UNFOLD(next[j]->point));
                }
                //jump to another LAV
                jtmp = j;
                LAV.splice(LAV.end(), *SLAV_it, i, ++jtmp);

                //create new vertex
                Vertex newV;
                newV.point = event.eventPoint;
                newV.prevEdgeHndl = j->nextEdgeHndl;
                newV.nextEdgeHndl = next[j]->prevEdgeHndl;
                pd.tmpVetices.push_back(newV);
                PlanData::Vertex_handle newVHandle = --(pd.tmpVetices.end());

                LAV.splice(LAV.end(), pd.tmpVetices, newVHandle);

                j->nextEdgeHndl->vh2 = newVHandle;
                next[j]->prevEdgeHndl->vh1 = newVHandle;

                //update chains
                for(list<list<PlanData::Vertex_handle> >::iterator chainsIter = chains.begin();
                    chainsIter != chains.end(); ++chainsIter)
                {
                    if(*chainsIter->begin() == j)
                    {
                        //assert(chainsIter->size() == 2);

                        chainsIter->insert(++(chainsIter->begin()), newVHandle);
                        break;
                    }
                }


                if (next[j] == LAV.begin())
                {
                    //check if one LAV is done
                    end = true;
                }
                else
                {
                    i = next[j];
                    SLAV_it = vmap[i].first;
                }
                break;
            }
        }
        //in the same LAV
        if (j == SLAV_it->end())
        {
            my_printf("j == SLAV_it->end()\n");
            jtmp = j;
            --jtmp;
            LAV.splice(LAV.end(), *SLAV_it, i, j);

            j = jtmp;
            if (j->nextEdgeHndl->vh2 == LAV.begin())
            {
                end = true;
            } else
            {
                if (SLAV_it->empty())
                {
                    //this should not happen
                    my_printf("WARNING: SLAV_it->empty() in interchain\n");
                    end = true;
                } else
                {
                    i = SLAV_it->begin();
                }
            }
        }
    }


}


void remove_empty_LAV(PlanData &pd)
{
    //remove empty chains;
    for(list<list<Vertex> >::iterator k = pd.vertices.begin(); k != pd.vertices.end(); ++k)
    {
        if(k->empty())
        {
            pd.vertices.erase(k);
        }
    }

}



void interchain_process(list<list<PlanData::Vertex_handle> > &chains, set<PlanData::Vertex_handle, compare_vertex_handle> &deleteFlag, const Event &event, PlanData &pd, MyPolyhedron &ret)
{
    for(list<list<PlanData::Vertex_handle> >::iterator i = chains.begin(); i != chains.end(); ++i)
    {
        assert(i->size() >= 2);
    }
    PlanData::Vertex_Map vmap = pd.get_vertex_map();
    if(chains.size() == 1)
    {
        my_printf("-------------------interchain_process: 1 chain--------------------\n");

        //Only one chain, so no interchain process, but needs to output arcs
        list<PlanData::Vertex_handle> &chain = chains.front();
        PlanData::Vertex_handle v1 = chain.front(), v2 = chain.back();
            if(v1->nextEdgeHndl->vh2 == v2 || v2->prevEdgeHndl->vh1 == v1)
            {
                assert(chain.size() == 2);
                my_printf("Interchain_process:Only a chain with length one,Bug?\n");
                return;
        }else
        {
            ret.push_back(Triangle_3(v1->point, v1->nextEdgeHndl->vh2->point, event.eventPoint));
            ret.push_back(Triangle_3(v2->prevEdgeHndl->vh1->point, v2->point, event.eventPoint));


            delete_with_flags(chains, deleteFlag, vmap);
            //clear_empty_LAV(pd);

            /*is this right?*/
            vmap = pd.get_vertex_map();


            Vertex newV;
            newV.point = event.eventPoint;
            newV.prevEdgeHndl = v1->nextEdgeHndl;
            newV.nextEdgeHndl = v2->prevEdgeHndl;
            PlanData::Vertex_handle newVHandle = (vmap[v2].first)->insert(vmap[v2].second, newV);
            v1->nextEdgeHndl->vh2 = newVHandle;
            v2->prevEdgeHndl->vh1 = newVHandle;
            chain.insert(++chain.begin(), newVHandle);
        }
    }
    else
    {
        my_printf("-------------------interchain_process: Multi chains--------------------\n");
        //output_SLAV(pd.vertices);

        map<PlanData::Vertex_handle, PlanData::Vertex_handle, compare_vertex_handle> next;

        list<list<PlanData::Vertex_handle> >::iterator prev = --chains.end();
        list<list<PlanData::Vertex_handle> > newChains;
        //record the new connections
        for(list<list<PlanData::Vertex_handle> >::iterator i = chains.begin(); i != chains.end(); ++i)
        {
            if (i->size() == 2)
            {
                //if length one, add a new edge
                PlanData::Vertex_handle tmpVh = i->front();
                PlanData::Edge_handle newE = pd.duplicateEdge(tmpVh->nextEdgeHndl);

                tmpVh->nextEdgeHndl = newE;

                /*Is this right?*/
                tmpVh->nextEdgeHndl->nextDirectionPlane = (prev->back())->prevEdgeHndl->directionPlane;
                (prev->back())->prevEdgeHndl->prevDirectionPlane = tmpVh->nextEdgeHndl->directionPlane;
                if(parallel(tmpVh->nextEdgeHndl, (prev->back())->prevEdgeHndl))
                {
                    /*Is this right?*/
                    tmpVh->nextEdgeHndl->nextDirectionPlane = (prev->back())->prevEdgeHndl->prevDirectionPlane;
                    (prev->back())->prevEdgeHndl->prevDirectionPlane = tmpVh->nextEdgeHndl->nextDirectionPlane;

                }
            }

            PlanData::Vertex_handle v1 = i->front(), v2 = i->back();
            ret.push_back(Triangle_3(v1->point, v1->nextEdgeHndl->vh2->point, event.eventPoint));
            ret.push_back(Triangle_3(v2->prevEdgeHndl->vh1->point, v2->point, event.eventPoint));

            list<PlanData::Vertex_handle> newChain;
            newChain.push_back(v1);
            newChain.push_back(prev->back());
            newChains.push_back(newChain);
            next[v1] = prev->back();
            prev = i;
        }

        delete_with_flags(chains, deleteFlag, vmap);
        chains = newChains;
        for(list<list<PlanData::Vertex_handle> >::iterator i = chains.begin(); i != chains.end(); ++i)
        {
            assert(i->size() == 2);
        }
        //clear_empty_LAV(pd);


        std::list<std::list<Vertex> > &newVertices = pd.newVertices;
        //keep moving vertices into correct place
        while(pd.vertices.size() != 0)
        {
            newVertices.push_back(list<Vertex>());
            construct_LAV(--(newVertices.end()), vmap, next, event, pd, chains);

            remove_empty_LAV(pd);
        }

        for(std::list<std::list<Vertex> >::iterator i = newVertices.begin();
            i != newVertices.end(); ++i)
        {
            pd.vertices.push_back(std::list<Vertex >());
            std::list<std::list<Vertex> >::iterator j = --(pd.vertices.end());
            j->splice(j->end(), *i);
        }
        //delete temporary mems
        pd.newVertices.clear();
        pd.tmpVetices.clear();

        //output_SLAV(pd.vertices);
    }

}
void intrachain_process(list<list<PlanData::Vertex_handle> > &chains, set<PlanData::Vertex_handle, compare_vertex_handle> &deleteFlag, const Event &event, PlanData &pd, MyPolyhedron &ret)
{
    //test validity
    //chains_corresponds_SLAV(chains, pd);
    PlanData::Vertex_Map vmap = pd.get_vertex_map();
    //NOTE:remember to update i
    for(list<list<PlanData::Vertex_handle> >::iterator i = chains.begin(); i != chains.end(); )
    {
        if((*i->begin()) == (*i->rbegin()))
        {
            my_printf("+++++++++++++++intrachain_process: Ring!!!++++++++++\n");

            for(list<PlanData::Vertex_handle>::iterator j = i->begin(); j != i->end(); ++j)
            {
                //myprintf("%lf, %lf, %lf\n", UNFOLD((*j)->point));
                list<PlanData::Vertex_handle>::iterator jNext = j;
                ++jNext;
                if(jNext != i->end())
                    ret.push_back(Triangle_3((*j)->point, (*jNext)->point, event.eventPoint));
            }
            //remove corresponding LAV
            pd.vertices.erase(vmap[*i->begin()].first);
            list<list<PlanData::Vertex_handle> >::iterator tmpI = i;
            ++tmpI;
            chains.erase(i);
            i = tmpI;
        }
        else
        {
            my_printf("+++++++++++++++intrachain_process: Not ring++++++++++\n");

            //NOTE: remember to update m
            for(list<PlanData::Vertex_handle>::iterator m = i->begin(); m != i->end(); ++m )
            {
                //bool erased = false;
                if(m != i->begin())
                {
                    list<PlanData::Vertex_handle>::iterator tmpM;
                    tmpM = m;
                    ++tmpM;
                    if(tmpM != i->end())
                    {
                        ++tmpM;
                        if(tmpM != i->end())
                        {
                            --tmpM;
                            ret.push_back(Triangle_3((*m)->point, (*tmpM)->point, event.eventPoint));
                        }
                        else
                            --tmpM;
                        deleteFlag.insert(*m);
                        //vmap[*m].first->erase(*m);
                        //i->erase(m);
                        //m = tmpM;
                        //erased = true;

                    }
                }
                //if(!erased) ++m;
            }

            ++i;
       }

    }

    remove_empty_LAV(pd);

}
extern void render();
void show_chains(list<list<PlanData::Vertex_handle> > chains)
{

    my_printf("chains_begin:\n");
    for(list<list<PlanData::Vertex_handle> >::iterator i = chains.begin(); i != chains.end(); ++i)
    {
        my_printf("chain_begin:\n");
        for(list<PlanData::Vertex_handle>::iterator j = i->begin(); j != i->end(); ++j)
        {
            my_printf("%lf, %lf, %lf\n", UNFOLD((*j)->point));
        }
        my_printf("chain_end\n");
    }
    output = chains;
    display_test_sw3();
    //render();
    //glutPostRedisplay();
    my_printf("chains_end\n\n");
}


void handle_direction_event(MyPolyhedron &ret, PlanData &pd, const Event &event)
{
    my_printf("Handle direction event, tag=%d\n", event.profileTag);

    double z = event.eventPoint.z();
    Plane_3 sweepPlane(Point_3(0,0,z), Point_3(0,1,z), Point_3(1, 0, z));
    Point_3 p1, p2;

    std::map<PlanData::Edge_handle, std::pair<Point_3, Point_3>, compare_edge_handl > mapForUpdate;

    for(PlanData::SLAV_iterator SLAVIter = pd.SLAV_begin(); SLAVIter != pd.SLAV_end(); ++SLAVIter)
    {
        for(list<Vertex>::iterator vIter = SLAVIter->begin(); vIter != SLAVIter->end(); ++vIter)
        {
            PlanData::Edge_handle edge = vIter->nextEdgeHndl;
            if(edge->profileTag != event.profileTag)
                continue;
            if(!my_intersect(edge->directionPlane,
                              edge->vh1->prevEdgeHndl->directionPlane,
                              sweepPlane, p1))
            {
                if(parallel(edge, edge->vh1->prevEdgeHndl))
                {
                    p1 = edge->vh1->point;
                    /*if(!my_intersect(edge->directionPlane,
                                                  edge->prevDirectionPlane,
                                                  sweepPlane, p1)) continue;*/
                }else
                    continue;
            }

            if(!my_intersect(edge->directionPlane,
                              edge->vh2->nextEdgeHndl->directionPlane,
                              sweepPlane, p2))
            {
                if(parallel(edge, edge->vh2->nextEdgeHndl))
                {
                    p2 = edge->vh2->point;
                    /*if(!my_intersect(edge->directionPlane,
                                                  edge->nextDirectionPlane,
                                                  sweepPlane, p2)) continue;*/
                }else
                    continue;
            }

            //output
            ret.push_back(Triangle_3(edge->vh1->point, edge->vh2->point, p2));
            ret.push_back(Triangle_3(p2, p1, edge->vh1->point));

            //create new edge and update later, if update now, the output part will be invalid
            PlanData::Edge_handle newE = pd.duplicateEdge(edge);
            edge->vh1->nextEdgeHndl = newE;
            edge->vh2->prevEdgeHndl = newE;
            assert(mapForUpdate.find(newE) == mapForUpdate.end());
            mapForUpdate[newE] = make_pair(p1, p2);
        }
    }


    for(PlanData::SLAV_iterator SLAVIter = pd.SLAV_begin(); SLAVIter != pd.SLAV_end(); ++SLAVIter)
    {
        for(list<Vertex>::iterator vIter = SLAVIter->begin(); vIter != SLAVIter->end(); ++vIter)
        {
            PlanData::Edge_handle edge = vIter->nextEdgeHndl;
            if(edge->profileTag != event.profileTag) continue;
            if(mapForUpdate.find(edge) != mapForUpdate.end())
            {
                p1 = mapForUpdate[edge].first;
                p2 = mapForUpdate[edge].second;
                //update
                edge->p1 = p1;
                edge->p2 = p2;
                edge->vh1->point = p1;
                edge->vh2->point = p2;

                edge->directionPlane = get_plane_from_edge_and_angle(p1, p2, event.newAngle);

                /*Is this right?*/
                edge->vh1->prevEdgeHndl->nextDirectionPlane = edge->directionPlane;
                edge->vh2->nextEdgeHndl->prevDirectionPlane = edge->directionPlane;

      /*          myprintf("after: plane_3 %lf, %lf, %lf, %lf\n", edge->directionPlane.a(),
                       edge->directionPlane.b(),
                       edge->directionPlane.c(),
                       edge->directionPlane.d());
        */
            }
          }
    }
}

void remove_area0_LAV(PlanData &pd)
{

    for(PlanData::SLAV_iterator SLAVIter = pd.SLAV_begin(); SLAVIter != pd.SLAV_end(); )
    {

        if(SLAVIter->size() <= 2)
        {
            PlanData::SLAV_iterator tmpI = SLAVIter;
            ++tmpI;
            pd.vertices.erase(SLAVIter);
            SLAVIter = tmpI;
            continue;
        }

        bool hasArea = false;
        for(list<Vertex>::iterator vIter = SLAVIter->begin(); vIter != SLAVIter->end(); ++vIter)
        {
            if(!parallel(vIter->nextEdgeHndl, vIter->prevEdgeHndl))
            {
                hasArea = true;
                break;
            }
        }
        if(!hasArea)
        {
            //all parallel
            PlanData::SLAV_iterator tmpI = SLAVIter;
            ++tmpI;
            pd.vertices.erase(SLAVIter);
            SLAVIter = tmpI;
            continue;
        }

        ++SLAVIter;

    }
}

void do_extrusion(MyPolyhedron &ret,  PlanData &pd, double startHeight, double endHeight);
//helper function
void construct_tmp_floor( Editor &tmpFloor, PlanData &pd, const Event& event)
{


    double z = event.eventPoint.z();
    Plane_3 sweepPlane(Point_3(0,0,z), Point_3(0,1,z), Point_3(1, 0, z));
    Point_3 p1, p2;
    for(PlanData::SLAV_iterator SLAVIter = pd.SLAV_begin(); SLAVIter != pd.SLAV_end(); ++SLAVIter)
    {
        for(list<Vertex>::iterator vIter = SLAVIter->begin(); vIter != SLAVIter->end(); ++vIter)
        {
            PlanData::Edge_handle edge = vIter->nextEdgeHndl;

            if(!my_intersect(edge->directionPlane,
                              edge->vh1->prevEdgeHndl->directionPlane,
                              sweepPlane, p1))
            {
                if(parallel(edge, edge->vh1->prevEdgeHndl))
                {
                    p1 = edge->vh1->point;

                }else
                    continue;
            }

            if(!my_intersect(edge->directionPlane,
                              edge->vh2->nextEdgeHndl->directionPlane,
                              sweepPlane, p2))
            {
                if(parallel(edge, edge->vh2->nextEdgeHndl))
                {
                    p2 = edge->vh2->point;
                }else
                    continue;
            }
            tmpFloor.add_edge(Point(p1.x(), p1.y()), Point(p2.x(), p2.y()), false);
        }
    }
}


void construct_floor_editor_from_SLAV(Editor &floor, PlanData &pd, int tag)
{
    for(PlanData::SLAV_iterator i = pd.SLAV_begin(); i != pd.SLAV_end(); ++i)
    {
        for(PlanData::Vertex_handle j = i->begin(); j != i->end(); ++j)
        {
            Point_3 p1, p2;
            p1 = j->nextEdgeHndl->p1;
            p2 = j->nextEdgeHndl->p2;
            floor.add_edge(Point(p1.x(), p1.y()), Point(p2.x(), p2.y()), tag, false);

        }

    }
}
void construct_profile_editor_from_profile(Editor &profileEditor, std::vector<Point> profile, int tag)
{
    vector<Point>::iterator prev;
    for(vector<Point>::iterator i = profile.begin(); i != profile.end(); ++i)
    {
        if(i != profile.begin())
            profileEditor.add_edge(*prev, *i, tag, false);
        prev = i;
    }

}

extern MyPolyhedron shell;
void add_faces_from_traingulation(MyPolyhedron &ret, std::vector<Polygon_2> pgnsNew, double startHeight);

void merge_overhang(PlanData &pd, Editor &floor, Editor&profile, double startHeight)
{
    pd.do_construction(floor, profile, startHeight);
}

//handle profiel offset events
void handle_profile_offset_event(MyPolyhedron &ret, PlanData &pd, const Event& event)
{
    assert(event.type == EVENT_TYPE_PROFILE_OFFSET);
    if(no_profile_offset_event) return;
    my_printf("Handle profile offset event...\n");
    //construct temporary profile
    Editor tmpProfile1, tmpProfile2;
    Point prop1(0,0), prop2(event.dist_inside, 1), prop3(event.dist_outside, 2);
    tmpProfile1.add_edge(prop1, prop2, false);
    tmpProfile1.add_edge(prop2, prop3, false);
    tmpProfile2.add_edge(prop1, prop2, false);
    tmpProfile2.add_edge(prop2, prop3, false);
    //this point should not matter
    tmpProfile2.add_edge(prop3, Point(0, 3), false);

    //construct temporary floor
    Editor tmpFloor1, tmpFloor2;
    construct_tmp_floor(tmpFloor1, pd, event);
    construct_tmp_floor(tmpFloor2, pd, event);

    //save context
    //MyPolyhedron shellSaved = shell;

    Editor projectedFloor, projectedProfile;
    int inside_tag = pd.profile.get_overhanging_inside_tag(event.profileTag);
    int outside_tag = pd.profile.get_overhanging_outside_tag(event.profileTag);

    MyPolyhedron tmpRet1, tmpRet2;

    //extrude for inside roof
    PlanData tmpPd1(tmpFloor1, tmpProfile1);
    //shell = tmpRet1;
    do_extrusion(tmpRet1, tmpPd1, 0, 1);
    construct_floor_editor_from_SLAV(projectedFloor, tmpPd1, inside_tag);
    construct_profile_editor_from_profile(projectedProfile, event.profile_inside, inside_tag);

    //extrude for outside roof
    PlanData tmpPd2(tmpFloor2, tmpProfile2);
    //shell = tmpRet2;
    do_extrusion(tmpRet2, tmpPd2, 0, 2);
    construct_floor_editor_from_SLAV(projectedFloor, tmpPd2, outside_tag);
    construct_profile_editor_from_profile(projectedProfile, event.profile_outside, outside_tag);

    //merge the overhanging data to pd
    //merge_overhang(pd, projectedFloor, projectedProfile, event.eventPoint.z());

    //get the cross section of the overhang
    std::vector<Polygon_2> pgns = projectedFloor.get_polygons();
    my_printf("pgn.size()=%d\n", pgns.size());
    std::vector<Polygon_2> pgnsNew = Extrusion().triangulate(pgns);
    my_printf("pgnsNew.size()=%d\n", pgnsNew.size());
    add_faces_from_traingulation(ret, pgnsNew, event.eventPoint.z());

    //insert the overhang to plan data
    //add_offset_region_to_SLAV(pd, projectedFloor, outside_tag);

    //shell = shellSaved;

}

//Extrusion algorithm
void do_extrusion(MyPolyhedron &ret,  PlanData &pd, double startHeight, double endHeight)
{




    //main loop priority queue
    std::priority_queue<Event, std::vector<Event>, compare_event> eventQueue, historyEvents;
    //Intersection Event;
    insert_intersection_events(eventQueue, historyEvents, pd, startHeight);
    insert_direction_events(eventQueue, pd);
    insert_profile_offset_events(eventQueue, pd);

    //assume all coordinates >=0
    double sweepZ = -1;
    output_SLAV(pd.vertices);

    //pd.update_plane_to_edge_map();
    //printf("do extrusion\n");
    //int cnt = 0, validCnt=0;
    int cnt = 0;
    while(!eventQueue.empty())
    {
        ++ cnt;
        //my_printf("\ncnt = %d, eventQueue size:%d\n", cnt, eventQueue.size());

        Event event = eventQueue.top();
        historyEvents.push(event);
        /*if(event.eventPoint.z() < sweepZ - 1e-4)
        {
            myprintf("why?\n");
            continue;
        }*/
        sweepZ = event.eventPoint.z();
        if(endHeight > 0 && sweepZ > endHeight)
        {
            break;
        }
        if(pd.vertices.empty())
        {
            break;
        }
        my_printf("sweepZ = %lf\n", sweepZ);

        if(event.type == EVENT_TYPE_GENERALIZED_INTERSECTION)
        {
            //cluster events
            list<Event> clustered = cluster_intersect_events(eventQueue, sweepZ, event.eventPoint);
            {
                my_printf("===clusted %d event(s)\n", clustered.size());
                for(list<Event>::iterator it = clustered.begin(); it != clustered.end(); ++it)
                {
                    my_printf("point=%lf, %lf, %lf\n", UNFOLD(it->eventPoint));
                    my_printf("edge (%lf, %lf, %lf)->(%lf,%lf,%lf)\n", UNFOLD(it->edge->p1), UNFOLD(it->edge->p2) );
                    debug(it->edge);
                    my_printf("prev (%lf, %lf, %lf)->(%lf,%lf,%lf)\n", UNFOLD(it->cornerPrevEdge->p1), UNFOLD(it->cornerPrevEdge->p2) );
                    debug(it->cornerPrevEdge);
                    my_printf("next (%lf, %lf, %lf)->(%lf,%lf,%lf)\n", UNFOLD(it->cornerNextEdge->p1), UNFOLD(it->cornerNextEdge->p2) );
                    debug(it->cornerNextEdge);
                }
                my_printf("\n");
            }

            //Filter out invalid events
            list<list<PlanData::Vertex_handle> > chains;// = get_chains_old(clustered,pd);
            //sort_chains(chains);
            //show_chains(chains);

            set<PlanData::Edge_handle, Extrsn::compare_edge_handl> intersect_edges_from_event;
            remove_invalid_intersect_events(clustered, intersect_edges_from_event, pd);
            if(clustered.size() == 0)
                continue;

            chains = get_chains(intersect_edges_from_event, pd);
            set<PlanData::Vertex_handle, Extrsn::compare_vertex_handle> deleteFlag;
            sort_chains(chains);

            //show_chains(chains);
            intrachain_process(chains, deleteFlag, event, pd, ret);
            //show_chains(chains);
            //chains_corresponds_SLAV(chains, pd);
            interchain_process(chains, deleteFlag, event, pd, ret);
            //show_chains(chains);

            //chains_corresponds_SLAV(chains, pd);

            remove_area0_LAV(pd);


        } else if(event.type == EVENT_TYPE_EDGE_DIRECTION_NON_HORIZONTAL)
        {
            eventQueue.pop();
            handle_direction_event(ret, pd, event);


            //display_test_sw3();
            //***!SLAV_sanity_check(pd);
            //For standard edge direction(in main loop do intersection first), now no edges should shrink, so
            //Just need to replace every edge
        } else if(event.type == EVENT_TYPE_PROFILE_OFFSET)
        {
            eventQueue.pop();
            handle_profile_offset_event(ret, pd, event);
            //display_test_sw3();
            //assert(false);
        }

        output_SLAV(pd.vertices);
        insert_intersection_events(eventQueue,historyEvents, pd, sweepZ);
        //pd.update_plane_to_edge_map();
    }

}


void add_faces_from_traingulation(MyPolyhedron &ret, std::vector<Polygon_2> pgnsNew, double startHeight)
{

    for(std::vector<Polygon_2>::iterator it = pgnsNew.begin(); it != pgnsNew.end(); ++it)
    {
        Polygon_2 triangle = *it;
        Point_3 p[3];
        assert(triangle.is_counterclockwise_oriented());
        for(int i=0; i<3; ++i)
        {
            p[i] = Point_3(triangle[i].x(), triangle[i].y(), startHeight);
        }
        ret.push_back(Triangle_3(p[1], p[0], p[2]));
    }
}

void Extrusion::build_polyhedron( Editor &floor,  Editor& profile, MyPolyhedron &ret)
{
    if(profile.get_edges().empty() || floor.get_edges().empty()) return;

    //Triangulate the initial floor plan
    //MyPolyhedron ret;
    ret = std::vector<Triangle_3>();
    std::vector<Polygon_2> pgns = floor.get_polygons();
    std::vector<Polygon_2> pgnsNew = this->triangulate(pgns);
    double startHeight = profile.get_start_height();

    add_faces_from_traingulation(ret, pgnsNew, startHeight);
    /*
    for(std::vector<Polygon_2>::iterator it = pgnsNew.begin(); it != pgnsNew.end(); ++it)
    {
        Polygon_2 triangle = *it;
        Point_3 p[3];
        assert(triangle.is_counterclockwise_oriented());
        for(int i=0; i<3; ++i)
        {
            p[i] = Point_3(triangle[i].x(), triangle[i].y(), startHeight);
        }
        ret.push_back(Triangle_3(p[1], p[0], p[2]));
    }*/

    try{
        //extrude the plan
        //double startHeight = profile.get_start_height();
        //construct plan data structure and direction plane
        PlanData pd(floor, profile);
        do_extrusion(ret, pd, startHeight, -1);
    }catch(...)
    {
        printf("Exception");
    }
}

Extrusion::Extrusion(){}

//check if the triangle is in the hole of the model, it will be removed if it is
bool triangle_is_hole(std::vector<Polygon_2> &allPgns, Point_2 ps[])
{
    Polygon_2 clockPgn, counterClockPgn;
    bool hasClockwisePgn=false;
    Polygon_2 pgn[3];
    //get the incident polygons for each point of the triangle
    for(int j=0; j<3; ++j)
    {
        for(std::vector<Polygon_2>::iterator i = allPgns.begin(); i != allPgns.end(); ++i)
        {
            if(i->has_on_boundary(ps[j]))
            {
                pgn[j] = *i;
            }
        }
    }
    //record orientation for later use
    for(int j=0; j<3; ++j)
    {
        if(pgn[j].is_clockwise_oriented())
        {
            hasClockwisePgn = true;
            clockPgn = pgn[j];
        } else
        {
            counterClockPgn = pgn[j];
        }
    }
    //check if the triangle is a hole by checking the centroid position
    Point_2 centroid = CGAL::centroid(ps[0], ps[1], ps[2]);
    if(!hasClockwisePgn)
    {
        // all polygons counter Clock oriented
        return counterClockPgn.has_on_unbounded_side(centroid);
    }
    else
    {
        return clockPgn.has_on_bounded_side(centroid);
    }
}



//Generate the triangulation of the pgns
std::vector<Polygon_2> Extrusion::triangulate(std::vector<Polygon_2> pgns)
{
    CDT cdt;
    std::map<Point_2, Vertex_handle> map;
    //map vertices to their handles for later use
    for(std::vector<Polygon_2>::iterator i = pgns.begin(); i != pgns.end(); ++i)
    {
        my_printf("+++pgn:\n");
        for(Polygon_2::Vertex_iterator j = i->vertices_begin(); j!= i->vertices_end(); ++j)
        {
            CDT::Vertex_handle h = cdt.insert(*j);
            map[*j] = h;
            my_printf("%lf, %lf\n", j->x(), j->y());
        }
        my_printf("+++\n");

    }
    //insert constraints to triangulation library
    for(std::vector<Polygon_2>::iterator i = pgns.begin(); i != pgns.end(); ++i)
    {
        Vertex_handle lastH, firstH;
        bool isFirst = true;
        for(Polygon_2::Vertex_iterator j = i->vertices_begin(); j!= i->vertices_end(); ++j)
        {
            if(isFirst)
            {
                firstH = map[*j];
                isFirst = false;
            } else
            {
                cdt.insert_constraint(lastH, map[*j]);
            }
            lastH = map[*j];
        }
        cdt.insert_constraint(lastH, firstH);
    }
    //remove the holes
    std::vector<Polygon_2> pgnsNew;
    for(CDT::Face_iterator i = cdt.faces_begin(); i!= cdt.faces_end(); ++i)
    {
        Point_2 ps[] = {i->vertex(0)->point(), i->vertex(1)->point(), i->vertex(2)->point()};
        //myprintf("TriIsHole=%d\n", TriIsHole(pgns, ps));
        if(triangle_is_hole(pgns, ps)) continue;
        Polygon_2 pgn;
        for(int j=0; j<3; ++j)
        {
            pgn.push_back(i->vertex(j)->point());
            //myprintf("test: %lf %lf\n", i->vertex(j)->point().x(), i->vertex(j)->point().y());
        }
        pgnsNew.push_back(pgn);
    }
    return pgnsNew;
}



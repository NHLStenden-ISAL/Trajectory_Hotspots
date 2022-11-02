#include "pch.h"
#include "segment_intersection.h"
#include "sweep_line_status_structure.h"

#include <functional>

using namespace Segment_Intersection_Sweep_Line;
std::vector<Vec2> Segment_Intersection_Sweep_Line::find_segment_intersections(const std::vector<Segment>& segments)
{
    //std::vector<const Vec2*> top_points;
    //std::vector<const Vec2*> bottom_points;


    //Event queue sorted on point, values are the event type and index in the segment list
    
    std::multimap<const Vec2, Event, std::less<Vec2>> event_queue;

    for (int i = 0; i < segments.size(); i++)
    {
        //TODO: There is a copy here, reference instead?
        event_queue.emplace(*segments.at(i).get_top_point(), Event(Intersection_Event_Type::TOP, i));
        event_queue.emplace(*segments.at(i).get_bottom_point(), Event(Intersection_Event_Type::BOTTOM, i));
    }

    //Initialize status structure with the highest event point
    Sweep_Line_Status_structure status_structure(event_queue.begin()->first.y);

    while (!event_queue.empty())
    {
        auto event = event_queue.begin();
        

        switch (event->second.event_type)
        {
        case Intersection_Event_Type::TOP:
            top(status_structure, segments, event_queue, event->second.segment_index);   
            break;
        case Intersection_Event_Type::BOTTOM:
            bottom(status_structure, event_queue, event->second.segment_index);
            break;
        case Intersection_Event_Type::INTERSECTION:
            
            break;
        default:
            break;
        }
        //TODO: dont add before first event!!
        event_queue.erase(event_queue.begin());
    }
    //Create event_class
    //Add ordering function (y<y, x<x)
    //Add top and bottom events, add reference to segment
    //Add events to sorted queue, take into account same points..
    //Handle events, add intersection event to queue, switch order of segment, do intersect test.



    //If event == INTERSECTION, std::swap
    //If event == TOP, Add to tree
    //If event == BOTTOM, Remove from tree

    return std::vector<Vec2>();
}

//TODO Should this be a class instead of a function?
void Segment_Intersection_Sweep_Line::top(Sweep_Line_Status_structure& status_structure, const std::vector<Segment>& segments, std::multimap<const Vec2, Event, std::less<Vec2>>& event_queue, const int new_segment)
{
    int left = -1;
    int right = -1;
    status_structure.insert(segments,new_segment, left, right);
    //status_structure.set_line_position(&segment.get_top_point()->y);
    
    Vec2* intersection;
    if (left != -1)
    {
        if (intersection_two_segments(&segments.at(new_segment), &segments.at(left), intersection))
        {
            event_queue.emplace(status_structure,Event(Intersection_Event_Type::INTERSECTION, ), intersection ));
           //add intersection // check neighbour?
        }
    }
    if(right != -1)
    {
        if (intersection_two_segments(&segment, right, intersection))
        {
            //add intersection // check neighbour?
        }
    }
    // returns node (with neighbours pointers) or null???
    // if( && ==!null)
    // insert segment
    //status_structure.checkneighbours(&segment);
    //check for intersection new neighbour
    // intersect(get neighbours)
    // insert intersection point to event queue
}

void Segment_Intersection_Sweep_Line::bottom(Sweep_Line_Status_structure& status_structure, const std::vector<Segment>& segments, std::multimap < const Vec2, Event, std::less<Vec2>>& event_queue, const int segment)
{
    status_structure.remove(&segment);
    //check for intersection new neighbour
    // intersect(get neighbours)
    // insert intersection point to event
}

void Segment_Intersection_Sweep_Line::intersection() 
{
   
    //swap Si(links) Sj(rechts) in status structure
    //check intersection between Si-1 Sj and Si+1 Sj
    //add new intersection event in queue
}


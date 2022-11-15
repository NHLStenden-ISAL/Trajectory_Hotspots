#include "pch.h"
#include "segment_intersection.h"
#include "sweep_line_status_structure.h"

#include <functional>


using namespace Segment_Intersection_Sweep_Line;

using map = std::map<const Vec2, Event, std::greater<Vec2>>;

std::vector<Vec2> Segment_Intersection_Sweep_Line::find_segment_intersections(const std::vector<Segment>& segments)
{
    //std::vector<const Vec2*> top_points;
    //std::vector<const Vec2*> bottom_points;


    //Event queue sorted on point, values are the event type and index in the segment list
    
    //all multimap needs to be map typedef?
    //std::map<const Vec2, Event, std::greater<Vec2>> event_queue;
    map event_queue;
    for (int i = 0; i < segments.size(); i++)
    {
        //TODO: There is a copy here, reference instead?
        event_queue.emplace(*segments.at(i).get_top_point(), Event(Intersection_Event_Type::TOP, i));
        event_queue.emplace(*segments.at(i).get_bottom_point(), Event(Intersection_Event_Type::BOTTOM, i));
    }

    //Initialize status structure with the highest event point
    Sweep_Line_Status_structure status_structure(event_queue.begin()->first.y);
    std::vector<Vec2> result;
    while (!event_queue.empty())
    {
        auto event = event_queue.begin();


        switch (event->second.event_type)
        {
        case Intersection_Event_Type::TOP:
            top(status_structure, segments, event_queue, event->second.segment_index, event_queue.begin()->first);
            break;
        case Intersection_Event_Type::BOTTOM:
            bottom(status_structure, segments, event_queue, event->second.segment_index, event_queue.begin()->first);
            break;
        case Intersection_Event_Type::INTERSECTION:
            result.emplace_back(event->first);
            intersection(status_structure, segments, event_queue, event->second.segment_index, event->second.other_segment_index, event_queue.begin()->first);
			
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

    return result;
}

//TODO Should this be a class instead of a function?
void Segment_Intersection_Sweep_Line::top(Sweep_Line_Status_structure& status_structure, const std::vector<Segment>& segments, map& event_queue, const int new_segment, Vec2 event_point)
{
    status_structure.set_line_position(event_point.y);
    int left_segment = -1;
    int right_segment = -1;
    status_structure.insert(segments, new_segment, left_segment, right_segment);
    
    
    Vec2 intersection;
    if (left_segment != -1)
    {
        if (Segment::intersection_two_segments(&segments.at(new_segment), &segments.at(left_segment), intersection))
        {
           event_queue.emplace(intersection, Event(Intersection_Event_Type::INTERSECTION, left_segment, new_segment));
           //add intersection // check neighbour?
        }
    }
    if(right_segment != -1)
    {
        if (Segment::intersection_two_segments(&segments.at(new_segment), &segments.at(right_segment), intersection))
        {
            event_queue.emplace(intersection, Event(Intersection_Event_Type::INTERSECTION, new_segment, right_segment));
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

void Segment_Intersection_Sweep_Line::bottom(Sweep_Line_Status_structure& status_structure, const std::vector<Segment>& segments, map& event_queue, const int segment, Vec2 event_point)
{
    status_structure.set_line_position(event_point.y);
	int left_segment = -1;
	int right_segment = -1;
    status_structure.remove(segments, segment, left_segment, right_segment);
    

    Vec2 intersection;
    if (left_segment != -1)
    {
        if (right_segment != -1)
        {
            if (Segment::intersection_two_segments(&segments.at(left_segment), &segments.at(right_segment), intersection))
            {
                event_queue.emplace(intersection, Event(Intersection_Event_Type::INTERSECTION, left_segment, right_segment));
                //add intersection // check neighbour?
            }
        }
    }
    //check for intersection new neighbour
    // intersect(get neighbours)
    // insert intersection point to event
}


void Segment_Intersection_Sweep_Line::intersection(Sweep_Line_Status_structure& status_structure,  const std::vector<Segment>& segments, map& event_queue, int p1, int p2, Vec2 event_point)
{
    Vec2 intersection;
    int left_segment = -1;
    int right_segment = -1;
    
    status_structure.swap_elements(segments, p1, p2, left_segment, right_segment);
        // p1 = left p2 = right after swap reversed
    status_structure.set_line_position(event_point.y);
    if (left_segment != -1)
    {
        if (Segment::intersection_two_segments(&segments.at(p1), &segments.at(left_segment), intersection))
        {
            if (intersection.y < event_point.y)
            {
                event_queue.emplace(intersection, Event(Intersection_Event_Type::INTERSECTION, p1, right_segment));
            }
        }
        
    }
    if (right_segment != -1)
    {
        if (Segment::intersection_two_segments(&segments.at(p2), &segments.at(right_segment), intersection))
        {
            if (intersection.y < event_point.y)
            {
                event_queue.emplace(intersection, Event(Intersection_Event_Type::INTERSECTION, p2, right_segment));
            }
        }
    }


    //swap Si(links) Sj(rechts) in status structure
    //check intersection between Si-1 Sj and Si+1 Sj
    //add new intersection event in queue
}




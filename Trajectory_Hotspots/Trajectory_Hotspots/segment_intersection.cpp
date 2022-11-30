#include "pch.h"
#include "sweep_line_status_structure.h"
#include "segment_intersection.h"


#include <functional>

using namespace Segment_Intersection_Sweep_Line;
std::vector<Vec2> Segment_Intersection_Sweep_Line::find_segment_intersections(const std::vector<Segment>& segments)
{
    map event_queue;

    //std::vector<const Vec2*> top_points;
    //std::vector<const Vec2*> bottom_points;

    for (int i = 0; i < segments.size(); i++)
    {

        //TODO: There is a copy of Vec2 here, reference instead?
        auto event_pair = event_queue.emplace(*segments.at(i).get_top_point(), std::vector<int>());
        event_pair.first->second.push_back(i);

        event_pair = event_queue.emplace(*segments.at(i).get_bottom_point(), std::vector<int>());
    }

    //Initialize status structure with the highest event point
    Sweep_Line_Status_structure status_structure(event_queue.begin()->first.y);

    std::vector<Segment> result;

    std::vector<Vec2> intersections;

    while (!event_queue.empty())
    {
        Handle_Event(status_structure, event_queue, segments, event_queue.begin()->first, event_queue.begin()->second, result);
        if (result.size() > 0)
        {
            intersections.push_back(event_queue.begin()->first);
        }
        event_queue.erase(event_queue.begin()); 
    }



    //while (!event_queue.empty())
    //{
    //    auto event = event_queue.begin();
    //
    //
    //    switch (event->second.event_type)
    //    {
    //    case Intersection_Event_Type::TOP:
    //        top(status_structure, segments, event_queue, event->second.segment_index, event_queue.begin()->first);
    //        break;
    //    case Intersection_Event_Type::BOTTOM:
    //        bottom(status_structure, segments, event_queue, event->second.segment_index, event_queue.begin()->first);
    //        break;
    //    case Intersection_Event_Type::INTERSECTION:
    //        result.emplace_back(intersection(status_structure, event_queue, segments, event->second.segment_index, event->second.other_segment_index, event_queue.begin()->first));
    //        break;
    //    default:
    //        break;
    //    }
    //    //TODO: dont add before first event!!
    //    event_queue.erase(event_queue.begin());
    //}

    //Create event_class
    //Add ordering function (y<y, x<x)
    //Add top and bottom events, add reference to segment
    //Add events to sorted queue, take into account same points..
    //Handle events, add intersection event to queue, switch order of segment, do intersect test.



    //If event == INTERSECTION, std::swap
    //If event == TOP, Add to tree
    //If event == BOTTOM, Remove from tree

    return intersections;
}


void Segment_Intersection_Sweep_Line::Handle_Event(
    Sweep_Line_Status_structure& status_structure,
    map& event_queue,
    const std::vector<Segment>& segments,
    const Vec2& event_point,
    const std::vector<int>& top_segments,
    std::vector<Segment>& result_segments)
{
    std::vector<int> intersections;
    std::vector<int> bottom_segments;
    int left_neighbour = -1;
    int right_neighbour = -1;
    int most_left_segment = -1;
    int most_right_segment = -1;

    //TODO: float not best solution here
    float line_pos = event_point.y;
    status_structure.set_line_position(line_pos);

    status_structure.get_all_nodes_on(segments, event_point, intersections, bottom_segments, most_left_segment, most_right_segment);

    if (top_segments.size() + bottom_segments.size() + intersections.size() > 1)
    {
        for (int segment : top_segments)
        {
            result_segments.push_back(segments.at(segment));
        }
        for (int segment : bottom_segments)
        {
            result_segments.push_back(segments.at(segment));
        }
        for (int segment : intersections)
        {
            result_segments.push_back(segments.at(segment));
        }
        //TODO return intersection + segments, check later if from same dcel
        //add to result
    }

    // after the final remove we have the right and left neighbour of all intersecting segments
    for (int segment : bottom_segments)
    {
        status_structure.remove(segments, segment, left_neighbour, right_neighbour);
    }
    for (int segment : intersections)
    {
        status_structure.remove(segments, segment, left_neighbour, right_neighbour);
    }

    //delete lower and intersection
    // set_lineposition 

    

    bool neighbours_filled = false;
    if (left_neighbour != -1 || right_neighbour != -1)
    {
        neighbours_filled = true;
    }

    int left_node = -1;
    int right_node = -1;
    for (int segment : intersections)
    {
        status_structure.insert(segments, segment, left_node, right_node);
    }
    for (int segment : top_segments)
    {
        status_structure.insert(segments, segment, left_node, right_node);
        if (!neighbours_filled)
        {
            left_neighbour = left_node;
            right_neighbour = right_node;
            neighbours_filled = true;
        }
        if (left_node == left_neighbour || left_neighbour == -1)
        {
            most_left_segment = segment;
        }
        if (right_node == right_neighbour || right_neighbour == -1)
        {
            most_right_segment = segment;
        }
    }



    //If top+intersection+bottom > 1 
    //report intersection + segments

    //Delete bottom segments from status_structure
    //Swap intersection segments
    //Insert top segments
    if (top_segments.size() + intersections.size() == 0)
    {
        if (left_node != -1 && right_node != -1)
        {
            Vec2 intersection;
            if (Segment::intersection_two_segments(&segments.at(left_node), &segments.at(right_node), intersection))
            {
                // get left_neighbour and right_neighbour of event point in status structure before call find new event
                auto event_pair = event_queue.emplace(intersection, std::vector<int>());
            }
        }
    }
    else
    {

        Vec2 intersection;
        if (most_left_segment != -1 && left_neighbour != -1)
        {
            if (Segment::intersection_two_segments(&segments.at(most_left_segment), &segments.at(left_neighbour), intersection))
            {
                auto event_pair = event_queue.emplace(intersection, std::vector<int>());
            }
        }
        if (most_right_segment != -1 && right_neighbour != -1)
        {
            if (Segment::intersection_two_segments(&segments.at(most_right_segment), &segments.at(right_neighbour), intersection))
            {
                auto event_pair = event_queue.emplace(intersection, std::vector<int>());
            }

        }
        // never intersects if theres only 1 neighbour
         
         
        //Else if upper + intersection == 0
        //Remove bottom, check neighbours, add new events when needed (bellow or to the right of current event_point


    }
}

//TODO Should this be a class instead of a function?
//void Segment_Intersection_Sweep_Line::top(Sweep_Line_Status_structure& status_structure, const std::vector<Segment>& segments, map& event_queue, const int new_segment, Vec2 event_point)
//{
//	status_structure.set_line_position(event_point.y);
//	int left_segment = -1;
//	int right_segment = -1;
//	status_structure.insert(segments, new_segment, left_segment, right_segment);
//
//
//	Vec2 intersection;
//	if (left_segment != -1)
//	{
//		if (Segment::intersection_two_segments(&segments.at(new_segment), &segments.at(left_segment), intersection))
//		{
//			auto event_pair = event_queue.emplace(intersection, Event(Intersection_Event_Type::INTERSECTION, left_segment, new_segment));
//			//add intersection // check neighbour?
//		}
//	}
//	if (right_segment != -1)
//	{
//		if (Segment::intersection_two_segments(&segments.at(new_segment), &segments.at(right_segment), intersection))
//		{
//			auto event_pair = event_queue.emplace(intersection, Event(Intersection_Event_Type::INTERSECTION, new_segment, right_segment));
//			//add intersection // check neighbour?
//		}
//	}
//	// returns node (with neighbours pointers) or null???
//	// if( && ==!null)
//	// insert segment
//	//status_structure.checkneighbours(&segment);
//	//check for intersection new neighbour
//	// intersect(get neighbours)
//	// insert intersection point to event queue
//}
//
//void Segment_Intersection_Sweep_Line::bottom(Sweep_Line_Status_structure& status_structure, const std::vector<Segment>& segments, map& event_queue, const int segment, Vec2 event_point)
//{
//	status_structure.set_line_position(event_point.y);
//	int left_segment = -1;
//	int right_segment = -1;
//	status_structure.remove(segments, segment, left_segment, right_segment);
//
//
//	Vec2 intersection;
//	if (left_segment != -1)
//	{
//		if (right_segment != -1)
//		{
//			if (Segment::intersection_two_segments(&segments.at(left_segment), &segments.at(right_segment), intersection))
//			{
//				auto event_pair = event_queue.emplace(intersection, Event(Intersection_Event_Type::INTERSECTION, left_segment, right_segment));
//
//				//add intersection // check neighbour?
//			}
//		}
//	}
//	//check for intersection new neighbour
//	// intersect(get neighbours)
//	// insert intersection point to event
//}
//
//void Segment_Intersection_Sweep_Line::intersection(Sweep_Line_Status_structure& status_structure, const std::vector<Segment>& segments, map& event_queue, int p1, int p2, Vec2 event_point)
//{
//	Vec2 intersection;
//	int left_segment = -1;
//	int right_segment = -1;
//
//	status_structure.swap_elements(segments, p1, p2, left_segment, right_segment);
//	// p1 = left p2 = right after swap reversed
//	status_structure.set_line_position(event_point.y);
//	if (left_segment != -1)
//	{
//		if (Segment::intersection_two_segments(&segments.at(p1), &segments.at(left_segment), intersection))
//		{
//			if (intersection.y < event_point.y)
//			{
//				auto event_pair = event_queue.emplace(intersection, Event(Intersection_Event_Type::INTERSECTION, p1, right_segment));
//			}
//		}
//
//	}
//	if (right_segment != -1)
//	{
//		if (Segment::intersection_two_segments(&segments.at(p2), &segments.at(right_segment), intersection))
//		{
//			if (intersection.y < event_point.y)
//			{
//				auto event_pair = event_queue.emplace(intersection, Event(Intersection_Event_Type::INTERSECTION, p2, right_segment));
//			}
//		}
//	}
//
//
//	//swap Si(links) Sj(rechts) in status structure
//	//check intersection between Si-1 Sj and Si+1 Sj
//	//add new intersection event in queue
//}




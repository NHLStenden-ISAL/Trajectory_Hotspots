#include "pch.h"
#include "sweep_line_status_structure.h"
#include "segment_intersection.h"


#include <functional>

using namespace Segment_Intersection_Sweep_Line;
std::vector<Vec2> Segment_Intersection_Sweep_Line::find_segment_intersections(const std::vector<Segment>& segments)
{
    map event_queue;

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
            result.clear();
        }
        event_queue.erase(event_queue.begin());
    }

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
    std::vector<int> intersection_segments;
    std::vector<int> bottom_segments;
    int left_neighbour = -1;
    int right_neighbour = -1;
    int most_left_segment = -1;
    int most_right_segment = -1;

    Float line_pos = event_point.y;
    status_structure.set_line_position(line_pos);

    //Get all nodes containing segments that intersect this event point
    int most_left_intersecting_segment = -1;
    int most_right_intersecting_segment = -1;
    status_structure.get_all_nodes_on_point(segments, event_point, intersection_segments, bottom_segments, most_left_intersecting_segment, most_right_intersecting_segment, left_neighbour, right_neighbour);

    //The intersecting segments swap after the event point so we swap the outer indices
    most_left_segment = most_right_intersecting_segment;
    most_right_segment = most_left_intersecting_segment;

    //Report all segments that intersect this point
    if (top_segments.size() + bottom_segments.size() + intersection_segments.size() > 1)
    {
        for (int segment : top_segments)
        {
            result_segments.push_back(segments.at(segment));
        }

        for (int segment : bottom_segments)
        {
            result_segments.push_back(segments.at(segment));
        }

        for (int segment : intersection_segments)
        {
            result_segments.push_back(segments.at(segment));
        }
        //TODO: return intersection + segments, check later if from same dcel
        //add to result
    }

    //Remove the segment that intersect with the bottom endpoint and internally
    //After the final remove we have the right and left neighbour of all intersecting segments
    //TODO: Do we need neighbours here if we get them from get_all_nodes_on???? Follow up: No, new TODO: Remove those parameters from remove..
    for (int segment : bottom_segments)
    {
        int l, r;
        status_structure.remove(segments, segment, l, r);
    }

    for (int segment : intersection_segments)
    {
        int l, r;
        status_structure.remove(segments, segment, l, r);
    }

    //Reinsert the segments that intersect internally on this point
    for (int segment : intersection_segments)
    {
        int l, r;
        status_structure.insert(segments, segment, l, r);
    }

    //If we only have top segments we did not find the neighbouring nodes yet
    bool neighbours_found = !bottom_segments.empty() || !intersection_segments.empty();

    //Insert the segments that intersect this event point with their top point,
    //finding the outer neighbours if we didn't already.
    int new_left_neighbour = -1;
    int new_right_neighbour = -1;
    for (int segment : top_segments)
    {
        status_structure.insert(segments, segment, new_left_neighbour, new_right_neighbour);

        //If we did not have intersecting or bottom segments we don't know the neighbours of this point.
        //In this case, the first top insert gives us these neighbours
        if (!neighbours_found)
        {
            left_neighbour = new_left_neighbour;
            right_neighbour = new_right_neighbour;
            neighbours_found = true;
        }

        //If the neighbour of this inserted segment is the first non-intersecting segment, this top segment is the new outer segment
        if (new_left_neighbour == left_neighbour || left_neighbour == -1)
        {
            most_left_segment = segment;
        }

        if (new_right_neighbour == right_neighbour || right_neighbour == -1)
        {
            most_right_segment = segment;
        }
    }

    //TODO: Might be better to just do a left and right inorder tree walk to get most left and right segment instead.

    //Check if there are new intersection events between the outer segments and their outer neighbours
    if (top_segments.empty() && intersection_segments.empty())
    {
        //Only bottom segments on this point, after their removal we need to check if their neighbours intersect in the future
        if (left_neighbour != -1 && right_neighbour != -1)
        {
            test_for_intersection(segments, left_neighbour, right_neighbour, event_point, event_queue);
        }
    }
    else
    {
        //If we added or swapped segments we need to check if the most left and right of these segments intersects their left and right neighbour respectively.
        Vec2 intersection_point;
        if (left_neighbour != -1)
        {
            test_for_intersection(segments, left_neighbour, most_left_segment, event_point, event_queue);
        }

        if (right_neighbour != -1)
        {
            test_for_intersection(segments, most_right_segment, right_neighbour, event_point, event_queue);
        }
    }
}

//Check if two potentially intersecting segments intersect. Add the event to the event_queue if they do and the intersection point is in the future.
void Segment_Intersection_Sweep_Line::test_for_intersection(const std::vector<Segment>& segments, const int left_segment, const int right_segment, const Vec2& event_point, Segment_Intersection_Sweep_Line::map& event_queue)
{
    Vec2 intersection_point;
    Segment::Intersection_Type intersection_type = segments.at(left_segment).intersects(segments.at(right_segment), intersection_point);
    if (intersection_type == Segment::Intersection_Type::point)
    {
        //Check if the intersection is in the future, intersections above the sweepline (and left of this point) should've been reported already
        if (intersection_point < event_point)
        {
            auto event_pair = event_queue.emplace(intersection_point, std::vector<int>());
        }
    }
}
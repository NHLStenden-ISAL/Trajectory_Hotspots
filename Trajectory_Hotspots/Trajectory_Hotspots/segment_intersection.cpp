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
    std::vector<int> intersections;
    std::vector<int> bottom_segments;
    int left_neighbour = -1;
    int right_neighbour = -1;
    int most_left_segment = -1;
    int most_right_segment = -1;

    float line_pos = event_point.y;
    status_structure.set_line_position(line_pos);
    status_structure.get_all_nodes_on_point(segments, event_point, intersections, bottom_segments, most_left_segment, most_right_segment, left_neighbour, right_neighbour);

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
    //TODO: Do we need neighbours here if we get them from get_all_nodes_on???? Follow up: No, new TODO: Remove those parameters from remove..
    for (int segment : bottom_segments)
    {
        int l, r;
        status_structure.remove(segments, segment, l, r);
    }

    for (int segment : intersections)
    {
        int l, r;
        status_structure.remove(segments, segment, l, r);
    }

    //delete lower and intersection
    // set_lineposition 



    for (int segment : intersections)
    {
        int l, r;
        status_structure.insert(segments, segment, l, r);
    }

    //If we only have top segments we did not find the neighbouring nodes yet
    bool neighbours_found = !bottom_segments.empty() && !intersections.empty();

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

    //If top+intersection+bottom > 1 
    //report intersection + segments

    //Delete bottom segments from status_structure
    //Swap intersection segments
    //Insert top segments

    //Check if there are new intersection events between the outer segments and their outer neighbours
    if (top_segments.size() + intersections.size() == 0)
    {
        //Only bottom segments on this point, after their removal we need to check if their neighbours intersect in the future
        if (left_neighbour != -1 && right_neighbour != -1)
        {
            Vec2 intersection;
            if (Segment::intersection_two_segments(&segments.at(left_neighbour), &segments.at(right_neighbour), intersection))
            {
                //Check if the intersection is in the future, intersections above the sweepline should've been reported already
                if (intersection < event_point)
                {
                    auto event_pair = event_queue.emplace(intersection, std::vector<int>());
                }
            }
        }
    }
    else
    {
        //If we added or swapped segments we need to check if the most left and right of these segments intersects their left and right neighbour respectively.
        Vec2 intersection;
        if (left_neighbour != -1)
        {
            if (Segment::intersection_two_segments(&segments.at(most_left_segment), &segments.at(left_neighbour), intersection))
            {
                if (intersection < event_point)
                {
                    auto event_pair = event_queue.emplace(intersection, std::vector<int>());
                }
            }
        }

        if (right_neighbour != -1)
        {
            if (Segment::intersection_two_segments(&segments.at(most_right_segment), &segments.at(right_neighbour), intersection))
            {
                if (intersection < event_point)
                {
                    auto event_pair = event_queue.emplace(intersection, std::vector<int>());
                }
            }
        }

        // never intersects if theres only 1 neighbour


        //Else if upper + intersection == 0
        //Remove bottom, check neighbours, add new events when needed (bellow or to the right of current event_point


    }
}
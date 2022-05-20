#include "pch.h"
#include "segment_intersection.h"

std::vector<Vec2> find_segment_intersections(const std::vector<Segment>& segments)
{
    std::vector<const Vec2*> top_points;
    std::vector<const Vec2*> bottom_points;

    std::map;

    for (const Segment& segment : segments)
    {
        top_points.push_back(segment.get_top_point());
        bottom_points.push_back(segment.get_bottom_point());
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
#pragma once

namespace Segment_Intersection_Sweep_Line
{
    class Sweep_Line_Status_structure; //forward declaration

    struct Intersection_Info
    {
        std::unordered_set<int> edges; //Segments that have the intersection point in the middle.
        std::vector<int> top_points; //Segments that intersect at the top point.
        std::vector<int> bottom_points; //Segments that intersect at the bottom point.
        std::vector<int> collinear_edges; //List of collinear edges at this point, every two indices are a pair that overlap.
    };

    //Event queue sorted on point, values are a segment list where the event point is the top point of the segment
    typedef std::map<const Vec2, Intersection_Info, std::greater<Vec2>> map;


    std::vector<Vec2> find_segment_intersections(const std::vector<Segment>& segments);

    void Handle_Event(
        Sweep_Line_Status_structure& status_structure,
        map& event_queue,
        const std::vector<Segment>& segments,
        const Vec2& event_point,
        const std::vector<int>& top_segments,
        std::vector<Segment>& result_segments);

    //Check if two potentially intersecting segments intersect. Add the event to the event_queue if they do and the intersection point is in the future.
    void test_for_intersection(const std::vector<Segment>& segments, const int left_segment, const int right_segment, const Vec2& event_point, Segment_Intersection_Sweep_Line::map& event_queue);
}
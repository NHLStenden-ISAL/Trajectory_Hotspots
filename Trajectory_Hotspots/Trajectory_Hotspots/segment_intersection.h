#pragma once

namespace Segment_Intersection_Sweep_Line
{
    class Sweep_Line_Status_structure;

    //Event queue sorted on point, values are a segment list where the event point is the top point of the segment
    typedef std::map<const Vec2, std::vector<int>, std::greater<Vec2>> map;
    

    std::vector<Vec2> find_segment_intersections(const std::vector<Segment>& segments);

    void Handle_Event(
        Sweep_Line_Status_structure& status_structure,
        map& event_queue,
        const std::vector<Segment>& segments,
        const Vec2& event_point,
        const std::vector<int>& top_segments,
        std::vector<Segment>& result_segments);

    void test_for_intersection(const std::vector<Segment>& segments, const int left_segment, const int right_segment, const Vec2& event_point, Segment_Intersection_Sweep_Line::map& event_queue);
}
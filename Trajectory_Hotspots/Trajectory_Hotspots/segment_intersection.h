#pragma once


namespace Segment_Intersection_Sweep_Line
{
    class Sweep_Line_Status_structure;

    //Event queue sorted on point, values are a segment list where the event point is the top point of the segment
    typedef std::map<const Vec2, std::vector<int>> map;

    //void top(Sweep_Line_Status_structure& status_structure, const std::vector<Segment>& segments, map& event_queue, const int segment, Vec2 event_point);
    //void bottom(Sweep_Line_Status_structure& status_structure, const std::vector<Segment>& segments, map& event_queue, const int segment, Vec2 event_point);
    //void intersection(Sweep_Line_Status_structure& status_structure, const std::vector<Segment>& segments, map& event_queue, int p1, int p2, Vec2 event_point);

    std::vector<Vec2> find_segment_intersections(const std::vector<Segment>& segments);

    void Handle_Event(
        Sweep_Line_Status_structure& status_structure,
        map& event_queue,
        const std::vector<Segment>& segments,
        const Vec2& event_point,
        const std::vector<int>& top_segments,
        Vec2& intersection,
        std::vector<Segment>& result_segments);
}
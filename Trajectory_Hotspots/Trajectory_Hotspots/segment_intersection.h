#pragma once


namespace Segment_Intersection_Sweep_Line
{
    class Sweep_Line_Status_structure;
    enum class Intersection_Event_Type
    {
        TOP,
        BOTTOM,
        INTERSECTION
    };

    class Event
    {
    public:
        Event(Intersection_Event_Type event_type, int segment_index, int other_segment_index) : event_type(event_type), segment_index(segment_index), other_segment_index(other_segment_index)
        {
        };
        Event(Intersection_Event_Type event_type, int segment_index) : event_type(event_type), segment_index(segment_index)
        {
        };
        Intersection_Event_Type event_type;
        int segment_index;
        int other_segment_index;

       
    };

    using map = std::map<const Vec2, Event, std::greater<Vec2>>;
   
    void top(Sweep_Line_Status_structure& status_structure, const std::vector<Segment>& segments, map& event_queue, const int segment, Vec2 event_point);
    void bottom(Sweep_Line_Status_structure& status_structure, const std::vector<Segment>& segments, map& event_queue, const int segment, Vec2 event_point);
    void intersection(Sweep_Line_Status_structure& status_structure, const std::vector<Segment>& segments, map& event_queue,  int p1, int p2, Vec2 event_point);
    std::vector<Vec2> find_segment_intersections(const std::vector<Segment>& segments);
}
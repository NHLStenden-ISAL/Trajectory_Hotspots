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

    class Segment_Intersection_Event
    {
    public:
        Segment_Intersection_Event() = default;

        Segment& segment;
        Vec2 event_point;
        Intersection_Event_Type event_type;
    };
    
    void top(Sweep_Line_Status_structure& status_structure, const Segment& segment);
    void bottom(Sweep_Line_Status_structure& status_structure, const Segment& segment);
    void intersection();
    std::vector<Vec2> find_segment_intersections(const std::vector<Segment>& segments);

}
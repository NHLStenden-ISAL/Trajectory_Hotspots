#pragma once

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

std::vector<Vec2> find_segment_intersections(const std::vector<Segment>& segments);
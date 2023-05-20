#pragma once

namespace Segment_Intersection_Sweep_Line
{
    class Sweep_Line_Status_structure; //forward declaration

    struct Intersection_Info
    {
        std::unordered_set<int> interior_segments; //Segments that have an internal intersection with the event point.
        std::vector<int> top_segments; //Segments that intersect at the top point.
        std::vector<int> bottom_segments; //Segments that intersect at the bottom point.
        std::vector<int> collinear_segments; //List of collinear segments at this point, every two indices are a pair that overlap.

        size_t segment_count() { return interior_segments.size() + top_segments.size() + bottom_segments.size() + collinear_segments.size(); };
    };

    struct Event_Point_Comparer
    {
        //Higher (in the y-axis) events go first. If on the same y-axis, most left goes first (smaller x-axis).
        //Checks if a occurs before b
        bool operator()(const Vec2& a, const Vec2& b) const
        {
            if (a.y == b.y)
            {
                if (a.x == b.x)
                {
                    return false;
                }

                return a.x < b.x;
            }

            return a.y > b.y;
        }
    };

    //Event queue sorted on point, values are a segment list where the event point is the top point of the segment
    typedef std::map<const Vec2, std::vector<int>, Event_Point_Comparer> map;


    std::vector<Vec2> find_segment_intersections(const std::vector<Segment>& segments);

    Intersection_Info Handle_Event(
        Sweep_Line_Status_structure& status_structure,
        map& event_queue,
        const std::vector<Segment>& segments,
        const Vec2& event_point,
        const std::vector<int>& top_segments);

    //Check if two potentially intersecting segments intersect. Add the event to the event_queue if they do and the intersection point is in the future.
    void test_for_intersection(const std::vector<Segment>& segments, const int left_segment, const int right_segment, const Vec2& event_point, Segment_Intersection_Sweep_Line::map& event_queue);
}
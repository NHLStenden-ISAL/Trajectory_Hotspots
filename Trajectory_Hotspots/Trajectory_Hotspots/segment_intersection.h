#pragma once

#include "sweep_line_status_structure.h"

namespace Segment_Intersection_Sweep_Line
{
    template<typename SegmentT>
    class Sweep_Line_Status_structure; //forward declaration

    struct Intersection_Info
    {
        Intersection_Info() = default;
        Intersection_Info(Intersection_Info&& other) noexcept = default;

        std::vector<int> top_segments; //Segments that intersect at the top point.
        std::vector<int> bottom_segments; //Segments that intersect at the bottom point.
        std::unordered_set<int> interior_segments; //Segments that have an internal intersection with the event point.
        std::vector<int> collinear_segments; //List of collinear segments at this point, every two indices are a pair that overlap.

        //TODO: Add reference to segment vector here.. prevent all the passing in the DCEL functions..

        size_t segment_count() const { return interior_segments.size() + top_segments.size() + bottom_segments.size() + collinear_segments.size(); };

        int get_first_segment() const
        {
            if (!top_segments.empty()) { return top_segments[0]; }
            else if (!bottom_segments.empty()) { return bottom_segments[0]; }
            else if (!interior_segments.empty()) { return *interior_segments.begin(); }
            else if (!collinear_segments.empty()) { return collinear_segments[0]; }
        };
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
    using map = std::map<const Vec2, std::vector<int>, Event_Point_Comparer>;

    template<typename SegmentT>
    std::vector<Vec2> find_segment_intersections(const std::vector<SegmentT>& segments);

    template<typename SegmentT>
    Intersection_Info Handle_Event(
        Sweep_Line_Status_structure<SegmentT>& status_structure,
        map& event_queue,
        const std::vector<SegmentT>& segments,
        const Vec2& event_point,
        const std::vector<int>& top_segments);

    //Check if two potentially intersecting segments intersect. Add the event to the event_queue if they do and the intersection point is in the future.
    template<typename SegmentT>
    void test_for_intersection(const std::vector<SegmentT>& segments, const int left_segment, const int right_segment, const Vec2& event_point, map& event_queue);


    template<typename SegmentT>
    std::vector<Vec2> find_segment_intersections(const std::vector<SegmentT>& segments);

    template<typename SegmentT>
    Intersection_Info Handle_Event(
        Sweep_Line_Status_structure<SegmentT>& status_structure,
        map& event_queue,
        const std::vector<SegmentT>& segments,
        const Vec2& event_point,
        const std::vector<int>& top_segments);

    template<typename SegmentT>
    void test_for_intersection(const std::vector<SegmentT>& segments, const int left_segment, const int right_segment, const Vec2& event_point, map& event_queue);


    template<typename SegmentT>
    Intersection_Info Handle_Event(
        Sweep_Line_Status_structure<SegmentT>& status_structure,
        map& event_queue,
        const std::vector<SegmentT>& segments,
        const Vec2& event_point,
        const std::vector<int>& top_segments)
    {
        Intersection_Info intersection_info;

        //TODO: Remove?
        std::vector<int> intersection_segments;
        std::vector<int> bottom_segments;

        Float line_pos = event_point.y;
        status_structure.set_line_position(line_pos);

        int left_neighbour = -1;
        int right_neighbour = -1;

        //Get all nodes containing segments that intersect this event point
        std::vector<int> ordered_intersecting_segments = status_structure.get_all_nodes_on_point(segments, event_point, left_neighbour, right_neighbour);

        int most_left_segment = -1;
        int most_right_segment = -1;

        if (!ordered_intersecting_segments.empty())
        {
            //Split the intersecting segments in interior and bottom intersections
            for (const auto& segment_index : ordered_intersecting_segments)
            {
                if (*segments[segment_index].get_bottom_point() == event_point)
                {
                    bottom_segments.push_back(segment_index);
                }
                else
                {
                    intersection_segments.push_back(segment_index);
                }
            }

            //The intersecting segments swap after the event point so we swap the outer indices
            most_left_segment = ordered_intersecting_segments.back();
            most_right_segment = ordered_intersecting_segments.front();
        }

        //Report all segments that intersect this point
        if (top_segments.size() + bottom_segments.size() + intersection_segments.size() > 0)
        {
            intersection_info.top_segments = top_segments;
            intersection_info.interior_segments.insert(intersection_segments.begin(), intersection_segments.end());
            intersection_info.bottom_segments = bottom_segments;

            //TODO: We need to do a linear search through the segments to check for collinearity, they should be neighbours. (tops cant collinear with bottoms)
            //TODO: Check if we need to report collinear with bottom intersections or if we can handle with top only?
        }

        //Remove the segment that intersect with the bottom endpoint and internally
        //After the final remove we have the right and left neighbour of all intersecting segments
        for (int segment : bottom_segments)
        {
            status_structure.remove(segments, segment);
        }

        for (int segment : intersection_segments)
        {
            status_structure.remove(segments, segment);
        }

        //Reinsert the segments that intersect internally on this point
        for (int segment : intersection_segments)
        {
            status_structure.insert(segments, segment);
        }

        //If we only have top segments we did not find the neighboring nodes yet
        bool neighbours_found = !ordered_intersecting_segments.empty();

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

            //If the neighbour of this inserted segment does not intersect the event point, this segment is the new outer segment.
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

        return intersection_info;
    }


    template<typename SegmentT>
    void test_for_intersection(const std::vector<SegmentT>& segments, const int left_segment, const int right_segment, const Vec2& event_point, map& event_queue)
    {
        Vec2 intersection_point;
        Segment::Intersection_Type intersection_type = segments.at(left_segment).intersects(segments.at(right_segment), intersection_point);

        //Report the intersection if the intersection is in the future, 
        //intersections above the sweep line (and left of this point) have been reported already
        switch (intersection_type)
        {

        case Segment::Intersection_Type::point:
        {
            if (Event_Point_Comparer comp; comp(event_point, intersection_point))
            {
                auto [event_point_key, event_top_segments] = event_queue.try_emplace(intersection_point);
            }

            break;
        }
        case Segment::Intersection_Type::collinear:
        {
            //Collinear intersections are handled around the start and end of their overlap.
            //If these points are in the future, add them to the event queue
            Vec2 overlap_start;
            Vec2 overlap_end;

            if (collinear_overlap(segments.at(left_segment), segments.at(right_segment), overlap_start, overlap_start))
            {
                Event_Point_Comparer comp;
                if (comp(event_point, overlap_start))
                {
                    auto [event_point_key, event_top_segments] = event_queue.try_emplace(overlap_start);
                }

                if (comp(event_point, overlap_end))
                {
                    auto [event_point_key, event_top_segments] = event_queue.try_emplace(overlap_end);
                }
            }
            else
            {
                assert(false); //Collinear but no overlap? Should never happen.
            }

            break;
        }
        case Segment::Intersection_Type::parallel:
        case Segment::Intersection_Type::none:
        {
            break;
        }
        }
    }

    template<typename SegmentT>
    std::vector<Vec2> find_segment_intersections(const std::vector<SegmentT>& segments)
    {
        map event_queue;

        for (int i = 0; i < segments.size(); i++)
        {
            auto event_pair = event_queue.emplace(*segments.at(i).get_top_point(), std::vector<int>());
            event_pair.first->second.push_back(i);

            event_pair = event_queue.emplace(*segments.at(i).get_bottom_point(), std::vector<int>());
        }

        //Initialize status structure with the highest event point
        Sweep_Line_Status_structure<SegmentT> status_structure(event_queue.begin()->first.y);

        std::vector<Vec2> intersections;

        while (!event_queue.empty())
        {
            Intersection_Info intersection_results = Handle_Event(status_structure, event_queue, segments, event_queue.begin()->first, event_queue.begin()->second);

            if (intersection_results.segment_count() > 1)
            {
                intersections.push_back(event_queue.begin()->first);
            }

            event_queue.erase(event_queue.begin());
        }

        return intersections;
    }
}
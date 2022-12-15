#pragma once

#include "vec2.h"
#include "segment.h"

//Represents an axis-aligned bounding box
class AABB
{
public:

    AABB() : min(0.f, 0.f), max(0.f, 0.f)
    {
    }

    AABB(const Vec2& min, const Vec2& max) :
        min(min),
        max(max)
    {
    }

    AABB(const Float min_x, const Float min_y, const Float max_x, const Float max_y) :
        min(min_x, min_y),
        max(max_x, max_y)
    {
    }


    AABB(const Segment& segment) :
        min(std::min(segment.start.x, segment.end.x), std::min(segment.start.y, segment.end.y)),
        max(std::max(segment.start.x, segment.end.x), std::max(segment.start.y, segment.end.y))
    {
    }

    //Combine this AABB with another by keeping the extremes in all four directions
    //TODO: Ensure this stays a square
    void combine(const AABB& other)
    {
        min.x = std::min(min.x, other.min.x);
        min.y = std::min(min.y, other.min.y);

        max.x = std::max(max.x, other.max.x);
        max.y = std::max(max.y, other.max.y);
    }

    //Combine two AABBs with each other by keeping the extremes in all four directions
    //TODO: Ensure this stays a square
    static AABB combine(const AABB& a, const AABB& b)
    {
        Float min_x = std::min(a.min.x, b.min.x);
        Float min_y = std::min(a.min.y, b.min.y);

        Float max_x = std::max(a.max.x, b.max.x);
        Float max_y = std::max(a.max.y, b.max.y);

        return AABB(Vec2(min_x, min_y), Vec2(max_x, max_y));
    }

    Vec2 min;
    Vec2 max;


    Float width() const
    {
        Float width = max.x - min.x;

        if (width.is_inf())
        {
            return std::numeric_limits<float>::infinity();
        }

        return width;
    }
};
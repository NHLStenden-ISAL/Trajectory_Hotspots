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
    void combine(const AABB& other)
    {
        min.x = std::min(min.x, other.min.x);
        min.y = std::min(min.y, other.min.y);

        max.x = std::max(max.x, other.max.x);
        max.y = std::max(max.y, other.max.y);
    }

    //Combine two AABBs with each other by keeping the extremes in all four directions
    static AABB combine(const AABB& a, const AABB& b)
    {
        Float min_x = std::min(a.min.x, b.min.x);
        Float min_y = std::min(a.min.y, b.min.y);

        Float max_x = std::max(a.max.x, b.max.x);
        Float max_y = std::max(a.max.y, b.max.y);

        return AABB(Vec2(min_x, min_y), Vec2(max_x, max_y));
    }

    //Potentially increase the size of the aabb based on the given point
    void augment(const Vec2& point)
    {
        min.x = std::min(min.x, point.x);
        min.y = std::min(min.y, point.y);
        max.x = std::max(max.x, point.x);
        max.y = std::max(max.y, point.y);
    }

    //Potentially increase the size of given aabb based on the given point
    static AABB augment(const AABB& aabb, const Vec2& point)
    {
        return AABB(
            std::min(aabb.min.x, point.x),
            std::min(aabb.min.y, point.y),
            std::max(aabb.max.x, point.x),
            std::max(aabb.max.y, point.y));
    }

    Float width() const
    {
        Float width = max.x - min.x;

        if (width.is_inf())
        {
            return std::numeric_limits<float>::infinity();
        }

        return width;
    }

    Float height() const
    {
        Float width = max.y - min.y;

        if (width.is_inf())
        {
            return std::numeric_limits<float>::infinity();
        }

        return width;
    }

    Float max_size() const
    {
        return std::max(width(), height());
    }

    Vec2 min;
    Vec2 max;

};
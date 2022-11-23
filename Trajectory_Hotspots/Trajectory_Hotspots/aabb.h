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

    AABB(const float min_x, const float min_y, const float max_x, const float max_y) : 
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
        float min_x = std::min(a.min.x, b.min.x);
        float min_y = std::min(a.min.y, b.min.y);

        float max_x = std::max(a.max.x, b.max.x);
        float max_y = std::max(a.max.y, b.max.y);

        return AABB(Vec2(min_x, min_y), Vec2(max_x, max_y));
    }

    Vec2 min;
    Vec2 max;

    float width() const
    {
        float width = max.x - min.x;
		
        if (isinf(width))
        {
			return std::numeric_limits<float>::infinity();
		}

        return width; //TODO: We would like to return angle and width
    }
};
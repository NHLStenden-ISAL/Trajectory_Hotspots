#include "pch.h"
#include "dcel.h"

void DCEL::overlay_point_on_edge(DCEL_Half_Edge* edge, DCEL_Vertex* point)
{
    std::unique_ptr<DCEL_Half_Edge> new_half_edge_1; //new twin of edge
    std::unique_ptr<DCEL_Half_Edge> new_half_edge_2; //new twin of edge->twin

    DCEL_Half_Edge* twin = edge->twin;

    //Old half_edges point to point, the new twins originate from it.
    new_half_edge_1->origin = point;
    new_half_edge_2->origin = point;

    //Switch twins to new half-edges
    edge->twin = new_half_edge_1.get();
    new_half_edge_1->twin = edge;

    twin->twin = new_half_edge_2.get();
    new_half_edge_2->twin = twin;

    //Set next of new to next of shortened half-edges
    new_half_edge_1->next = twin->next;
    new_half_edge_2->next = edge->next;
    new_half_edge_1->next->prev = new_half_edge_1.get();
    new_half_edge_2->next->prev = new_half_edge_2.get();

    //Update prev and next pointers around the point
    //Find positions of new half-edges around the point and insert

    DCEL_Half_Edge* CCW_half_edge = nullptr;
    DCEL_Half_Edge* CW_half_edge = nullptr;

    DCEL_Half_Edge* prev_half_edge = point->incident_half_edge;
    DCEL_Half_Edge* current_half_edge = point->incident_half_edge->twin->next;

    find_adjacent_half_edges_around_point(point, edge, current_half_edge, CCW_half_edge, CW_half_edge);

    //TODO: update prev/next

    //Find other side, start from previous end
    find_adjacent_half_edges_around_point(point, twin, CW_half_edge, CCW_half_edge, CW_half_edge);

    //TODO: update prev/next
}

void DCEL::find_adjacent_half_edges_around_point(DCEL::DCEL_Vertex* point, DCEL::DCEL_Half_Edge* query_edge, DCEL::DCEL_Half_Edge* starting_half_edge, DCEL::DCEL_Half_Edge*& CCW_half_edge, DCEL::DCEL_Half_Edge*& CW_half_edge)
{
    DCEL::DCEL_Half_Edge* prev_half_edge = starting_half_edge->prev->twin;
    DCEL::DCEL_Half_Edge* current_half_edge = starting_half_edge;

    Float prev_order = Vec2::order_around_center(point->position, query_edge->origin->position, prev_half_edge->target()->position);

    do
    {
        Float new_order = Vec2::order_around_center(point->position, query_edge->origin->position, current_half_edge->target()->position);

        if (prev_order < 0.f && new_order > 0.f)
        {
            CCW_half_edge = prev_half_edge;
            CW_half_edge = current_half_edge;
            break;
        }
        else
        {
            prev_half_edge = current_half_edge;
            prev_order = new_order;
            current_half_edge = current_half_edge->twin->next;
        }
    } while (current_half_edge != starting_half_edge);

    assert(false);
}

void DCEL::overlay_edge_on_edge()
{

}

void DCEL::overlay_point_on_point()
{

}
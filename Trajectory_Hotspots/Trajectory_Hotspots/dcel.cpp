#include "pch.h"
#include "dcel.h"

void DCEL::overlay_point_on_edge(DCEL_Half_Edge* edge, DCEL_Vertex* point)
{
    //Note: Technically there's some duplicate code here because the handling of the original half-edges is almost identical.
    //Splitting this up gives some trouble with overwriting the next pointers though.
    //Also, this way we can start the search for the second adjacent half-edges where the first ended.

    //TODO: Precalc new size in top function? Can leave this anyway? because it does nothing when already at right size.
    half_edges.reserve(half_edges.size() + 2); //Reserve two extra spots to prevent invalidating the first reference when exceeding space on the second emplace_back
    std::unique_ptr<DCEL_Half_Edge>& new_half_edge_1 = half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>()); //new twin of edge
    std::unique_ptr<DCEL_Half_Edge>& new_half_edge_2 = half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>()); //new twin of edge->twin

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

    //face is left of halfedge, so halfedge with vertex as origin has CCW halfedge as prev 
    new_half_edge_1->prev = CCW_half_edge;
    CCW_half_edge->next = new_half_edge_1.get();

    //and halfedge with vertex as target has CW halfedge as next
    edge->next = CW_half_edge;
    CW_half_edge->prev = edge;

    //Find other side, start from previous end
    find_adjacent_half_edges_around_point(point, twin, CW_half_edge, CCW_half_edge, CW_half_edge);

    new_half_edge_2->prev = CCW_half_edge;
    CCW_half_edge->next = new_half_edge_2.get();

    twin->next = CW_half_edge;
    CW_half_edge->prev = twin;
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

void DCEL::overlay_edge_on_edge(DCEL_Half_Edge* edge_1, DCEL_Half_Edge* edge_2, const Vec2& intersection_point)
{
    std::unique_ptr<DCEL_Vertex>& new_dcel_vertex = vertices.emplace_back(std::make_unique<DCEL_Vertex>(intersection_point));

    DCEL_Half_Edge* edge_1_old_twin = edge_1->twin;

    //Create two new twins for the first edge
    half_edges.reserve(half_edges.size() + 2); //Reserve two extra spots to prevent invalidating the first reference when exceeding space on the second emplace_back
    std::unique_ptr<DCEL_Half_Edge>& edge_1_new_twin_1 = half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>());
    std::unique_ptr<DCEL_Half_Edge>& edge_1_new_twin_2 = half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>());

    edge_1_new_twin_1->origin = new_dcel_vertex.get();
    edge_1_new_twin_2->origin = new_dcel_vertex.get();

    edge_1_new_twin_1->twin = edge_1;
    edge_1->twin = edge_1_new_twin_1.get();

    edge_1_new_twin_2->twin = edge_1_old_twin;
    edge_1_old_twin->twin = edge_1_new_twin_2.get();

    //Set next of new to next of shortened half-edges
    edge_1_new_twin_2->next = edge_1->next;
    edge_1_new_twin_2->next->prev = edge_1_new_twin_2.get();

    edge_1_new_twin_1->next = edge_1_old_twin->next;
    edge_1_new_twin_1->next->prev = edge_1_new_twin_1.get();

    //Chain the four half-edges under edge_1 together
    //so we can simply call the edge on point function for the second edge.
    edge_1_new_twin_1->prev = edge_1_old_twin;
    edge_1_old_twin->next = edge_1_new_twin_1.get();

    edge_1_new_twin_2->prev = edge_1;
    edge_1->next = edge_1_new_twin_2.get();

    //Insert second edge using the overlay edge over point function
    overlay_point_on_edge(edge_2, new_dcel_vertex.get());
}

void DCEL::overlay_point_on_point()
{
    //Gather half-edges around second vertex
    //Set origins to first vertex
    //Add half-edges one-by-one

    //TODO: Do we want to check if prev/next needs to change? Probably not, cost of checking is same or more of updating..
}

std::vector<Vec2> DCEL::DCEL_Face::get_vertices() const
{
    const DCEL_Half_Edge* starting_half_edge = outer_component;
    const DCEL_Half_Edge* current_half_edge = starting_half_edge;

    std::vector<Vec2> vertices;
    do
    {
        //Report vertex and move to the next half-edge in the chain
        vertices.push_back(current_half_edge->origin->position);
        current_half_edge = current_half_edge->next;
    } while (current_half_edge != starting_half_edge);

    return vertices;
}
#include "pch.h"
#include "dcel.h"

#include "segment.h"
#include "segment_intersection.h"

void DCEL::overlay_dcel(DCEL& other_dcel)
{
    //TODO:
    //-Call sweep line algorithm
    //For each intersection call appropriate overlay helper function
    //Not sure if we want to just make a new DCEL..

    size_t original_half_edge_count = this->half_edge_count();

    //Move all the half edges from the other dcel into this one.
    this->half_edges.insert(this->half_edges.end(),
        std::make_move_iterator(other_dcel.half_edges.begin()),
        std::make_move_iterator(other_dcel.half_edges.end()));


    ////Construct the edges representing the half edge twins for use in the sweep line algorithm.
    std::vector<DCEL_Overlay_Edge_Wrapper> DCEL_edges;
    DCEL_edges.reserve(half_edge_count() / 2);

    for (size_t i = 0; i < this->half_edge_count(); i++)
    {
        if (this->half_edges[i]->is_orientated_left_right())
        {
            bool is_original_half_edge = i < original_half_edge_count;
            DCEL_edges.emplace_back(this->half_edges[i].get(), is_original_half_edge);
        }
    }

    //Call the sweep line algorithm handling overlay cases from top to bottom.
    Segment_Intersection_Sweep_Line::find_segment_intersections(DCEL_edges);
}

void DCEL::overlay_vertex_on_edge(DCEL_Half_Edge* edge, DCEL_Vertex* vertex)
{
    //Note: Technically there's some duplicate code here because the handling of the original half-edges is almost identical.
    //Splitting this up gives some trouble with overwriting the next pointers though.
    //Also, this way we can start the search for the second adjacent half-edges where the first ended.

    //TODO: Precalculate new size in top function? Can leave this anyway? because it does nothing when already at right size.
    half_edges.reserve(half_edges.size() + 2); //Reserve two extra spots to prevent invalidating the first reference when exceeding space on the second emplace_back
    std::unique_ptr<DCEL_Half_Edge>& new_half_edge_1 = half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>()); //new twin of edge
    std::unique_ptr<DCEL_Half_Edge>& new_half_edge_2 = half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>()); //new twin of edge->twin

    DCEL_Half_Edge* twin = edge->twin;

    //Old half_edges point to vertex, the new twins originate from it.
    new_half_edge_1->origin = vertex;
    new_half_edge_2->origin = vertex;

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

    //Update prev and next pointers around the vertex
    //Find positions of new half-edges around the vertex and insert

    DCEL_Half_Edge* CW_half_edge = nullptr;
    DCEL_Half_Edge* CCW_half_edge = nullptr;

    DCEL_Half_Edge* prev_half_edge = vertex->incident_half_edge;
    DCEL_Half_Edge* current_half_edge = vertex->incident_half_edge->twin->next;

    vertex->find_adjacent_half_edges(edge, current_half_edge, CW_half_edge, CCW_half_edge);

    //face is left of halfedge, so halfedge with vertex as origin has CCW halfedge as prev 
    new_half_edge_1->prev = CCW_half_edge;
    CCW_half_edge->next = new_half_edge_1.get();

    //and halfedge with vertex as target has CW halfedge as next
    edge->next = CW_half_edge;
    CW_half_edge->prev = edge;

    //Find other side, start from previous end (it is impossible for it to lie between the just added and CW half-edge)
    vertex->find_adjacent_half_edges(twin, CW_half_edge, CW_half_edge, CCW_half_edge);

    new_half_edge_2->prev = CCW_half_edge;
    CCW_half_edge->next = new_half_edge_2.get();

    twin->next = CW_half_edge;
    CW_half_edge->prev = twin;
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
    //so we can simply call the edge on vertex function for the second edge.
    edge_1_new_twin_1->prev = edge_1_old_twin;
    edge_1_old_twin->next = edge_1_new_twin_1.get();

    edge_1_new_twin_2->prev = edge_1;
    edge_1->next = edge_1_new_twin_2.get();

    //Insert second edge using the overlay edge over vertex function
    overlay_vertex_on_edge(edge_2, new_dcel_vertex.get());
}

void DCEL::overlay_vertex_on_vertex(DCEL_Vertex* vertex_1, DCEL_Vertex* vertex_2)
{
    //Gather half-edges around second vertex
    //For each: Set origins to first vertex
    //          and switch prev, prev->next,
    //          twin->next, and twin->next->prev pointers

    std::vector<DCEL_Half_Edge*> incident_half_edges_v2 = vertex_2->get_incident_half_edges();

    DCEL_Half_Edge* CW_half_edge = nullptr;
    DCEL_Half_Edge* CCW_half_edge = nullptr;

    DCEL_Half_Edge* current_half_edge = vertex_1->incident_half_edge;

    for (auto& incident_half_edge_v2 : incident_half_edges_v2)
    {
        incident_half_edge_v2->origin = vertex_1;

        vertex_1->find_adjacent_half_edges(incident_half_edge_v2, current_half_edge, CW_half_edge, CCW_half_edge);

        //We can prevent the two twin writes by checking if the CCW is the previous added half-edge
        //but adding branching is probably slower
        incident_half_edge_v2->next = CW_half_edge;
        incident_half_edge_v2->twin->prev = CCW_half_edge;

        CW_half_edge->prev = incident_half_edge_v2;
        CCW_half_edge->next = incident_half_edge_v2->twin;
    }
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

void DCEL::DCEL_Vertex::find_adjacent_half_edges(DCEL::DCEL_Half_Edge* query_edge, DCEL::DCEL_Half_Edge* starting_half_edge, DCEL::DCEL_Half_Edge*& CW_half_edge, DCEL::DCEL_Half_Edge*& CCW_half_edge) const
{
    DCEL::DCEL_Half_Edge* prev_half_edge = starting_half_edge->prev->twin;
    DCEL::DCEL_Half_Edge* current_half_edge = starting_half_edge;

    Float prev_order = Vec2::order_around_center(this->position, query_edge->origin->position, prev_half_edge->target()->position);

    do
    {
        Float new_order = Vec2::order_around_center(this->position, query_edge->origin->position, current_half_edge->target()->position);

        if (prev_order < 0.f && new_order > 0.f)
        {
            CW_half_edge = current_half_edge;
            CCW_half_edge = prev_half_edge;
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

std::vector<DCEL::DCEL_Half_Edge*> DCEL::DCEL_Vertex::get_incident_half_edges() const
{
    DCEL::DCEL_Half_Edge* starting_half_edge = incident_half_edge;
    DCEL::DCEL_Half_Edge* current_half_edge = incident_half_edge;

    std::vector<DCEL_Half_Edge*> incident_half_edges;

    do
    {
        incident_half_edges.emplace_back(current_half_edge);
        current_half_edge = current_half_edge->twin->next;

    } while (current_half_edge != starting_half_edge);

    return incident_half_edges;
}

bool DCEL::DCEL_Half_Edge::is_orientated_left_right() const
{
    return orientation_left_right(origin->position, twin->origin->position);
}

Float DCEL::DCEL_Overlay_Edge_Wrapper::y_intersect(Float y) const
{
    return edge_segment.y_intersect(y);
}

const Vec2* DCEL::DCEL_Overlay_Edge_Wrapper::get_top_point() const
{
    return edge_segment.get_top_point();
}

const Vec2* DCEL::DCEL_Overlay_Edge_Wrapper::get_bottom_point() const
{
    return edge_segment.get_bottom_point();
}

Segment::Intersection_Type DCEL::DCEL_Overlay_Edge_Wrapper::intersects(const DCEL_Overlay_Edge_Wrapper& other, Vec2& intersection_point) const
{
    return Segment::Intersection_Type();
}

bool collinear_overlap(const DCEL::DCEL_Overlay_Edge_Wrapper& segment1, const DCEL::DCEL_Overlay_Edge_Wrapper& segment2, Vec2& overlap_start, Vec2& overlap_end)
{
    return collinear_overlap(segment1.edge_segment, segment2.edge_segment, overlap_start, overlap_start);
}
#pragma once

class DCEL
{
public:

    class DCEL_Vertex;
    class DCEL_Half_Edge;
    class DCEL_Face;

    //Class representing a vertex of the DCEL.
    //The class points to one of the outgoing half-edges (with the vertex as its origin).
    class DCEL_Vertex
    {
    public:
        Vec2 position;
        DCEL_Half_Edge* incident_half_edge;
    };

    //Class representing a half-edge of the DCEL.
    //Every half-edge has an accompanying twin half-edge.
    //It also stores the incident face that lies to the left of the half-edge 
    //and the next and previous half-edges in the chain that lies around the face.
    class DCEL_Half_Edge
    {
    public:
        DCEL_Vertex* origin;

        DCEL_Face* incident_face;

        DCEL_Half_Edge* twin;
        DCEL_Half_Edge* next;
        DCEL_Half_Edge* prev;
    };

    //Class representing a face of the DCEL, surrounded by half-edges.
    //It stores a pointer to one of the half-edges on its boundary
    //and pointers to the outer boundary of any optional holes.
    class DCEL_Face
    {
    public:
        DCEL_Half_Edge* outer_component;
        std::vector<DCEL_Half_Edge*> inner_components;
    };

    size_t vertex_count() const { return vertices.size(); };
    size_t half_edge_count() const { return half_edges.size(); };
    size_t face_count() const { return faces.size(); };

private:

    std::vector<std::unique_ptr<DCEL_Vertex>> vertices;
    std::vector<std::unique_ptr<DCEL_Half_Edge>> half_edges;
    std::vector<std::unique_ptr<DCEL_Face>> faces;

    void overlay_point_on_edge();
    void overlay_edge_on_edge();
    void overlay_point_on_point();


};


#include "pch.h"


#include "halfedge.h"
#include "face.h"
#include "vertex.h"
Vertex Vertex::get_point(Face face)
{
	return Vertex(Vec2(0,0), std::vector<HalfEdge*>());
}

Vertex Vertex::create_vertex(Vec2 position) 
{
	return Vertex(Vec2(0, 0), std::vector<HalfEdge*>());
}

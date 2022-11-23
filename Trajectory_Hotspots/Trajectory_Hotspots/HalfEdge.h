#pragma once

#include "Vertex.h"
#include "Face.h"
#include "dcel.h"
#include "pch.h"

class HalfEdge {

public:

	HalfEdge(Vertex* origin, HalfEdge* twin, Face* incidentFace, HalfEdge* next, HalfEdge* prev) : origin(origin), twin(twin), incidentFace(incidentFace), next(next), prev(prev)
	{

	}

	//Loops through all the half edges around a vertex
	std::vector<HalfEdge*> through_vertex_halfedges(Vertex v);

	//Splits edges on an intersection
	std::vector<HalfEdge*> split_edges(HalfEdge edge1, HalfEdge edge2);

	HalfEdge splice_edges();

	//Insert an intersecting edge
	HalfEdge insert_edge();

	//Gets the first clockwise half edge with origin V
	HalfEdge get_cw_halfedge(HalfEdge* halfedge);

	Vertex* origin;
	HalfEdge* twin;
	Face* incidentFace;
	HalfEdge* next;
	HalfEdge* prev;

};
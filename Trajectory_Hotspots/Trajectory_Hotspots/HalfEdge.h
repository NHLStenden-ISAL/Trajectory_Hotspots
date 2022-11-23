#pragma once

class HalfEdge {

public:

	HalfEdge(Vertex* origin, HalfEdge* twin, Face* incidentFace, HalfEdge* next, HalfEdge* prev) : origin(origin), twin(twin), incidentFace(incidentFace), next(next), prev(prev)
	{

	}

	//Loops through all the half edges around a vertex
	std::vector<HalfEdge*> through_vertex_halfedges(Vertex* v);

	//Splits edges on an intersection
	std::vector<HalfEdge*> split_edges(HalfEdge* edge1, HalfEdge* edge2, Vertex* intersection);

	std::vector<float*> get_polar_angles(std::vector<HalfEdge*> halfedges, Vertex* v);
	
	HalfEdge* splice_edges(HalfEdge* edge1, HalfEdge* edge2, HalfEdge* edge11, HalfEdge* edge22);

	//Gets the first clockwise half edge with origin V
	HalfEdge* get_cw_halfedge(HalfEdge* halfedge, Vertex* v);
	
	//Insert an intersecting edge
	HalfEdge* insert_edge(HalfEdge* halfedge, Dcel dcel);

	

	Vertex* origin;
	HalfEdge* twin;
	Face* incidentFace;
	HalfEdge* next;
	HalfEdge* prev;

};
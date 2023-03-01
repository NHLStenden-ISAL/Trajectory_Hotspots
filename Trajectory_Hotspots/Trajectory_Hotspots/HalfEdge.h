#pragma once

class Vertex;
class Dcel;
class Face;

class HalfEdge {

public:

	HalfEdge(Vertex* origin, HalfEdge* twin, Face* incidentFace, HalfEdge* next, HalfEdge* prev) : origin(origin), twin(twin), incidentFace(incidentFace), next(next), prev(prev)
	{

	}

	//Loops through all the half edges around a vertex
	std::vector<HalfEdge*> get_incident_edges(Vertex* v);

	void correct_prev_next_ptrs(Vertex* v, std::vector<float> angles, float new_angle, HalfEdge* new_edge, std::vector<HalfEdge*> halfedges);

	//Splits edges on an intersection
	std::vector<HalfEdge*> split_edges(HalfEdge* edge1, HalfEdge* edge2, Vertex* intersection);

	static std::vector<float> get_polar_angles(std::vector<HalfEdge*> halfedges, Vertex* v);

	static std::vector<Vec2> get_vertex_positions(std::vector<HalfEdge*> halfedges, Vertex* v);
	
	//Insert an intersecting edge
	//HalfEdge* insert_edge(HalfEdge* halfedge, Dcel dcel);

	

	Vertex* origin;
	HalfEdge* twin;
	Face* incidentFace;
	HalfEdge* next;
	HalfEdge* prev;

};
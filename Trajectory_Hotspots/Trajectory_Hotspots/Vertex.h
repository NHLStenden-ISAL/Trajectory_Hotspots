#pragma once

class HalfEdge;
class Face;

class Vertex {

public:

	Vertex(Vec2 position, std::vector<HalfEdge*> incidentEdge) : position(position), incidentEdge(incidentEdge)
	{
		
	}

	//Get a point inside the face (not on its edge) houd rekening met drie vertices op een lijn. (lijn 725 in main.cpp)
	Vertex get_point(Face face);

	Vertex create_vertex(Vec2 position);

	Vec2 position;
	std::vector<HalfEdge*> incidentEdge;  
	
    

};
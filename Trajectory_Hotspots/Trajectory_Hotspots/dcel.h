#pragma once

class HalfEdge;

class Dcel 
{
public:

	
	Dcel(std::vector<std::unique_ptr<HalfEdge>> &&halfedges,
		 std::vector<std::unique_ptr<Face>> &&faces,
		 std::vector<std::unique_ptr<Vertex>> &&vertices)
	{
			this->halfedges = std::move(halfedges);
			this->faces = std::move(faces);
			this->vertices = std::move(vertices);
	}

	Dcel map_overlay(Dcel dcel1, Dcel dcel2);
	
	std::vector<std::unique_ptr<HalfEdge>> halfedges;
	std::vector<std::unique_ptr<Face>> faces;
	std::vector<std::unique_ptr<Vertex>> vertices;

};


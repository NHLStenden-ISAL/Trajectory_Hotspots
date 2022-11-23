#pragma once

class Dcel 
{
public:

	Dcel(std::vector<std::unique_ptr<HalfEdge>> halfedges, std::vector<std::unique_ptr<Face>> faces, std::vector<std::unique_ptr<Vertex>> vertices) : halfedges(halfedges), faces(faces), vertices(vertices)
	{

	}

	Dcel map_overlay(Dcel dcel1, Dcel dcel2);
	
	std::vector<std::unique_ptr<HalfEdge>> halfedges;
	std::vector<std::unique_ptr<Face>> faces;
	std::vector<std::unique_ptr<Vertex>> vertices;

};


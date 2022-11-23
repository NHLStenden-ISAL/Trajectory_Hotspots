#pragma once

#include "pch.h"
#include "HalfEdge.h"
#include "Vertex.h"
#include "Face.h"


class dcel 
{
public:

	dcel(std::vector<std::unique_ptr<HalfEdge>> halfedges, std::vector<std::unique_ptr<Face>> faces, std::vector<std::unique_ptr<Vertex>> vertices) : halfedges(halfedges), faces(faces), vertices(vertices)
	{

	}

	dcel map_overlay(dcel dcel1, dcel dcel2);
	
	std::vector<std::unique_ptr<HalfEdge>> halfedges;
	std::vector<std::unique_ptr<Face>> faces;
	std::vector<std::unique_ptr<Vertex>> vertices;

};


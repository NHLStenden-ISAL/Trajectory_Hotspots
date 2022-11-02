#pragma once

#include "pch.h"
#include "HalfEdge.h"

class Vertex {

public:

	Vertex(Vec2 position, HalfEdge* incidentEdge) : position(position), incidentEdge(incidentEdge)
	{
		
	}

private:

	Vec2 position;
	//TODO: make it point to half edge with v as origin 
	HalfEdge* incidentEdge;

};
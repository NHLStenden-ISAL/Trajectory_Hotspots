#pragma once

#include "Vertex.h"
#include "Face.h"
#include "pch.h"

class HalfEdge {

public:

	HalfEdge(Vertex* origin, HalfEdge* twin, Face* incidentFace, HalfEdge* next, HalfEdge* prev) : origin(origin), twin(twin), incidentFace(incidentFace), next(next), prev(prev)
	{

	}

private:

	Vertex* origin;
	HalfEdge* twin;
	Face* incidentFace;
	HalfEdge* next;
	HalfEdge* prev;

};
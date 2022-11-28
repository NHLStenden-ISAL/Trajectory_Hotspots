#include "pch.h"

#include "face.h"
#include "halfedge.h"
#include "vertex.h"
#include "dcel.h"

Dcel Dcel::map_overlay(Dcel dcel1, Dcel dcel2)
{
	return Dcel(std::vector<std::unique_ptr<HalfEdge>>(), std::vector<std::unique_ptr<Face>>(), std::vector<std::unique_ptr<Vertex>>());
}


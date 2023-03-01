#include "pch.h"

#include "face.h"
#include "halfedge.h"
#include "vertex.h"
#include "dcel.h"

Dcel Dcel::map_overlay(Dcel dcel1, Dcel dcel2)
{
	//Copy dcel1 and 2 into a new dcel
	//Dcel dcel3 = dcel3.move(dcel1.halfedges, dcel1.faces, dcel1.vertices);


	
	return Dcel(std::vector<std::unique_ptr<HalfEdge>>(), std::vector<std::unique_ptr<Face>>(), std::vector<std::unique_ptr<Vertex>>());
	
}

std::vector<std::unique_ptr<HalfEdge>>& Dcel::get_halfedges()
{
	return halfedges;
}


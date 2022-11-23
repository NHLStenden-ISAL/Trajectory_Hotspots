#include "pch.h"
#include "HalfEdge.h"

#include "Vertex.h"
#include "Face.h"
#include "dcel.h"

//First offset_edge index is N. First twin_edge index is N+1.
std::vector<HalfEdge*> HalfEdge::through_vertex_halfedges(Vertex* v)
{
	//Take an incident edge(de edge die naar de gegeven vertex point)
	//Is taking the first element in the incidentedge a good idea?
	HalfEdge* edge = v->incidentEdge[0];

	HalfEdge* offset_edge = edge->next->twin;

	HalfEdge* twin_edge = offset_edge->twin;

	std::vector<HalfEdge*> result;
	
	//Vanaf de incident edge loop door de rest van de edges heen. Stopt als hij weer bij zichzelf is.
	while (offset_edge != edge)
	{
		result.emplace_back(offset_edge);
		result.emplace_back(twin_edge);

		offset_edge = offset_edge->next->twin;
		twin_edge = offset_edge->twin;
	}
	
	return result;
	
}

std::vector<HalfEdge*> HalfEdge::split_edges(HalfEdge* edge1, HalfEdge* edge2, Vertex* intersection)
{
	//Je splitst eigenlijk de edge door nieuwe records te creëeren en daarbij veel dingen te doen zodat alle ptrs goed staan.
	std::vector<HalfEdge*> result;
	//incidentFace is the face that is bound by (unique) half edges. Every half edge only bounds one face.

	//Copy the next and prev ptrs of the old edges which we later put into the new half edges.
	HalfEdge* next_edge1 = edge1->next;
	HalfEdge* next_edge2 = edge2->next;
	
	HalfEdge* prev_edge1 = edge1->prev;
	HalfEdge* prev_edge2 = edge2->prev;

	//Create the 2 new edges
	//TODO: Change incidentFace so that it is correct.(need sweepline/tree)
	HalfEdge* new_edge1 = new HalfEdge(intersection, edge1, edge1->incidentFace, next_edge2, prev_edge1);
	HalfEdge* new_edge2 = new HalfEdge(intersection, edge2, edge2->incidentFace, next_edge1, prev_edge2);
	
	//The edges that the next pointers of the new edges point to must change their prev to the new edges.
	next_edge1->prev = new_edge1;
	next_edge2->prev = new_edge2;

	//When rotating through the halfedges, calculate all the radians from the neighboring vertices and compare that 
	//to the radians from the intersection vertex to the neighboring vertices.

	//Get angles from all the halfedges around the intersection
	std::vector<HalfEdge*> halfedges = through_vertex_halfedges(intersection);
	std::vector<float*> angles = get_polar_angles(halfedges, intersection);

	//Get the angles from the new edges to compare them
	std::vector<HalfEdge*> new_edges = { new_edge1, new_edge2 };
	std::vector<float*> new_angles = get_polar_angles(new_edges, intersection);


	
	//Place 2 new half edges in vector
	result.emplace_back(new_edge1);
	result.emplace_back(new_edge2);
	
	return result;
}

std::vector<float*> HalfEdge::get_polar_angles(std::vector<HalfEdge*> halfedges, Vertex* v)
{
	std::vector<HalfEdge*> incidentedges;

	for (size_t i = 0; i < halfedges.size(); i++)
	{
		//If the origin of the halfedge isn't the intersection, its incident to the intersection
		if (halfedges[i]->origin != v)
		{
			//So we place it into the incidentedges vector to get the origin (a vec2) so we can use it for the polar coordinate system.
			incidentedges.emplace_back(halfedges[i]);
		}
	}

	std::vector<Vec2> vertex_positions;

	for (size_t i = 0; i < incidentedges.size(); i++)
	{
		Vec2 xy = incidentedges[i]->origin->position;

		vertex_positions.emplace_back(xy);
	}

	std::vector<float*> polar_angles;

	for (size_t i = 0; i < vertex_positions.size(); i++)
	{
		//If vertex_positions[i].y, vertex_positions[i].x are both larger than 0 we just calculate the radians
		if (vertex_positions[i].y > 0 && vertex_positions[i].x > 0)
		{
			float radians = (atan2(vertex_positions[i].y, vertex_positions[i].x));
			polar_angles.emplace_back(radians);
		}
		//If vertex_positions[i].x is smaller than 0 and vertex_positions[i].y is bigger than 0 or both x and y are negative we add 180 degrees to the radians
		else if (vertex_positions[i].x < 0 && vertex_positions[i].y > 0 || vertex_positions[i].x < 0 && vertex_positions[i].y < 0)
		{
			float radians = (atan2(vertex_positions[i].y, vertex_positions[i].x)) + 180;
			polar_angles.emplace_back(radians);
		}
		//If vertex_positions[i].y is positive and vertex_positions[i].x is negative we add 360 degrees to the radians
		else if (vertex_positions[i].y < 0 && vertex_positions[i].x > 0)
		{
			float radians = (atan2(vertex_positions[i].y, vertex_positions[i].x)) + 360;
			polar_angles.emplace_back(radians);
		}
	}
	
	return polar_angles;
}

HalfEdge* HalfEdge::splice_edges(HalfEdge* edge1, HalfEdge* edge2, HalfEdge* edge11, HalfEdge* edge22)
{
	//If the vertex is an intersection, add 2 new half edge records to the half edge list with origin vertex position.
	//Pair the new half edges by setting their twin pointers to the other half edge.
	//Copy the next pointers of the half edges to the next pointer of the old half edge that is not its twin.
	//Update the prev pointer by pointing to the new half edges.
//	Vec2 vertex_position = v.position;
//	Vec2 intersection_position = intersection.position;
//	
//	std::vector<HalfEdge> halfedges;
//	
//	if (vertex_position == intersection_position)
//	{
//		HalfEdge new_halfedges = HalfEdge(&intersection, &halfedge, halfedge.incidentFace, nullptr, nullptr);
//		halfedges.emplace_back(new_halfedges);
//	}
  }

HalfEdge* HalfEdge::get_cw_halfedge(HalfEdge* halfedge, Vertex* v)
{
	HalfEdge* edge = v->incidentEdge[0]->twin;

	//Krijg de eerstvolgende clockwise halfedge met: HE->Twin->Next
	if (edge->origin != halfedge->origin)
	{
		return halfedge->twin->next;
	}
}

HalfEdge* HalfEdge::insert_edge(HalfEdge* halfedge, Dcel dcel)
{
	return dcel.halfedges.emplace_back(halfedge);
}


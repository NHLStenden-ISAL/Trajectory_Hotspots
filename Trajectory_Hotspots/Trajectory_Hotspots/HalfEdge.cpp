#include "pch.h"

#include "vertex.h"
#include "face.h"
#include "dcel.h"
#include "halfedge.h"

//First offset_edge index is N. First twin_edge index is N+1.
std::vector<HalfEdge*> HalfEdge::get_incident_edges(Vertex* v)
{
	//Take an incident edge(The edge that points away from a vertex)
	//Is taking the first element in the incidentedge a good idea?
	HalfEdge* edge = v->incidentEdge[0];

	HalfEdge* offset_edge = edge->twin->next;

	//HalfEdge* twin_edge = offset_edge->twin;

	std::vector<HalfEdge*> result;
	
	result.emplace_back(edge);
	
	//Vanaf de incident edge loop door de rest van de edges heen. Stopt als hij weer bij zichzelf is.
	while (offset_edge != edge)
	{
		result.emplace_back(offset_edge);
		//result.emplace_back(twin_edge);

		offset_edge = offset_edge->twin->next;
		//twin_edge = offset_edge->twin;
	}
	
	return result;
	
}

void HalfEdge::correct_prev_next_ptrs(Vertex* v, std::vector<float> angles, float new_angle, HalfEdge* new_edge, std::vector<HalfEdge*> halfedges)
{
	for (size_t i = 0; i < angles.size(); i++)
	{
		//Finds which 2 edges the new edge is between
		if (new_angle < angles[i + 1] && new_angle > angles[i])
		{
			//Use angles index on halfedges to change angles[i + 1] halfedge's pointing away from intersection his previous pointer to the new halfedge
			//And change the counter clockwise halfedge which points to the intersection his next pointer to the new halfedge
					
			//if the vertex incident edge's twin's origin is the intersection we change the previous pointer of the twin
			//Since we used get_incident_edges, we already got the incident edges
			halfedges[(i + 1) % halfedges.size()]->prev = new_edge->twin;
						
			new_edge->twin->next = halfedges[(i + 1) % halfedges.size()];
					
			halfedges[i]->next = new_edge;
						
			new_edge->prev = halfedges[i];
		}
	}
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
	HalfEdge* new_edge1 = new HalfEdge(intersection, edge2, edge1->incidentFace, next_edge2, prev_edge1);
	HalfEdge* new_edge2 = new HalfEdge(intersection, edge1, edge2->incidentFace, next_edge1, prev_edge2);

	//Set the old twin pointers to the new edges that is not its twin
	edge1->twin = new_edge2;
	edge2->twin = new_edge1;
	
	//The edges that the next pointers of the new edges point to must change their prev to the new edges.
	next_edge1->prev = new_edge1;
	next_edge2->prev = new_edge2;

	//When rotating through the halfedges, calculate all the radians from the neighboring vertices and compare that 
	//to the radians from the intersection vertex to the neighboring vertices.

	//Get angles from all the halfedges around the intersection
	std::vector<HalfEdge*> halfedges = get_incident_edges(intersection);
	std::vector<float> angles = get_polar_angles(halfedges, intersection);

	//Get the angles from the new edges to compare them
	std::vector<HalfEdge*> new_edges = { new_edge1, new_edge2 };
	std::vector<float> new_angles = get_polar_angles(new_edges, intersection);
	
	correct_prev_next_ptrs(intersection, angles, new_angles[0], new_edge1, halfedges);
	correct_prev_next_ptrs(intersection, angles, new_angles[1], new_edge2, halfedges);
	
	
	//TODO: Check if function should return a list or add straight to a dcel.
	result.emplace_back(new_edge1);
	result.emplace_back(new_edge2);
	
	return result;
}

std::vector<Vec2> HalfEdge::get_vertex_positions(std::vector<HalfEdge*> halfedges, Vertex* v)
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

	//Get the origin of the incidentedges, which is a position.
	std::vector<Vec2> vertex_positions;

	for (size_t i = 0; i < incidentedges.size(); i++)
	{
		Vec2 xy = incidentedges[i]->origin->position;

		vertex_positions.emplace_back(xy);
	}
	
	return vertex_positions;
}

std::vector<float> HalfEdge::get_polar_angles(std::vector<HalfEdge*> halfedges, Vertex* v)
{
	//Get the origin of the incidentedges, which is a position.
	std::vector<Vec2> vertex_positions = get_vertex_positions(halfedges, v);

	//Turn the vertex positions into polar coordinates
	std::vector<float> polar_angles;

	for (size_t i = 0; i < vertex_positions.size(); i++)
	{
		Vec2 target_vector = vertex_positions[i] - v->position;
		
		//If target_vector.y, target_vector.x are both larger than 0 we just calculate the radians
		if (target_vector.y > 0 && target_vector.x > 0)
		{
			float radians = (atan2(target_vector.y, target_vector.x));
			polar_angles.emplace_back(radians);
		}
		//If target_vector.x is smaller than 0 and target_vector.y is bigger than 0 or both x and y are negative we add 180 degrees to the radians
		else if (target_vector.x < 0 && target_vector.y > 0 || target_vector.x < 0 && target_vector.y < 0)
		{
			float radians = (atan2(target_vector.y, target_vector.x)) + 180;
			polar_angles.emplace_back(radians);
		}
		//If target_vector.y is positive and target_vector.x is negative we add 360 degrees to the radians
		else if (target_vector.y < 0 && target_vector.x > 0)
		{
			float radians = (atan2(target_vector.y, target_vector.x)) + 360;
			polar_angles.emplace_back(radians);
		}
	}
	
	return polar_angles;
}


//HalfEdge* HalfEdge::insert_edge(HalfEdge* halfedge, Dcel dcel)
//{
//	return dcel.halfedges.emplace_back(halfedge);
//}


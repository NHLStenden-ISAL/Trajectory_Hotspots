#pragma once

class Face {

public:

	Face(HalfEdge* outerComponent) : outerComponent(outerComponent)
	{

	}

	//Loopt door de faces en returnt alle neighboring faces, niet de infinite face
	Face get_neighbors(Face face);

	//Method name change?
	Face delta_function();

private:

	HalfEdge* outerComponent;


};
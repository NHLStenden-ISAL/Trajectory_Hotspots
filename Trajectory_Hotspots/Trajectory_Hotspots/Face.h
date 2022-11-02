#pragma once

#include "pch.h"
#include "HalfEdge.h"

class Face {

public:

	Face(HalfEdge* outerComponent) : outerComponent(outerComponent)
	{

	}

private:

	HalfEdge* outerComponent;

};
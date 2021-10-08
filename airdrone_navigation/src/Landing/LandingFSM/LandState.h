#pragma once

#include "Land.h"

class Land;

class LandState
{
public:
	virtual void enter(Land* land) = 0;
	
	virtual void toggle(Land* land) = 0;
	
	virtual void exit(Land* land) = 0;
	
	virtual ~LandState() {}
};

#pragma once

#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <ctype.h>
#include <time.h>

#include <cmath>
#include <ctime>
#include <cstdio>
#include <vector>
#include "Vector3.h"

#include "main.h"


struct OrbitalElements {
	float ap;
	float pe;
	float sma;
	float ecc;
	float inc;
	float lan;
	float argpe;
};

class Orbit
{
private:
	
	OrbitalElements parameters;
	std::vector<float> vertices;
	int apIndex;
	int peIndex;

	float gradientStart;

	bool vessel;
	
	void			calculateOrbitalElements(Vector3<float>, Vector3<float>);
	void			calculateEllipticalOrbit();
	void			calculateHyperbolicOrbit();

	Vector3<float>	applyOrbitalRotations(Vector3<float>, float, float, float);
	Vector3<float>	calculateHyperbolicPosition(float, float, float);
	Vector3<float>	calculateEllipticalPosition(float, float, float, float);

	Vector3<float>	calculateNormalVector() const;

	

public:
	Orbit();
	Orbit(Vector3<float>, Vector3<float>);
	Orbit(float, float, float, float, float);

	OrbitalElements		getParameters() const;
	std::vector<float>	getVertices() const;
	int			getApIndex() const;
	int			getPeIndex() const;
	bool		isVesselOrbit() const;
	float		getGradientStart() const;
	void		setGradientStart(float);

	void		calculateOrbit();
	void		calculateOrbit(Vector3<float>, Vector3<float>);

	void		findANDN(const Orbit& otherOrbit, int& ascendingNodeIndex, int& descendingNodeIndex) const;
	void		findANDN(int& ascendingNodeIndex, int& descendingNodeIndex) const;
	
	float		calculateTransferDv(Orbit otherOrbit) const;

	int			getVertex(float) const;

};

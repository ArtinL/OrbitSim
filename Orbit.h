#pragma once

#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <ctype.h>
#include <time.h>

#include <cmath>
#include <ctime>
#include <algorithm>
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
	std::vector<Vector3<float>> vertices;
	int apIndex;
	int peIndex;

	int referenceIndex;
	
	void			calculateOrbitalElements(Vector3<float>, Vector3<float>);
	void			calculateEllipticalOrbit();
	void			calculateHyperbolicOrbit();

	Vector3<float>	applyOrbitalRotations(Vector3<float>, float, float, float);
	Vector3<float>	calculateHyperbolicPosition(float, float, float);
	Vector3<float>	calculateEllipticalPosition(float, float, float, float);

	Vector3<float>	calculateHVector() const;
	

public:
	Orbit();
	Orbit(Vector3<float>, Vector3<float>);
	Orbit(float, float, float, float, float);

	std::vector<Vector3<float>>	getVertices() const;
	OrbitalElements				getParameters() const;

	int		getApIndex() const;
	int		getPeIndex() const;
	int		getReferenceIndex() const;
	void	setReferenceIndex(int);
	void	setReferenceEccAnomaly(float);

	void	calculateOrbit();
	void	calculateOrbit(Vector3<float>, Vector3<float>);

	float	findANEccAnomaly(const Orbit*, float&) const;
	void	findOrbitalNodeIndecies(const Orbit*, int&, int&, float&) const;

	float	calculateTransferDV(const Orbit*) const;

	int		getVertex(float) const;

};


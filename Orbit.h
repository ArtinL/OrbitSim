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

	std::vector<Vector3<float>>	getVertices() const;
	OrbitalElements				getParameters() const;

	int		getApIndex() const;
	int		getPeIndex() const;
	bool	isVesselOrbit() const;
	float	getGradientStart() const;
	void	setGradientStart(float);

	void	calculateOrbit();
	void	calculateOrbit(Vector3<float>, Vector3<float>);

	float	findRelativeInc(const Orbit* otherOrbit) const;

	int		getVertex(float) const;

};


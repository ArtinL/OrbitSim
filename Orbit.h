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
#include <functional>
#include <stdexcept>

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

	float atlasTrueAnomaly;
	int atlasIndex;

	Orbit* targetOrbit;

	int ANindex;
	int DNindex;
	float relativeInc;
	
	void			calculateOrbitalElements(Vector3<float>, Vector3<float>);

	void			calculateEllipticalOrbit();
	void			calculateHyperbolicOrbit();

	Vector3<float>	applyOrbitalRotations(Vector3<float>, float, float, float) const;
	void			generateOrbitVertices(const std::function<Vector3<float>(float)>&, float, float, int);

	Vector3<float>	calculateHVector() const;

	float			findANEccAnomaly();
	void			calcOrbitalNodeIndecies();
	

public:
	Orbit();
	Orbit(Vector3<float>, Vector3<float>);
	Orbit(float, float, float, float, float);

	std::vector<Vector3<float>>	getVertices() const;
	OrbitalElements				getParameters() const;

	int		getApIndex() const;
	int		getPeIndex() const;

	
	void    setAtlasTrueAnomaly(float);
	float	getAtlasTrueAnomaly() const;

	int		getAtlasIndex() const;
	void	setAtlasIndex(int);

	void	setTargetOrbit(Orbit*);
	Orbit*	getTargetOrbit() const;
	void	clearTargetOrbit();

	int		getANIndex() const;
	int		getDNIndex() const;
	float	getRelativeInc() const;

	void	calculateOrbit();
	void	calculateOrbit(Vector3<float>, Vector3<float>);

	float	calculateTransferDV(const Orbit*) const;

	int		getVertexFromTrueAnomaly(float) const;
	float   getTrueAnomalyFromVertex(int) const;



};


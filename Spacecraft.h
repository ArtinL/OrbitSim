#pragma once
#ifndef SPACECRAFT_H
#define SPACECRAFT_H

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
#include <string>

#include "glut.h"
#include "Vector3.h"
#include "Orbit.h"
#include "main.h"

enum ImpulseDirection {
	PROGRADE,
	RETROGRADE,
	RADIALIN,
	RADIALOUT,
	NORMAL,
	ANTINORMAL
};

enum AttitudeDirection {
	PITCH_UP,
	PITCH_DOWN,
	ROLL_LEFT,
	ROLL_RIGHT,
	YAW_LEFT,
	YAW_RIGHT
};

class Spacecraft {
private:

	static int count;

	int			id;
	bool		destroyed;
	std::string name;

    Vector3<float>	position;
    Vector3<float>	velocity;

	Orbit*	orbit;

	Vector3<float> computeAcceleration(const Vector3<float>&);
	void rungeKuttaStep(const Vector3<float>&, const Vector3<float>&, float, Vector3<float>&, Vector3<float>&);

public:
    Spacecraft();
	Spacecraft(std::string, int initAlt, int initVel);
	Spacecraft(std::string, Vector3<float> position, Vector3<float> velocit);

	int getId() const;
	std::string getName() const;

	bool isDestroyed() const;
	void destroy();

	Vector3<float> getPosition() const;
	Vector3<float> getVelocity() const;

	Orbit*	getOrbit() const;
	void	recalculateOrbit();

    void	applyImpulse(ImpulseDirection, float);
    void	updateState(float);

	float	getTrueAnomaly() const;
	//float   getEccentricAnomaly() const;

	int		getVertexIndex() const;

	void    assignTarget(Orbit*);
	void    removeTarget();

};

#endif // SPACECRAFT_H

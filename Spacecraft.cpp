#include "Spacecraft.h"
#include "Orbit.h"


// Default constructor
Spacecraft::Spacecraft() : 
	Spacecraft("Default", Vector3<float>(), Vector3<float>(), Vector3<float>(), Vector3<float>(), Vector3<float>()) {}

Spacecraft::Spacecraft(std::string name, int initAlt, int initVel)
	: Spacecraft(
		name,
		Vector3<float>{ KM_TO_U(initAlt) + 1.0f, 0.0f, 0.0f },
		Vector3<float>{ 0.0f, 0.0f, -M_TO_U(initVel) },
		Vector3<float>{ 0.0f, 0.0f, 1.0f },
		Vector3<float>{ 1.0f, 0.0f, 0.0f },
		Vector3<float>{ 0.0f, 1.0f, 0.0f }
	) {}


Spacecraft::Spacecraft(std::string name, Vector3<float> position, Vector3<float> velocity, Vector3<float> forward, Vector3<float> right, Vector3<float> up) {
	this->name = name;
	this->position = position;
	this->velocity = velocity;
	this->forward = forward;
	this->right = right;
	this->up = up;

	this->destroyed = false;
	orbit = new Orbit(position, velocity);
	id = count++;
}

int Spacecraft::count = 0;

int Spacecraft::getId() const {
	return id;
}

std::string Spacecraft::getName() const {
	return name;
}

bool Spacecraft::isDestroyed() const {
	return destroyed;
}

void Spacecraft::destroy() {
	destroyed = true;
}

Vector3<float> Spacecraft::getPosition() const {
	return position;
}

Vector3<float> Spacecraft::getVelocity() const {
	return velocity;
}

Vector3<float> Spacecraft::getForward() const {
	return forward;
}

Vector3<float> Spacecraft::getRight() const {
	return right;
}

Vector3<float> Spacecraft::getUp() const {
	return up;
}

Orbit* Spacecraft::getOrbit() const {
	return orbit;
}

void Spacecraft::recalculateOrbit() {
	orbit->calculateOrbit(position, velocity);
}

Vector3<float> Spacecraft::computeAcceleration(const Vector3<float>& position) {
	float r = position.magnitude();
	Vector3<float> direction = position.normalize();
	float accelerationMagnitude = -MU / (r * r);
	return direction * accelerationMagnitude;
}

void Spacecraft::rungeKuttaStep(const Vector3<float>& position, const Vector3<float>& velocity, float deltaTime, Vector3<float>& k_r, Vector3<float>& k_v) {
	Vector3<float> acceleration = computeAcceleration(position);
	k_v = acceleration * deltaTime;
	k_r = velocity * deltaTime;
}


void Spacecraft::updateState(float ScaledDeltaTime) {
	// Ensure ScaledDeltaTime is positive to avoid negative time steps
	float deltaTime = ScaledDeltaTime / 1000.0f; // Convert milliseconds to seconds

	float PosMag = position.magnitude();

	// Convert position and velocity from OpenGL units to SI units
	Vector3<float> positionSI = U_TO_M(position);
	Vector3<float> velocitySI = U_TO_M(velocity);

	// Runge-Kutta 4th order integration
	Vector3<float> k1_r, k1_v, k2_r, k2_v, k3_r, k3_v, k4_r, k4_v;

	// k1 for position and velocity
	rungeKuttaStep(positionSI, velocitySI, deltaTime, k1_r, k1_v);

	// k2 for position and velocity
	Vector3<float> midVelocity = velocitySI + k1_v * 0.5f;
	Vector3<float> midPosition = positionSI + k1_r * 0.5f;
	rungeKuttaStep(midPosition, midVelocity, deltaTime, k2_r, k2_v);

	// k3 for position and velocity
	midVelocity = velocitySI + k2_v * 0.5f;
	midPosition = positionSI + k2_r * 0.5f;
	rungeKuttaStep(midPosition, midVelocity, deltaTime, k3_r, k3_v);

	// k4 for position and velocity
	Vector3<float> finalVelocity = velocitySI + k3_v;
	Vector3<float> finalPosition = positionSI + k3_r;
	rungeKuttaStep(finalPosition, finalVelocity, deltaTime, k4_r, k4_v);

	// Final velocity and position after all 4 stages
	Vector3<float> newVelocitySI = velocitySI + (k1_v + (k2_v + k3_v) * 2.0f + k4_v) / 6.0f;
	Vector3<float> newPositionSI = positionSI + (k1_r + (k2_r + k3_r) * 2.0f + k4_r) / 6.0f;

	// Convert back to OpenGL units
	position = M_TO_U(newPositionSI);
	velocity = M_TO_U(newVelocitySI);

}


void Spacecraft::applyImpulse(ImpulseDirection direction, float magnitude) {

	magnitude = M_TO_U(magnitude);

	if (velocity.magnitude() == 0) {
		velocity = { 0.0f, 0.0f, -0.00001f };
		return;
	}

	Vector3<float> pro = velocity.normalize();
	Vector3<float> norm = position.cross(velocity).normalize();
	Vector3<float> rad = position.normalize();


	Vector3<float> impulseDirection;
	switch (direction) {
	case PROGRADE:
		impulseDirection = pro;
		break;
	case RETROGRADE:
		impulseDirection = pro * -1;
		break;
	case RADIALIN:
		impulseDirection = rad * -1;
		break;
	case RADIALOUT:
		impulseDirection = rad;
		break;
	case NORMAL:
		impulseDirection = norm;
		break;
	case ANTINORMAL:
		impulseDirection = norm * -1;
		break;
	default:
		return;
	}
	
	Vector3<float> impulseVector = impulseDirection * magnitude;

	velocity = velocity + impulseVector;

	orbit->calculateOrbit(position, velocity);
}

float Spacecraft::getTrueAnomaly() const {
	Vector3<float > r = U_TO_M(position);
	Vector3<float > v = U_TO_M(velocity);

	float rMag = r.magnitude();
	float vMag = v.magnitude();

	Vector3<float> h = r.cross(v);

	Vector3<float> e = ((v.cross(h) / MU) - r / rMag);
	float eMag = e.magnitude();

	float cosTheta = (r.dot(e)) / (rMag * eMag);

	float theta = acos(cosTheta);

	return theta;
}

int Spacecraft::getVertexIndex() const {
	return orbit->getVertex(getTrueAnomaly());
}
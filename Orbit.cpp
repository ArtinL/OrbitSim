#include "Orbit.h"

Orbit::Orbit() {
	parameters = OrbitalElements();
	vertices = std::vector<float>();
	apIndex = 0;
	peIndex = 0;
    vessel = true;
    gradientStart = 0.0f;

}

Orbit::Orbit(Vector3<float> position, Vector3<float> velocity) {
	parameters = OrbitalElements();
	vertices = std::vector<float>();
	apIndex = 0;
	peIndex = 0;
	vessel = true;
    gradientStart = 0.0f;

	calculateOrbit(position, velocity);
}

Orbit::Orbit(float ap, float pe, float inc, float lan, float argpe) {
    OrbitalElements parameters {
		ap,
		pe,
		(ap + pe) / 2,
		(ap - pe) / (ap + pe),
		inc,
		lan,
		argpe
	};


    this->parameters = parameters;
    vertices = std::vector<float>();
    apIndex = 0;
    peIndex = 0;
	vessel = false;
	gradientStart = 0.0f;

    calculateOrbit();
}

OrbitalElements Orbit::getParameters() const {
	return parameters;
}

std::vector<float> Orbit::getVertices() const {
	return vertices;
}

int Orbit::getApIndex() const {
	return apIndex;
}

int Orbit::getPeIndex() const {
	return peIndex;
}

bool Orbit::isVesselOrbit() const {
	return vessel;
}

float Orbit::getGradientStart() const {
	return gradientStart;
}

void Orbit::setGradientStart(float s) {
	gradientStart = s;
}

void Orbit::calculateOrbit() {
    vertices.clear();

    if (parameters.ecc < 1.0f)
        calculateEllipticalOrbit();
    else
        calculateHyperbolicOrbit();
}

void Orbit::calculateOrbit(Vector3<float> pos, Vector3<float> vel) {
    calculateOrbitalElements(pos, vel);
    vertices.clear();

    if (parameters.ecc < 1.0f)
        calculateEllipticalOrbit();
    else
        calculateHyperbolicOrbit();

}

void Orbit::findANDN(const Orbit& otherOrbit, int& ascendingNodeIndex, int& descendingNodeIndex) const {
    Vector3<float> normal1 = this->calculateNormalVector();
    Vector3<float> normal2 = otherOrbit.calculateNormalVector();

    Vector3<float> lineOfNodes = normal1.cross(normal2).normalize();

    float ascendingNodeAngle = atan2(lineOfNodes.y, lineOfNodes.x);
    if (ascendingNodeAngle < 0) ascendingNodeAngle += 2 * M_PI;

    float descendingNodeAngle = ascendingNodeAngle + M_PI;
    if (descendingNodeAngle >= 2 * M_PI) descendingNodeAngle -= 2 * M_PI;

    ascendingNodeIndex = getVertex(ascendingNodeAngle);
    descendingNodeIndex = getVertex(descendingNodeAngle);
}

void Orbit::findANDN(int& ascendingNodeIndex, int& descendingNodeIndex) const {
    Vector3<float> normal = this->calculateNormalVector();

    float ascendingNodeAngle = atan2(normal.y, normal.x);
    if (ascendingNodeAngle < 0) ascendingNodeAngle += 2 * M_PI;

    float descendingNodeAngle = ascendingNodeAngle + M_PI;
    if (descendingNodeAngle >= 2 * M_PI) descendingNodeAngle -= 2 * M_PI;

    ascendingNodeIndex = getVertex(ascendingNodeAngle);
    descendingNodeIndex = getVertex(descendingNodeAngle);
}


float Orbit::calculateTransferDv(Orbit targetOrbit) const {

    float sma1 = U_TO_M(parameters.sma);
    float ecc1 = parameters.ecc;
    float inc1 = parameters.inc;
    float lan1 = parameters.lan;
    float argpe1 = parameters.argpe;

    float sma2 = U_TO_M(targetOrbit.parameters.sma);
    float ecc2 = targetOrbit.parameters.ecc;
    float inc2 = targetOrbit.parameters.inc;
    float lan2 = targetOrbit.parameters.lan;
    float argpe2 = targetOrbit.parameters.argpe;

    float totalDeltaV = 0.0f;
  
    return totalDeltaV;
}




int Orbit::getVertex(float trueAnomaly) const {
    size_t numVertices = vertices.size() / 3;
    float thetaStep = 2.0f * M_PI / numVertices;

    size_t closestIndex = static_cast<size_t>(trueAnomaly / thetaStep);
    if (closestIndex >= numVertices) closestIndex = numVertices - 1;

    return static_cast<int>(closestIndex*3);
}


void Orbit::calculateOrbitalElements(Vector3<float> pos, Vector3<float> vel) {
	Vector3<float> y = { 0, 1, 0 };
	Vector3<float> x = { 1, 0, 0 };

	Vector3<float> r = U_TO_M(pos);
	Vector3<float> v = U_TO_M(vel);

	Vector3<float> h = r.cross(v);

	float rMag = r.magnitude();
	float vMag = v.magnitude();
	float hMag = h.magnitude();

	float specificEnergy = (vMag * vMag) / 2 - (MU / rMag);

	float sma = -MU / (2 * specificEnergy);

	Vector3<float> e = (v.cross(h) / MU) - (r / rMag);
	float eMag = e.magnitude();

	float ap = sma * (1 + eMag);
	float pe = sma * (1 - eMag);

	float inc = acos(h.y / hMag);
	if (h.y < 0) inc = -1 * inc;

	Vector3<float> n = y.cross(h);
	float nMag = n.magnitude();

	float lan;
	if (abs(inc) < 1e-6f)
		lan = 0.0f;
	else
		lan = acos(n.x / nMag);

	if (n.z < 0) lan = 2 * M_PI - lan;

	float argpe;

	if (abs(inc) < 1e-8) {
		argpe = atan2(e.z, e.x);
		if (argpe < 0) argpe = 2 * M_PI + argpe;
	}
	else if (eMag > 1e-8) {
		argpe = acos((n.dot(e)) / (nMag * eMag));
		if (n.cross(e).dot(h) > 0) argpe = 2 * M_PI - argpe;
	}
	else argpe = 0;


	parameters.ap = M_TO_U(ap);
	parameters.pe = M_TO_U(pe);
	parameters.sma = M_TO_U(sma);
	parameters.ecc = eMag;
	parameters.inc = inc;
	parameters.lan = lan;
	parameters.argpe = argpe;

}


Vector3<float> Orbit::calculateEllipticalPosition(float theta, float a, float b, float cx) {
    float x = a * cos(theta) + cx;
    float z = b * sin(theta);
    return Vector3<float>(x, 0.0f, z);
}


Vector3<float> Orbit::applyOrbitalRotations(Vector3<float> pos, float inc, float argpe, float lan) {
    float x_rot = pos.x * cos(argpe) - pos.z * sin(argpe);
    float z_rot = pos.x * sin(argpe) + pos.z * cos(argpe);
    float y_rot = pos.y;

    float x_incl = x_rot;
    float y_incl = y_rot * cos(inc) - z_rot * sin(inc);
    float z_incl = y_rot * sin(inc) + z_rot * cos(inc);

    float x_final = x_incl * cos(lan) - z_incl * sin(lan);
    float z_final = x_incl * sin(lan) + z_incl * cos(lan);
    float y_final = y_incl;

    return Vector3<float>(x_final, y_final, z_final);
}

Vector3<float> Orbit::calculateHyperbolicPosition(float theta, float ecc, float pl) {
    float r = pl / (1 + ecc * cos(theta));
    float x = r * cos(theta);
    float y = 0.0f;
    float z = r * sin(theta);

    return Vector3<float>(x, y, z);
}

void Orbit::calculateEllipticalOrbit() {
    int numSegments = 100 + (100 * ((int)parameters.sma / 10));
    numSegments = numSegments > 600 ? 1000 : numSegments;
    float thetaStep = 2.0f * M_PI / numSegments;

    float a = parameters.sma;
    float ecc = parameters.ecc;
    float b = a * sqrt(1 - ecc * ecc);
    float cx = -a * ecc;

    float inc = parameters.inc;
    float lan = parameters.lan;
    float argpe = parameters.argpe;

    int apoapsisIndex = -1;
    int periapsisIndex = -1;
    float maxDistance = -1.0f;
    float minDistance = 1000000;

    for (int i = 0; i < numSegments; i++) {
        float theta = i * thetaStep;
        Vector3<float> pos = calculateEllipticalPosition(theta, a, b, cx);
        pos = applyOrbitalRotations(pos, inc, argpe, lan);

        vertices.push_back(pos.x);
        vertices.push_back(pos.y);
        vertices.push_back(pos.z);

        float distance = std::sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);

        if (distance > maxDistance) {
            maxDistance = distance;
            apoapsisIndex = i;
        }
        if (distance < minDistance) {
            minDistance = distance;
            periapsisIndex = i;
        }
    }

    apIndex = apoapsisIndex;
    peIndex = periapsisIndex;
}

void Orbit::calculateHyperbolicOrbit() {
    int numSegments = 1000;
    

    float ecc = parameters.ecc;
    float pl = parameters.sma * (1 - ecc * ecc);  // Semi-latus rectum approximation for hyperbolic orbits

    float inc = parameters.inc;
    float lan = parameters.lan;
    float argpe = parameters.argpe;

    int apoapsisIndex = -1;
    int periapsisIndex = -1;
    float maxDistance = -1.0f;
    float minDistance = 1000000;
    
    float nuMax = acos(fmin(-1.0f / ecc, 1.0f));


    float thetaStep = 2.0f * nuMax / numSegments;
    for (int i = 0; i < numSegments; i++) {
        float theta = i * thetaStep - nuMax;
        float r = pl / (1 + ecc * cos(theta));
        if (r < 0 || isnan(r) || isinf(r)) continue; // Skip invalid values

        Vector3<float> pos = calculateHyperbolicPosition(theta, ecc, pl);
        pos = applyOrbitalRotations(pos, inc, argpe, lan);

        vertices.push_back(pos.x);
        vertices.push_back(pos.y);
        vertices.push_back(pos.z);

        float distance = std::sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);

        
        if (distance < minDistance) {
            minDistance = distance;
            periapsisIndex = i;
        }
    }

    apIndex = -1;
    peIndex = periapsisIndex;
}

Vector3<float> Orbit::calculateNormalVector() const {
    float sinInc = sin(parameters.inc);
    float cosInc = cos(parameters.inc);
    float sinLan = sin(parameters.lan);
    float cosLan = cos(parameters.lan);

    return Vector3<float>(sinInc * sinLan, -sinInc * cosLan, cosInc);
}

#include "Orbit.h"

Orbit::Orbit() {
	parameters = OrbitalElements();
	vertices = std::vector<Vector3<float>>();
	apIndex = 0;
	peIndex = 0;
    referenceIndex = 0;

}

Orbit::Orbit(Vector3<float> position, Vector3<float> velocity) {
	parameters = OrbitalElements();
	vertices = std::vector<Vector3<float>>();
	apIndex = 0;
	peIndex = 0;
    referenceIndex = 0;

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
    vertices = std::vector<Vector3<float>>();
    apIndex = 0;
    peIndex = 0;
	referenceIndex = 0;

    calculateOrbit();
}

OrbitalElements Orbit::getParameters() const {
	return parameters;
}

std::vector<Vector3<float>> Orbit::getVertices() const {
	return vertices;
}

int Orbit::getApIndex() const {
	return apIndex;
}

int Orbit::getPeIndex() const {
	return peIndex;
}

int Orbit::getReferenceIndex() const {
	return referenceIndex;
}

void Orbit::setReferenceIndex(int i) {
	referenceIndex = i;
}

void Orbit::setReferenceEccAnomaly(float a) {
	referenceIndex = getVertex(a);
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
    calculateOrbit();

}

Vector3<float> Orbit::calculateHVector() const {
	float i = parameters.inc;
	float lan = parameters.lan;

    Vector3<float> h = { sin(i) * sin(lan), cos(i), -sin(i) * cos(lan) };

    return h;
}

float Orbit::findANEccAnomaly(const Orbit* otherOrbit, float& angle) const {

	Vector3<float> h1 = calculateHVector();
	Vector3<float> h2 = otherOrbit->calculateHVector();

    float cosAngle = h1.dot(h2) / (h1.magnitude() * h2.magnitude());
    angle = acos(cosAngle);

	Vector3<float> n = h1.cross(h2);
	n = n.normalize();

    Vector3<float> pe = vertices[peIndex].normalize();

    Vector3<float> e = pe * parameters.ecc;

    float cosTrueAn = (n.dot(e)) / parameters.ecc;
	float sinTrueAn = (h1.cross(n)).dot(e) / parameters.ecc;

	float trueAnomaly = atan2(sinTrueAn, cosTrueAn);


    if (trueAnomaly < 0.0f) trueAnomaly += 2.0f * M_PI;

	float ecc = parameters.ecc;
    float eccentricAnomaly = 2.0f * atan(sqrt((1 - ecc) / (1 + ecc)) * tan(trueAnomaly / 2.0f));
    if (eccentricAnomaly < 0) eccentricAnomaly += 2.0f * M_PI;


    return eccentricAnomaly;

}

void Orbit::findOrbitalNodeIndecies(const Orbit* otherOrbit, int& ANindex, int& DNindex, float& relativeInc) const {

    if (parameters.ecc >= 1) {
		ANindex = -1;
		DNindex = -1;
		relativeInc = 0.0f;
		return;
    }

	float ANEccAcnomaly = findANEccAnomaly(otherOrbit, relativeInc);
	ANindex = getVertex(ANEccAcnomaly);

	float DNEccAnomaly = ANEccAcnomaly + M_PI;
	DNindex = getVertex(DNEccAnomaly);

	relativeInc = RAD_TO_DEG(relativeInc);
}


int Orbit::getVertex(float eccAnomaly) const {

    if (parameters.ecc < 1.0f) {
        eccAnomaly = fmod(eccAnomaly, 2.0f * M_PI);
        if (eccAnomaly < 0) eccAnomaly += 2.0f * M_PI;
    }

    int numSegments = vertices.size();
    float thetaStep;

    if (parameters.ecc < 1.0f) { 
        thetaStep = 2.0f * M_PI / numSegments;
    }
    else { 
        float nuMax = acos(-1.0f / parameters.ecc);
        thetaStep = 2.0f * nuMax / numSegments;
        eccAnomaly = fmax(fmin(eccAnomaly, nuMax), -nuMax);
    }

    int index = static_cast<int>(round(eccAnomaly / thetaStep));
    index = static_cast<int>(fmax(fmin(index, numSegments - 1), 0));

    return (index + 1) % numSegments;
}


float Orbit::calculateTransferDV(const Orbit* otherOrbit) const {
    
    OrbitalElements current = this->getParameters();
    OrbitalElements target = otherOrbit->getParameters();

    float currentSMA = U_TO_M(current.sma);
	float targetSMA = U_TO_M(target.sma);

    float planeChangeDV = 0.0f;
    if (abs(current.inc - target.inc) > 0.001f ||
        abs(current.lan - target.lan) > 0.001f) {
        float v1 = sqrt(MU * (2.0f / currentSMA));
        float angleChange = acos(cos(current.inc) * cos(target.inc) +
            sin(current.inc) * sin(target.inc) *
            cos(current.lan - target.lan));
        planeChangeDV = 2.0f * v1 * sin(angleChange / 2.0f);
    }

    float r1 = currentSMA;
    float r2 = targetSMA;
    float at = (r1 + r2) / 2.0f;

    float v1 = sqrt(MU / r1);  
    float vt1 = sqrt(MU * (2.0f / r1 - 1.0f / at));  

    float v2 = sqrt(MU / r2); 
    float vt2 = sqrt(MU * (2.0f / r2 - 1.0f / at));  

    float dv1 = abs(vt1 - v1);  
    float dv2 = abs(v2 - vt2);  

    if (current.ecc > 0.001f || target.ecc > 0.001f) {
        float vdep = sqrt(MU * (2.0f / r1 - 1.0f / currentSMA));
        float varr = sqrt(MU * (2.0f / r2 - 1.0f / targetSMA));
        dv1 = abs(vt1 - vdep);
        dv2 = abs(varr - vt2);
    }

    return dv1 + dv2 + planeChangeDV;
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
    int numSegments = MIN_ORBIT_VERTICES + (100 * ((int)parameters.sma / 10));
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

        vertices.push_back(pos);

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
    float pl = parameters.sma * (1 - ecc * ecc);  

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
        if (r < 0 || isnan(r) || isinf(r)) continue; 

        Vector3<float> pos = calculateHyperbolicPosition(theta, ecc, pl);
        pos = applyOrbitalRotations(pos, inc, argpe, lan);

        vertices.push_back(pos);

        float distance = std::sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);

        
        if (distance < minDistance) {
            minDistance = distance;
            periapsisIndex = i;
        }
    }

    apIndex = -1;
    peIndex = periapsisIndex;
}
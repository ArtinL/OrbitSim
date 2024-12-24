#include "Orbit.h"
#include "main.h"

Orbit::Orbit() {
	parameters = OrbitalElements();
	vertices = std::vector<Vector3<float>>();
	apIndex = 0;
	peIndex = 0;
	atlasTrueAnomaly = 0.0f;
    atlasIndex = 0;
	targetOrbit = nullptr;
    ANindex = -1;
    DNindex = -1;
    relativeInc = -1.0f;

}

Orbit::Orbit(Vector3<float> position, Vector3<float> velocity) {
	parameters = OrbitalElements();
	vertices = std::vector<Vector3<float>>();
	apIndex = 0;
	peIndex = 0;
    atlasTrueAnomaly = 0.0f;
    atlasIndex = 0;
	targetOrbit = nullptr;
    ANindex = -1;
    DNindex = -1;
    relativeInc = -1.0f;

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
    atlasTrueAnomaly = 0.0f;
	atlasIndex = 0;
	targetOrbit = nullptr;
    ANindex = -1;
    DNindex = -1;
    relativeInc = -1.0f;

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

int Orbit::getAtlasIndex() const {
	return atlasIndex;
}

void Orbit::setAtlasIndex(int index) {
	atlasIndex = index;
	atlasTrueAnomaly = getTrueAnomalyFromVertex(index);
}

void Orbit::setAtlasTrueAnomaly(float a) {
	atlasTrueAnomaly = a;
	atlasIndex = getVertexFromTrueAnomaly(a);
}

float Orbit::getAtlasTrueAnomaly() const {
	return atlasTrueAnomaly;
}

void Orbit::setTargetOrbit(Orbit* target) {
	targetOrbit = target;
	calcOrbitalNodeIndecies();
}

Orbit* Orbit::getTargetOrbit() const {
	return targetOrbit;
}

void Orbit::clearTargetOrbit() {
    targetOrbit = nullptr;
    ANindex = -1;
    DNindex = -1;
    relativeInc = -1.0f;
}

int Orbit::getANIndex() const {
	return ANindex;
}

int Orbit::getDNIndex() const {
	return DNindex;
}

float Orbit::getRelativeInc() const {
	return relativeInc;
}

Vector3<float> Orbit::calculateHVector() const {
	float i = parameters.inc;
	float lan = parameters.lan;

    Vector3<float> h = { sin(i) * sin(lan), cos(i), -sin(i) * cos(lan) };

    return h;
}

float Orbit::findANEccAnomaly() {


	if (targetOrbit == nullptr) throw std::runtime_error("Target orbit not set");

	Vector3<float> h1 = calculateHVector();
	Vector3<float> h2 = targetOrbit->calculateHVector();

    float cosAngle = h1.dot(h2) / (h1.magnitude() * h2.magnitude());
    this->relativeInc = RAD_TO_DEG(acos(cosAngle));

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

void Orbit::calcOrbitalNodeIndecies() {

    if (parameters.ecc >= 1) {
		this->ANindex = -1;
		this->DNindex = -1;
		this->relativeInc = 0.0f;
		return;
    }

	float ANEccAcnomaly = findANEccAnomaly();
	this->ANindex = getVertexFromTrueAnomaly(ANEccAcnomaly);

	float DNEccAnomaly = ANEccAcnomaly + M_PI;
	this->DNindex = getVertexFromTrueAnomaly(DNEccAnomaly);

}


int Orbit::getVertexFromTrueAnomaly(float trueAnomaly) const {

	float eccAnomaly = 2.0f * atan(sqrt((1 - parameters.ecc) / (1 + parameters.ecc)) * tan(trueAnomaly / 2.0f));
    if (eccAnomaly < 0) eccAnomaly += 2.0f * M_PI;

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

float Orbit::getTrueAnomalyFromVertex(int index) const {
	int numSegments = vertices.size();
	float thetaStep = 2.0f * M_PI / numSegments;

	float eccAnomaly = index * thetaStep;

    float trueAnomaly = 2.0f * atan2(
        sqrt(1.0f + parameters.ecc) * sin(eccAnomaly / 2.0f),
        sqrt(1.0f - parameters.ecc) * cos(eccAnomaly / 2.0f)
    );
	return trueAnomaly;
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


void Orbit::calculateOrbit() {
    if (parameters.ecc < 1.0f)
        calculateEllipticalOrbit();
    else
        calculateHyperbolicOrbit();

	if (targetOrbit != nullptr) {
		calcOrbitalNodeIndecies();
	}
}

void Orbit::calculateOrbit(Vector3<float> pos, Vector3<float> vel) {
    calculateOrbitalElements(pos, vel);
    calculateOrbit();
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

Vector3<float> Orbit::applyOrbitalRotations(Vector3<float> pos, float inc, float argpe, float lan) const {
    float xRot = pos.x * cos(argpe) - pos.z * sin(argpe);
    float zRot = pos.x * sin(argpe) + pos.z * cos(argpe);
    float yRot = pos.y;

    float xIncl = xRot;
    float yIncl = yRot * cos(inc) - zRot * sin(inc);
    float zIncl = yRot * sin(inc) + zRot * cos(inc);

    return Vector3<float>(
        xIncl * cos(lan) - zIncl * sin(lan),
        yIncl,
        xIncl * sin(lan) + zIncl * cos(lan)
    );
}

void Orbit::generateOrbitVertices(const std::function<Vector3<float>(float)>& pointGenerator,
    float startTheta, float endTheta, int numSegments) {
    float thetaStep = (endTheta - startTheta) / numSegments;
    float maxDistance = -1.0f;
    float minDistance = std::numeric_limits<float>::max();

    vertices.clear();
    vertices.reserve(numSegments);

    for (int i = 0; i < numSegments; i++) {
        float theta = startTheta + i * thetaStep;
        Vector3<float> pos = pointGenerator(theta);

        if (pos.magnitude() == 0) continue;

        pos = applyOrbitalRotations(pos, parameters.inc, parameters.argpe, parameters.lan);
        vertices.push_back(pos);

        float distance = pos.magnitude();
        if (distance > maxDistance) {
            maxDistance = distance;
            apIndex = i;
        }
        if (distance < minDistance) {
            minDistance = distance;
            peIndex = i;
        }
    }
}

void Orbit::calculateEllipticalOrbit() {
    float a = parameters.sma;
    float ecc = parameters.ecc;
    float b = a * sqrt(1 - ecc * ecc);
    float cx = -a * ecc;

    int numSegments = MIN_ORBIT_VERTICES + (100 * ((int)parameters.sma / 10));

    auto pointGen = [a, b, cx](float theta) {
        return Vector3<float>(
            a * cos(theta) + cx,
            0.0f,
            b * sin(theta)
        );
        };

    generateOrbitVertices(pointGen, 0, 2.0f * M_PI, numSegments);
}

void Orbit::calculateHyperbolicOrbit() {
    float ecc = parameters.ecc;
    float pl = parameters.sma * (1 - ecc * ecc);
    float nuMax = acos(fmin(-1.0f / ecc, 1.0f));

    auto pointGen = [pl, ecc](float theta) {
        float r = pl / (1 + ecc * cos(theta));
        if (r < 0 || isnan(r) || isinf(r))
            return Vector3<float>(0, 0, 0);

        return Vector3<float>(
            r * cos(theta),
            0.0f,
            r * sin(theta)
        );
        };

    generateOrbitVertices(pointGen, -nuMax, nuMax, 1000);
    apIndex = -1;
}


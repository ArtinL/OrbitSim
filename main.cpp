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
#include <sstream>
#include <iomanip>

#ifdef WIN32
#include <windows.h>
#pragma warning(disable:4996)
#endif


#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include "glew.h"
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include "glut.h"

#include "Spacecraft.h"
#include "Vector3.h"
#include "Orbit.h"
#include "main.h"


// title of these windows:

#define WINDOWTITLE  "Orbital Simulation"
#define GLUITITLE    "User Interface Window"

// texture selectors

#define EARTH_TEX_NAME "earth.bmp"
#define STARFIELD_TEX_NAME "stars.bmp"
#define NAVBALL_TEX_NAME "navball.bmp"


// what the glui package defines as true and false:

const int GLUITRUE  = true;
const int GLUIFALSE = false;

// the escape key:

const int ESCAPE = 0x1b;

// initial window size:

const int INIT_WINDOW_W = 720;
const int INIT_WINDOW_H = 1280;

// min/max allowable camera radius:

const float MAX_CAM_RADIUS = 100.0f;
const float MIN_CAM_RADIUS_V = 0.5f;
const float MIN_CAM_RADIUS_P = 1.3f;

// scroll wheel button values:

const int SCROLL_WHEEL_UP   = 3;
const int SCROLL_WHEEL_DOWN = 4;

// mouse movement sensitivity:

const float SCROLL_SENSITIVITY = 0.1f;
const float MOUSE_SENSITIVITY = 0.008f;


// active mouse buttons (or them together):

const int	LEFT   = 4;
const int	MIDDLE = 2;
const int	RIGHT  = 1;

// how long an alert message should be displayed

const int	ALERT_DURATION = 3; // In seconds

// limits for warp levels

const int	MIN_WARP_LEVEL = 0;
const int	MAX_WARP_LEVEL = 9;

// limits for throttle levels

const int	MIN_THROTTLE_LEVEL = 0;
const int	MAX_THROTTLE_LEVEL = 10;

// distance limit for simulation end

const int	SOI_LIMIT = 10000000; // In km

const int	INIT_ALT = 2000; // In km
const int	INIT_VEL = 8000; // In m/s


enum ButtonVals
{
	SPAWN,
	RESET,
	QUIT
};

enum GameMode {
	SANDBOX,
	CHALLENGE,
	NUM_MODES
};

enum OrbitType {
	INACTIVE,
	ACTIVE,
	TARGET
};

std::string GameModeLabels[] = {
	"Sandbox",
	"Challenge"
};

const GLfloat BACKCOLOR[ ] = { 0., 0., 0., 1. };

const GLfloat AXES_WIDTH   = 3.;

// scaling factors for time warp

const int WarpScales[] = {
	1,
	5,
	10,
	50,
	100,
	1000,
	10000,
	100000,
	1000000,
};

// altitude limits for time warp

const int WarpAltLimits[]{
	0,
	0,
	0,
	0,
	0,
	100,
	8000,
	50000,
	500000,
};

// scaling factors for engine throttle

const int ThrottleScales[] = {
	1,
	5,
	10,
	25,
	50,
	75,
	100,
	250,
	500,
	1000
};

// win condition thresholds for challenge mode

const float SMA_WIN_THRESHOLD = 1000.0f;
const float INC_WIN_THRESHOLD = 1.0f;
const float LAN_WIN_THRESHOLD = 5.0f;
const float ARGPE_WIN_THRESHOLD = 5.0f;


// for lighting

const float	WHITE[ ] = { 1.,1.,1.,1. };

enum Color
{
	RED,
	YELLOW,
	GREEN,
	CYAN,
	BLUE,
	MAGENTA,
	DARK_CYAN,
	DARK_MAGENTA,
};


const GLfloat Colors[][3] =
{
	{ 1., 0., 0. },		// red
	{ 1., 1., 0. },		// yellow
	{ 0., 1., 0. },		// green
	{ 0., 1., 1. },		// cyan
	{ 0., 0., 1. },		// blue
	{ 1., 0., 1. },		// magenta
	{ 0., 0.5, 0.5 },	// dark cyan
	{ 0.5, 0., 0.5 },	// dark magenta
};


// non-constant global variables:

int			ActiveButton;
int			AxesOn;					
int			DebugOn;				
int			MainWindow;
int			Xmouse, Ymouse;			

float		cameraRadius;
float		cameraTheta;
float		cameraPhi;

Vector3<float>	cameraPos;

unsigned long	scaledElapsedTime;
unsigned long	trueElapsedTime;
int				scaledDeltaTime;

int			WarpLevel;
int			ThrottleLevel;

GLuint		AxesList;				

GLuint		SphereDL;
GLuint		EarthDL;
GLuint		SunDL;
GLuint		StarfieldDL;
GLuint		VesselDL;

GLuint		EarthTex;
GLuint		StarfieldTex;

bool		lookAtVessel;
int			DoOrbitDetail;
bool		ended;
	
std::string	AlertMsg;
bool		AlertOn = false;
int			AlertStartTime = 0;

std::vector<Spacecraft*> vessels;
int			activeVessel;

int			currGameMode;
std::string	currModeLabel;

Orbit*		targetOrbit;
int			lastThrust;

// function prototypes:

void		Animate( );
void		SetViewportAndProjection();
void		Display( );
void		InitGraphics( );
void		InitLists( );
void		InitMenus( );
void		Keyboard( unsigned char, int, int );
void		MouseButton( int, int, int, int );
void		MouseMotion( int, int );
void		Reset( int );
void		Resize( int, int );
void		Visibility( int );

void		DoAxesMenu(int);
void		DoGameModeMenu(int);
void		DoOrbitDetailMenu(int);
void		DoDebugMenu(int);
void		DoMainMenu(int);
void		DoRasterString(float, float, float, const std::string&);
void		DoStrokeString(float, float, float, float, const std::string&);

void		Alert(const std::string&);

GLuint		LoadBmpTexture(const char*);
void		RenderSun();
void		RenderEarth();
void		RenderStarfield();
void		RenderIndicators(Orbit*, int);
void		PlaceIndicatorWidget(Vector3<float>, Color, const std::string&);

void		RenderTimeWarpWidget();
std::string	GenerateWarpInfo();
std::string	GenerateTimeInfo();
void		RenderOrbitalInfoWidget(Orbit orbit);
void		RenderExtendedOrbitalWidget(Orbit orbit);
void		RenderVelocityWidget();
void		RenderAltitudeWidget();
void		RenderGameModeTitle();
void		RenderThrottleWidget();
void		RenderAlert();

void		DrawOrbit(Orbit*, int type);
void		DrawVessel(Spacecraft* vessel, bool active);

void		warpControl(int);
void		impulseControl(ImpulseDirection);
	
void		cullDestroyedVessels();
void		limitWarpLevel();
void		updateVessels();
void		spawnNew(bool announce = true);

void 		InitSandbox(bool cont = false);
void		InitChallenge();
void		CheckWin();




void	Axes( float );
float	limitExtremes(float, float, float);
float*	MulArray3(float, float[] );
float*	MulArray3(float, float, float, float );
float*	Array3(float, float, float );
float	Ranf(float , float );
void	TimeOfDaySeed();
void	HsvRgb( float[3], float [3] );
void	Cross(float[3], float[3], float[3]);
float	Dot(float [3], float [3]);
float	Unit(float [3], float [3]);
float	Unit(float [3]);

std::string ProcessIntStr(int);
std::string ProcessFloatStr(float);

#include "setmaterial.cpp"
#include "setlight.cpp"
#include "osusphere.cpp"
#include "osucone.cpp"
#include "osutorus.cpp"
#include "bmptotexture.cpp"


// main program:
int
main( int argc, char *argv[ ] )
{
	srand(time(NULL));

	glutInit( &argc, argv );

	InitGraphics( );

	InitLists( );

	Reset(SANDBOX);

	glutSetWindow( MainWindow );
	glutMainLoop( );

	return 0;
}


void
Animate( )
{
	static int lastTime = 0;
	int ms = glutGet(GLUT_ELAPSED_TIME);

	if (lastTime == 0) lastTime = ms;

	int DeltaTime = ms - lastTime;
	lastTime = ms;

	if (ended) return;

	trueElapsedTime += DeltaTime;

	scaledDeltaTime = DeltaTime * WarpScales[WarpLevel];

	scaledElapsedTime += scaledDeltaTime;
	

	glutSetWindow( MainWindow );
	glutPostRedisplay( );
}


void SetViewportAndProjection() {
	GLsizei windowWidth = glutGet(GLUT_WINDOW_WIDTH);
	GLsizei windowHeight = glutGet(GLUT_WINDOW_HEIGHT);

	float windowAspect = (float)windowWidth / (float)windowHeight;

	const float desiredAspect = 16.0f / 9.0f;

	GLsizei viewportWidth, viewportHeight;
	if (windowAspect > desiredAspect) {
		viewportHeight = windowHeight;
		viewportWidth = (GLsizei)(windowHeight * desiredAspect);
	}
	else {
		viewportWidth = windowWidth;
		viewportHeight = (GLsizei)(windowWidth / desiredAspect);
	}

	
	GLint viewportX = (windowWidth - viewportWidth) / 2;
	GLint viewportY = (windowHeight - viewportHeight) / 2;

	
	glViewport(viewportX, viewportY, viewportWidth, viewportHeight);

	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(70.0f, desiredAspect, 0.1f, Z_FAR_CLIP);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

// draw the  scene:

void
Display( )
{

	
	glutSetWindow( MainWindow );

	
	glDrawBuffer( GL_BACK );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	glEnable( GL_DEPTH_TEST );

	glShadeModel( GL_FLAT );
	
	SetViewportAndProjection();

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity( );
	
	glDisable( GL_FOG );

	if (ended) return;

	// Get the position of the vessel
	Vector3<float> vesselPosition = vessels[activeVessel]->getPosition();
	float centerX = lookAtVessel ? vesselPosition.x : 0.0f;
	float centerY = lookAtVessel ? vesselPosition.y : 0.0f;
	float centerZ = lookAtVessel ? vesselPosition.z : 0.0f;

	
	float cameraX = centerX + cameraRadius * sin(cameraPhi) * cos(cameraTheta);
	float cameraY = centerY + cameraRadius * cos(cameraPhi);
	float cameraZ = centerZ + cameraRadius * sin(cameraPhi) * sin(cameraTheta);

	float distance = sqrt(cameraX * cameraX + cameraY * cameraY + cameraZ * cameraZ);

	cameraPos = { cameraX, cameraY, cameraZ };



	// Set the eye position, look-at position, and up-vector
	gluLookAt(cameraX, cameraY, cameraZ,    
		centerX, centerY, centerZ,    
		0.0f, 1.0f, 0.0f);                    


	if( AxesOn != 0 )
	{
		glColor3f(1.0f, 1.0f, 0.0f );
		glCallList( AxesList );
	}

	glEnable( GL_NORMALIZE );

	SetPointLight(GL_LIGHT0, SUN_DISTANCE, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);

	RenderStarfield();
	RenderSun();
	RenderEarth();

	if (currGameMode == CHALLENGE) {
		DrawOrbit(targetOrbit, TARGET);
		CheckWin();
	}
	
	limitWarpLevel();
	updateVessels();
	
	
	glDisable( GL_DEPTH_TEST );
	glDisable(GL_LIGHTING);
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity( );
	glOrtho(0.0f, 160.0f, 0.0f, 90.0f, -15.0f, 15.0f); // Extended Z range
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity( );
	glColor3f(1.0f, 1.0f, 1.0f);

	RenderTimeWarpWidget();
	RenderVelocityWidget();
	RenderAltitudeWidget();
	RenderGameModeTitle();

	if (DoOrbitDetail) { 
		RenderExtendedOrbitalWidget(*(vessels[activeVessel]->getOrbit()));
		if (currGameMode == CHALLENGE) RenderExtendedOrbitalWidget(*targetOrbit);
	}
	else {
		RenderOrbitalInfoWidget(*(vessels[activeVessel]->getOrbit()));
		if (currGameMode == CHALLENGE) RenderOrbitalInfoWidget(*targetOrbit);

	}



	RenderThrottleWidget();

	cullDestroyedVessels();

	if (vessels.empty()) {
		ended = true;
		Alert("All vessels have been destroyed!");
	}
	
	RenderAlert();

	glutSwapBuffers( );

	glFlush( );
}

void updateVessels() {
	

	for (Spacecraft* vessel : vessels) {
		vessel->updateState(scaledDeltaTime);

		if (WarpLevel > 6) vessel->recalculateOrbit();

		DrawOrbit(vessel->getOrbit(), (int) (vessel == vessels[activeVessel]));
		DrawVessel(vessel, vessel == vessels[activeVessel]);


		float kmTrueAlt = U_TO_KM(vessel->getPosition().magnitude() - EARTH_RADIUS);

		if (kmTrueAlt <= 0) {
			Alert(vessel->getName() + " has crashed into the Earth!");
			vessel->destroy();
			activeVessel = 0;
		}
		else if (kmTrueAlt > SOI_LIMIT) {
			Alert(vessel->getName() + " is lost to space!");
			vessel->destroy();
			activeVessel = 0;
		}

	}
}

void limitWarpLevel() {
	float kmTrueAlt = U_TO_KM(vessels[activeVessel]->getPosition().magnitude() - EARTH_RADIUS);
	if (WarpAltLimits[WarpLevel] > kmTrueAlt)
		Alert("Warp limited to x" + ProcessIntStr(WarpScales[--WarpLevel]));
	
}

void cullDestroyedVessels() {
	for (auto it = vessels.begin(); it != vessels.end(); ) {
		if ((*it)->isDestroyed()) {
			delete* it;
			it = vessels.erase(it);
		}
		else {
			++it;
		}
	}
}

void RenderSun() {
	glPushMatrix();

	glDisable(GL_LIGHTING);

	glColor3f(1.0f, 1.0f, 0.0f);
	glTranslatef(SUN_DISTANCE, 0.0f, 0.0f);
	glCallList(SunDL);
	
	glEnable(GL_LIGHTING);

	glPopMatrix();
}

void RenderEarth() {
    glPushMatrix();

    
    float rotationSpeed = 0.0041667f; 
	float earthRotation = rotationSpeed * MS_TO_S(scaledElapsedTime); 

    glRotatef(earthRotation, 0.0f, 1.0f, 0.0f);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glEnable(GL_TEXTURE_2D);
    glCallList(EarthDL);
    glDisable(GL_TEXTURE_2D);

    glPopMatrix();
}

void RenderStarfield() {
	glPushMatrix();
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	glCallList(StarfieldDL);
	glEnable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
}


void DrawOrbit(Orbit* orbit, int type) {
	std::vector<Vector3<float>> OrbitPoints = orbit->getVertices();
	OrbitalElements elements = orbit->getParameters();
	float gradientStart = orbit->getGradientStart();

	size_t numVertices = OrbitPoints.size();
	float gradientStep = 1.0f / numVertices;

	int vesselStartIndex = vessels[activeVessel]->getVertexIndex();

	if (type == TARGET) {
		gradientStart -= gradientStep * 0.1f;
		if (gradientStart < 0.0f) {
			gradientStart += 1.0f;
		}
	}

	glDisable(GL_LIGHTING);

	if (elements.ecc < 1.0f)
		glBegin(GL_LINE_LOOP);
	else
		glBegin(GL_LINE_STRIP);

	for (size_t i = 0; i < numVertices; ++i) {
		float brightness = 1.0f;

		float t = static_cast<float>(i) / numVertices;

		if (t >= gradientStart) {
			brightness = 0.5f + 0.5f * (1.0f - (t - gradientStart));
		}
		else {
			brightness = 0.5f + 0.5f * (1.0f - (t + 1.0f - gradientStart));
		}

		switch (type) {
		case ACTIVE:
			glColor3f(0.0f, 0.7f, 0.7f);
			break;
		case INACTIVE:
			glColor3f(0.4f, 0.4f, 0.4f);
			break;
		case TARGET:
			glColor3f(0.7f * brightness, 0.0f, 0.7f * brightness);
			break;
		}

		const Vector3<float>& point = OrbitPoints[i];
		glVertex3f(point.x, point.y, point.z);
	}
	glEnd();

	RenderIndicators(orbit, type);

	glEnable(GL_LIGHTING);
	orbit->setGradientStart(gradientStart);
}


void PlaceIndicatorWidget(Vector3<float> pos, Color color, const std::string& label) {

	glPushMatrix();

	float cameraDistance = (cameraPos - pos).magnitude();
	float scale = cameraDistance * 0.3;
	glTranslatef(pos.x, pos.y, pos.z);
	
	
	glColor3fv(&Colors[color][0]);
	
	GLfloat modelview[16];
	
	glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == j) {
				modelview[i * 4 + j] = 1.0;
			}
			else {
				modelview[i * 4 + j] = 0.0;
			}
		}
	}
	glLoadMatrixf(modelview);
	glScalef(scale, scale, scale);

	glBegin(GL_TRIANGLES);
	float width = 0.05f;
	float height = 0.08f;

	glVertex3f(-width, height, 0.0f);
	glVertex3f(width, height, 0.0f);
	glVertex3f(0.0f, 0.0, 0.0f);
	glEnd();


	float charWidth = 0.05f; // Adjust this factor based on the font size
	float labelLength = label.length() * charWidth;
	float labelX = -labelLength / 2.0f;

	glColor3f(1.0f, 1.0f, 1.0f);
	DoRasterString(labelX, height + 0.05f, 0.0f, label);


	


	glPopMatrix();
}



void RenderIndicators(Orbit* orbit, int type) {
	std::vector<Vector3<float>> OrbitPoints = orbit->getVertices();
	int apIndex = orbit->getApIndex();
	int peIndex = orbit->getPeIndex();

	if (type == INACTIVE) return;
	Color color = CYAN;

	switch (type) {
		case ACTIVE:
			color = DARK_CYAN;
			break;
		case TARGET:
			color = DARK_MAGENTA;
			break;
	}

	glDisable(GL_LIGHTING);

	if (apIndex != -1) {
		const Vector3<float>& apPoint = OrbitPoints[apIndex];
		PlaceIndicatorWidget(apPoint, color, "AP");
	}

	if (peIndex != -1) {
		const Vector3<float>& pePoint = OrbitPoints[peIndex];
		PlaceIndicatorWidget(pePoint, color, "PE");
	}

	glEnable(GL_LIGHTING);
}



void DrawVessel(Spacecraft* vessel, bool active) {
	glPushMatrix();
	glDisable(GL_LIGHTING);
	glTranslatef(vessel->getPosition().x, vessel->getPosition().y, vessel->getPosition().z);

	float camDistance = (cameraPos - vessel->getPosition()).magnitude();

	glScalef(camDistance, camDistance, camDistance);
	glCallList(VesselDL);
	glEnable(GL_LIGHTING);
	glPopMatrix();
}

void RenderGameModeTitle() {
	float x = (160 - currModeLabel.length()) / 2;
	DoRasterString(x, 5.0f, 0.0f, currModeLabel);
}

void RenderTimeWarpWidget() {
	std::string warpInfo = GenerateWarpInfo();
	DoRasterString(5.0f, 85.0f, 0.0f, warpInfo);

	std::string elapsedTimeInfo = GenerateTimeInfo();
	DoRasterString(5.0f, 80.0f, 0.0f, elapsedTimeInfo);
}


std::string GenerateWarpInfo() {
	int numArrows = (WarpLevel > MAX_WARP_LEVEL) ? MAX_WARP_LEVEL : WarpLevel + 1;
	std::string warpInfo = "Time Warp |" + std::string(numArrows, '>') + std::string(MAX_WARP_LEVEL - numArrows, ' ') + "|";
	return warpInfo;
}

std::string GenerateTimeInfo() {
	unsigned long elapsedSeconds = MS_TO_S(scaledElapsedTime);

	int days = elapsedSeconds / (24 * 3600);
	elapsedSeconds = fmod(elapsedSeconds, 24 * 3600);
	int hours = elapsedSeconds / 3600;
	elapsedSeconds = fmod(elapsedSeconds, 3600);
	int minutes = elapsedSeconds / 60;
	int seconds = fmod(elapsedSeconds, 60);

	char timeInfo[50];
	sprintf(timeInfo, "T+%dd, %02d:%02d:%02d", days, hours, minutes, seconds);

	return std::string(timeInfo);
}


void RenderVelocityWidget() {
	float velocity = U_TO_M(vessels[activeVessel]->getVelocity().magnitude());
	std::string velocityInfo = "Velocity: " + ProcessFloatStr(velocity) + " m/s";
	DoRasterString(5.0f, 15.0f, 0.0f, velocityInfo);
}

void RenderAltitudeWidget() {
	float altitude = U_TO_KM(vessels[activeVessel]->getPosition().magnitude() - EARTH_RADIUS);
	std::string altitudeInfo = ProcessFloatStr(altitude) + " km";
	int length = altitudeInfo.length();
	DoRasterString(76, 85.0f, 0.0f, "Altitude");
	DoRasterString((160 - length) / 2, 80.0f, 0.0f, altitudeInfo);
}

void RenderOrbitalInfoWidget(Orbit orbit) {
	OrbitalElements OrbitInfo = orbit.getParameters();

	float ap = U_TO_KM(OrbitInfo.ap - EARTH_RADIUS);
	float pe = U_TO_KM(OrbitInfo.pe - EARTH_RADIUS);
	float inc = RAD_TO_DEG(OrbitInfo.inc);

	std::string label = orbit.isVesselOrbit() ? vessels[activeVessel]->getName() : "Target";
	float startOffset = orbit.isVesselOrbit() ? 85.0f : 40.0f;

	DoRasterString(135.0f, startOffset, 0.0f, label);

	DoRasterString(135.0, startOffset - 5, 0.0f, "Ap: " + ProcessFloatStr(ap) + " km");
	DoRasterString(135.0, startOffset - 10, 0.0f, "Pe: " + ProcessFloatStr(pe) + " km");
	DoRasterString(135.0, startOffset - 15, 0.0f, "Inc: " + ProcessFloatStr(inc) + " deg");
}

void RenderExtendedOrbitalWidget(Orbit orbit) {
	OrbitalElements OrbitInfo = orbit.getParameters();

	float sma = U_TO_KM(OrbitInfo.sma);
	float ecc = OrbitInfo.ecc;
	float lan = RAD_TO_DEG(OrbitInfo.lan);
	float argpe = RAD_TO_DEG(OrbitInfo.argpe);

	RenderOrbitalInfoWidget(orbit);

	float startOffset = orbit.isVesselOrbit() ? 65.0f : 20.0f;

	DoRasterString(135.0, startOffset, 0.0f, "SMA: " + ProcessFloatStr(sma) + " km");
	DoRasterString(135.0, startOffset - 5, 0.0f, "ecc: " + ProcessFloatStr(ecc));
	DoRasterString(135.0, startOffset - 10, 0.0f, "lan: " + ProcessFloatStr(lan) + " deg");
	DoRasterString(135.0, startOffset - 15, 0.0f, "argpe: " + ProcessFloatStr(argpe) + " deg");
}


void RenderThrottleWidget() {
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(2) << ThrottleScales[ThrottleLevel];
	std::string throttleInfo = "Throttle: " + oss.str() + " m/s";
	DoRasterString(5.0f, 10.0f, 0.0f, throttleInfo);
}



void Alert(const std::string& msg) {
	AlertMsg = msg;
	AlertOn = true;
	AlertStartTime = MS_TO_S(trueElapsedTime);
}

void RenderAlert() {
	if (!AlertOn) return;

	int elapsedAlertTime = MS_TO_S(trueElapsedTime) - AlertStartTime;

	if (elapsedAlertTime > ALERT_DURATION) {
		AlertOn = false;
		return;
	}

	int messageLength = AlertMsg.length();
	int x = (160 - messageLength) / 2;

	glColor3f(0.9f, 0.9f, 0.3f);

	DoRasterString(x, 70, 0.0f, AlertMsg);
}


// PLACEHOLDER NAMES UNTIL I FIGURE OUT A BETTER WAY

std::vector<std::string> names = {
	"Cool ship",
	"Awesome ship",
	"Amazing ship",
	"Fantastic ship",
	"Super ship",
	"Great ship",
	"Breathtaking ship",
	"Stunning ship",
	"Astonishing ship",
	"Unbelievable ship",
	"Mind-blowing ship",
	"Jaw-dropping ship",
	"Spectacular ship",
	"Remarkable ship",
	"Extraordinary ship",
	"Incredible ship",
	"Phenomenal ship",
	"Outstanding ship"
};

std::vector<int> usages(18, 0);

void spawnNew(bool announce) {
	int randAltOffset = Ranf(-500, 2000);
	int randVelOffset = Ranf(-100, 500);

	int nameIndex = rand() % names.size();
	int usage = usages[nameIndex] + 1;
	usages[nameIndex]++;

	std::string name = names[nameIndex] + " #" + std::to_string(usage);

	vessels.push_back(new Spacecraft(name, INIT_ALT + randAltOffset, INIT_VEL + randVelOffset));

	if (announce)
		Alert("Spawned new vessel: " + name);
}



void InitSandbox(bool cont) {
	currGameMode = SANDBOX;
	currModeLabel = GameModeLabels[SANDBOX];
	if (targetOrbit != NULL) {
		delete targetOrbit;
		targetOrbit = NULL;

	}

	if (cont) {
		InitMenus();
	} else {
		spawnNew(false);
		Alert("Sandbox mode: create spacecraft and fly around!");
	}

}

void InitChallenge() {
	currGameMode = CHALLENGE;
	currModeLabel = GameModeLabels[CHALLENGE];

	Spacecraft* ship = new Spacecraft("Player", 500, 7615);
	vessels.push_back(ship);

	if (targetOrbit != NULL) delete targetOrbit;

	float init1 = Ranf(500.0f, 35000.0f);
	float init2 = Ranf(500.0f, 35000.0f);
	
	float ap = max(init1, init2);
	float pe = min(init1, init2);

	float inc = Ranf(0.0f, 75.0f);
	float lan = Ranf(0.0f, 360.0f);
	float argpe = Ranf(0.0f, 360.0f);

	targetOrbit = new Orbit(
		KM_TO_U(ap) + 1.0f,
		KM_TO_U(pe) + 1.0f,
		DEG_TO_RAD(inc),
		DEG_TO_RAD(lan),
		DEG_TO_RAD(argpe)
	);
	
	Alert("Challenge mode: match the designated orbit to win!");

}

void CheckWin() {


	if (trueElapsedTime - lastThrust < 1000) return;

	OrbitalElements vesselParams = vessels[activeVessel]->getOrbit()->getParameters();
	OrbitalElements targetParams = targetOrbit->getParameters();

	float smaDiff = U_TO_KM(fabs(vesselParams.sma - targetParams.sma));
	float incDiff = RAD_TO_DEG(fabs(vesselParams.inc - targetParams.inc));
	float lanDiff = RAD_TO_DEG(fabs(vesselParams.lan - targetParams.lan));
	float argpeDiff = RAD_TO_DEG(fabs(vesselParams.argpe - targetParams.argpe));

	if (
		smaDiff < SMA_WIN_THRESHOLD &&
		incDiff < INC_WIN_THRESHOLD &&
		lanDiff < LAN_WIN_THRESHOLD &&
		argpeDiff < ARGPE_WIN_THRESHOLD
		) {
		Alert("Orbit matched! You have completed the challenge");
		InitSandbox(true);
		
	}

}



// initialize the glut and OpenGL libraries:

void
InitGraphics()
{
	if (DebugOn != 0)
		fprintf(stderr, "Starting InitGraphics.\n");

	// request the display modes

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);

	// set the initial window configuration:

	glutInitWindowPosition(0, 0);
	glutInitWindowSize(INIT_WINDOW_H, INIT_WINDOW_W);

	// open the window and set its title:

	MainWindow = glutCreateWindow(WINDOWTITLE);
	glutSetWindowTitle(WINDOWTITLE);

	// set the framebuffer clear values:

	glClearColor(BACKCOLOR[0], BACKCOLOR[1], BACKCOLOR[2], BACKCOLOR[3]);

	glutSetWindow(MainWindow);
	glutDisplayFunc(Display);
	glutReshapeFunc(Resize);
	glutKeyboardFunc(Keyboard);
	glutMouseFunc(MouseButton);
	glutMotionFunc(MouseMotion);
	glutPassiveMotionFunc(MouseMotion);
	//glutPassiveMotionFunc( NULL );
	glutVisibilityFunc(Visibility);
	glutEntryFunc(NULL);
	glutSpecialFunc(NULL);
	glutSpaceballMotionFunc(NULL);
	glutSpaceballRotateFunc(NULL);
	glutSpaceballButtonFunc(NULL);
	glutButtonBoxFunc(NULL);
	glutDialsFunc(NULL);
	glutTabletMotionFunc(NULL);
	glutTabletButtonFunc(NULL);
	glutMenuStateFunc(NULL);
	glutTimerFunc(-1, NULL, 0);

	glutIdleFunc(Animate);

	

#ifdef WIN32
	GLenum err = glewInit();
	if (err != GLEW_OK)
	{
		fprintf(stderr, "glewInit Error\n");
	}
	else
		fprintf(stderr, "GLEW initialized OK\n");
	fprintf(stderr, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
#endif


	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// Set light parameters
	GLfloat light_position[] = { SUN_DISTANCE, 0.0f, 0.0f, 1.0f };
	GLfloat light_ambient[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat light_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);


	EarthTex = LoadBmpTexture(EARTH_TEX_NAME);
	StarfieldTex = LoadBmpTexture(STARFIELD_TEX_NAME);
}


GLuint LoadBmpTexture(const char* filename) {
	int width, height;
	unsigned char* texture = BmpToTexture((char*)filename, &width, &height);
	if (texture == NULL) {
		fprintf(stderr, "Cannot open texture '%s'\n", filename);
		return 0;
	}
	else
		fprintf(stderr, "Opened '%s': width = %d ; height = %d\n", filename, width, height);


	GLuint tex;

	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, texture);

	return tex;
}

// initialize the display lists

void
InitLists()
{
	if (DebugOn != 0)
		fprintf(stderr, "Starting InitLists.\n");


	SphereDL = glGenLists(1);
	glNewList(SphereDL, GL_COMPILE);
		OsuSphere(1., 200, 200);
	glEndList();

	EarthDL = glGenLists(1);
	glNewList(EarthDL, GL_COMPILE);
		glPushMatrix();
		glScalef(EARTH_RADIUS, EARTH_RADIUS, EARTH_RADIUS);
		glBindTexture(GL_TEXTURE_2D, EarthTex);
		glCallList(SphereDL);
		glPopMatrix();
	glEndList();

	SunDL = glGenLists(1);
	glNewList(SunDL, GL_COMPILE);
		glPushMatrix();
		glScalef(SUN_RADIUS, SUN_RADIUS, SUN_RADIUS);
		glColor3f(1.0f, 1.0f, 0.0f);
		glCallList(SphereDL);
		glPopMatrix();
	glEndList();

	StarfieldDL = glGenLists(1);
	glNewList(StarfieldDL, GL_COMPILE);
		glPushMatrix();
		glScalef(STAR_FIELD_RADIUS, STAR_FIELD_RADIUS, STAR_FIELD_RADIUS);
		glBindTexture(GL_TEXTURE_2D, StarfieldTex);
		glCallList(SphereDL);
		glPopMatrix();
	glEndList();

	// placeholder sphere for vessel
	VesselDL = glGenLists(1);
	glNewList(VesselDL, GL_COMPILE);
		glPushMatrix();
		glScalef(0.01f, 0.01f, 0.01f);
		glColor3f(0.7f, 0.7f, 0.7f);
		glCallList(SphereDL);
		glPopMatrix();
	glEndList();

	AxesList = glGenLists(1);
	glNewList(AxesList, GL_COMPILE);
		glLineWidth(AXES_WIDTH);
		Axes(1.5);
		glLineWidth(1.);
	glEndList();
}



void
DoAxesMenu( int id )
{
	AxesOn = id;

	glutSetWindow( MainWindow );
	glutPostRedisplay( );
}



void
DoDebugMenu( int id )
{
	DebugOn = id;

	glutSetWindow( MainWindow );
	glutPostRedisplay( );
}


void DoGameModeMenu ( int id ) {
	

	Reset(id);

	glutSetWindow(MainWindow);
	glutPostRedisplay();

}


void
DoOrbitDetailMenu( int id )
{
	DoOrbitDetail = id;
	glutSetWindow( MainWindow );
	glutPostRedisplay( );
}


// main menu:

void
DoMainMenu( int id )
{
	switch( id )
	{
		case SPAWN:
			spawnNew();
			break;
		case RESET:
			Reset(currGameMode);
			break;

		case QUIT:
			glutSetWindow( MainWindow );
			glFinish( );
			glutDestroyWindow( MainWindow );
			exit( 0 );
			break;

		default:
			fprintf( stderr, "Don't know what to do with Main Menu ID %d\n", id );
	}

	glutSetWindow( MainWindow );
	glutPostRedisplay( );
}

// initialize the glui window:

void
InitMenus( )
{
	if (DebugOn != 0)
		fprintf(stderr, "Starting InitMenus.\n");

	glutSetWindow( MainWindow );

	int gamemodeenu = glutCreateMenu( DoGameModeMenu );
	for (int i = 0; i < NUM_MODES; i++) {
		if (i == currGameMode) continue;
		glutAddMenuEntry(GameModeLabels[i].c_str(), i);
	}

	int axesmenu = glutCreateMenu( DoAxesMenu );
	glutAddMenuEntry( "Off",  0 );
	glutAddMenuEntry( "On",   1 );

	int orbitDetailMenu = glutCreateMenu( DoOrbitDetailMenu );
	glutAddMenuEntry( "Basic", 0 );
	glutAddMenuEntry( "Advanced", 1);

	int debugmenu = glutCreateMenu( DoDebugMenu );
	glutAddMenuEntry( "Off",  0 );
	glutAddMenuEntry( "On",   1 );

	int mainmenu = glutCreateMenu( DoMainMenu );

	glutAddSubMenu(	  "Game Mode",	   gamemodeenu);
	if (currGameMode == SANDBOX)
		glutAddMenuEntry( "Spawn new craft",	   SPAWN);
	glutAddSubMenu(   "Orbital Info",  orbitDetailMenu);
	glutAddSubMenu(	  "Axes",		   axesmenu);
	glutAddMenuEntry( "Reset",         RESET );
	glutAddSubMenu(   "Debug",         debugmenu);
	glutAddMenuEntry( "Quit",          QUIT );

// attach the pop-up menu to the right mouse button:

	glutAttachMenu( GLUT_RIGHT_BUTTON );
}

void warpControl(int up) {
	if (up == -1) {
		WarpLevel = 0;
		Alert("Time warp stopped");
		return;
	}

	if (	(WarpLevel == MAX_WARP_LEVEL && up) 
		||	(WarpLevel == MIN_WARP_LEVEL && !up)
		)
		return;
	
	float kmTrueAlt = U_TO_KM(vessels[activeVessel]->getPosition().magnitude() - EARTH_RADIUS);

	if (up && WarpAltLimits[WarpLevel + 1] > kmTrueAlt) {
		Alert("Cannot warp more than x" + ProcessIntStr(WarpScales[WarpLevel]) + 
			" while under " + ProcessIntStr(WarpAltLimits[WarpLevel + 1]) + " km");
		return;
	}
	
	
	WarpLevel += up ? 1 : -1;

	Alert("x" + ProcessIntStr(WarpScales[WarpLevel]));
}

void impulseControl(ImpulseDirection direction) {
	if (WarpLevel > 0) {
		Alert("Cannot thrust while time warp is active");
		return;
	}
	lastThrust = trueElapsedTime;
	vessels[activeVessel]->applyImpulse(direction, ThrottleScales[ThrottleLevel]);
}

// the keyboard callback:

void
Keyboard( unsigned char c, int x, int y )
{
	if( DebugOn != 0 )
		fprintf( stderr, "Keyboard: '%c' (0x%0x)\n", c, c );

	switch( c )
	{
		case '.':
		case '>':
			warpControl(1);
			break;
		case ',':
		case '<':
			warpControl(0);
			break;

		case '/':
		case '?':
			warpControl(-1);
			break;

		case 'z':
		case 'Z':
			if (ThrottleLevel < MAX_THROTTLE_LEVEL) {
				ThrottleLevel++;
			}
			break;

		case 'x':
		case 'X':
			if (ThrottleLevel > MIN_THROTTLE_LEVEL) {
				ThrottleLevel--;
			}
			break;

		case 'w':
		case 'W':
			impulseControl(PROGRADE);
			break;

		case 's':
		case 'S':
			impulseControl(RETROGRADE);
			break;

		case 'a':
		case 'A':
			impulseControl(RADIALIN);
			break;

		case 'd':
		case 'D':
			impulseControl(RADIALOUT);
			break;
		case 'q':
		case 'Q':
			impulseControl(NORMAL);
			break;

		case 'e':
		case 'E':
			impulseControl(ANTINORMAL);
			break;

		case '`':
		case '~':

			if (lookAtVessel) break;

			lookAtVessel = true;
			cameraRadius = 1.0f;

			Alert("Focus: " + vessels[activeVessel]->getName());

			break;
		case '\t':

			if (!lookAtVessel) break;

			lookAtVessel = false;
			cameraRadius = 2.0f;

			Alert("Focus: Earth");

			break;

		case '[':
		case '{':
			if (vessels.size() == 1) {
				Alert("No vessels to switch to");
				break;
			}
			
			if (activeVessel > 0) 
				activeVessel--;
			else 
				activeVessel = vessels.size() - 1;

			lookAtVessel = true;
			WarpLevel = 0;

			Alert("Switched to " + vessels[activeVessel]->getName());

			break;

		case ']':
		case '}':
			if (vessels.size() == 1) {
				Alert("No vessels to switch to");
				break;
			}

			if (activeVessel < vessels.size() - 1) 
				activeVessel++;
			else 
				activeVessel = 0;
			
			lookAtVessel = true;
			WarpLevel = 0;

			Alert("Switched to " + vessels[activeVessel]->getName());

			break;

		default:
			fprintf( stderr, "Don't know what to do with keyboard hit: '%c' (0x%0x)\n", c, c );
	}

	// force a call to Display( ):

	glutSetWindow( MainWindow );
	glutPostRedisplay( );
}


// called when the mouse button transitions down or up:

void
MouseButton( int button, int state, int x, int y )
{
	int b = 0;			// LEFT, MIDDLE, or RIGHT

	if( DebugOn != 0 )
		fprintf( stderr, "MouseButton: %d, %d, %d, %d\n", button, state, x, y );

	
	
	float newRadius;

	switch( button )
	{
		case GLUT_LEFT_BUTTON:
			b = LEFT;		break;

		case GLUT_MIDDLE_BUTTON:
			b = MIDDLE;		break;

		case GLUT_RIGHT_BUTTON:
			b = RIGHT;		break;

		case SCROLL_WHEEL_UP:

			 newRadius = cameraRadius - SCROLL_SENSITIVITY * (cameraRadius / 2);

			 if (newRadius < (lookAtVessel ? MIN_CAM_RADIUS_V : MIN_CAM_RADIUS_P)) break;
			cameraRadius = newRadius;

			break;

		case SCROLL_WHEEL_DOWN:

			newRadius = cameraRadius + SCROLL_SENSITIVITY * (cameraRadius / 2);

			if (newRadius > MAX_CAM_RADIUS) break;
			cameraRadius = newRadius;

			break;

		default:
			b = 0;
			fprintf( stderr, "Unknown mouse button: %d\n", button );
	}

	

	if( state == GLUT_DOWN )
	{
		Xmouse = x;
		Ymouse = y;
		ActiveButton |= b;		
	}
	else
	{
		ActiveButton &= ~b;		
	}

	glutSetWindow(MainWindow);
	glutPostRedisplay();

}

// called when the mouse moves while a button is down:

void
MouseMotion( int x, int y )
{
	int dx = x - Xmouse;		
	int dy = y - Ymouse;

	if( ( ActiveButton & LEFT ) != 0 )
	{

		cameraTheta += MOUSE_SENSITIVITY * dx;
		cameraPhi -= MOUSE_SENSITIVITY * dy;

		cameraPhi = limitExtremes(cameraPhi, 0.1f, M_PI - 0.1f);

	}

	if( ( ActiveButton & MIDDLE ) != 0 )
	{
		float newRadius = cameraRadius - (MOUSE_SENSITIVITY * (cameraRadius/2)) * (dx - dy);

		if (newRadius > (lookAtVessel ? MIN_CAM_RADIUS_V : MIN_CAM_RADIUS_P)
			&& newRadius < MAX_CAM_RADIUS) {
			cameraRadius = newRadius;
		}

	}


	Xmouse = x;			
	Ymouse = y;

	glutSetWindow( MainWindow );
	glutPostRedisplay( );
}

// reset the simulation

void
Reset(int gamemode)
{
	ActiveButton = 0;
	AxesOn = 0;
	DebugOn = 0;
	DoOrbitDetail = 0;


	lookAtVessel = true;

	cameraRadius = 1.0f;
	cameraTheta = 0.0f;
	cameraPhi = (M_PI / 2) - 0.5;

	scaledElapsedTime = 0;
	WarpLevel = 0;
	ThrottleLevel = 3;

	ended = false;
	AlertOn = false;
	AlertMsg = "";
	AlertStartTime = 0;


	for (Spacecraft* vessel : vessels) {
		delete vessel;
	}

	vessels.clear();
	activeVessel = 0;


	
	switch (gamemode) {
		case SANDBOX:
			InitSandbox();
			break;
		case CHALLENGE:
			InitChallenge();
			break;
	}
	

	InitMenus();

}


// display a string using a raster font:

void DoRasterString(float x, float y, float z, const std::string& s) {
	glRasterPos3f((GLfloat)x, (GLfloat)y, (GLfloat)z);

	for (char c : s) {
		glutBitmapCharacter(GLUT_BITMAP_8_BY_13, c);
	}
}


// display a string using a stroke font:

void DoStrokeString(float x, float y, float z, float ht, const std::string& s) {
	glPushMatrix();
	glTranslatef((GLfloat)x, (GLfloat)y, (GLfloat)z);
	float sf = ht / (119.05f + 33.33f);
	glScalef((GLfloat)sf, (GLfloat)sf, (GLfloat)sf);
	for (char c : s)
	{
		glutStrokeCharacter(GLUT_STROKE_ROMAN, c);
	}
	glPopMatrix();
}

// called when user resizes the window:

void
Resize( int width, int height )
{

	glutSetWindow( MainWindow );
	glutPostRedisplay( );
}


// handle a change to the window's visibility:

void
Visibility ( int state )
{
	if( DebugOn != 0 )
		fprintf( stderr, "Visibility: %d\n", state );

	if( state == GLUT_VISIBLE )
	{
		glutSetWindow( MainWindow );
		glutPostRedisplay( );
	}
}



/////////////////////////   UTILITIES:  //////////////////////////


// the stroke characters 'X' 'Y' 'Z' :

static float xx[ ] = { 0.f, 1.f, 0.f, 1.f };

static float xy[ ] = { -.5f, .5f, .5f, -.5f };

static int xorder[ ] = { 1, 2, -3, 4 };

static float yx[ ] = { 0.f, 0.f, -.5f, .5f };

static float yy[ ] = { 0.f, .6f, 1.f, 1.f };

static int yorder[ ] = { 1, 2, 3, -2, 4 };

static float zx[ ] = { 1.f, 0.f, 1.f, 0.f, .25f, .75f };

static float zy[ ] = { .5f, .5f, -.5f, -.5f, 0.f, 0.f };

static int zorder[ ] = { 1, 2, 3, 4, -5, 6 };

// fraction of the length to use as height of the characters:
const float LENFRAC = 0.10f;

// fraction of length to use as start location of the characters:
const float BASEFRAC = 1.10f;

float limitExtremes(float value, float minValue, float maxValue) {
	if (value < minValue) return minValue;
	if (value > maxValue) return maxValue;
	return value;
}

//	Draw a set of 3D axes:
//	(length is the axis length in world coordinates)

void
Axes( float length )
{
	glBegin( GL_LINE_STRIP );
		glVertex3f( length, 0., 0. );
		glVertex3f( 0., 0., 0. );
		glVertex3f( 0., length, 0. );
	glEnd( );
	glBegin( GL_LINE_STRIP );
		glVertex3f( 0., 0., 0. );
		glVertex3f( 0., 0., length );
	glEnd( );

	float fact = LENFRAC * length;
	float base = BASEFRAC * length;

	glBegin( GL_LINE_STRIP );
		for( int i = 0; i < 4; i++ )
		{
			int j = xorder[i];
			if( j < 0 )
			{
				
				glEnd( );
				glBegin( GL_LINE_STRIP );
				j = -j;
			}
			j--;
			glVertex3f( base + fact*xx[j], fact*xy[j], 0.0 );
		}
	glEnd( );

	glBegin( GL_LINE_STRIP );
		for( int i = 0; i < 5; i++ )
		{
			int j = yorder[i];
			if( j < 0 )
			{
				
				glEnd( );
				glBegin( GL_LINE_STRIP );
				j = -j;
			}
			j--;
			glVertex3f( fact*yx[j], base + fact*yy[j], 0.0 );
		}
	glEnd( );

	glBegin( GL_LINE_STRIP );
		for( int i = 0; i < 6; i++ )
		{
			int j = zorder[i];
			if( j < 0 )
			{
				
				glEnd( );
				glBegin( GL_LINE_STRIP );
				j = -j;
			}
			j--;
			glVertex3f( 0.0, fact*zy[j], base + fact*zx[j] );
		}
	glEnd( );

}

// utility to create an array from a multiplier and an array:

float*
MulArray3(float factor, float array0[])
{
	static float array[4];

	array[0] = factor * array0[0];
	array[1] = factor * array0[1];
	array[2] = factor * array0[2];
	array[3] = 1.;
	return array;
}


float*
MulArray3(float factor, float a, float b, float c)
{
	static float array[4];

	float* abc = Array3(a, b, c);
	array[0] = factor * abc[0];
	array[1] = factor * abc[1];
	array[2] = factor * abc[2];
	array[3] = 1.;
	return array;
}

float*
Array3(float a, float b, float c)
{
	static float array[4];

	array[0] = a;
	array[1] = b;
	array[2] = c;
	array[3] = 1.;
	return array;
}


float
Ranf(float low, float high)
{
	float r = (float)rand();               // 0 - RAND_MAX
	float t = r / (float)RAND_MAX;       // 0. - 1.

	return   low + t * (high - low);
}

// call this if you want to force your program to use
// a different random number sequence every time you run it:
void
TimeOfDaySeed()
{
	struct tm y2k;
	y2k.tm_hour = 0;    y2k.tm_min = 0; y2k.tm_sec = 0;
	y2k.tm_year = 2000; y2k.tm_mon = 0; y2k.tm_mday = 1;

	time_t  now;
	time(&now);
	double seconds = difftime(now, mktime(&y2k));
	unsigned int seed = (unsigned int)(1000. * seconds);    // milliseconds
	srand(seed);
}


// function to convert HSV to RGB


void
HsvRgb( float hsv[3], float rgb[3] )
{
	// guarantee valid input:

	float h = hsv[0] / 60.f;
	while( h >= 6. )	h -= 6.;
	while( h <  0. ) 	h += 6.;

	float s = hsv[1];
	if( s < 0. )
		s = 0.;
	if( s > 1. )
		s = 1.;

	float v = hsv[2];
	if( v < 0. )
		v = 0.;
	if( v > 1. )
		v = 1.;

	// if sat==0, then is a gray:

	if( s == 0.0 )
	{
		rgb[0] = rgb[1] = rgb[2] = v;
		return;
	}

	// get an rgb from the hue itself:
	
	float i = (float)floor( h );
	float f = h - i;
	float p = v * ( 1.f - s );
	float q = v * ( 1.f - s*f );
	float t = v * ( 1.f - ( s * (1.f-f) ) );

	float r=0., g=0., b=0.;			// red, green, blue
	switch( (int) i )
	{
		case 0:
			r = v;	g = t;	b = p;
			break;
	
		case 1:
			r = q;	g = v;	b = p;
			break;
	
		case 2:
			r = p;	g = v;	b = t;
			break;
	
		case 3:
			r = p;	g = q;	b = v;
			break;
	
		case 4:
			r = t;	g = p;	b = v;
			break;
	
		case 5:
			r = v;	g = p;	b = q;
			break;
	}


	rgb[0] = r;
	rgb[1] = g;
	rgb[2] = b;
}

void
Cross(float v1[3], float v2[3], float vout[3])
{
	float tmp[3];
	tmp[0] = v1[1] * v2[2] - v2[1] * v1[2];
	tmp[1] = v2[0] * v1[2] - v1[0] * v2[2];
	tmp[2] = v1[0] * v2[1] - v2[0] * v1[1];
	vout[0] = tmp[0];
	vout[1] = tmp[1];
	vout[2] = tmp[2];
}

float
Dot(float v1[3], float v2[3])
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}


float
Unit(float vin[3], float vout[3])
{
	float dist = vin[0] * vin[0] + vin[1] * vin[1] + vin[2] * vin[2];
	if (dist > 0.0)
	{
		dist = sqrtf(dist);
		vout[0] = vin[0] / dist;
		vout[1] = vin[1] / dist;
		vout[2] = vin[2] / dist;
	}
	else
	{
		vout[0] = vin[0];
		vout[1] = vin[1];
		vout[2] = vin[2];
	}
	return dist;
}


float
Unit( float v[3] )
{
	float dist = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	if (dist > 0.0)
	{
		dist = sqrtf(dist);
		v[0] /= dist;
		v[1] /= dist;
		v[2] /= dist;
	}
	return dist;
}

std::string ProcessIntStr(int n) {
	std::string number = std::to_string(n);
	std::string result = "";
	int count = 0;
	int start = 0;

	if (number[0] == '-') {
		result += '-';
		start = 1;
	}

	for (int i = number.length() - 1; i >= start; i--) {
		result = number[i] + result;
		count++;
		if (count % 3 == 0 && i != start) {
			result = "," + result;
		}
	}
	return result;
}

std::string ProcessFloatStr(float n) {
	std::stringstream stream;
	stream << std::fixed << std::setprecision(2) << n;
	std::string number = stream.str();

	std::string result = "";
	int count = 0;
	size_t decimalPos = number.find('.');
	int start = 0;

	if (number[0] == '-') start = 1;
	

	int intPartEnd = (decimalPos == std::string::npos) ? number.length() - 1 : decimalPos - 1;
	for (int i = intPartEnd; i >= start; i--) {
		result = number[i] + result;
		count++;
		if (count % 3 == 0 && i != start) {
			result = "," + result;
		}
	}

	if (start == 1) result = "-" + result;

	if (decimalPos != std::string::npos) result += number.substr(decimalPos);
	

	return result;
}


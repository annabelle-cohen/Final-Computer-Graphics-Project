#define _CRT_SECURE_NO_WARNINGS
#include "glut.h"
#include <math.h>
#include <time.h>
#include <stdio.h>
#include "Train.h"

const int W = 600;
const int H = 600;
const int TW = 512;
const int TH = 512;
const int NUM_CARRIAGES = 10;

double ground[GSZ][GSZ] = { 0 }; // this is the matrix of heights

// texture definitions
unsigned char tx0[TH][TW][3]; // 3 stands for rgb
unsigned char tx1[512][1024][3]; // fence
unsigned char tx2[512][512][3]; // wall with window
unsigned char tx3[1024][2048][3]; // skynight 


unsigned char* bmp;

// locomotion (ego-motion)
double eyex = 0, eyey = 14, eyez = 26;

double yaw = PI, pitch = -0.1, roll;
double speed = 0, angularSpeed = 0;
double dir[3] = { sin(yaw),sin(pitch),cos(yaw) };
// aircraft motion
double airyaw = PI, airpitch = 0, airroll;
double airspeed = 0, airangularSpeed = 0;
double airdir[3] = { sin(airyaw),sin(airpitch),cos(airyaw) };
double airLocation[3] = { 0,18,0 };
double left_control = -0.8, right_control;

double offset = 0;

Train* pTrain[NUM_CARRIAGES] = { 0 };

bool buildTerrain = false;


void UpdateTerrain2();
void UpdateTerrain3();
void Smooth();

// first light source
float lamb0[4] = { 0.2,0.2,0.2,0 };
float ldiff0[4] = { 0.8,0.5,0.2,0 };
float lspec0[4] = { 0.5,0.5,0.5,0 };
float lpos0[4] = { -1,0.2,0,0 };// the last element defines kind of light source:
// if it is 1 the the light is positional; if it is 0 the the light is directional

// second light source
float lamb1[4] = { 0.2,0.2,0.2,0 };
float ldiff1[4] = { 0.8,0.8,0.8,0 };
float lspec1[4] = { 0.7,0.7,0.7,0 };
float lpos1[4] = { 1,1,1,0 }; // directional

// material: red plastic
float mamb0[4] = { 0.4,0.2,0.2,0 };
float mdiff0[4] = { 0.9,0.0,0.0,0 };
float mspec0[4] = { 0.9,0.7,0.7,0 };

// material: green plastic
float mamb1[4] = { 0.2,0.4,0.2,0 };
float mdiff1[4] = { 0.0,0.4,0.0,0 };
float mspec1[4] = { 0.7,0.9,0.7,0 };

// material: gold
float mamb2[4] = { 0.24,0.2,0.07,0 };
float mdiff2[4] = { 0.75,0.6,0.22,0 };
float mspec2[4] = { 0.62,0.55,0.37,0 };

// material: silver
float mamb3[4] = { 0.2,0.2,0.2,0 };
float mdiff3[4] = { 0.7,0.7,0.7,0 };
float mspec3[4] = { 0.7,0.7,0.7,0 };


void ReadBitmap(const char fname[])
{
	FILE* pf;
	// BITMAP file starts with 
	BITMAPFILEHEADER bf;
	// then goes
	BITMAPINFOHEADER bi;
	// and then goes stream: bgrbgrbgr.....
	int size; // how many BYTES will be read

	pf = fopen(fname, "rb");
	fread(&bf, sizeof(BITMAPFILEHEADER), 1, pf);
	fread(&bi, sizeof(BITMAPINFOHEADER), 1, pf);
	size = bi.biHeight * bi.biWidth * 3;

	bmp = (unsigned char*)malloc(size);
	fread(bmp, 1, size, pf);
	fclose(pf);
}




double Distance(int i1, int j1, int i2, int j2)
{
	return sqrt((double)(i1 - i2) * (i1 - i2) + (j1 - j2) * (j1 - j2));
}


void SetTexture(int kind)
{
	int i, j, delta, k;
	switch (kind)
	{
	case 0: // brick wall
		for (i = 0; i < TH; i++)
			for (j = 0; j < TW; j++)
			{
				delta = -20 + rand() % 40; // random value in range [-20,20)
				if (i % (TH / 2) > (TH / 2 - 15) ||
					(i < TH / 2) && (j % (TW / 2) > (TW / 2 - 15)) ||
					(i >= TH / 2) && (j % (TW / 4) > (TW / 4 - 15)) && j < TW / 4 ||
					(i >= TH / 2) && (j % (TW / 4) > (TW / 4 - 15)) && j>3 * TW / 4 - 15 && j < 3 * TW / 4)// cement
				{
					tx0[i][j][0] = 190 + delta;//red
					tx0[i][j][1] = 190 + delta;//green
					tx0[i][j][2] = 190 + delta;//blue
				}
				else				//brick
				{
					tx0[i][j][0] = 190 + delta;//red
					tx0[i][j][1] = 50 + delta;//green
					tx0[i][j][2] = 30 + delta;//blue
				}
			}
		break;
	case 1: // rails
		for (i = 0; i < TH; i++)
			for (j = 0; j < TW; j++)
			{
				delta = -20 + rand() % 40; // random value in range [-20,20)
				if (i < 120 && i>50 || i<TH - 50 && i>TH - 120)// white sign
				{
					tx0[i][j][0] = 70 + delta;//red
					tx0[i][j][1] = 80 + delta;//green
					tx0[i][j][2] = 70 + delta;//blue
				}
				else if (j > 128 && j < 198 || j<TW - 128 && j>TW - 198)				//road
				{
					tx0[i][j][0] = 190 + delta;//red
					tx0[i][j][1] = 150 + delta;//green
					tx0[i][j][2] = 130 + delta;//blue
				}
				else {
					tx0[i][j][0] = 200 + delta;//red
					tx0[i][j][1] = 200 + delta;//green
					tx0[i][j][2] = 200 + delta;//blue
				}
			}
		break;
	case 2: // pedestrians walk
		for (i = 0; i < TH; i++)
			for (j = 0; j < TW; j++)
			{
				delta = -20 + rand() % 40; // random value in range [-20,20)
				tx0[i][j][0] = 200 + delta;//red
				tx0[i][j][1] = 200 + delta;//green
				tx0[i][j][2] = 200 + delta;//blue
			}
		break;

	case 3: // wooden fence
		for (i = 0, k = 0; i < 512; i++)
			for (j = 0; j < 1024; j++, k += 3)
			{
				tx1[i][j][0] = bmp[k + 2]; // red
				tx1[i][j][1] = bmp[k + 1]; // green
				tx1[i][j][2] = bmp[k]; // blue
			}
		break;
	case 4: // wall with window
	case 5: // and roof
	case 6: // and AFEKA
		for (i = 0, k = 0; i < 512; i++)
			for (j = 0; j < 512; j++, k += 3)
			{
				tx2[i][j][0] = bmp[k + 2]; // red
				tx2[i][j][1] = bmp[k + 1]; // green
				tx2[i][j][2] = bmp[k]; // blue
			}
		break;
	case 7: // sunrise
		for (i = 0, k = 0; i < 1024; i++)
			for (j = 0; j < 2048; j++, k += 3)
			{
				tx3[i][j][0] = bmp[k + 2]; // red
				tx3[i][j][1] = bmp[k + 1]; // green
				tx3[i][j][2] = bmp[k]; // blue
			}
		break;


	}
}

// align heights along z=0
void PrepareRoad()
{
	int j, i;

	for (j = 0; j < GSZ; j++)
		ground[GSZ / 2 + 1][j] = ground[GSZ / 2 - 1][j] = ground[GSZ / 2][j];

	// aligh along x = 0
	for (i = 0; i < GSZ; i++)
		ground[i][GSZ / 2 + 1] = ground[i][GSZ / 2 - 1] = ground[i][GSZ / 2];

}

void PrepareGround(int startx, int startz, int endx, int endz, int height)
{
	for (int i = startz; i <= endz; i++)
		for (int j = startx; j <= endx; j++)
			ground[i][j] = height;
}



void init()
{
	int i, j;
	srand(time(0)); // seed or init random numbers generator

	glClearColor(0.0, 0.6, 0.8, 0);// color of window background
	glEnable(GL_DEPTH_TEST); // allow to show the nearest object

	for (i = 1; i <= 1000; i++)
		UpdateTerrain2();


	Smooth();
	Smooth();

	PrepareRoad();
	PrepareGround(GSZ / 2 + 5, GSZ / 2 - 20, GSZ / 2 + 20, GSZ / 2 - 5, 1);

	glEnable(GL_NORMALIZE);// needs for lighting to normalize the vectors

	// Textures definitions
	SetTexture(0); // brick wall
	glBindTexture(GL_TEXTURE_2D, 0); // 0 stands for texture number
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, TW, TH, 0, GL_RGB, GL_UNSIGNED_BYTE, tx0);

	SetTexture(1); //rails
	glBindTexture(GL_TEXTURE_2D, 1); // 0 stands for texture number
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, TW, TH, 0, GL_RGB, GL_UNSIGNED_BYTE, tx0);

	SetTexture(2); // pedestrian walk
	glBindTexture(GL_TEXTURE_2D, 2); // 0 stands for texture number
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, TW, TH, 0, GL_RGB, GL_UNSIGNED_BYTE, tx0);

	// read wooden fence from file
	ReadBitmap("fence.bmp");
	SetTexture(3);
	free(bmp);
	glBindTexture(GL_TEXTURE_2D, 3);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1024, 512, 0, GL_RGB, GL_UNSIGNED_BYTE, tx1);


	// read wall with window from file
	ReadBitmap("window_on_wall.bmp");
	SetTexture(4);
	free(bmp);
	glBindTexture(GL_TEXTURE_2D, 4);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 512, 512, 0, GL_RGB, GL_UNSIGNED_BYTE, tx2);


	// read roof from file
	ReadBitmap("roof.bmp");
	SetTexture(5);
	free(bmp);
	glBindTexture(GL_TEXTURE_2D, 5);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 512, 512, 0, GL_RGB, GL_UNSIGNED_BYTE, tx2);

	// read Afeka from file
	ReadBitmap("afeka.bmp");
	SetTexture(6);
	free(bmp);
	glBindTexture(GL_TEXTURE_2D, 6);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 512, 512, 0, GL_RGB, GL_UNSIGNED_BYTE, tx2);


	// read skynight from file
	ReadBitmap("skynight.bmp");
	SetTexture(7);
	free(bmp);
	glBindTexture(GL_TEXTURE_2D, 7);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 2048, 1024, 0, GL_RGB, GL_UNSIGNED_BYTE, tx3);
	int col = 50;


	// define a car
	// we still have to compute diry
	for (i = 0; i < NUM_CARRIAGES; i++)
	{
		double r, g, b;


		r = (rand() % 100) / 100.0;
		g = (rand() % 100) / 100.0;
		b = (rand() % 100) / 100.0;
		// to set up the initial slope we initialize the cars with speed = 0
		if (ground[GSZ / 2][col] > 0) 	pTrain[i] = new Train(r, g, b, col - GSZ / 2, ground[GSZ / 2][col], 0, 1, 0, 0, 0);
		else	pTrain[i] = new Train(r, g, b, col - GSZ / 2, 1, 0, 1, 0, 0, 0); // on the bridge
		// updates the slope
		col++;
		pTrain[i]->Move(ground);
		pTrain[i]->SetSpeed(0.01);
	}




}

// comlete random
void UpdateTerrain1()
{
	int row, col;
	double delta = 0.4;

	if (rand() % 2 == 0)
		delta = -delta;
	row = rand() % GSZ;
	col = rand() % GSZ;

	ground[row][col] += delta;
}

void DrawColorCylinder(int n, double r, double g, double b)
{
	double alpha;
	double teta = 2 * PI / n;

	for (alpha = 0; alpha < 2 * PI; alpha += teta)
	{
		glBegin(GL_POLYGON);
		glColor3d(0.5 * r + 0.5 * fabs(sin(alpha)), 0.5 * g + 0.5 * fabs(sin(alpha)), 0.5 * b + 0.5 * fabs(sin(alpha)));
		glVertex3d(sin(alpha), 1, cos(alpha));
		glVertex3d(sin(alpha + teta), 1, cos(alpha + teta));
		glVertex3d(sin(alpha + teta), -1, cos(alpha + teta));
		glVertex3d(sin(alpha), -1, cos(alpha));
		glEnd();
	}
}


void DrawColorCylinder2(int n, double topr, double bottomr, double r, double g, double b)
{
	double alpha;
	double teta = 2*PI/n; 

	glColor3d(r,g,b);
	for(alpha=0;alpha<2*PI;alpha+=n*teta)
	{
		glBegin(GL_POLYGON);
			glVertex3d(topr*sin(alpha),1,topr*cos(alpha));
			glVertex3d(topr*sin(alpha+teta),1,topr*cos(alpha+teta));
			glVertex3d(bottomr*sin(alpha+teta),0,bottomr*cos(alpha+teta));
			glVertex3d(bottomr*sin(alpha),0,bottomr*cos(alpha));
		glEnd();
	}
}

void DrawOpenSphere(int n, int l, double open_angle)
{
	double beta;
	double delta = PI / l, teta = 2 * PI / n;
	int i;

	for (i = 1; i <= n; i++)
	{
		glPushMatrix();
		glRotated(i * teta * 180 / PI, 0, 1, 0);
		glTranslated(0, -0.5, 0);
		glRotated(open_angle, 1, 0, 0);
		glTranslated(0, 0.5, 0);
		glRotated(-0.5 * teta * 180 / PI, 0, 1, 0);
		for (beta = -PI / 2; beta < PI / 2; beta += delta)
		{
			glPushMatrix();
			glTranslated(0, sin(beta), 0);
			glScaled(1, (sin(beta + delta) - sin(beta)), 1);
			DrawColorCylinder2(n, cos(beta + delta), cos(beta), 1, cos((beta + PI / 2) / 2), 0);
			glPopMatrix();
		}
		glPopMatrix();
	}
}

void DrawFlower()
{
	double open_angle = 90 * (0.8 + left_control) / 2;

	glPushMatrix();
	glScaled(0.5, 0.5, 0.5);
	glRotated(45, 0, 1, 0);
	DrawColorCylinder(40, 0, 1, 0);
	glPopMatrix();
	glPushMatrix();
	glTranslated(0, 12, 0);
	glScaled(2, 3, 2);
	DrawOpenSphere(20 * (left_control + 1.1) / 2, 32, open_angle);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, 12, 0);
	glRotated(45, 0, 1, 0);
	glScaled(1.5, 2.5, 1.5);
	DrawOpenSphere(20 * (left_control + 1.1) / 2, 32, open_angle - 10);
	glPopMatrix();

}


// "seismologic" algorithm
void UpdateTerrain2()
{
	int i1, j1, i2, j2, i, j;
	double a, b;
	double delta = 0.06;
	double delta2 = 1;
	
	//draw green area on the left
	for (i = 0; i < GSZ; i++)
		for (j = 0; j < 40; j++)
		{
			if (i<GSZ / 2 - 1 && i>GSZ / 2 - 70 && j<30 ) {
				ground[i][j] += 0.005;
			}
			else if (i>GSZ / 2 + 1 && i<GSZ / 2 + 70 && j<30) {
				ground[i][j] += 0.005;
			}
			else {
				if (delta2 > 0) {
					ground[i][j] = 1;
					delta2 = -delta2;
				}
				else {
					ground[i][j] = 1;
					delta2 = -delta2;
				}
			}
		}
	//draw green area on the right
	for (i = 0; i < GSZ; i++)
		for (j = 100; j < GSZ; j++)
		{
			if (i<GSZ / 2 - 1 && i>GSZ / 2 - 70 && j>120 || i>GSZ / 2 + 1 && i<GSZ / 2 + 70 &&j>120) {
				ground[i][j] += 0.006;
			
			}
			else {
				if (delta2 > 0) {
					ground[i][j] = 1;
					delta2 = -delta2;
				}
				else {
					ground[i][j] = 1;
					delta2 = -delta2;
				}
			}
		}


	if (rand() % 2 == 0)
		delta = -delta;

	i1 = rand() % GSZ;
	j1 = rand() % GSZ;
	i2 = rand() % GSZ;
	j2 = rand() % GSZ;

	//draw random terrains 
	if (j1 != j2)
	{
		a = ((double)i2 - i1) / (j2 - j1);
		b = i1 - a * j1;

		for (i = 0; i < GSZ; i++)
			for (j = 10; j < 60; j++)
			{
				if (i<GSZ / 2 - 1 || i>GSZ / 2 + 1) {
					if (i < a * j + b)
						ground[i][j] += delta;
					else
						ground[i][j] -= delta;
				}
			}
			
		for (i = 0; i < GSZ; i++)
			for (j = 90; j < GSZ-10; j++)
			{
				if (i<GSZ / 2 - 1 || i>GSZ / 2 + 1) {
					if (i < a * j + b)
						ground[i][j] += 0.01 + delta;
					else
						ground[i][j] -= 0.01 + delta;
				}
			}
			
		//draw river 
		for (i = 0; i < GSZ; i++)
			for (j = 60; j < 90; j++)
			{
				ground[i][j] = -1.5 + delta;
			}
	}
}

// random walk
void UpdateTerrain3()
{
	int counter = 1200;
	int i, j;
	double delta = 0.08;

	if (rand() % 2 == 0)
		delta = -delta;

	// initial point
	i = rand() % GSZ;
	j = rand() % GSZ;
	while (counter > 0)
	{
		counter--;
		ground[i][j] += delta;
		// choose direction
		switch (rand() % 4)
		{
		case 0: // up
			i++;
			break;
		case 1: // right
			j++;
			break;
		case 2: // down
			i--;
			break;
		case 3: // left
			j--;
			break;
		}

		i = (i + GSZ) % GSZ;
		j = (j + GSZ) % GSZ;
	}
}


void Smooth()
{
	double tmp[GSZ][GSZ];
	int i, j;

	// compute smoothing signal
	for (i = 1; i < GSZ - 1; i++)
		for (j = 1; j < GSZ - 1; j++)
			tmp[i][j] = (0.25 * ground[i - 1][j - 1] + ground[i - 1][j] + 0.25 * ground[i - 1][j + 1] +
				ground[i][j - 1] + 4 * ground[i][j] + ground[i][j + 1] +
				0.25 * ground[i + 1][j - 1] + ground[i + 1][j] + 0.25 * ground[i + 1][j + 1]) / 9.0;

	// copy the new signal
	for (i = 1; i < GSZ - 1; i++)
		for (j = 1; j < GSZ - 1; j++)
			ground[i][j] = tmp[i][j];

}

// height is +-4
void SetColor(double h)
{
	if (fabs(h) < 0.2) // sand
		glColor3d(0.8, 0.7, 0.5);
	else
		if (fabs(h) < 7) // green fields
			glColor3d(0.3 + h / 40, 0.6 - fabs(h) / 20, 0);
		else // rocks and snow
			glColor3d(sqrt(h) / 5, sqrt(h) / 5, sqrt(h) / 4.0);
}

void SetNormal(int i, int j)
{
	double n[3];

	n[0] = ground[i][j] - ground[i][j + 1];
	n[1] = 1;
	n[2] = ground[i][j] - ground[i + 1][j];
	glNormal3d(n[0], n[1], n[2]);

}

// the parameters are indices in ground matrix
void DrawFence(int startx, int startz, int endx, int endz, double h)
{
	int sx = startx - GSZ / 2, sz = startz - GSZ / 2, ex = endx - GSZ / 2, ez = endz - GSZ / 2;
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 3); // Fence texture
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // GL_REPLACE

	// draw near fence

	glColor3d(0.7, 0.6, 0.5);
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);	glVertex3d(sx, h, ez);
	glTexCoord2d(0, 1);		glVertex3d(sx, h + 2, ez);
	glTexCoord2d(3, 1);		glVertex3d(ex, h + 2, ez);
	glTexCoord2d(3, 0);		glVertex3d(ex, h, ez);
	glEnd();

	// draw far fence
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);	glVertex3d(sx, h, sz);
	glTexCoord2d(0, 1);		glVertex3d(sx, h + 2, sz);
	glTexCoord2d(3, 1);		glVertex3d(ex, h + 2, sz);
	glTexCoord2d(3, 0);		glVertex3d(ex, h, sz);
	glEnd();

	// draw left fence
	glColor3d(1, 1, 1);
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);	glVertex3d(sx, h, ez);
	glTexCoord2d(0, 1);		glVertex3d(sx, h + 2, ez);
	glTexCoord2d(3, 1);		glVertex3d(sx, h + 2, sz);
	glTexCoord2d(3, 0);		glVertex3d(sx, h, sz);
	glEnd();

	// draw right fence
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);	glVertex3d(ex, h, ez);
	glTexCoord2d(0, 1);		glVertex3d(ex, h + 2, ez);
	glTexCoord2d(3, 1);		glVertex3d(ex, h + 2, sz);
	glTexCoord2d(3, 0);		glVertex3d(ex, h, sz);
	glEnd();

	glDisable(GL_TEXTURE_2D);

}

// the parameters are indices in ground matrix
void DrawHouse(int startx, int startz, int endx, int endz, double h, int hrepeat, int vrepeat)
{
	int sx = startx - GSZ / 2, sz = startz - GSZ / 2, ex = endx - GSZ / 2, ez = endz - GSZ / 2;
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 4); // wall with window texture
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // GL_REPLACE

	// draw near wall

	glColor3d(0.7, 0.6, 0.5);
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);	glVertex3d(sx, h, ez);
	glTexCoord2d(0, vrepeat);		glVertex3d(sx, h + 8, ez);
	glTexCoord2d(hrepeat, vrepeat);		glVertex3d(ex, h + 8, ez);
	glTexCoord2d(hrepeat, 0);		glVertex3d(ex, h, ez);
	glEnd();

	// draw far wall
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);	glVertex3d(sx, h, sz);
	glTexCoord2d(0, vrepeat);		glVertex3d(sx, h + 8, sz);
	glTexCoord2d(hrepeat, vrepeat);		glVertex3d(ex, h + 8, sz);
	glTexCoord2d(hrepeat, 0);		glVertex3d(ex, h, sz);
	glEnd();

	// draw left wall
	glColor3d(1, 1, 1);
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);	glVertex3d(sx, h, ez);
	glTexCoord2d(0, vrepeat);		glVertex3d(sx, h + 8, ez);
	glTexCoord2d(hrepeat, vrepeat);		glVertex3d(sx, h + 8, sz);
	glTexCoord2d(hrepeat, 0);		glVertex3d(sx, h, sz);
	glEnd();

	// draw right wall
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);	glVertex3d(ex, h, ez);
	glTexCoord2d(0, vrepeat);		glVertex3d(ex, h + 8, ez);
	glTexCoord2d(hrepeat, vrepeat);		glVertex3d(ex, h + 8, sz);
	glTexCoord2d(hrepeat, 0);		glVertex3d(ex, h, sz);
	glEnd();

	// top floor
	// near
	glColor3d(0.7, 0.6, 0.5);
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);	glVertex3d(sx, h + 8, ez);
	glTexCoord2d(0.5 * hrepeat, vrepeat / 2.0);		glVertex3d((sx + ex) / 2, h + 12, ez);
	glTexCoord2d(hrepeat, 0);		glVertex3d(ex, h + 8, ez);
	glEnd();
	// far
	glColor3d(0.7, 0.6, 0.5);
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);	glVertex3d(sx, h + 8, sz);
	glTexCoord2d(0.5 * hrepeat, vrepeat / 2.0);		glVertex3d((sx + ex) / 2, h + 12, sz);
	glTexCoord2d(hrepeat, 0);		glVertex3d(ex, h + 8, sz);
	glEnd();

	// draw roof
	glBindTexture(GL_TEXTURE_2D, 5); //roof texture
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // GL_REPLACE

	// draw left side of roof
	glColor3d(1, 1, 1);
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);	glVertex3d(sx, h + 8, ez);
	glTexCoord2d(0, vrepeat);		glVertex3d((sx + ex) / 2, h + 12, ez);
	glTexCoord2d(hrepeat, vrepeat);		glVertex3d((sx + ex) / 2, h + 12, sz);
	glTexCoord2d(hrepeat, 0);		glVertex3d(sx, h + 8, sz);
	glEnd();
	// draw left side of roof

	glColor3d(0.7, 0.7, 0.7);
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);	glVertex3d(ex, h + 8, ez);
	glTexCoord2d(0, vrepeat);		glVertex3d((sx + ex) / 2, h + 12, ez);
	glTexCoord2d(hrepeat, vrepeat);		glVertex3d((sx + ex) / 2, h + 12, sz);
	glTexCoord2d(hrepeat, 0);		glVertex3d(ex, h + 8, sz);
	glEnd();


	glDisable(GL_TEXTURE_2D);

}

void DrawBridgeL2R() {
	int j;
	int i = 0;
	bool isBridgeNeeded = 1;
	bool isWireNeeded = 0;
	for (j = 0; j < GSZ - 4; j++)
	{
		//if ((j + 3) < GSZ / 2 - 2 || (j + 3) > GSZ / 2 + 1) {
			if (ground[GSZ / 2][j + 3] < 0 && isBridgeNeeded) {
				i = j + 3;
				//back
				glBegin(GL_POLYGON);
				glVertex3d(i - GSZ / 2, 1, -1);
				glVertex3d(i - GSZ / 2, 3, -1);
				glVertex3d(i - GSZ / 2 + 0.1, 3, -1);
				glVertex3d(i - GSZ / 2 + 0.1, 1, -1);
				glEnd();

				//front
				glBegin(GL_POLYGON);
				glVertex3d(i - GSZ / 2, 1, 1);
				glVertex3d(i - GSZ / 2, 3, 1);
				glVertex3d(i - GSZ / 2 + 0.1, 3, 1);
				glVertex3d(i - GSZ / 2 + 0.1, 1, 1);
				glEnd();
				isBridgeNeeded = 0;
			}
			else {
				isBridgeNeeded = 1;

			}
		//}
	}

}

void DrawWireL2R() {
	int j;
	int i = 0;
	bool isWireNeeded = 1;
	for (j = 0; j < GSZ - 4; j++)
	{
		//if ((j + 3) < GSZ / 2 - 2 || (j + 3) > GSZ / 2 + 1) {
			if (ground[GSZ / 2][j + 3] < 0 && isWireNeeded) {
				i = j + 3;
				//back
				//first wire from top to left
				glBegin(GL_LINES);
				glVertex3d(i - GSZ / 2, 3, -1);
				glVertex3d(j - GSZ / 2 + 2, 1, -1);
				glEnd();

				//second wire from top to left
				glBegin(GL_LINES);
				glVertex3d(i - GSZ / 2, 2.5, -1);
				glVertex3d(j - GSZ / 2 + 2, 1, -1);
				glEnd();

				//third wire from top to left
				glBegin(GL_LINES);
				glVertex3d(i - GSZ / 2, 2, -1);
				glVertex3d(j - GSZ / 2 + 2, 1, -1);
				glEnd();

				//first wire from top to right
				glBegin(GL_LINES);
				glVertex3d(i - GSZ / 2 + 0.1, 3, -1);
				glVertex3d(i - GSZ / 2 + 1, 1, -1);
				glEnd();

				//second wire from top to right
				glBegin(GL_LINES);
				glVertex3d(i - GSZ / 2 + 0.1, 2.5, -1);
				glVertex3d(i - GSZ / 2 + 1, 1, -1);
				glEnd();

				//third wire from top to right
				glBegin(GL_LINES);
				glVertex3d(i - GSZ / 2 + 0.1, 2, -1);
				glVertex3d(i - GSZ / 2 + 1, 1, -1);
				glEnd();


				//front
				//first wire from top to left
				glBegin(GL_LINES);
				glVertex3d(i - GSZ / 2, 3, 1);
				glVertex3d(j - GSZ / 2 + 2, 1, 1);
				glEnd();

				//second wire from top to left
				glBegin(GL_LINES);
				glVertex3d(i - GSZ / 2, 2.5, 1);
				glVertex3d(j - GSZ / 2 + 2, 1, 1);
				glEnd();

				//third wire from top to left
				glBegin(GL_LINES);
				glVertex3d(i - GSZ / 2, 2, 1);
				glVertex3d(j - GSZ / 2 + 2, 1, 1);
				glEnd();

				//first wire from top to right
				glBegin(GL_LINES);
				glVertex3d(i - GSZ / 2 + 0.1, 3, 1);
				glVertex3d(i - GSZ / 2 + 1, 1, 1);
				glEnd();

				//second wire from top to right
				glBegin(GL_LINES);
				glVertex3d(i - GSZ / 2 + 0.1, 2.5, 1);
				glVertex3d(i - GSZ / 2 + 1, 1, 1);
				glEnd();

				//third wire from top to right
				glBegin(GL_LINES);
				glVertex3d(i - GSZ / 2 + 0.1, 2, 1);
				glVertex3d(i - GSZ / 2 + 1, 1, 1);
				glEnd();
				isWireNeeded = 0;
			}
			else {
				isWireNeeded = 1;

			}
		//}

	}
}

void DrawBridgeN2F()
{

	int j;
	int i = 0;
	bool isBridgeNeeded = 1;
	bool isWireNeeded = 0;
	for (i = 0; i < GSZ - 4; i++)
	{
		if ((i + 3) < GSZ / 2 - 2 || (i + 3) > GSZ / 2 + 1) {
			if (ground[i + 3][GSZ / 2] < 0 && isBridgeNeeded) {
				j = i + 3;
				//back
				glBegin(GL_POLYGON);
				glVertex3d(-1, 1, j - GSZ / 2);
				glVertex3d(-1, 3, j - GSZ / 2);
				glVertex3d(-1, 3, j - GSZ / 2 + 0.1);
				glVertex3d(-1, 1, j - GSZ / 2 + 0.1);
				glEnd();

				//front
				glBegin(GL_POLYGON);
				glVertex3d(1, 1, j - GSZ / 2);
				glVertex3d(1, 3, j - GSZ / 2);
				glVertex3d(1, 3, j - GSZ / 2 + 0.1);
				glVertex3d(1, 1, j - GSZ / 2 + 0.1);
				glEnd();
				isBridgeNeeded = 0;
			}
			else {
				isBridgeNeeded = 1;

			}
		}
	}

}

void DrawWireN2F()
{

	int j;
	int i = 0;
	bool isWireNeeded = 1;
	for (i = 0; i < GSZ - 4; i++)
	{
		if ((i + 3) < GSZ / 2 - 2 || (i + 3) > GSZ / 2 + 1) {
			if (ground[i + 3][GSZ / 2] < 0 && isWireNeeded) {
				j = i + 3;

				//left side
				//first wire from top to left
				glBegin(GL_LINES);
				glVertex3d(-1, 3, j - GSZ / 2);
				glVertex3d(-1, 1, i - GSZ / 2 + 2);
				glEnd();

				//second wire from top to left
				glBegin(GL_LINES);
				glVertex3d(-1, 2.5, j - GSZ / 2);
				glVertex3d(-1, 1, i - GSZ / 2 + 2);
				glEnd();

				//third wire from top to left
				glBegin(GL_LINES);
				glVertex3d(-1, 2, j - GSZ / 2);
				glVertex3d(-1, 1, i - GSZ / 2 + 2);
				glEnd();

				//first wire from top to right
				glBegin(GL_LINES);
				glVertex3d(-1, 3, j - GSZ / 2);
				glVertex3d(-1, 1, j - GSZ / 2 + 1);
				glEnd();

				//second wire from top to right
				glBegin(GL_LINES);
				glVertex3d(-1, 2.5, j - GSZ / 2);
				glVertex3d(-1, 1, j - GSZ / 2 + 1);
				glEnd();

				//third wire from top to right
				glBegin(GL_LINES);
				glVertex3d(-1, 2, j - GSZ / 2);
				glVertex3d(-1, 1, j - GSZ / 2 + 1);
				glEnd();



				//right side
				//first wire from top to right
				glBegin(GL_LINES);
				glVertex3d(1, 3, j - GSZ / 2);
				glVertex3d(1, 1, j - GSZ / 2 + 1);
				glEnd();

				//second wire from top to right
				glBegin(GL_LINES);
				glVertex3d(1, 2.5, j - GSZ / 2);
				glVertex3d(1, 1, j - GSZ / 2 + 1);
				glEnd();

				//third wire from top to right
				glBegin(GL_LINES);
				glVertex3d(1, 2, j - GSZ / 2);
				glVertex3d(1, 1, j - GSZ / 2 + 1);
				glEnd();

				//first wire from top to left
				glBegin(GL_LINES);
				glVertex3d(1, 3, j - GSZ / 2);
				glVertex3d(1, 1, i - GSZ / 2 + 2);
				glEnd();

				//second wire from top to left
				glBegin(GL_LINES);
				glVertex3d(1, 2.5, j - GSZ / 2);
				glVertex3d(1, 1, i - GSZ / 2 + 2);
				glEnd();

				//third wire from top to left
				glBegin(GL_LINES);
				glVertex3d(1, 2, j - GSZ / 2);
				glVertex3d(1, 1, i - GSZ / 2 + 2);
				glEnd();
				isWireNeeded = 0;
			}
			else {
				isWireNeeded = 1;

			}
		}
	}

}

void DrawRoad()
{
	int j, i;
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 1); // road texture
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); // GL_MODULATE

	// Road from left to right
	for (j = 0; j < GSZ - 1; j++)
	{
		//if (j<GSZ / 2 - 1 || j>GSZ / 2) // not on crossroads
		//{
			glBegin(GL_POLYGON);
			glTexCoord2d(0, 0);
			if (ground[GSZ / 2][j] > 0) 			glVertex3d(j - GSZ / 2, ground[GSZ / 2][j] + 0.1, -1); // point 1;
			else  glVertex3d(j - GSZ / 2, 1, -1); // bridge
			glTexCoord2d(1, 0);
			if (ground[GSZ / 2][j + 1] > 0)
			{
				glVertex3d(j - GSZ / 2 + 1, ground[GSZ / 2][j + 1] + 0.1, -1); // point 2
				glTexCoord2d(1, 2);   		glVertex3d(j - GSZ / 2 + 1, ground[GSZ / 2][j + 1] + 0.1, 1); // point 3
			}
			else
			{
				glVertex3d(j - GSZ / 2 + 1, 1, -1);
				glTexCoord2d(1, 2);   		glVertex3d(j - GSZ / 2 + 1, 1, 1); // point 3 on bridge
			}

			glTexCoord2d(0, 2);
			if (ground[GSZ / 2][j] > 0) 	glVertex3d(j - GSZ / 2, ground[GSZ / 2][j] + 0.1, 1); // point 4
			else glVertex3d(j - GSZ / 2, 1, 1); // point 4
			glEnd();
		//}
	}

	DrawBridgeL2R();
	DrawWireL2R();



	
	// pedestrian walk
	glBindTexture(GL_TEXTURE_2D, 2); // zebra texture
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); // GL_MODULATE

	// Cross roads

	//else // this is a bridge
	{
	/*	glBegin(GL_POLYGON);
		/*glTexCoord2d(0.5, 0.1);*/		glVertex3d(-1, 1, -1);
		/*glTexCoord2d(1, 0.1);	*/	glVertex3d(1, 1, -1);
		/*glTexCoord2d(1, 0.9); */   	glVertex3d(1, 1, 1);
		/*glTexCoord2d(0.5, 0.9);*/    	glVertex3d(-1, 1, 1);
	//	glEnd();
	}
	glDisable(GL_TEXTURE_2D);
	
	/*for (i = 0; i < GSZ; i++)
		for (j = 100; j < GSZ; j++)
		{
			if (i<GSZ / 2 - 1 && i>GSZ / 2 - 70 && j > 120 || i > GSZ / 2 + 1 && i < GSZ / 2 + 70 && j>120) {
				
				DrawFlower();

			}
		
		}*/




}


// draws the ground matrix
void DrawFloor()
{
	int i, j;

	glColor3d(0, 0, 0.3);
	// i goes on z, j goes on x, y is taken from ground[i][j]
	for (i = 0; i < GSZ - 2; i++)
		for (j = 0; j < GSZ - 2; j++)
		{
			/*		if (fabs(ground[i][j]) >= 6) // snow
					{
						// set material: silver
						glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mamb3);
						glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mdiff3);
						glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mspec3);
						glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50);

					}
					else // sand
					{
						// gold
						glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mamb2);
						glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mdiff2);
						glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mspec2);
						glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 80);

					}
		*/			glBegin(GL_POLYGON);// GL_LINE_LOOP);
		SetColor(ground[i][j]);
		//			SetNormal(i, j);
		glVertex3d(j - GSZ / 2, ground[i][j], i - GSZ / 2);
		SetColor(ground[i][j + 1]);
		//			SetNormal(i, j+1);
		glVertex3d(j + 1 - GSZ / 2, ground[i][j + 1], i - GSZ / 2);
		SetColor(ground[i + 1][j + 1]);
		//			SetNormal(i+1, j+1);
		glVertex3d(j + 1 - GSZ / 2, ground[i + 1][j + 1], i + 1 - GSZ / 2);
		SetColor(ground[i + 1][j]);
		//			SetNormal(i+1, j);
		glVertex3d(j - GSZ / 2, ground[i + 1][j], i + 1 - GSZ / 2);
		glEnd();
		}

	DrawRoad();

	// add water surface
	//	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4d(0, 0.3, 0.7, 0.6);
	glBegin(GL_POLYGON);
	glVertex3d(-GSZ / 2, 0, -GSZ / 2);
	glVertex3d(-GSZ / 2, 0, GSZ / 2);
	glVertex3d(GSZ / 2, 0, GSZ / 2);
	glVertex3d(GSZ / 2, 0, -GSZ / 2);
	glEnd();
	glDisable(GL_BLEND);
	//	glEnable(GL_LIGHTING);

}


void DrawAxes()
{
	glLineWidth(2);
	glBegin(GL_LINES);
	glColor3d(1, 0, 0); // x
	glVertex3d(0, 0, 0);
	glVertex3d(15, 0, 0);
	glColor3d(0, 1, 0); // y
	glVertex3d(0, 0, 0);
	glVertex3d(0, 5, 0);
	glColor3d(0, 0, 1); // z
	glVertex3d(0, 0, 0);
	glVertex3d(0, 0, 15);

	glEnd();
	glLineWidth(1);

}

void DrawCube()
{

	// top side
	glBegin(GL_POLYGON);
	glColor3d(1, 0, 0); // Red
	glVertex3d(1, 1, 1);
	glColor3d(1, 1, 0); // Yellow
	glVertex3d(1, 1, -1);
	glColor3d(0, 1, 0); // Green
	glVertex3d(-1, 1, -1);
	glColor3d(1, 1, 1); // White
	glVertex3d(-1, 1, 1);
	glEnd();

	// bottom side
	glBegin(GL_POLYGON);
	glColor3d(1, 0, 1); // Magenta
	glVertex3d(1, -1, 1);
	glColor3d(0, 0, 0); // Black
	glVertex3d(1, -1, -1);
	glColor3d(0, 1, 1); // Cyan
	glVertex3d(-1, -1, -1);
	glColor3d(0, 0, 1); // Blue
	glVertex3d(-1, -1, 1);
	glEnd();

	// front side
	glBegin(GL_POLYGON);
	glColor3d(1, 0, 0); // Red
	glVertex3d(1, 1, 1);
	glColor3d(1, 1, 1); // White
	glVertex3d(-1, 1, 1);
	glColor3d(0, 0, 1); // Blue
	glVertex3d(-1, -1, 1);
	glColor3d(1, 0, 1); // Magenta
	glVertex3d(1, -1, 1);
	glEnd();

	// rear side
	glBegin(GL_POLYGON);
	glColor3d(1, 1, 0); // Yellow
	glVertex3d(1, 1, -1);
	glColor3d(0, 1, 0); // Green
	glVertex3d(-1, 1, -1);
	glColor3d(0, 1, 1); // Cyan
	glVertex3d(-1, -1, -1);
	glColor3d(0, 0, 0); // Black
	glVertex3d(1, -1, -1);
	glEnd();

	// left side
	glBegin(GL_POLYGON);
	glColor3d(1, 1, 1); // White
	glVertex3d(-1, 1, 1);
	glColor3d(0, 1, 0); // Green
	glVertex3d(-1, 1, -1);
	glColor3d(0, 1, 1); // Cyan
	glVertex3d(-1, -1, -1);
	glColor3d(0, 0, 1); // Blue
	glVertex3d(-1, -1, 1);
	glEnd();

	// right side
	glBegin(GL_POLYGON);
	glColor3d(1, 0, 0); // Red
	glVertex3d(1, 1, 1);
	glColor3d(1, 1, 0); // Yellow
	glVertex3d(1, 1, -1);
	glColor3d(0, 0, 0); // Black
	glVertex3d(1, -1, -1);
	glColor3d(1, 0, 1); // Magenta
	glVertex3d(1, -1, 1);
	glEnd();

}

void DrawCube1()
{

	// top side
	glBegin(GL_POLYGON);
	glColor3d(1, 0, 0); // Red
	glVertex3d(1, 1, 1);
	glColor3d(1, 1, 0); // Yellow
	glVertex3d(1, 1, -1);
	glColor3d(0, 1, 0); // Green
	glVertex3d(-1, 1, -1);
	glColor3d(1, 1, 1); // White
	glVertex3d(-1, 1, 1);
	glEnd();

	// bottom side
	glBegin(GL_POLYGON);
	glColor3d(1, 0, 1); // Magenta
	glVertex3d(1, -1, 1);
	glColor3d(0, 0, 0); // Black
	glVertex3d(1, -1, -1);
	glColor3d(0, 1, 1); // Cyan
	glVertex3d(-1, -1, -1);
	glColor3d(0, 0, 1); // Blue
	glVertex3d(-1, -1, 1);
	glEnd();

	// front side
	glBegin(GL_LINE_LOOP);
	glColor3d(1, 0, 0); // Red
	glVertex3d(1, 1, 1);
	glColor3d(1, 1, 1); // White
	glVertex3d(-1, 1, 1);
	glColor3d(0, 0, 1); // Blue
	glVertex3d(-1, -1, 1);
	glColor3d(1, 0, 1); // Magenta
	glVertex3d(1, -1, 1);
	glEnd();

	// rear side
	glBegin(GL_LINE_LOOP);
	glColor3d(1, 1, 0); // Yellow
	glVertex3d(1, 1, -1);
	glColor3d(0, 1, 0); // Green
	glVertex3d(-1, 1, -1);
	glColor3d(0, 1, 1); // Cyan
	glVertex3d(-1, -1, -1);
	glColor3d(0, 0, 0); // Black
	glVertex3d(1, -1, -1);
	glEnd();

	// left side
	glBegin(GL_LINE_LOOP);
	glColor3d(1, 1, 1); // White
	glVertex3d(-1, 1, 1);
	glColor3d(0, 1, 0); // Green
	glVertex3d(-1, 1, -1);
	glColor3d(0, 1, 1); // Cyan
	glVertex3d(-1, -1, -1);
	glColor3d(0, 0, 1); // Blue
	glVertex3d(-1, -1, 1);
	glEnd();

	// right side
	glBegin(GL_LINE_LOOP);
	glColor3d(1, 0, 0); // Red
	glVertex3d(1, 1, 1);
	glColor3d(1, 1, 0); // Yellow
	glVertex3d(1, 1, -1);
	glColor3d(0, 0, 0); // Black
	glVertex3d(1, -1, -1);
	glColor3d(1, 0, 1); // Magenta
	glVertex3d(1, -1, 1);
	glEnd();

}

void DrawCylinder(int n)
{
	double alpha, teta = 2 * PI / n;

	for (alpha = 0; alpha <= 2 * PI; alpha += teta)
	{
		// defines one side

		glBegin(GL_POLYGON);
		glColor3d(fabs(sin(alpha)), (1 + cos(alpha)) / 2, 1 - fabs(sin(alpha + PI / 2)));
		glVertex3d(sin(alpha), 1, cos(alpha)); // vertex 1
		glVertex3d(sin(alpha + teta), 1, cos(alpha + teta)); // vertex 2
		glColor3d(fabs(sin(alpha)) / 2, (1 + cos(alpha)) / 4, 1 - fabs(sin(alpha + PI / 2)));
		glVertex3d(sin(alpha + teta), 0, cos(alpha + teta)); // vertex 3
		glVertex3d(sin(alpha), 0, cos(alpha)); // vertex 4
		glEnd();
	}
}

void DrawCylinder3(int n, double r, double g, double b)
{
	double alpha, teta = 2 * PI / n;

	glColor3d(r, g, b);
	for (alpha = 0; alpha <= 2 * PI; alpha += teta)
	{
		// defines one side

		glBegin(GL_POLYGON);
		glVertex3d(sin(alpha), 1, cos(alpha)); // vertex 1
		glVertex3d(sin(alpha + teta), 1, cos(alpha + teta)); // vertex 2
		glVertex3d(sin(alpha + teta), 0, cos(alpha + teta)); // vertex 3
		glVertex3d(sin(alpha), 0, cos(alpha)); // vertex 4
		glEnd();
	}
}


// tnum is texture number
// num_repeat is a number of texture repeats
void DrawTexCylinder(int n, int tnum, int num_repeat)
{
	double alpha, teta = 2 * PI / n;
	double part = num_repeat / (double)n;
	int counter;

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, tnum); // wall with window texture
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // GL_REPLACE


	for (alpha = 0, counter = 0; alpha <= 2 * PI; alpha += teta, counter++)
	{
		// defines one side

		glBegin(GL_POLYGON);
		glColor3d(0.3 + 0.7 * fabs(sin(alpha)), 0.3 + 0.7 * fabs(sin(alpha)), 0.3 + 0.7 * fabs(sin(alpha)));
		glTexCoord2d(counter * part, 1);    glVertex3d(sin(alpha), 1, cos(alpha)); // vertex 1
		glTexCoord2d((counter + 1) * part, 1);    		glVertex3d(sin(alpha + teta), 1, cos(alpha + teta)); // vertex 2
		glTexCoord2d((counter + 1) * part, 0);    		glVertex3d(sin(alpha + teta), 0, cos(alpha + teta)); // vertex 3
		glTexCoord2d(counter * part, 0);    		glVertex3d(sin(alpha), 0, cos(alpha)); // vertex 4
		glEnd();
	}

	glDisable(GL_TEXTURE_2D);

}


// tnum is texture number
// num_repeat is a number of texture repeats
void DrawTexCylinder1(int n, int tnum, int num_repeat, double tr, double br)
{
	double alpha, teta = 2 * PI / n;
	double part = num_repeat / (double)n;
	int counter;

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, tnum); // wall with window texture
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // GL_REPLACE


	for (alpha = 0, counter = 0; alpha <= 2 * PI; alpha += teta, counter++)
	{
		// defines one side

		glBegin(GL_POLYGON);
		glColor3d(0.3 + 0.7 * fabs(sin(alpha)), 0.3 + 0.7 * fabs(sin(alpha)), 0.3 + 0.7 * fabs(sin(alpha)));
		glTexCoord2d(counter * part, 1);    glVertex3d(tr * sin(alpha), 1, tr * cos(alpha)); // vertex 1
		glTexCoord2d((counter + 1) * part, 1);    		glVertex3d(tr * sin(alpha + teta), 1, tr * cos(alpha + teta)); // vertex 2
		glTexCoord2d((counter + 1) * part, 0);    		glVertex3d(br * sin(alpha + teta), 0, br * cos(alpha + teta)); // vertex 3
		glTexCoord2d(counter * part, 0);    		glVertex3d(br * sin(alpha), 0, br * cos(alpha)); // vertex 4
		glEnd();
	}

	glDisable(GL_TEXTURE_2D);

}

// DrawTexCylinder2 attaches vertical and horizontal part of texture
// on cylinder
// tnum is texture number
// num_repeat is a number of texture repeats
// tpart and bpart are vertical texture coordinates
void DrawTexCylinder2(int n, int tnum, int num_repeat, double tr, double br, double tpart, double bpart)
{
	double alpha, teta = 2 * PI / n;
	double part = num_repeat / (double)n;
	int counter;

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, tnum); // wall with window texture
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);


	for (alpha = 0, counter = 0; alpha <= 2 * PI; alpha += teta, counter++)
	{
		// defines one side

		glBegin(GL_POLYGON);
		//		glColor3d(0.3 + 0.7 * fabs(sin(alpha)), 0.3 + 0.7 * fabs(sin(alpha)), 0.3 + 0.7 * fabs(sin(alpha)));
		glTexCoord2d(counter * part, tpart);    glVertex3d(tr * sin(alpha), 1, tr * cos(alpha)); // vertex 1
		glTexCoord2d((counter + 1) * part, tpart);    		glVertex3d(tr * sin(alpha + teta), 1, tr * cos(alpha + teta)); // vertex 2
		glTexCoord2d((counter + 1) * part, bpart);    		glVertex3d(br * sin(alpha + teta), 0, br * cos(alpha + teta)); // vertex 3
		glTexCoord2d(counter * part, bpart);    		glVertex3d(br * sin(alpha), 0, br * cos(alpha)); // vertex 4
		glEnd();
	}

	glDisable(GL_TEXTURE_2D);

}


// topr is top radius, bottomr is bottom radius
void DrawCylinder1(int n, double topr, double bottomr)
{
	double alpha, teta = 2 * PI / n;

	for (alpha = 0; alpha <= 2 * PI; alpha += teta)
	{
		// defines one side
		glBegin(GL_POLYGON);
		glColor3d(fabs(sin(alpha)), (1 + cos(alpha)) / 2, 1 - fabs(sin(alpha + PI / 2)));
		glVertex3d(topr * sin(alpha), 1, topr * cos(alpha)); // vertex 1
		glVertex3d(topr * sin(alpha + teta), 1, topr * cos(alpha + teta)); // vertex 2
		glColor3d(fabs(sin(alpha)) / 2, (1 + cos(alpha)) / 4, 1 - fabs(sin(alpha + PI / 2)));
		glVertex3d(bottomr * sin(alpha + teta), 0, bottomr * cos(alpha + teta)); // vertex 3
		glVertex3d(bottomr * sin(alpha), 0, bottomr * cos(alpha)); // vertex 4
		glEnd();
	}
}

// topr is top radius, bottomr is bottom radius
// defines vector of normal
void DrawLitCylinder1(int n, double topr, double bottomr)
{
	double alpha, teta = 2 * PI / n, h, len;

	for (alpha = 0; alpha <= 2 * PI; alpha += teta)
	{
		h = (bottomr - topr) * bottomr;

		// defines one side
		glBegin(GL_POLYGON);
		glNormal3d(topr * sin(alpha), h, topr * cos(alpha));
		glVertex3d(topr * sin(alpha), 1, topr * cos(alpha)); // vertex 1
		glNormal3d(topr * sin(alpha + teta), h, topr * cos(alpha + teta));
		glVertex3d(topr * sin(alpha + teta), 1, topr * cos(alpha + teta)); // vertex 2
		glNormal3d(bottomr * sin(alpha + teta), h, bottomr * cos(alpha + teta));
		glVertex3d(bottomr * sin(alpha + teta), 0, bottomr * cos(alpha + teta)); // vertex 3
		glNormal3d(bottomr * sin(alpha), h, bottomr * cos(alpha));
		glVertex3d(bottomr * sin(alpha), 0, bottomr * cos(alpha)); // vertex 4
		glEnd();
	}
}

void DrawConus(int n)
{
	double alpha, teta = 2 * PI / n;

	for (alpha = 0; alpha <= 2 * PI; alpha += teta)
	{
		// defines one side

		glBegin(GL_POLYGON);
		glColor3d(1, 1, 1);
		glVertex3d(0, 1, 0); // vertex 1
		glColor3d(fabs(sin(alpha)) / 2, (1 + cos(alpha)) / 4, 1 - fabs(sin(alpha + PI / 2)));
		glVertex3d(sin(alpha + teta), 0, cos(alpha + teta)); // vertex 2
		glVertex3d(sin(alpha), 0, cos(alpha)); // vertex 3
		glEnd();
	}
}


void DrawConus2(int n)
{
	double alpha, teta = 2 * PI / n;

	for (alpha = 0; alpha <= 2 * PI; alpha += teta)
	{
		// defines one side

		glBegin(GL_POLYGON);
		glColor3d(1, 1, 1);
		glVertex3d(0, 1, 0); // vertex 1
		glColor3d(0, 0.3,0.1);
		glVertex3d(sin(alpha + teta), 0, cos(alpha + teta)); // vertex 2
		glVertex3d(sin(alpha), 0, cos(alpha)); // vertex 3
		glEnd();
	}
}

// n is the number of sectors, sl is the number of slices
void DrawSphere(int n, int sl)
{
	double beta, gamma = PI / sl;


	for (beta = -PI / 2; beta <= PI / 2; beta += gamma)
	{
		glPushMatrix();
		glTranslated(0, sin(beta), 0);
		glScaled(1, sin(beta + gamma) - sin(beta), 1);
		DrawCylinder1(n, cos(beta + gamma), cos(beta));
		glPopMatrix();
	}
}

// n is the number of sectors, sl is the number of slices
// num_rep is amount of horizontal repeats of a texture
// vert_rep is amount of vertical repeats of a texture
void DrawTexSphere(int n, int sl, int tnum, int num_rep, int vert_rep)
{
	double beta, gamma = PI / sl;
	int counter;
	double part = vert_rep / (double)sl;

	for (beta = -PI / 2, counter = 0; beta <= PI / 2; beta += gamma, counter++)
	{
		glPushMatrix();
		glTranslated(0, sin(beta), 0);
		glScaled(1, sin(beta + gamma) - sin(beta), 1);
		DrawTexCylinder2(n, tnum, num_rep, cos(beta + gamma), cos(beta), (counter + 1) * part, counter * part);
		glPopMatrix();
	}
}

// n is the number of sectors, sl is the number of slices
void DrawLitSphere(int n, int sl)
{
	double beta, gamma = PI / sl;


	for (beta = -PI / 2; beta <= PI / 2; beta += gamma)
	{
		glPushMatrix();
		glTranslated(0, sin(beta), 0);
		glScaled(1, sin(beta + gamma) - sin(beta), 1);
		DrawLitCylinder1(n, cos(beta + gamma), cos(beta));
		glPopMatrix();
	}
}



void DrawView()
{
	DrawFloor();
	DrawAxes();

	//	glRotated(offset, 0, 1, 0);
	glPushMatrix();
	glTranslated(airLocation[0], airLocation[1], airLocation[2]);
	glRotated(airyaw * 180 / PI, 0, 1, 0);
	glRotated(airpitch * 180 / PI, -1, 0, 0);
	glRotated(-100 * airangularSpeed * 180 / PI, 0, 0, 1);
	glPopMatrix();
}



void drawTree() {
	glPushMatrix();
	glScaled(0.2, 3, 0.2);
	glTranslated(0, 1, 0);
	DrawCylinder3(50, 0.5, 0.2, 0.2);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, 6, 0);
	glScaled(1, 3, 1);
	DrawConus2(20);
	glPopMatrix();
}

void display()
{
	double x, y, beta;

		// once again frame buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clean frame buffer and depth buffer
	glViewport(0, 0, W, H);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity(); // loads the Identity matrix to the Projection Matrix
	// define the camera model
	glFrustum(-1, 1, -1, 1, 1, 300);
	// define the viewing parameters
	gluLookAt(eyex, eyey, eyez, // eye coordinates
		eyex + dir[0], eyey + dir[1], eyez + dir[2], // point of interest coordinates
		0, 1, 0); // vector UP reflects roll

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity(); // loads the Identity matrix to the TRASFORMATION Matrix
	
	int k;
	int x1=-38;
	int y1=1;
	int z1=-5;
	bool isFront = true;
	//first row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1-3;
			z1--;
			isFront = false;
	
		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}

	x1 = -38;
	y1 = 1;
	z1 = -10;


	//second deep row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}

	x1 = -38;
	y1 = 1;
	z1 = -14;

	//third deep row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}


	 x1 = -38;
	 y1 = 1;
	 z1 = -8;

	 

	 //front row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}
	x1 = -38;
	y1 = 1;
	z1 = -13;
	//front deep row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}


	x1 = -38;
	y1 = 1;
	z1 = -16;

	//third deep front row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}

	DrawFloor();
	DrawFence(GSZ / 2 + 6, GSZ / 2 - 19, GSZ / 2 + 19, GSZ / 2 - 6, ground[GSZ / 2 - 6][GSZ / 2 + 6]);

	DrawHouse(GSZ / 2 + 8, GSZ / 2 - 17, GSZ / 2 + 17, GSZ / 2 - 8, ground[GSZ / 2 - 8][GSZ / 2 + 8], 3, 2);


	// textured sphere
	glPushMatrix();
	glTranslated(0, -30, 0);
	glRotated(offset / 10, 0, 1, 0);
	glScaled(180, 180, 180);
	DrawTexSphere(140, 40, 7, 1, 1);
	glPopMatrix();

	//draw car
	bool isLoco = true;
	for (int i = NUM_CARRIAGES - 1; i>=0 ; i--) {
		pTrain[i]->Draw(offset, isLoco);
		isLoco = false;
	}


	glutSwapBuffers(); // show all
}
void displayFromLocomotive()
{
	double x, y, beta;
	double cx, cy, cz; // car center
	double dx = pTrain[0]->GetDirx();
	double dy = pTrain[0]->GetDiry();
	double dz = pTrain[0]->GetDirz();

	cx = pTrain[0]->GetCx();
	cy = pTrain[0]->GetCy();
	cz = pTrain[0]->GetCz();

	// define texture of mirror
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clean frame buffer and depth buffer
	glViewport(0, 0, W, H);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity(); // loads the Identity matrix to the Projection Matrix
	// define the camera model
	glFrustum(0.2, -0.2, -0.2, 0.2, 0.3, 300);
	// define the viewing parameters
	gluLookAt(pTrain[0]->GetCx() , pTrain[0]->GetCy() + 0.6, pTrain[0]->GetCz()  , // eye coordinates
		pTrain[0]->GetCx() - pTrain[0]->GetDirx(), pTrain[0]->GetCy() + 0.6 - pTrain[0]->GetDiry(),
		pTrain[0]->GetCz() - pTrain[0]->GetDirz(), // point of interest coordinates
		0,1, 0); // vector UP reflects roll

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity(); // loads the Identity matrix to the TRASFORMATION Matrix


	int k;
	int x1 = -38;
	int y1 = 0.5;
	int z1 = -5;
	bool isFront = true;
	//first row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}

	x1 = -38;
	y1 = 0.5;
	z1 = -10;


	//second deep row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}

	x1 = -38;
	y1 = 0.5;
	z1 = -14;

	//third deep row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}


	x1 = -38;
	y1 = 0.5;
	z1 = -8;



	//front row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}
	x1 = -38;
	y1 = 0.5;
	z1 = -13;
	//front deep row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}


	x1 = -38;
	y1 = 0.5;
	z1 = -16;

	//third deep front row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}

	DrawFloor();
	DrawFence(GSZ / 2 + 6, GSZ / 2 - 19, GSZ / 2 + 19, GSZ / 2 - 6, ground[GSZ / 2 - 6][GSZ / 2 + 6]);

	DrawHouse(GSZ / 2 + 8, GSZ / 2 - 17, GSZ / 2 + 17, GSZ / 2 - 8, ground[GSZ / 2 - 8][GSZ / 2 + 8], 3, 2);


	// textured sphere (sky)
	glPushMatrix();
	glTranslated(0, -30, 0);
	glRotated(offset / 10, 0, 1, 0);
	glScaled(180, 180, 180);
	DrawTexSphere(140, 40, 7, 1, 1);
	glPopMatrix();


	bool isLoco = true;
	for (int i = NUM_CARRIAGES - 1; i >= 0; i--) {
		pTrain[i]->Draw(offset, isLoco);
		isLoco = false;
	}


	glBindTexture(GL_TEXTURE_2D, 8); // new texture name
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	// creates texture basing on "Frame Buffer"
	glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 0, 0, 512, 512, 0);


	// once again frame buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clean frame buffer and depth buffer
	glViewport(0, 0, W, H);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity(); // loads the Identity matrix to the Projection Matrix
	// define the camera model
	glFrustum(-0.2, 0.2, -0.2, 0.2, 0.3, 300);
	// define the viewing parameters
	gluLookAt(pTrain[0]->GetCx(), pTrain[0]->GetCy() + 0.4, pTrain[0]->GetCz(), // eye coordinates
		pTrain[0]->GetCx() + pTrain[0]->GetDirx(), pTrain[0]->GetCy() + 0.3 + pTrain[0]->GetDiry(),
		pTrain[0]->GetCz() + pTrain[0]->GetDirz(), // point of interest coordinates
		0, 1, 0); // vector UP reflects roll

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity(); // loads the Identity matrix to the TRASFORMATION Matrix

	DrawFloor();
	DrawFence(GSZ / 2 + 6, GSZ / 2 - 19, GSZ / 2 + 19, GSZ / 2 - 6, ground[GSZ / 2 - 6][GSZ / 2 + 6]);

	DrawHouse(GSZ / 2 + 8, GSZ / 2 - 17, GSZ / 2 + 17, GSZ / 2 - 8, ground[GSZ / 2 - 8][GSZ / 2 + 8], 3, 2);


	// textured sphere (sky)
	glPushMatrix();
	glTranslated(0, -30, 0);
	glRotated(offset / 10, 0, 1, 0);
	glScaled(180, 180, 180);
	DrawTexSphere(140, 40, 7, 1, 1);
	glPopMatrix();


	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 8); // new texture name
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); // GL_MODULATE

	// before setting a vertex we should bind the texture coordinate to this vertex
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);		glVertex3d(cx + dx, cy + dy, cz - 0.7 + dz);
	glTexCoord2d(0, 1);		glVertex3d(cx + dx, cy + 0.5 + dy, cz - 0.7 + dz);
	glTexCoord2d(1, 1);		glVertex3d(cx + dx, cy + 0.5 + dy, cz - 0.2 + dz);
	glTexCoord2d(1, 0);		glVertex3d(cx + dx, cy + dy, cz - 0.2 + dz);
	glEnd();
	glDisable(GL_TEXTURE_2D);


	glutSwapBuffers(); // show all
}

void displayFromRight()
{
	double x, y, beta;
	double cx, cy, cz; // car center
	double dx = pTrain[1]->GetDirx();
	double dy = pTrain[1]->GetDiry();
	double dz = pTrain[1]->GetDirz();

	cx = pTrain[1]->GetCx();
	cy = pTrain[1]->GetCy();
	cz = pTrain[1]->GetCz();


	int k;
	int x1 = -38;
	int y1 = 0.5;
	int z1 = -5;
	bool isFront = true;
	//first row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}

	x1 = -38;
	y1 = 0.5;
	z1 = -10;


	//second deep row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}

	x1 = -38;
	y1 = 0.5;
	z1 = -14;

	//third deep row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}


	x1 = -38;
	y1 = 0.5;
	z1 = -8;



	//front row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}
	x1 = -38;
	y1 = 0.5;
	z1 = -13;
	//front deep row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}


	x1 = -38;
	y1 = 0.5;
	z1 = -16;

	//third deep front row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}

	DrawFloor();
	DrawFence(GSZ / 2 + 6, GSZ / 2 - 19, GSZ / 2 + 19, GSZ / 2 - 6, ground[GSZ / 2 - 6][GSZ / 2 + 6]);

	DrawHouse(GSZ / 2 + 8, GSZ / 2 - 17, GSZ / 2 + 17, GSZ / 2 - 8, ground[GSZ / 2 - 8][GSZ / 2 + 8], 3, 2);


	// textured sphere (sky)
	glPushMatrix();
	glTranslated(0, -30, 0);
	glRotated(offset / 10, 0, 1, 0);
	glScaled(180, 180, 180);
	DrawTexSphere(140, 40, 7, 1, 1);
	glPopMatrix();


	bool isLoco = true;
	for (int i = NUM_CARRIAGES - 1; i >= 0; i--) {
		pTrain[i]->Draw(offset, isLoco);
		isLoco = false;
	}




	glBindTexture(GL_TEXTURE_2D, 8); // new texture name
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	// creates texture basing on "Frame Buffer"
	glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 0, 0, 512, 512, 0);


	// once again frame buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clean frame buffer and depth buffer
	glViewport(0, 0, W, H);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity(); // loads the Identity matrix to the Projection Matrix
	// define the camera model
	glFrustum(-0.2, 0.2, -0.2, 0.2, 0.3, 300);
	// define the viewing parameters
	gluLookAt(pTrain[1]->GetCx(), pTrain[1]->GetCy() + 0.4, pTrain[1]->GetCz(), // eye coordinates
		pTrain[1]->GetCx() + pTrain[1]->GetDirx(), pTrain[0]->GetCy() + 0.3 + pTrain[1]->GetDiry(),
		pTrain[1]->GetCz() + pTrain[1]->GetDirz() + 45, // point of interest coordinates
		0, 1, 0); // vector UP reflects roll

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity(); // loads the Identity matrix to the TRASFORMATION Matrix

	DrawFloor();
	DrawFence(GSZ / 2 + 6, GSZ / 2 - 19, GSZ / 2 + 19, GSZ / 2 - 6, ground[GSZ / 2 - 6][GSZ / 2 + 6]);

	DrawHouse(GSZ / 2 + 8, GSZ / 2 - 17, GSZ / 2 + 17, GSZ / 2 - 8, ground[GSZ / 2 - 8][GSZ / 2 + 8], 3, 2);


	// textured sphere (sky)
	glPushMatrix();
	glTranslated(0, -30, 0);
	glRotated(offset / 10, 0, 1, 0);
	glScaled(180, 180, 180);
	DrawTexSphere(140, 40, 7, 1, 1);
	glPopMatrix();


	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 8); // new texture name
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); // GL_MODULATE

	// before setting a vertex we should bind the texture coordinate to this vertex
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);		glVertex3d(cx + dx, cy + dy, cz - 0.7 + dz);
	glTexCoord2d(0, 1);		glVertex3d(cx + dx, cy + 0.5 + dy, cz - 0.7 + dz);
	glTexCoord2d(1, 1);		glVertex3d(cx + dx, cy + 0.5 + dy, cz - 0.2 + dz);
	glTexCoord2d(1, 0);		glVertex3d(cx + dx, cy + dy, cz - 0.2 + dz);
	glEnd();
	glDisable(GL_TEXTURE_2D);




	glutSwapBuffers(); // show all
}


void displayFromLeft()
{
	double x, y, beta;
	double cx, cy, cz; // car center
	double dx = pTrain[1]->GetDirx();
	double dy = pTrain[1]->GetDiry();
	double dz = pTrain[1]->GetDirz();

	cx = pTrain[1]->GetCx();
	cy = pTrain[1]->GetCy();
	cz = pTrain[1]->GetCz();


	int k;
	int x1 = -38;
	int y1 = 0.5;
	int z1 = -5;
	bool isFront = true;
	//first row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}

	x1 = -38;
	y1 = 0.5;
	z1 = -10;


	//second deep row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}

	x1 = -38;
	y1 = 0.5;
	z1 = -14;

	//third deep row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}


	x1 = -38;
	y1 = 0.5;
	z1 = -8;



	//front row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}
	x1 = -38;
	y1 = 0.5;
	z1 = -13;
	//front deep row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}


	x1 = -38;
	y1 = 0.5;
	z1 = -16;

	//third deep front row
	for (k = 0; k < 14; k++) {
		if (isFront) {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1--;
			isFront = false;

		}
		else {
			glPushMatrix();
			glTranslated(x1, y1, -z1);
			drawTree();
			glPopMatrix();
			x1 = x1 - 3;
			z1++;
			isFront = true;
		}

	}

	DrawFloor();
	DrawFence(GSZ / 2 + 6, GSZ / 2 - 19, GSZ / 2 + 19, GSZ / 2 - 6, ground[GSZ / 2 - 6][GSZ / 2 + 6]);

	DrawHouse(GSZ / 2 + 8, GSZ / 2 - 17, GSZ / 2 + 17, GSZ / 2 - 8, ground[GSZ / 2 - 8][GSZ / 2 + 8], 3, 2);


	// textured sphere (sky)
	glPushMatrix();
	glTranslated(0, -30, 0);
	glRotated(offset / 10, 0, 1, 0);
	glScaled(180, 180, 180);
	DrawTexSphere(140, 40, 7, 1, 1);
	glPopMatrix();


	bool isLoco = true;
	for (int i = NUM_CARRIAGES - 1; i >= 0; i--) {
		pTrain[i]->Draw(offset, isLoco);
		isLoco = false;
	}




	glBindTexture(GL_TEXTURE_2D, 8); // new texture name
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	// creates texture basing on "Frame Buffer"
	glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 0, 0, 512, 512, 0);


	// once again frame buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clean frame buffer and depth buffer
	glViewport(0, 0, W, H);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity(); // loads the Identity matrix to the Projection Matrix
	// define the camera model
	glFrustum(-0.2, 0.2, -0.2, 0.2, 0.3, 300);
	// define the viewing parameters
	gluLookAt(pTrain[1]->GetCx(), pTrain[1]->GetCy() + 0.6, pTrain[1]->GetCz()+0.5, // eye coordinates
		pTrain[1]->GetCx() + pTrain[1]->GetDirx(), pTrain[0]->GetCy() + 0.3 + pTrain[1]->GetDiry(),
		pTrain[1]->GetCz() + pTrain[1]->GetDirz() - 45, // point of interest coordinates
		0, 1, 0); // vector UP reflects roll

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity(); // loads the Identity matrix to the TRASFORMATION Matrix

	DrawFloor();
	DrawFence(GSZ / 2 + 6, GSZ / 2 - 19, GSZ / 2 + 19, GSZ / 2 - 6, ground[GSZ / 2 - 6][GSZ / 2 + 6]);

	DrawHouse(GSZ / 2 + 8, GSZ / 2 - 17, GSZ / 2 + 17, GSZ / 2 - 8, ground[GSZ / 2 - 8][GSZ / 2 + 8], 3, 2);


	// textured sphere (sky)
	glPushMatrix();
	glTranslated(0, -30, 0);
	glRotated(offset / 10, 0, 1, 0);
	glScaled(180, 180, 180);
	DrawTexSphere(140, 40, 7, 1, 1);
	glPopMatrix();


	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 8); // new texture name
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); // GL_MODULATE

	// before setting a vertex we should bind the texture coordinate to this vertex
	glBegin(GL_POLYGON);
	glTexCoord2d(0, 0);		glVertex3d(cx + dx, cy + dy, cz - 0.7 + dz);
	glTexCoord2d(0, 1);		glVertex3d(cx + dx, cy + 0.5 + dy, cz - 0.7 + dz);
	glTexCoord2d(1, 1);		glVertex3d(cx + dx, cy + 0.5 + dy, cz - 0.2 + dz);
	glTexCoord2d(1, 0);		glVertex3d(cx + dx, cy + dy, cz - 0.2 + dz);
	glEnd();
	glDisable(GL_TEXTURE_2D);




	glutSwapBuffers(); // show all
}


void idle()
{
	int i, j;
	double dist;
	offset = 0;

	// set locomotion direction 
	yaw += angularSpeed;
	// setup the sight direction
	dir[0] = sin(yaw);
	dir[1] = sin(pitch);
	dir[2] = cos(yaw);
	// setup the motion
	eyex += speed * dir[0];
	eyey += speed * dir[1];
	eyez += speed * dir[2];


	for (i = 0; i < NUM_CARRIAGES; i++)
		pTrain[i]->Move(ground);

	glutPostRedisplay(); // posts message (with request to show the frame ) to main window
}


void special_key(int key, int x, int y)
{
	switch (key)
	{
	case GLUT_KEY_LEFT:
		angularSpeed += 0.001;
		break;
	case GLUT_KEY_RIGHT:
		angularSpeed -= 0.001;
		break;
	case GLUT_KEY_UP:
		speed += 0.01;
		break;
	case GLUT_KEY_DOWN:
		speed -= 0.01;
		break;
	case GLUT_KEY_PAGE_UP:
		pitch += 0.1;
		break;
	case GLUT_KEY_PAGE_DOWN:
		pitch -= 0.1;
		break;


	}
}

void mouse(int button, int state, int x, int y)
{
	/*	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
		{
			//buildTerrain = !buildTerrain;
			Smooth();
		}
		*/
}

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'a':
		airangularSpeed += 0.001;
		break;
	case 'w':
		airspeed += 0.01;
		break;
	case 's':
		airspeed -= 0.01;
		break;
	case 'd':
		airangularSpeed -= 0.001;
		break;
	}
}

// mouse drag function
void motion(int x, int y)
{
	double xp, yp;

	xp = 2 * (x / 100.0) - 1;
	yp = 2 * (H - y) / 100.0 - 1;

	if (-0.2 < xp && xp < 0.2 && airpitch - 0.2 < yp && yp < airpitch + 0.2) // update airpitch
		airpitch = yp;

}

void menu(int choice)
{
	switch (choice)
	{
	case 1:
		glutDisplayFunc(display); // refresh window function
		break;
	case 2:
		glutDisplayFunc(displayFromRight);
		break;
	case 3:
		glutDisplayFunc(displayFromLeft);
		break;
	case 4:
		glutDisplayFunc(displayFromLocomotive);
		break;
	}
}

void main(int argc, char* argv[])
{
	glutInit(&argc, argv);
	// defines BUFFERS: Color buffer (frame buffer) and Depth buffer
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(W, H);
	glutInitWindowPosition(300, 100);
	glutCreateWindow("3D Example");

	glutDisplayFunc(display); // refresh window function
	glutIdleFunc(idle); // kind of timer function

	glutSpecialFunc(special_key);
	glutMouseFunc(mouse);
	glutKeyboardFunc(keyboard);
	glutMotionFunc(motion);
	// menu
	glutCreateMenu(menu);
	glutAddMenuEntry("Regular view", 1);
	glutAddMenuEntry("Display from right", 2);
	glutAddMenuEntry("Display from Left", 3);
	glutAddMenuEntry("Tomas view", 4);
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	init();

	glutMainLoop();
}
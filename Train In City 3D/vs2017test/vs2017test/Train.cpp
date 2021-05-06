#include "Train.h"
#include "glut.h"
#include <math.h>

Train::Train(double red, double green, double blue, double x, double y, double z, double dx, double dy, double dz, double speed)
{
	r = red;
	g = green;
	b = blue;
	cx = x;
	cy = y;
	cz = z;
	dirx = dx;
	diry = dy;
	dirz = dz;
	this->speed = speed;
}


void Train::DrawCylinder1(int n, double topr, double bottomr)
{
	double alpha, teta = 2 * PI / n;

	for (alpha = 0; alpha <= 2 * PI; alpha += teta)
	{
		// defines one side
		glBegin(GL_POLYGON);
		glColor3d(0, 0, 0);
		glVertex3d(topr * sin(alpha), 1, topr * cos(alpha)); // vertex 1
		glVertex3d(topr * sin(alpha + teta), 1, topr * cos(alpha + teta)); // vertex 2
		glColor3d(0, 0, 0);
		glVertex3d(bottomr * sin(alpha + teta), 0, bottomr * cos(alpha + teta)); // vertex 3
		glVertex3d(bottomr * sin(alpha), 0, bottomr * cos(alpha)); // vertex 4
		glEnd();
	}
}

void Train::DrawCylinder5(int n, double topr, double bottomr)
{
	double alpha, teta = 2 * PI / n;

	for (alpha = 0; alpha <= 2 * PI; alpha += teta)
	{
		// defines one side
		glBegin(GL_POLYGON);
		glColor3d(0.6, 0.6, 0.5);
		glVertex3d(topr * sin(alpha), 1, topr * cos(alpha)); // vertex 1
		glVertex3d(topr * sin(alpha + teta), 1, topr * cos(alpha + teta)); // vertex 2
		glColor3d(0.6, 0.6, 0.5);
		glVertex3d(bottomr * sin(alpha + teta), 0, bottomr * cos(alpha + teta)); // vertex 3
		glVertex3d(bottomr * sin(alpha), 0, bottomr * cos(alpha)); // vertex 4
		glEnd();
	}
}

void Train::DrawSphere(int n, int sl)
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

void Train::DrawSphere1(int n, int sl)
{
	double beta, gamma = PI / sl;


	for (beta = -PI / 2; beta <= PI / 2; beta += gamma)
	{
		glPushMatrix();
		glTranslated(0, sin(beta), 0);
		glScaled(1, sin(beta + gamma) - sin(beta), 1);
		DrawCylinder5(n, cos(beta + gamma), cos(beta));
		glPopMatrix();
	}
}


// kind of ring
void Train::DrawTyer(int n, double outer, double inner, double r, double g, double b, int jump)
{
	double alpha, teta = 2 * PI / n;
	double x, z;
	glColor3d(r, g, b);
	for (alpha = 0; alpha < 2 * PI; alpha += jump * teta)
	{
		glBegin(GL_POLYGON);
		x = outer * sin(alpha);
		z = outer * cos(alpha);
		glVertex3d(x, 0, z); // 1
		x = inner * sin(alpha);
		z = inner * cos(alpha);
		glVertex3d(x, 0, z); // 2
		x = inner * sin(alpha + teta);
		z = inner * cos(alpha + teta);
		glVertex3d(x, 0, z); // 3
		x = outer * sin(alpha + teta);
		z = outer * cos(alpha + teta);
		glVertex3d(x, 0, z); // 4
		glEnd();
	}
}

void Train::DrawCylinder3(int n, double r, double g, double b)
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

void Train::DrawWheel(double offset)
{
	glPushMatrix();
	glRotated(-5 * offset, 0, 1, 0);
	DrawCylinder3(50, 0.2, 0.2, 0.2);
	glPushMatrix();
	glScaled(0.7, 1, 0.7);
	DrawCylinder3(50, 0.2, 0.2, 0.2);
	glPopMatrix();

	DrawTyer(50, 1, 0.7, 0.4, 0.4, 0.4, 1);
	DrawTyer(20, 0.7, 0, 0.8, 0.8, 0.8, 2);
	glPushMatrix();
	glTranslated(0, 1, 0);
	DrawTyer(50, 1, 0.7, 0.4, 0.4, 0.4, 1);
	DrawTyer(20, 0.7, 0, 0.8, 0.8, 0.8, 2);
	glPopMatrix();
	glPopMatrix();

}

// compute the direction and update car center
void Train::Move(double ground[GSZ][GSZ])
{
	int row = 0, col = 0;
	if (fabs(dirz) < 0.01 && fabs(dirx) > 0.99) // if the direction is along X axis
	{
		// find column and row of cell in ground
		row = (int)(cz + GSZ / 2);
		col = (int)(cx + GSZ / 2);
		if (dirx > 0) // moving to the right
		{
			if (col >= GSZ - 1) // if the car goes out of matrix return it back to the start
			{
				col = 0;
				cx = -GSZ / 2;
				if (ground[row][col] > 0) cy = ground[row][col];
				else cy = 1;
			}
			if (ground[row][col] > 0) diry = ground[row][col + 1] - ground[row][col];
			else diry = 0;
		}
		else // moving to the left
		{
			if (col < 0)
			{
				col = GSZ - 2;
				cx = GSZ / 2;
				if (ground[row][col + 1] > 0) cy = ground[row][col + 1];
				else cy = 1;
			}
			if (ground[row][col + 1] > 0)
				diry = ground[row][col] - ground[row][col + 1];
			else
			{
				diry = 0;
				cy = 1;
			}
		}
	}
	// update center of a car
	cx += dirx * speed;
	cy += diry * speed;
	cz += dirz * speed;
	int tmp_col;
	tmp_col = (int)(cx + GSZ / 2);
	if (tmp_col != col && tmp_col < GSZ) // update cy
	{
		if (ground[row][tmp_col] > 0) cy = ground[row][tmp_col];
		else cy = 1;

	}

}

void Train::SetSpeed(double s)
{
	speed = s;
}

double Train::GetCx()
{
	return cx;
}

double Train::GetCy()
{
	return cy;
}

double Train::GetCz()
{
	return cz;
}

double Train::GetDirx()
{
	return dirx;
}

double Train::GetDiry()
{
	return diry;
}

double Train::GetDirz()
{
	return dirz;
}


void Train::Draw(double offset, bool isLoco)
{
	double beta;
	glPushMatrix();
	glTranslated(cx, cy + 0.2, cz); // position the car in its center (we lift the car by 0.2 to put wheels on road)
	// only when the motion is along X axis
	beta = atan(diry); // in rad
	glRotated(beta * 180 / PI, 0, 0, 1);
	glScaled(0.05, 0.05, 0.05);

	if (isLoco) {
		//first wheel
		glPushMatrix();
		glTranslated(5, 0, 2.5);
		glRotated(90, 1, 0, 0);
		DrawWheel(offset);
		glPopMatrix();

		//parallel first wheel
		glPushMatrix();
		glTranslated(5, 0, -3.5);
		glRotated(90, 1, 0, 0);
		DrawWheel(offset);
		glPopMatrix();

		//sceond wheel
		glPushMatrix();
		glTranslated(0, 0, 2.5);
		glRotated(90, 1, 0, 0);
		DrawWheel(offset);
		glPopMatrix();

		//parallel second wheel
		glPushMatrix();
		glTranslated(0, 0, -3.5);
		glRotated(90, 1, 0, 0);
		DrawWheel(offset);
		glPopMatrix();

		//third wheel
		glPushMatrix();
		glTranslated(-5, 0, 2.5);
		glRotated(90, 1, 0, 0);
		DrawWheel(offset);
		glPopMatrix();

		//parallel third wheel
		glPushMatrix();
		glTranslated(-5, 0, -3.5);
		glRotated(90, 1, 0, 0);
		DrawWheel(offset);
		glPopMatrix();



		// body
		glPushMatrix();
		glTranslated(0, 1, 0);
		glRotated(90, 0, 0, 1);
		glScaled(1.8, 1, 4);
		glRotated(45, 0, 1, 0);
		glTranslated(0, -7.5, 0);
		glScaled(1, 15, 1);
		DrawCylinder3(4, 0.6, 0.6, 0.5);
		glPopMatrix();




		// windows
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glColor4d(0.6, 0.3, 0.0, 0.5);
		// right side
		glBegin(GL_POLYGON);
		glVertex3d(-5, 5, 3);
		glVertex3d(0, 5, 3);
		glVertex3d(0, 2, 3);
		glVertex3d(-5, 2, 3);
		glEnd();

		// left side
		glBegin(GL_POLYGON);
		glVertex3d(-5, 5, -3);
		glVertex3d(0, 5, -3);
		glVertex3d(0, 2, -3);
		glVertex3d(-5, 2, -3);
		glEnd();

		glDisable(GL_BLEND);
		//drawfront body
		glPushMatrix();
		glTranslated(0, 2, 0);
		glScaled(2, 4, 4.1);
		glRotated(45, 0, 1, 0);
		DrawCylinder3(4, 0.6, 0.6, 0.5);
		glPopMatrix();

		//draw backbody
		glPushMatrix();
		glTranslated(-6, 2, 0);
		glScaled(2, 4, 4.1);
		glRotated(45, 0, 1, 0);
		DrawCylinder3(4, 0.6, 0.6, 0.5);
		glPopMatrix();

		glPushMatrix();
		glTranslated(-3, 5, 0);
		glScaled(6, 2, 4.1);
		glRotated(45, 0, 1, 0);
		DrawCylinder3(4, 0.6, 0.6, 0.5);
		glPopMatrix();

		glPushMatrix();
		glTranslated(-8, 0, 0);
		glScaled(1, 1, 4.1);
		glRotated(45, 0, 1, 0);
		DrawCylinder3(4, 0.6, 0.6, 0.5);
		glPopMatrix();

		//roof
		glPushMatrix();
		glTranslated(0, 7, 0);
		glRotated(90, 0, 0, 1);
		glScaled(0, 1.5, 4.2);
		glRotated(45, 0, 1, 0);
		glTranslated(0, -0.5, 0);
		glScaled(1, 5, 1);
		DrawCylinder3(4, 0.2, 0.2, 0.2);
		glPopMatrix();

		glPushMatrix();
		glTranslated(7.3, 4, 0);
		glRotated(90, 0, 0, 1);
		glScaled(1.5, 10, 1.5);
		DrawCylinder3(200, 0, 0, 0);
		glPopMatrix();

		glPushMatrix();
		glTranslated(4.5, 3, 0);
		glScaled(0.5, 5, 0.5);
		DrawCylinder3(200, 0, 0, 0);
		glPopMatrix();


		glPushMatrix();
		glTranslated(6, 4, 0);
		glScaled(0, 1.52, 1.52);
		DrawSphere(20, 20);
		glPopMatrix();

		//smoke

		glPushMatrix();
		glTranslated(4, 9.7, 0);
		glScaled(0.6, 0.6, 0.6);
		DrawSphere1(20, 20);
		glPopMatrix();

		//second
		glPushMatrix();
		glTranslated(3.8, 10.5, 0);
		glScaled(0.7, 0.7, 0.7);
		DrawSphere1(20, 20);
		glPopMatrix();

		//third
		glPushMatrix();
		glTranslated(3.8, 11.2, -0.5);
		glScaled(0.7, 0.7, 0.7);
		DrawSphere1(20, 20);
		glPopMatrix();

		//forth
		glPushMatrix();
		glTranslated(3.6, 11.5, 0);
		glScaled(0.8, 0.8, 0.8);
		DrawSphere1(20, 20);
		glPopMatrix();


		//fifth
		glPushMatrix();
		glTranslated(3.6, 11.6, -0.5);
		glScaled(0.8, 0.8, 0.8);
		DrawSphere1(20, 20);
		glPopMatrix();

		//six
		glPushMatrix();
		glTranslated(3.2, 10.6, 0);
		glScaled(0.8, 0.8, 0.8);
		DrawSphere1(20, 20);
		glPopMatrix();

		//six
		glPushMatrix();
		glTranslated(3.2, 10.6, -0.5);
		glScaled(0.8, 0.8, 0.8);
		DrawSphere1(20, 20);
		glPopMatrix();

		//seven
		glPushMatrix();
		glTranslated(2.8, 12, 0);
		glScaled(0.9, 0.9, 0.9);
		DrawSphere1(20, 20);
		glPopMatrix();


		//eight
		glPushMatrix();
		glTranslated(2.8, 11.6, -0.5);
		glScaled(0.9, 0.9, 0.9);
		DrawSphere1(20, 20);
		glPopMatrix();



		// front panel
		glColor3d(0.2, 0.2, 0.2);
		glBegin(GL_POLYGON);
		glVertex3d(7.5, 2.3, -2.9);
		glVertex3d(7.5, 2.3, 2.9);
		glVertex3d(7.5, -0.1, 2.9);
		glVertex3d(7.5, -0.1, -2.9);
		glEnd();

		// front lights
		glPushMatrix();
		glTranslated(7.5, 0.3, -2.2);
		glScaled(0.1, 0.3, 0.5);
		DrawCylinder3(20, 1, 1, 0.9);
		glPopMatrix();

		glPushMatrix();
		glTranslated(7.5, 0.3, 2.2);
		glScaled(0.1, 0.3, 0.5);
		DrawCylinder3(20, 1, 1, 0.9);
		glPopMatrix();

		// rear panel
		glColor3d(0.2, 0.2, 0.2);
		glBegin(GL_POLYGON);
		glVertex3d(-7.5, 3.0, -2.9);
		glVertex3d(-7.5, 3.0, 2.9);
		glVertex3d(-7.5, -0.1, 2.9);
		glVertex3d(-7.5, -0.1, -2.9);
		glEnd();


		//for back roof
		glPushMatrix();
		glTranslated(-8, 1, 0);
		glRotated(90, 0, 0, 1);
		glScaled(0, 0.15, 4.1);
		glRotated(45, 0, 1, 0);
		glTranslated(0, -5, 0);
		glScaled(1, 10, 1);
		DrawCylinder3(4, 0.2, 0.2, 0.2);
		glPopMatrix();
	}else{

		glPushMatrix();
	glTranslated(5, 0, 2.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();

	glPushMatrix();
	glTranslated(5, 0, -3.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();



	glPushMatrix();
	glTranslated(2.5, 0, 2.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();

	glPushMatrix();
	glTranslated(2.5, 0, -3.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();


	glPushMatrix();
	glTranslated(0, 0, 2.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, 0, -3.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();

	glPushMatrix();
	glTranslated(-2.5, 0, 2.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();

	glPushMatrix();
	glTranslated(-2.5, 0, -3.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();

	glPushMatrix();
	glTranslated(-5, 0, 2.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();

	glPushMatrix();
	glTranslated(-5, 0, -3.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();



	// body


	glPushMatrix();
	glTranslated(0, 1, 0);
	glRotated(90, 0, 0, 1);
	glScaled(1.8, 1, 4);
	glRotated(45, 0, 1, 0);
	glTranslated(0, -7.5, 0);
	glScaled(1, 15, 1);
	DrawCylinder3(4, 0.6, 0.6, 0.5);
	glPopMatrix();




	// windows
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4d(0.6, 0.3, 0.0, 0.5);
	// right side
	glBegin(GL_POLYGON);
	glVertex3d(-5, 5, 3);
	glVertex3d(5, 5, 3);
	glVertex3d(5, 2, 3);
	glVertex3d(-5, 2, 3);
	glEnd();
	// left side
	glBegin(GL_POLYGON);
	glVertex3d(-5, 5, -3);
	glVertex3d(5, 5, -3);
	glVertex3d(5, 2, -3);
	glVertex3d(-5, 2, -3);
	glEnd();

	glDisable(GL_BLEND);

	glPushMatrix();
	glTranslated(6, 2, 0);
	glScaled(2, 4, 4.1);
	glRotated(45, 0, 1, 0);
	DrawCylinder3(4, 0.6, 0.6, 0.5);
	glPopMatrix();

	glPushMatrix();
	glTranslated(-6, 2, 0);
	glScaled(2, 4, 4.1);
	glRotated(45, 0, 1, 0);
	DrawCylinder3(4, 0.6, 0.6, 0.5);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, 5, 0);
	glScaled(10.5, 2, 4.1);
	glRotated(45, 0, 1, 0);
	DrawCylinder3(4, 0.6, 0.6, 0.5);
	glPopMatrix();

	glPushMatrix();
	glTranslated(-8, 0, 0);
	glScaled(1, 1, 4.1);
	glRotated(45, 0, 1, 0);
	DrawCylinder3(4, 0.6, 0.6, 0.5);
	glPopMatrix();

	//roof
	glPushMatrix();
	glTranslated(0, 7, 0);
	glRotated(90, 0, 0, 1);
	glScaled(0, 1.5, 4.2);
	glRotated(45, 0, 1, 0);
	glTranslated(0, -5, 0);
	glScaled(1, 10, 1);
	DrawCylinder3(4, 0.2, 0.2, 0.2);
	glPopMatrix();

	// front panel
	glColor3d(0.2, 0.2, 0.2);
	glBegin(GL_POLYGON);
	glVertex3d(7.5, 3.0, -2.9);
	glVertex3d(7.5, 3.0, 2.9);
	glVertex3d(7.5, -0.1, 2.9);
	glVertex3d(7.5, -0.1, -2.9);
	glEnd();

	// front lights
	glPushMatrix();
	glTranslated(7.5, 0.3, -2.2);
	glScaled(0.1, 0.3, 0.5);
	DrawCylinder3(20, 1, 1, 0.9);
	glPopMatrix();

	glPushMatrix();
	glTranslated(7.5, 0.3, 2.2);
	glScaled(0.1, 0.3, 0.5);
	DrawCylinder3(20, 1, 1, 0.9);
	glPopMatrix();

	// rear panel
	glColor3d(0.2, 0.2, 0.2);
	glBegin(GL_POLYGON);
	glVertex3d(-7.5, 3.0, -2.9);
	glVertex3d(-7.5, 3.0, 2.9);
	glVertex3d(-7.5, -0.1, 2.9);
	glVertex3d(-7.5, -0.1, -2.9);
	glEnd();


	//for back roof
	glPushMatrix();
	glTranslated(-8, 1, 0);
	glRotated(90, 0, 0, 1);
	glScaled(0, 0.15, 4.1);
	glRotated(45, 0, 1, 0);
	glTranslated(0, -5, 0);
	glScaled(1, 10, 1);
	DrawCylinder3(4, 0.2, 0.2, 0.2);
	glPopMatrix();
	}
glPopMatrix();
}

void Train::DrawLocomotive(double offset)
{
	double beta;
	glPushMatrix();
	glTranslated(cx, cy + 0.2, cz); // position the car in its center (we lift the car by 0.2 to put wheels on road)
	// only when the motion is along X axis
	beta = atan(diry); // in rad
	glRotated(beta * 180 / PI, 0, 0, 1);
	glScaled(0.05, 0.05, 0.05);


	//first wheel
	glPushMatrix();
	glTranslated(5, 0, 2.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();

	//parallel first wheel
	glPushMatrix();
	glTranslated(5, 0, -3.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();

	//sceond wheel
	glPushMatrix();
	glTranslated(0, 0, 2.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();

	//parallel second wheel
	glPushMatrix();
	glTranslated(0, 0, -3.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();

	//third wheel
	glPushMatrix();
	glTranslated(-5, 0, 2.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();

	//parallel third wheel
	glPushMatrix();
	glTranslated(-5, 0, -3.5);
	glRotated(90, 1, 0, 0);
	DrawWheel(offset);
	glPopMatrix();



	// body
	glPushMatrix();
	glTranslated(0, 1, 0);
	glRotated(90, 0, 0, 1);
	glScaled(1.8, 1, 4);
	glRotated(45, 0, 1, 0);
	glTranslated(0, -7.5, 0);
	glScaled(1, 15, 1);
	DrawCylinder3(4, r, g, b);
	glPopMatrix();




	// windows
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4d(0.6, 0.3, 0.0, 0.5);
	// right side
	glBegin(GL_POLYGON);
	glVertex3d(-5, 5, 3);
	glVertex3d(0, 5, 3);
	glVertex3d(0, 2, 3);
	glVertex3d(-5, 2, 3);
	glEnd();

	// left side
	glBegin(GL_POLYGON);
	glVertex3d(-5, 5, -3);
	glVertex3d(0, 5, -3);
	glVertex3d(0, 2, -3);
	glVertex3d(-5, 2, -3);
	glEnd();

	glDisable(GL_BLEND);
	//drawfront body
	glPushMatrix();
	glTranslated(0, 2, 0);
	glScaled(2, 4, 4.1);
	glRotated(45, 0, 1, 0);
	DrawCylinder3(4, r, g, b);
	glPopMatrix();

	//draw backbody
	glPushMatrix();
	glTranslated(-6, 2, 0);
	glScaled(2, 4, 4.1);
	glRotated(45, 0, 1, 0);
	DrawCylinder3(4, r, g, b);
	glPopMatrix();

	glPushMatrix();
	glTranslated(-3, 5, 0);
	glScaled(6, 2, 4.1);
	glRotated(45, 0, 1, 0);
	DrawCylinder3(4, r, g, b);
	glPopMatrix();

	glPushMatrix();
	glTranslated(-8, 0, 0);
	glScaled(1, 1, 4.1);
	glRotated(45, 0, 1, 0);
	DrawCylinder3(4, r, g, b);
	glPopMatrix();

	//roof
	glPushMatrix();
	glTranslated(0, 7, 0);
	glRotated(90, 0, 0, 1);
	glScaled(0, 1.5, 4.2);
	glRotated(45, 0, 1, 0);
	glTranslated(0, -0.5, 0);
	glScaled(1, 5, 1);
	DrawCylinder3(4, r * 0.8, g * 0.8, b * 0.8);
	glPopMatrix();

	glPushMatrix();
	glTranslated(7.3, 4, 0);
	glRotated(90, 0, 0, 1);
	glScaled(1.5, 10, 1.5);
	//DrawCylinder(200);
	DrawCylinder3(200, 0, 0, 0);
	glPopMatrix();

	glPushMatrix();
	glTranslated(4.5, 3, 0);
	glScaled(0.5, 5, 0.5);
	DrawCylinder3(200, 0, 0, 0);
	glPopMatrix();


	glPushMatrix();
	glTranslated(6, 4, 0);
	glScaled(0, 1.52, 1.52);
	//DrawCylinder(200);
	DrawSphere(20, 20);
	glPopMatrix();

	//smoke

	glPushMatrix();
	glTranslated(4, 9.7, 0);
	glScaled(0.6, 0.6, 0.6);
	DrawSphere1(20, 20);
	glPopMatrix();

	//second
	glPushMatrix();
	glTranslated(3.8, 10.5, 0);
	glScaled(0.7, 0.7, 0.7);
	DrawSphere1(20, 20);
	glPopMatrix();

	//third
	glPushMatrix();
	glTranslated(3.8, 11.2, -0.5);
	glScaled(0.7, 0.7, 0.7);
	DrawSphere1(20, 20);
	glPopMatrix();

	//forth
	glPushMatrix();
	glTranslated(3.6, 11.5, 0);
	glScaled(0.8, 0.8, 0.8);
	DrawSphere1(20, 20);
	glPopMatrix();


	//fifth
	glPushMatrix();
	glTranslated(3.6, 11.6, -0.5);
	glScaled(0.8, 0.8, 0.8);
	DrawSphere1(20, 20);
	glPopMatrix();

	//six
	glPushMatrix();
	glTranslated(3.2, 10.6, 0);
	glScaled(0.8, 0.8, 0.8);
	DrawSphere1(20, 20);
	glPopMatrix();

	//six
	glPushMatrix();
	glTranslated(3.2, 10.6, -0.5);
	glScaled(0.8, 0.8, 0.8);
	DrawSphere1(20, 20);
	glPopMatrix();

	//seven
	glPushMatrix();
	glTranslated(2.8, 12, 0);
	glScaled(0.9, 0.9, 0.9);
	DrawSphere1(20, 20);
	glPopMatrix();


	//eight
	glPushMatrix();
	glTranslated(2.8, 11.6, -0.5);
	glScaled(0.9, 0.9, 0.9);
	DrawSphere1(20, 20);
	glPopMatrix();



	// front panel
	glColor3d(r * 0.8, g * 0.8, b * 0.8);
	glBegin(GL_POLYGON);
	glVertex3d(7.5, 2.3, -2.9);
	glVertex3d(7.5, 2.3, 2.9);
	glVertex3d(7.5, -0.1, 2.9);
	glVertex3d(7.5, -0.1, -2.9);
	glEnd();

	// front lights
	glPushMatrix();
	glTranslated(7.5, 0.3, -2.2);
	glScaled(0.1, 0.3, 0.5);
	DrawCylinder3(20, 1, 1, 0.9);
	glPopMatrix();

	glPushMatrix();
	glTranslated(7.5, 0.3, 2.2);
	glScaled(0.1, 0.3, 0.5);
	DrawCylinder3(20, 1, 1, 0.9);
	glPopMatrix();

	// rear panel
	glColor3d(r * 0.8, g * 0.8, b * 0.8);
	glBegin(GL_POLYGON);
	glVertex3d(-7.5, 3.0, -2.9);
	glVertex3d(-7.5, 3.0, 2.9);
	glVertex3d(-7.5, -0.1, 2.9);
	glVertex3d(-7.5, -0.1, -2.9);
	glEnd();


	//for back roof
	glPushMatrix();
	glTranslated(-8, 1, 0);
	glRotated(90, 0, 0, 1);
	glScaled(0, 0.15, 4.1);
	glRotated(45, 0, 1, 0);
	glTranslated(0, -5, 0);
	glScaled(1, 10, 1);
	DrawCylinder3(4, r * 0.8, g * 0.8, b * 0.8);
	glPopMatrix();



	glPopMatrix();
}

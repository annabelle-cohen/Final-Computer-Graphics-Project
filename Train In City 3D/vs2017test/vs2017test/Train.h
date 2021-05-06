#pragma once
const double PI = 3.14;
const int GSZ = 150;

class Train
{
private:
	double r, g, b;
	double cx, cy,cz; // center in world coordinates
	double dirx, diry, dirz;
	double speed;
public:
	Train(double r, double g, double b, double x, double y, double z, double dx, double dy, double dz, double speed);
	void DrawTyer(int n, double outer, double inner, double r, double g, double b, int jump);
	void DrawCylinder3(int n, double r, double g, double b);
	void DrawSphere(int n, int sl);
	void DrawCylinder1(int n, double topr, double bottomr);
	void DrawCylinder5(int n, double topr, double bottomr);
	void DrawSphere1(int n, int sl);
	void DrawWheel(double offset);
	void Draw(double offset,bool isLoco);
	void DrawLocomotive(double offset);
	void Move(double ground[GSZ][GSZ]);
	void SetSpeed(double s);
	double GetCx();
	double GetCy();
	double GetCz();
	double GetDirx();
	double GetDiry();
	double GetDirz();

};


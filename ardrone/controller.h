struct input {
	float th;
	float ph;
	float ps;
	float alt;
};
struct super_twisting {
	float uth;	// Inputs
	float uph;
	float ups;
	float K1;
	float K2;
	float thd;	// x deseada
	float phd;
	float psd;
	float thpd;	// x punto deseada
	float phpd;
	float pspd;
	float th;	// Valor de x
	float ph;
	float ps;
	float eth;	// Error de x
	float eph;
	float eps;
	float epth;	// Error de x punto
	float epph;
	float epps;
	float lambda;
	float sth;
	float sph;
	float sps;
	float intsgnth;	// Valor de la integral
	float intsgnph;
	float intsgnps;
	float antth;	// Valor anterior de la integral
	float antph;
	float antps;
	input u;
	double dt;	// Diferencia de tiempo
	double ta;	// Tiempo anterior
};
struct PID {
	float ualt;	// Input
	float kp;	// Constantes
	float ki;
	float kd;
	float I;
	float xd;	// x deseada
	float eant;	// Valor anterior del error
	input u;
	double dt;	// Diferencia de tiempo
	double ta;	// Tiempo anterior
};
void updateatt_u(super_twisting *c, float th, float thp, float ph, float php, float ps, float psp);
void updatealt_u(PID *c, float alt);
int init_controller(super_twisting *st, PID *pid);
float sign(float x);

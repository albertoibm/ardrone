/*
 * ATRAS | ADELANTE, Theta +|-
 * IZQUIERDA | DERECHA, Phi +|-
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include "util/type.h"
#include "util/util.h"
#include "attitude/attitude.h"
#include "controller.h"

float sign(float x)
{
	if(x<0) return -1;
	else if(x==0) return 0;
	else return 1;
}
void updateatt_u(super_twisting *c, float th, float thp, float ph, float php, float ps, float psp)
{
	float tmp; //thp, php, psp;
  float l = 0.13, Ixx = 24.1e-3, Iyy = 23.2e-3, Izz = 45.1e-2;
  float b = 0.0006646195542576290, d = b*9.72;
  float ginvth = Ixx/l, ginvph = Iyy/l, ginvps = Izz;
	c->dt = util_timestamp() - c->ta;
	c->ta = util_timestamp();
	c->th = th;
	c->ph = ph;
	c->ps = ps;
	c->eth = c->thd - c->th;
	c->eph = c->phd - c->ph;
	c->eps = c->psd - c->ps;
	c->epth = c->thpd - thp;
	c->epph = c->phpd - php;
	c->epps = c->pspd - psp;

	tmp = c->epth + c->lambda*c->eth;	// s
	c->intsgnth += c->dt * (sign(tmp) + sign(c->sth))/2; // Integrate
	c->sth = tmp;
  tmp = c->epph + c->lambda*c->eph;	// s
  c->intsgnph += c->dt * (sign(tmp) + sign(c->sph))/2; // Integrate
  c->sph = tmp;
  tmp = c->epps + c->lambda*c->eps;	// s
  c->intsgnps += c->dt * (sign(tmp) + sign(c->sps))/2; // Integrate
  c->sps = tmp;
  
  // Debug
  printf("e= %.3f %.3f %.3f\n",c->eth, c->eph, c->eps); //Imprime el error
	printf("\nxp= %.3f %.3f %.3f\n",thp, php, psp); // Imprime la derivada
	printf("s= %f %f %f\n",c->sth, c->sph, c->sps); // Imprime s
	printf("intsgn= %f %f %f\n",c->intsgnth, c->intsgnph, c->intsgnps);
	printf("lmda*ep= %f %f %f\n",c->lambda*c->epth, c->lambda*c->epph, c->lambda*c->epps);
	printf("K2·intsignps= %.3f\n",c->K2*c->intsgnps);
	printf("K1·sqrt(s)·sign(s)= %.3f",c->K1*sqrt(fabs(c->sth))*sign(c->sth));
	printf(" %.3f",c->K1*sqrt(fabs(c->sph))*sign(c->sph));
	printf(" %.3f\n",c->K1*sqrt(abs(c->sps))*sign(c->sps));

	c->uth = c->lambda * c->epth - c->K1 * sqrt(fabs(c->sth)) * sign(c->sth) - c->K2 * c->intsgnth;
	printf("ug= %.3f",c->uth);
	c->uth = ginvth * c->uth;

  c->uph = c->lambda * c->epph - c->K1 * sqrt(fabs(c->sph)) * sign(c->sph) - c->K2 * c->intsgnph;
	printf(" %.3f",c->uph);
  c->uph = ginvph * c->uph;

  c->ups = c->lambda * c->epps - c->K1 * sqrt(fabs(c->sps)) * sign(c->sps) - c->K2 * c->intsgnps;
	printf(" %.3f\n",c->ups);
  c->ups = 0;//ginvps * c->ups;
	printf("u= %.3f %.3f %.3f\n",c->uth,c->uph,c->ups);

}
void updatealt_u(PID *c, float alt)
{
	float P, D, e;
  c->dt = util_timestamp() - c->ta;
  c->ta = util_timestamp();
	e = c->xd - alt;
	P = e;
  c->I += c->dt * (e + c->eant)/2; // Integrate
	D =  (e - c->eant)/c->dt;
	c->eant = e;
	c->ualt = c->kp * P + c->ki * c->I + c->kd * D;
	if(c->ualt < 0) c->ualt = 0;
}
int init_controller(super_twisting *st, PID *pid)
{
	st->uth = 0;
	st->uph = 0;
	st->ups = 0;
	st->K1 = 4.5 * 12;
	st->K2 = 1.1 * 12;
	st->th = 0;
	st->ph = 0;
	st->ps = 0;
	st->thd = 0;
	st->phd = 0;
	st->psd = 0;
	st->thpd = 0;
	st->phpd = 0;
	st->pspd = 0;
	st->lambda = 3;
	st->intsgnth = 0;
	st->intsgnph = 0;
	st->intsgnps = 0;
	st->antth = 0;
	st->antph = 0;
	st->antps = 0;
	st->dt = 1;
	st->ta = util_timestamp();
	pid->ualt = 0;
	pid->kp = 80;
	pid->ki = 10;
	pid->kd = 10;
	pid->I = 0;
	pid->xd = 30;
	pid->eant = 0;
	pid->dt = 1;
	pid->ta = st->ta;
	return 0;
}

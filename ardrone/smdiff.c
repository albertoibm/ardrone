#include <stdio.h>  //printf
#include <stdlib.h>  //malloc
#include <math.h>  //pow
#include "smdiff.h"
#include "controller.h" //sign

float potencia(int n)
{
	return n/(n+1.0);
}
void init_DiffOrdN(DiffOrdN *self, int n, float *gains)
{
	int i;
	self->orden = n;
	self->diffs = (Diff*) malloc(n * 5 * sizeof(float));
	for(i=0; i < n + 1; i++)
	{
		Diff tmp;
		tmp.orden = i;
		tmp.gain = gains[n-i];
		tmp.integral = 0;
		tmp.preintval = 0;
		self->diffs[i] = tmp;
	}
	printf("Derivador de orden %d creado\n",n);
}
int update_DiffOrdN(DiffOrdN *self, float val, double dt)
{
	int i;
	for(i = self->orden; i >= 0; i--)
	{
		float zm1;
		if(i == 0) zm1 = 0;
		else zm1 = self->diffs[i-1].integral;
		val = f_Diff(&self->diffs[i], val, zm1);
		self->diffs[i].integral += (self->diffs[i].tmp + val)/2 * dt;
		self->diffs[i].tmp = val;
	}
	return 0;
}
float f_Diff(Diff *self, float val, float zm1)
{
	float v,e,pot,abse,sig;
	e = self->integral - val;
	abse = fabs(e);
	sig = sign(e);
	pot = pow(abse,potencia(self->orden));
	v = -sig*self->gain * pot;
	return v + zm1;
}

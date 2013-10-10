/*
 *  * ATRAS | ADELANTE, Theta +|-
 *   * IZQUIERDA | DERECHA, Phi +|-
 *    */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include "util/type.h"
#include "util/util.h"
#include "attitude/attitude.h"
#include "motorboard/mot.h"
#include "controller.h"
#include "smdiff.h"

int sleep(double s)
{
	double t0 = util_timestamp();
	while( util_timestamp() - t0 < s);
	return 0;
}
int main(int argc, char *argv[])
{
	att_struct att;
	double t0, dt, ta;
	float alt, th, ph, ps;	// Valores de X
	float thp, php, psp;	// Valores de Xp
	// Parametros del cuadricoptero
	float l = 0.13, Ixx = 24.1e-3, Iyy = 23.2e-3, Izz = 45.1e-2;
	float b = 0.0006646195542576290, d = b*9.72;
	float ginvth = Ixx/l, ginvph = Iyy/l, ginvps = Izz;

	float factor;
	int om1, om2, om3, om4;
	int ttotal;
	int rc,i;
	super_twisting st;
	PID pid;
	DiffOrdN filtroth, filtroph;
	float gains[] = {9.5, 8, 7.5, 5, 1.5, 1};
	float N = 5;
	printf("Iniciando derivadores numericos...\n");
	init_DiffOrdN(&filtroth, N, &gains[0]);
	init_DiffOrdN(&filtroph, N, &gains[0]);
	for( i = 0 ; i < N +1; i++){
		printf("Th: Orden %d. Ganancia: %f. Integral: %f\n",i,filtroth.diffs[i].gain,filtroth.diffs[i].integral);
		printf("Ph: Orden %d. Ganancia: %f. Integral: %f\n",i,filtroph.diffs[i].gain,filtroph.diffs[i].integral);
	}
	printf(" Listo!\n");
	printf("Iniciando el controlador...");
	rc = init_controller(&st, &pid);
	if(rc) return rc;
	printf("Listo!\n");
	if( argc < 3)
	{
		printf("Es necesario ingresar el tiempo de vuelo y el factor de multiplicacion\n");
		return 0;
	}
	ttotal = atoi(argv[1]);	// Tiempo total de vuelo
	factor = atoi(argv[2])/10.0; // Factor que para pruebas de vuelo
	printf("Prueba de vuelo de %d segundos\n", ttotal);
	printf("Iniciando lector de actitud...");
	rc = att_Init(&att);
	if(rc) return rc;
	printf("Listo!\n");
	
	printf("Iniciando la escritura en los motores...");
	mot_Init();
	printf(" Listo!\n");
	printf("Encendiendo los motores...");
	mot_Run(.01, .01, .01, .01);
	sleep(3.0);
	ta = t0 = util_timestamp();
	pthread_yield();
	printf("Iniciando ciclo de vuelo\n");
	while( util_timestamp() - t0 < ttotal )
	{
		dt = util_timestamp() -ta;
		ta = util_timestamp();
		rc = att_GetSample(&att);
		if (rc)
		{
			printf("Error al actualizar los datos de actitud\n");
			break;
		}
		alt = att.h;
		ph = att.roll;
		th = att.pitch;
		ps = 0;//att.yaw;
		update_DiffOrdN( &filtroth, th, dt);
		update_DiffOrdN( &filtroph, ph, dt);
		th = filtroth.diffs[(int)N].integral;
		ph = filtroph.diffs[(int)N].integral;
		thp = filtroth.diffs[(int)N-1].integral;
		php = filtroph.diffs[(int)N-1].integral;
		updateatt_u(&st, th, thp, ph, php, ps, 0);
		updatealt_u(&pid, alt);
		om1 = om2 = om3 = om4 = 0;
		//SOLVE EQUATIONS
		om1 = (int)(sqrt(-(b*st.ups + d*st.uph - d*pid.ualt - d*st.uth)/(4*b*d)*factor)/2);
		om2 = (int)(sqrt((d*st.uph + d*st.uth + b*st.ups + d*pid.ualt)/(4*b*d)*factor)/2);
		om3 = (int)(sqrt(-(d*st.uth + b*st.ups - d*pid.ualt - d*st.uph)/(4*b*d)*factor)/2);
		om4 = (int)(sqrt((d*pid.ualt + b*st.ups - d*st.uph - d*st.uth)/(4*b*d)*factor)/2);
		if( om1 < 5) om1 += 5;
		if( om2 < 5) om2 += 5;
		if( om3 < 5) om3 += 5;
		if( om4 < 5) om4 += 5;
		//OUTPUT TO MOTORS
		mot_Run(om1/256.0, om2/256.0, om3/256.0, om4/256.0);
		printf("uo= %.2f %.2f %.2f %.2f\nom= %d %d %d %d\ndt= %.4f\n", pid.ualt, st.uph, st.uth, st.ups, om1, om2, om3, om4, dt);
		pthread_yield();
	}
	printf("Tiempo de vuelo terminado.\n");
	printf("Deteniendo motores y cerrando el manejador de motores y navegacion\n");
	mot_Stop();
	mot_Close();
	att_Close();
	printf("Cerrado\n");
}

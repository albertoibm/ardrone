struct DiffOrdN {
	int orden;
	struct Diff *diffs;
};
struct Diff {
	int orden;
	float integral;
	float gain;
	float tmp;
	float preintval;
};
float potencia(int n);
void init_DiffOrdN(struct DiffOrdN *self, int n, float *gains);
int update_DiffOrdN(struct DiffOrdN *self, float val, double dt);
float f_Diff(struct Diff *self, float val, float zm1);

#define DEGR_MAX 360

typedef struct degree {
	int16_t i;
	int8_t frac;
	int8_t denom;
} degree;

void degr_sub(degree n1, degree n2, degree *r) {
	if(n1.denom < n2.denom) {
		n1.frac = n2.denom / n1.denom * n1.frac;
		r->denom = n2.denom;
	} else if (n2.denom < n1.denom) {
		n2.frac = n1.denom / n2.denom * n2.frac;
		r->denom = n1.denom;
	} else {
		r->denom = n1.denom;
	}

	r->frac = n1.frac - n2.frac;
	r->i = 0;

	// Normalise values
	if(r->frac < 0) {
		r->frac = r->denom + r->frac;
		r->i--;
	}
	r->i = n1.i - n2.i;
	if(r->i < 0)
		r->i = DEGR_MAX + r->i;
}

uint32_t degr_to_step(degree d) {
	uint32_t steps_i = d.i * steps_per_degr;
	uint32_t steps_f = (d.frac * steps_per_degr) / d.denom;
	return (steps_i + steps_f) >> 1;
}

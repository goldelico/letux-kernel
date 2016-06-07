#ifndef PWR_GENERIC_FUEL_GAUSE_H
#define PWR_GENERIC_FUEL_GAUSE_H

/* calculate remaining fuel level (in %) of a LiIon battery assuming
 * a standard chemistry model
 *    http://cseweb.ucsd.edu/~trosing/lectures/battery.pdf
 *
 * @mV: voltage measured outside the battery
 * @mA: current flowing out of the battery
 * @mOhm: assumed series resitance of the battery
 *
 * returns value between 0 and 100
 */
static inline int fuel_level_LiIon(int mV, int mA, int mOhm) {
	int u;

	/* internal battery voltage is higher than measured when discharging */
	mV += (mOhm * mA) /1000;

	/* apply formula */
	u = 3870000 - (14523 * (37835 - 10 * mV));
	/* use linear approx. below 3.756V / 19.66% assuming 3.3V / 0% */
	if (u < 0) {
		return  max(((mV - 3300) * ((3756 - 3300) * 1966)) / 100000, 0);
	}
	return min((int)(1966 + int_sqrt(u))/100, 100);	
}

#endif /* PWR_GENERIC_FUEL_GAUSE_H */

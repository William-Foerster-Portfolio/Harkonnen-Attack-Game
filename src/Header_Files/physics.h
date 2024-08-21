#ifndef TESTS_H
#define TESTS_H

#include "app.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>


int calc_velocity(int v0, int force, int mass, float time);
int gravity(int v0, int grav, float time);
int calc_position(int x0, int velocity0, float time);
long long int calc_kenetic_energy(int mass, int velocity);
int bounce(float kenetic_change, int mass, int velocity0);
float ball_meet_plat(int plat, int plat_v, int hm_x, int hm_y, int plat_height);


#endif  // TESTS_H

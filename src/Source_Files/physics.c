#include "physics.h"

int calc_velocity(int v0, int force, int mass, float time)
{
    int velocity = v0 + ((force/mass)*time);
    return velocity;
}
int gravity(int v0, int grav, float time)
{
    int velocity = v0 + (grav*time);
    return velocity;
}

int calc_position(int x0, int velocity0, float time)
{
    int x = x0 + (velocity0 * time);
    return x;
}
int bounce(float kenetic_change, int mass, int velocity0)
{
    float kenetic = kenetic_change * calc_kenetic_energy(mass, velocity0);
    velocity0 = -1 * sqrt((2*kenetic)/mass);
    return velocity0;
}

long long int calc_kenetic_energy(int mass, int velocity)
{
   long long int KE = .5*mass*(velocity * velocity);
   return KE;
}

float ball_meet_plat(int plat, int plat_v, int hm_x, int hm_y, int plat_height)
 {
//   int y = 0;

//   int time_zero = (sqrt((hm_y)+((hm_vy/140)*(hm_vy/104)))/((hm_vy/140)*(hm_vy/140)));
//   int x = hm_x + (hm_vx * time_zero);
//   int velocity = ((x-plat)/time_zero);
//   float percent = velocity/70000;
  float a = 0;
//  float x_ratio = 0;
//  float y_ratio =
  float b = 0;
  float c = 0;
  float temp1 = 0;
  float temp2 = 0;
  if(plat <= hm_x)
    {
      if(plat_v < 0)
        {
          a = hm_x - plat;
          a = a/100;
          b = plat_height-hm_y;
          b = b/100;
          c = sqrt((a*a)+(b*b));
          c *= 2;
        }
      else
        {
          a = hm_x - plat;
          a = a/100;
          b = plat_height-hm_y;
          b = b/100;
          c = sqrt((a*a)+(b*b));
        }
    }
  else
      {
        if(plat_v > 0)
          {
            a = plat - hm_x;
            a = a/100;
            b = plat_height-hm_y;
            b = b/100;
            c = sqrt((a*a)+(b*b));
            c *= 2;
          }
        else
          {
            a = plat - hm_x;
            a = a/100;
            b = plat_height-hm_y;
            b = b/100;
            c = sqrt((a*a)+(b*b));
          }
      }
  c /= 1000;
  c = 1 - c;


   return c;
 }



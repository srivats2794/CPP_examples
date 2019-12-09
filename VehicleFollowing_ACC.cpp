/* A simple control example for Adaptive Cruise Control.
Ego vehicle is controlled using acceleration as input to maintain a safe desired distance. */
#include <iostream>

using namespace std;

class VehicleFollowing
{
  double x_ego, vx_ego, x_tgt, vx_tgt, ax_des;

  void distanceFollower(VehicleFollowing& s1)
  {
    double d_des= 1.5*s1.v_tgt;
    double v_rel= s1.v_tgt-s1.v_ego;
    double d= s1.x_tgt-s1.x_des;

    if (d==d_des || v_rel==0)
    {
      s1.ax_des=0;
    }
    else
    {
      s1.ax_des= v_rel-d+d_des;
    };
  };
 };

 int main()
 {
   //given x_tgt, vx_tgt, x_ego, vx_ego

   VehicleFollowing t1;
   double Tsim=10, Ts=0.1;
   int n= std::floor(Tsim/Ts);

   t1.x_tgt= x_tgt; t1.vx_tgt= vx_tgt; t1.x_ego= x_ego; t1.vx_ego= vx_ego;

   for (int i=0; i<=n; i++)
   {
     t1.distanceFollower(t1);
     t1.vx_ego= t1.vx_ego+(t1.ax_des*Ts);
     t1.x_ego= t1.x_ego+(t1.vx_ego*Ts);
     t1.x_tgt= t1.x_tgt+(t1.vx_tgt*Ts);
   };
   x_tgt = t1.x_tgt; vx_tgt = t1.vx_tgt; x_ego = t1.x_ego; vx_ego = t1.vx_ego;
   cout<<"x_tgt"<<x_tgt<<endl;
   cout<<"vx_tgt"<<vx_tgt<<endl;
   cout<<"x_ego"<<x_ego<<endl;
   cout<<"vx_ego"<<vx_ego<<endl;

   return 0;
 }

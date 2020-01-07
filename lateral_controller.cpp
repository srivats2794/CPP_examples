// A simple lateral controller example when given current vehicle coordinates, heading and path information

struct Point
{
  double x,y,theta;
}

class Path
{
public:

  Point closest_p(Point P)
  {
    //Returns closest point in desired Path
  }

  void get_theta(Point& P)
  {
    //returns theta at a certain point
  }

  double R(Point P)
  {
    //Returns Radius of curvature at Point p
  }
}

class controller
{
public:
  double L //vehicle length m;
  double V; //Vehicle constant speed m/s
  double control_input (Point current_p, Path P, controller c)
  {
    //Assuming fwd driving and front wheel drive and given position is for center of vehicle.
    double x,y;
    double x_w,y_w; //New world coordinates
    double phi;
    double k=1; //initial untuned control paramter
    c.L=1;c.V=1; // 1m wheel base, 1m/s speed

    x= current_p.x*cos(current_p.theta)+current_p.y*sin(current_p.theta);
    y= -current_p.x*sin(current_p.theta)+current_p.y*cos(current_p.theta);

    x= x+c.L/2;
    double theta= P.get_theta
    x_w= x*cos(current_p.theta)-y*sin(current_p.theta);
    y_w= x*sin(current_p.theta)+y*cos(current_p.theta);

    Point d_axle_p; // drive axle Point
    d_axle_p.x= x_w; d_axle_p.y= y_w;
    double theta= P.get_theta(d_axle_p);
    Point closest_p = P.closest_p(d_axle_p); //closest point to drive axle point
    double x_diff= x_w-closest_p.x;
    double y_diff= y_w-closest_p.y;
    double dist_error= sqrt((x_diff*xdiff)+(y_diff*y_diff));
    double heading_error= closest_p.theta-current_p.theta;
    phi= heading_error+atan2((k*dist_error)/c.V); // k/v gives the look ahead distance in the path
  }
}

/* A simple example for vehicle trajectory tracking.
Nonlinear simple kinematic model propogation using euler forward */

#include <iostream>
#include <cmath>
#include <vector>
using namespace std;

class vehicle_data
{
public:
	double L;//vehicle parameters
	double x0, y0, theta0; //initial states
	std::vector<double> X, Y, Theta;// trajectory
};

class time_data
{
public:
	double Tsim; //Simulation time
	double Ts; // Sample time
};

class inputs
{
public:
	double V;//velocity
	std::vector<double> phi;//steering angles
};

void vehicle_trajectory(vehicle_data& vehicle, inputs in, time_data time)
{

	int n = std::floor(time.Tsim / time.Ts);
	if (vehicle.L <= 0)
	{
		cout << "Wheelbase cannot be 0 or lesser";
		return;
	};

	for (int i = 0; i <= n; i++)
	{
		const double pi = 3.14159265358979323846;

		if (in.phi[i] >= 90 * (pi / 180))
		{
			in.phi[i] = 89 * (pi / 180);
		}
		else if (in.phi[i] <= -90 * (pi / 180))
		{
			in.phi[i] = -89 * (pi / 180);
		}
		double x_dot, y_dot, theta_dot;
		x_dot = in.V*cos(vehicle.theta0);
		y_dot = in.V*sin(vehicle.theta0);
		theta_dot = (in.V / vehicle.L)*tan(in.phi[i]);
		vehicle.X.push_back(i); vehicle.Y.push_back(i); vehicle.Theta.push_back(i);
		vehicle.X[i] = vehicle.x0; vehicle.Y[i] = vehicle.y0; vehicle.Theta[i] = vehicle.theta0;

		vehicle.x0 = x_dot * time.Ts + vehicle.x0; vehicle.y0 = y_dot * time.Ts + vehicle.y0; vehicle.theta0 = theta_dot * time.Ts + vehicle.theta0;
	}
}
int main()
{
	vehicle_data vehicle1; time_data t; inputs u1;
	vehicle1.L = 1; vehicle1.x0 = 2; vehicle1.y0 = -1; vehicle1.theta0 = 0;
	t.Ts = 0.001; t.Tsim = 2;
	u1.V = 10; int n = std::floor(t.Tsim / t.Ts); u1.phi.assign(n+1, 0);
	vehicle_trajectory(vehicle1, u1, t);
	cout << vehicle1.X[n]<<endl;
	cout << vehicle1.Y[n];
	cin.get();
	return 0;
}

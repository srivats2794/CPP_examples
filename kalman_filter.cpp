/* Kalman filter implementation in C++ example. System: Compound pendulum, 0 input, pi/4 starting position*/ 

#include <iostream>
#include <cmath>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <random>
using namespace std;
using namespace Eigen;

const double pi = 3.14159265358979323846;

class time_data
{
public:
	double Tsim; //Simulation time
	double Ts; // Sample time
};

class system_data
{
public:
	double g = 9.81, m, M, l, b,I; // parameters
	void I_calc(system_data& S1)
	{
		S1.I = ((S1.m*(pow(S1.l, 2))) / 3) + (S1.M*(pow(S1.l, 2)));
	}
	MatrixXd A, B, C, D, Identity;
	VectorXd x_t, y_t, x_t_dot;
	int u;
	double sigmap1, sigmap2,randp1, randp2;
	VectorXd noise_p;

	vector<VectorXd> X_t{x_t};
	void print_system(system_data S2)
	{
		cout << S2.A << endl;
		cout << S2.B << endl;
		cout << S2.C << endl;
		cout << S2.D << endl;
		cin.get();
	};

	void system_sim(system_data& S3, time_data t1)
	{
		std::random_device rd1, rd2;
		std::mt19937 gen1(rd1());
		std::mt19937 gen2(rd2());
		std::normal_distribution<double> d1(0, S3.sigmap1);
		std::normal_distribution<double> d2(0, S3.sigmap2);
		S3.randp1 = d1(gen1); S3.randp2 = d2(gen2);
		S3.noise_p = VectorXd(2, 1); 
		S3.noise_p << (S3.randp1*sqrt(t1.Ts)),
			          (S3.randp2*sqrt(t1.Ts));
		S3.x_t_dot = S3.A * S3.x_t + S3.B * S3.u;
		S3.y_t = S3.C * S3.x_t + S3.D * S3.u;
		S3.X_t.push_back(S3.x_t);
		S3.x_t = S3.x_t_dot*t1.Ts + S3.x_t + S3.noise_p;;
	};
};

class kalman_filter
{
public:
	MatrixXd P, Q, R, K_k;
	VectorXd x_hat, y_tilde, y_hat, y_diff;
	vector<VectorXd> X_hat{ x_hat };
	vector<VectorXd> Y_tilde{ y_tilde };
	double sigmam1, sigmam2, randm1, randm2;
	VectorXd noise_m;
	
	void estimator(system_data& s3, kalman_filter& k1, time_data t2)
	{
		std::random_device rd3, rd4;
		std::mt19937 gen3(rd3());
		std::mt19937 gen4(rd4());
		std::normal_distribution<double> d3(0, k1.sigmam1);
		std::normal_distribution<double> d4(0, k1.sigmam2);
		k1.randm1 = d3(gen3); k1.randm2 = d4(gen4);
		k1.noise_m= VectorXd(2, 1);
		k1.y_diff= VectorXd(2, 1);
		k1.noise_m << k1.randm1,
					  k1.randm2;
		k1.y_diff << 0,
					 0;
		k1.y_tilde = s3.y_t + k1.noise_m;

		k1.Y_tilde.push_back(k1.y_tilde);

		// Kalman gain calculation
		k1.K_k = (k1.P*s3.C.transpose())*((s3.C*k1.P*s3.C.transpose() + k1.R).inverse());

		// Estimation covariance propagation
		k1.P = s3.A*(k1.P*(s3.Identity-(s3.C*k1.K_k)))*s3.A.transpose() + k1.Q*t2.Ts;

		k1.X_hat.push_back(k1.x_hat);

		k1.y_diff = k1.y_tilde - k1.x_hat;

		k1.x_hat = (k1.K_k*k1.y_diff)+k1.x_hat;  
		k1.x_hat = (s3.A*k1.x_hat + s3.B*s3.u)*t2.Ts+k1.x_hat;
	};
};

int main()
{
	system_data s1; time_data t; kalman_filter k;
	s1.m = 0.095; s1.M = 0.380; s1.l = 0.43; s1.b = 0.003; t.Ts = 0.001; t.Tsim = 0.01;
	s1.sigmap1 = sqrt(0.001); s1.sigmap2 = sqrt(0.025); k.sigmam1 = sqrt(0.020); k.sigmam2 = sqrt(0.050);
	s1.I_calc(s1);
	s1.A = MatrixXd(2, 2);
	s1.A<< 0, 1,
		(-((s1.g*s1.l) / s1.I)*(s1.M + (s1.m / 2))), (-s1.b/s1.I);
	
	s1.B = MatrixXd(2, 1);
	s1.B<< 0,
		 (1 / s1.I);

	s1.C = Matrix2d::Identity();
	
	s1.D = MatrixXd(2, 1);
	s1.D << 0,
		    0 ;
	
	s1.x_t = VectorXd(2, 1);
	s1.x_t << pi/4,
		       0;
	s1.y_t = VectorXd(2, 1);
	s1.y_t << 0,
			  0;
	s1.u = 0;

	s1.x_t_dot = VectorXd(2, 1);
	s1.x_t_dot << 0,
			      0;

	k.P = MatrixXd(2, 2);
	k.P << (pow(pi / 4, 2)), 0,
					   0, 0;

	k.Q = MatrixXd(2, 2);
	k.Q << pow(s1.sigmap1, 2), 0,
		    0, pow(s1.sigmap2, 2);

	k.R = MatrixXd(2, 2);
	k.R << pow(k.sigmam1, 2), 0,
		   0, pow(k.sigmam2, 2);
	
	k.K_k = MatrixXd(2, 2);
	k.K_k << 0, 0,
		     0, 0;

	k.x_hat = MatrixXd(2, 1);
	k.x_hat << pi / 4,
				0;

	s1.Identity= Matrix2d::Identity();

	int n = floor(t.Tsim / t.Ts);

	for (int i = 0; i <= n; i++)
	{
		s1.system_sim(s1, t);
		k.estimator(s1, k, t);
	};

	for (auto i = k.X_hat.begin(); i < k.X_hat.end(); i++)
	{
		std::cout << *i << ' ';
	};

	cin.get();
}
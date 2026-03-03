#ifndef TOOLS__TRAJECTORY_RK4_HPP
#define TOOLS__TRAJECTORY_RK4_HPP
#include <vector>

namespace tools
{
	class Trajectory //考虑空气阻力, 使用RK4近似打击点然后使用二分法得到最终的pitch角度
	{
		public:
			struct State {
				double x;
				double y;
				double vx;
				double vy;
			};
			//可视化使用
			struct SamplePoint {
				double x;
				double y;
				double vx;
				double vy;
				double t;
			};

			Trajectory(const double x0, const double y0, const double v0, const double target_x, const double target_y, const double max_angle, const double max_time);//单位是弧度
			const double isSolvable() {return is_solvable_;}
			const double getFlyTime() {return fly_time_;}
			const double getPitch() {return pitch_;}
			//可视化使用
			std::vector<SamplePoint> simulateTrajectory(double theta) const;
			std::vector<SamplePoint> simulateSolvedTrajectory() const;

		private:
			double x0_;
			double y0_;
			double v0_;
			double target_x_;
			double target_y_;
			double max_angle_;
			double max_time_;
			double fly_time_;
			double pitch_;
			bool is_solvable_;
			bool calculateHeightError(const double theta, double& heighterror, double& fly_time);
			bool solvePitchBisection(double theta_low, double theta_high);
			bool findBracket(double& bracket_low, double& bracket_high);
			State derivative(const State& s) const;
			State addState(const State& a, const State& b) const;
			State multState(const State& a, double c) const;
			State rk4Step(const State& s) const;
	};	
}
 #endif

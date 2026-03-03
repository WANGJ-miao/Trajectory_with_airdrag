#include "trajectory_RK4.hpp"
#include <cmath>

constexpr double H = 0.01;//RK4步长
constexpr int STEPS = 10;//定长扫描时的步数
namespace tools
{
	constexpr double g = 9.7946;
	/* 
	  假设空气阻力 f = k1 * v^2, 下方是k1的组成
	  C_d = 0.47   无量纲系数，一般球体都用这个值
	  p = 1.169    空气密度（kg/m^3）
	  r = 0.02125  弹丸半径（m）
	  m = 0.041    弹丸质量（kg）
	*/
	constexpr double k = (0.47 * 1.169 * M_PI * 0.02125 * 0.02125) / (2 * 0.041);

	Trajectory::Trajectory(const double x0, const double y0, const double v0, const double target_x, const double target_y, const double max_angle, double max_time) {
		v0_ = v0;
		x0_ = x0;
		y0_ = y0;
		target_x_ = target_x;
		target_y_ = target_y;
		max_angle_ = max_angle;
		max_time_ = max_time;

		//首次按定长扫描
		double bracket_low, bracket_high;
		bool found = findBracket(bracket_low, bracket_high);
		if (!found) is_solvable_ = false;
		else { //开始二分法找最合适的角度
			bool succeed = solvePitchBisection(bracket_low,  bracket_high);
			if (succeed) is_solvable_ = true; else is_solvable_ = false;
		}
	}

	Trajectory::State Trajectory::derivative(const Trajectory::State& s) const {
		double v = std::sqrt(s.vx * s.vx + s.vy * s.vy);

		State ds;
		ds.x = s.vx;
		ds.y = s.vy;
		ds.vx = -k * v * s.vx;
		ds.vy = -g - k * v * s.vy;
		return ds;
	}

	Trajectory::State Trajectory::addState(const Trajectory::State& a, const Trajectory::State& b) const{
		return {a.x + b.x, a.y + b.y, a.vx + b.vx, a.vy + b.vy};
	}

	Trajectory::State Trajectory::multState(const Trajectory::State& a, double c) const{
		return {c * a.x, c * a.y, c * a.vx, c * a.vy};
	}

	Trajectory::State Trajectory::rk4Step(const Trajectory::State& s) const {
		State k1 = derivative(s);
		State k2 = derivative(addState(s, multState(k1, 0.5 * H)));
		State k3 = derivative(addState(s, multState(k2, 0.5 * H)));
		State k4 = derivative(addState(s, multState(k3, H)));

		State sum;
		sum.x = k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x;
		sum.y = k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y;
		sum.vx = k1.vx + 2.0 * k2.vx + 2.0 * k3.vx + k4.vx;
		sum.vy = k1.vy + 2.0 * k2.vy + 2.0 * k3.vy + k4.vy;

		return addState(s, multState(sum, H / 6.0));
	}

	bool Trajectory::calculateHeightError(const double theta, double& heighterror, double& fly_time) {
		State s;
		s.x = x0_;
		s.y = y0_;
		s.vx = v0_ * std::cos(theta);
		s.vy = v0_ * std::sin(theta);

		double t = 0.0;
		while (t < max_time_) {
			State prev = s;
			s = rk4Step(s);
			t += H;

			//如果vx很小说明飞不动了
			if (s.vx <= 1e-6) {return false;}

			//判断有没有跨过target_x_
			if (prev.x <= target_x_ && s.x >= target_x_) {
				double ratio = (target_x_ - prev.x) / (s.x - prev.x);
				//线性插值
				double y_hit = prev.y + ratio * (s.y - prev.y);
				heighterror = y_hit - target_y_;
				fly_time = t + (ratio -1) * H;
				return true;
			}

			//如果已经掉的太低可以提前停止
			if (s.y < -10.0) { return false; }
		}
		return false;
	}

	bool Trajectory::findBracket(double& bracket_low, double& bracket_high) {
		double theta_min = std::atan2(target_y_, target_x_);
		double theta_max = max_angle_;
		double angle_range = theta_max - theta_min;
		double delta_theta = angle_range / STEPS;
		int flipping_index = -1;
		for (int i = 0; i < STEPS + 1; i++) {
			double heighterror;
			bool ok = calculateHeightError(theta_min + i * delta_theta, heighterror, fly_time_);
			if (ok == false) return false;//这里逻辑要改
			if (heighterror > 0) {
				flipping_index = i;
				break;
			}
		}
		if (flipping_index <= 0) return false;
		bracket_high = theta_min + flipping_index * delta_theta;
		bracket_low = bracket_high - delta_theta;
		return true;
	}
	
	bool Trajectory::solvePitchBisection(double theta_low, double theta_high) {
		constexpr int max_iter = 60;
		constexpr double theta_eps = 1e-6;//角度误差
		constexpr double height_eps = 1e-4;//高度误差

		for (int i = 0; i < max_iter; i++) {
			double theta_mid = (theta_low + theta_high) / 2;
			double height_error = 0.0;
			bool ok = calculateHeightError(theta_mid, height_error, fly_time_);
			if (!ok) {return false;}
			//如果足够接近
			if (std::abs(height_error) < height_eps) {
				pitch_ = theta_mid;
				return true;
			}
			if (std::abs(theta_high - theta_low) < theta_eps) {
				pitch_ = theta_mid;
				return true;
			}
			//选择新的区间
			if (height_error < 0) {
				theta_low = theta_mid;
			}
			else if (height_error > 0) {
				theta_high = theta_mid;
			}
		}
		pitch_ = (theta_low + theta_high) / 2;
		return true;
	}

	std::vector<Trajectory::SamplePoint> Trajectory::simulateTrajectory(double theta) const {
		std::vector<SamplePoint> points;

		State s;
		s.x = x0_;
		s.y = y0_;
		s.vx = v0_ * std::cos(theta);
		s.vy = v0_ * std::sin(theta);

		double t = 0.0;
		
		while (t < max_time_) {
			points.push_back({s.x, s.y, s.vx, s.vy, t});
			
			if (s.vx <= 1e-6) break;
			if (s.y < -10.0) break;

			s = rk4Step(s);
			t += H;
		}
		return points;
	}

	std::vector<Trajectory::SamplePoint> Trajectory::simulateSolvedTrajectory() const {
		return simulateTrajectory(pitch_);
	}		

}

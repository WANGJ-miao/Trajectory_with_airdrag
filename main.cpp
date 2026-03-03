#include "trajectory_RK4.hpp"
#include <iostream>
#include <vector>
#include <matplot/matplot.h>

int main() {
	double x0 = 0;
	double y0 = 0;
	double v0 = 30;
	//double target_x = 10;
	//double target_y = 1;
	double max_angle = 1.5;
	double max_time = 5;

	std::vector<std::pair<double, double>> targets = {
		{28, 9},
		{10, 1.5},
		{42, 12},
		{54, 2.2},
		{30, 10},
		{25, 5}
	};

	matplot::hold(matplot::on);

	for (const auto& target : targets) {
		double target_x = target.first;
		double target_y = target.second;
		tools::Trajectory trajectory{x0, y0, v0, target_x, target_y, max_angle, max_time};
		if (!trajectory.isSolvable()) {
			auto points = trajectory.simulateSolvedTrajectory();
			std::vector<double> xs;
			std::vector<double> ys;
			xs.reserve(points.size());
			ys.reserve(points.size());

			for (const auto& point : points) {
				xs.push_back(point.x);
				ys.push_back(point.y);
			}
			matplot::plot(xs, ys);
			auto tgt = matplot::scatter(std::vector<double>{target_x}, std::vector<double>{target_y});
			tgt->marker_face(true);
			tgt->marker_size(10);
			std::cout << "Trajectory is not solvable.\n";
		}

		auto points = trajectory.simulateSolvedTrajectory();
		std::vector<double> xs;
		std::vector<double> ys;
		xs.reserve(points.size());
		ys.reserve(points.size());

		for (const auto& point : points) {
			xs.push_back(point.x);
			ys.push_back(point.y);
		}
		matplot::plot(xs, ys);
		auto tgt = matplot::scatter(std::vector<double>{target_x}, std::vector<double>{target_y});
		tgt->marker_face(true);
		tgt->marker_size(10);
	}
		//for (int i = 0; i <= 50; i+=10) {
		//tools::Trajectory trajectory{x0, y0, v0, target_x + i, target_y, max_angle, max_time};
		//double flytime = trajectory.getFlyTime();
		//double is_solvable = trajectory.isSolvable();
		//double pitch = trajectory.getPitch() * 180 / 3.14;
		//std::cout << "x: " << target_x + i << "\t" << "y: " << target_y << "\t";
		//std::cout << "flytime: " << flytime << "\n";
		//std::cout << "is_solvable: " << is_solvable << "\n";
		//std::cout << "pitch: (degree)" << pitch << "\n";
	//}
	//tools::Trajectory trajectory{x0, y0, v0, target_x, target_y, max_angle, max_time};
	//if (!trajectory.isSolvable()) {
		//std::cout << "Trajectory is not solvable.\n";
		//return 0;
	//}
//
	//auto points = trajectory.simulateSolvedTrajectory();
//
	//std::vector<double> xs;
	//std::vector<double> ys;
	//xs.reserve(points.size());
	//ys.reserve(points.size());
//
	//for (const auto& point : points) {
		//xs.push_back(point.x);
		//ys.push_back(point.y);
	//}
//
	////接下来使用matplot画图
	//matplot::plot(xs, ys);
	//matplot::hold(matplot::on);
    //matplot::scatter(std::vector<double>{target_x}, std::vector<double>{target_y});

	auto start = matplot::scatter(std::vector<double>{x0}, std::vector<double>{y0});
	start->marker_face(true);
	start->marker_size(8);
	
    matplot::xlabel("x / m");
    matplot::ylabel("y / m");
    matplot::title("Trajectories with Target Points");
    matplot::grid(matplot::on);
	matplot::show();
	return 0;
}

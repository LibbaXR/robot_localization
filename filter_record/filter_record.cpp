#include <fstream>
#include <iostream>
#include <iomanip>
#include "robot_localization/measurement.hpp"
#include <vector>
#include "robot_localization/ekf.hpp"
#include "robot_localization/filter_common.hpp"
#include "robot_localization/filter_utilities.hpp"

using namespace robot_localization;

Measurement readLine(std::string line)
{
    Measurement meas;
    meas.update_vector_ = std::vector<bool>(STATE_SIZE, false);
    meas.update_vector_[StateMemberX] = 
    meas.update_vector_[StateMemberY] = 
    meas.update_vector_[StateMemberZ] = 
    meas.update_vector_[StateMemberRoll] = 
    meas.update_vector_[StateMemberPitch] = 
    meas.update_vector_[StateMemberYaw] = true;

    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurement_covariance(STATE_SIZE, STATE_SIZE);

    measurement.setZero();
    measurement_covariance.setZero();

    std::stringstream ss(line);
    std::string token;
    
    if (std::getline(ss, token, ','))
    {
        meas.time_ = MyTime(std::chrono::nanoseconds(long(std::stod(token) * 1e9)));
    }
    if (std::getline(ss, token, ','))
        measurement(StateMemberX) = std::stod(token);
    if (std::getline(ss, token, ','))
        measurement(StateMemberY) = std::stod(token);
    if (std::getline(ss, token, ','))
        measurement(StateMemberZ) = std::stod(token);

    if (std::getline(ss, token, ','))
        measurement(StateMemberYaw) = std::stod(token);
    if (std::getline(ss, token, ','))
        measurement(StateMemberPitch) = std::stod(token);
    if (std::getline(ss, token, ','))
        measurement(StateMemberRoll) = std::stod(token);

    measurement_covariance.setIdentity();
    measurement_covariance *= pow(0.2, 2); // Simulate covariance of 2 cm.

    // If pose frame is different - rotate pose covariance to current frame.
    // Since we estimate the camera frame and pose is given in camera frame - no need to rotate.

    meas.measurement_ = measurement;
    meas.covariance_ = measurement_covariance;

    return meas;
}

std::vector<Measurement> readFile(const std::string& filename)
{
    std::vector<Measurement> measurements;
    std::ifstream fin(filename);
    std::string line;
    while (std::getline(fin, line))
    {
        Measurement meas(readLine(line));
        measurements.push_back(meas);
    }
    return measurements;
}

std::vector<Measurement> processData(const std::vector<Measurement>& measurements)
{
    std::vector<Measurement> filtered_data;
    robot_localization::Ekf filter_;
    for (auto measurement : measurements)
    {
        // This will call predict and, if necessary, correct
        filter_.processMeasurement(measurement);
        if (filter_.getInitializedStatus()) {
            Measurement meas;
            // Grab our current state and covariance estimates
            const Eigen::VectorXd & state = filter_.getState();
            const Eigen::MatrixXd & estimate_error_covariance = filter_.getEstimateErrorCovariance();
            meas.time_ = filter_.getLastMeasurementTime();
            meas.measurement_ = state;
            meas.covariance_ = estimate_error_covariance;
            filtered_data.push_back(meas);
        }
    }

    return filtered_data;
}

void writeFile(std::string filename, std::vector<Measurement> measurements)
{
    std::ofstream fout(filename);
    fout << std::fixed << std::setprecision(8);
    for (auto& meas : measurements)
    {
        fout << filter_utilities::toSec(meas.time_) << ", "
        << meas.measurement_(StateMemberX) << ", "
        << meas.measurement_(StateMemberY) << ", "
        << meas.measurement_(StateMemberZ) << ", "
        << meas.measurement_(StateMemberYaw) << ", "
        << meas.measurement_(StateMemberPitch) << ", "
        << meas.measurement_(StateMemberRoll) << std::endl;
    }
}

int main(int argc, char *argv[])
{
    if(argc < 3)
    {
        std::cout << std::endl << "Usage: ./filter_record <record_file> <results_file> " << std::endl;
        std::cout << std::endl;
        std::cout << "Example:" << std::endl;
        std::cout << "./filter_record results_b.txt filtered_results_b.txt" << std::endl;
        std::cout << std::endl;
        return 1;
    }

    std::string in_filename(argv[1]);
    std::string out_filename(argv[2]);

    std::vector<Measurement> measurements(readFile(in_filename));
    std::cout << "measurements.size(): " << measurements.size() << std::endl;
    std::vector<Measurement> fixed_measurements(processData(measurements));
    writeFile(out_filename, fixed_measurements);

}
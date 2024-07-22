#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <IMU_Preintegrator.h>
#include <fstream>

#include <opencv2/core/core.hpp>


bool LoadEuRocIMUData(const std::string &strImuPath, std::vector<IMUData> &imu_datas);
bool LoadSrulikData(const std::string &strImuPath, std::vector<IMUData> &imu_datas);
Eigen::Vector4d RotationMatrixToQuat(const Eigen::Matrix3d& q);

int main(int argc, char **argv) {
    std::string input_filepath = "in.csv";  // default input file
    std::string output_file_path = "out.csv";  // default output file

    // Process command-line arguments
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--help") == 0) {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --input <path>      Set the input IMU file path (default: in.csv)\n"
                      << "                      in.csv - timestamp[ns], wx[rad/s], wy[rad/s], wz[rad/s], ax[m/s^2], ay[m/s^2], az[m/s^2]\n"
                      << "  --output <path>     Set the output file path (default: out.csv)\n"
                      << "                      out.csv - quaternion[wxyz], position[xyz], velocity[xyz]\n"
                      << "  --help              Display this help message and exit\n";
            return 0;
        } else if (std::strcmp(argv[i], "--input") == 0 && i + 1 < argc) {
            input_filepath = argv[++i];  // Increment i to skip next argument since it's used here
        } else if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
            output_file_path = argv[++i];
        } else {
            std::cerr << "Unknown option or missing argument for " << argv[i] << ". Use --help for usage information.\n";
            return 1;
        }
    }

    std::ofstream output(output_file_path);

    std::vector<IMUData> imu_datas;
    LoadEuRocIMUData(input_filepath, imu_datas);

    IMUPreintegrator IMUPreintegrator;
    output << "qw,qx,qy,qz,px,py,pz,vx,vy,vz" << std::endl;
    for(int i=0; i < imu_datas.size(); ++i)
    {
        IMUData imu_data = imu_datas[i];
        double dt = imu_datas[i+1].timestamp_ - imu_data.timestamp_;

        IMUPreintegrator.Update(imu_data.gyroscope_, imu_data.accelerometer_, dt);


        auto delta_rot_R = IMUPreintegrator.GetDeltaRot();
        auto delta_rot_quat = RotationMatrixToQuat(delta_rot_R);
        output << delta_rot_quat[0] << "," << delta_rot_quat[1] << "," << delta_rot_quat[2] << "," << delta_rot_quat[3] << ",";
        auto delta_p = IMUPreintegrator.GetDeltaP();
        output << delta_p[0] << "," << delta_p[1] << "," << delta_p[2] << ",";
        auto delta_v = IMUPreintegrator.GetDeltaV();
        output << delta_v[0] << "," << delta_v[1] << "," << delta_v[2] << ",";
        output << std::endl;
        

        // output << IMUPreintegrator.GetDeltaV() << std::endl;
        // output << "Pre R " << i << std::endl;
        // output << IMUPreintegrator.GetDeltaRot() << std::endl;
        // output << "Pre Cov " << i << std::endl;
        // output << IMUPreintegrator.GetCovPVRot() << std::endl;

        // output << "Pre P jacobian gyr " << i << std::endl;
        // output << IMUPreintegrator.GetPJacoBiasgur() << std::endl;
        // output << "Pre P jacobian acc " << i << std::endl;
        // output << IMUPreintegrator.GetPJacoBiasacc() << std::endl;
        // output << "Pre V jacobian gyr " << i << std::endl;
        // output << IMUPreintegrator.GetVJacobBiasgyro() << std::endl;
        // output << "Pre V jacobian acc " << i << std::endl;
        // output << IMUPreintegrator.GetVJacobBiasacc() << std::endl;
        // output << "Pre Rot jacobian acc " << i << std::endl;
        // output << IMUPreintegrator.GetRotJacobBiasgyr() << std::endl;

    }
    printf("finished integration of %d imu data samples\n", imu_datas.size());
    return 0;
}

bool LoadEuRocIMUData(const std::string &strImuPath, std::vector<IMUData> &imu_datas)
{

    std::ifstream fImus;
    fImus.open(strImuPath.c_str());
    if (!fImus.is_open())
    {
        std::cerr << "Failed to open imu file: " << strImuPath << std::endl;
        return false;
    }
    imu_datas.reserve(30000);
    //int testcnt = 10;


    while (!fImus.eof())
    {
        std::string s;
        getline(fImus, s);
        if (!s.empty())
        {
            char c = s.at(0);
            if (c < '0' || c > '9')
                continue;

            std::stringstream ss;
            ss << s;
            double tmpd;
            int cnt = 0;
            double data[10];    // timestamp, wx,wy,wz, ax,ay,az
            while (ss >> tmpd)
            {
                data[cnt] = tmpd;
                cnt++;
                if (cnt == 7)
                    break;
                if (ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }
            data[0] *= 1e-9;
            IMUData imudata(data[1], data[2], data[3],
                                       data[4], data[5], data[6], data[0]);
            imu_datas.push_back(imudata);

        }
    }
    return true;
}

Eigen::Vector4d RotationMatrixToQuat(const Eigen::Matrix3d& q) {
    Eigen::Vector4d quat;
    double tr = q.trace();

    if (tr > 0) {
        double S = sqrt(tr + 1.0) * 2; // S=4*qw
        quat[0] = 0.25 * S;
        quat[1] = (q(2,1) - q(1,2)) / S;
        quat[2] = (q(0,2) - q(2,0)) / S;
        quat[3] = (q(1,0) - q(0,1)) / S;
    } else if ((q(0,0) > q(1,1)) && (q(0,0) > q(2,2))) {
        double S = sqrt(1.0 + q(0,0) - q(1,1) - q(2,2)) * 2; // S=4*qx
        quat[0] = (q(2,1) - q(1,2)) / S;
        quat[1] = 0.25 * S;
        quat[2] = (q(0,1) + q(1,0)) / S;
        quat[3] = (q(0,2) + q(2,0)) / S;
    } else if (q(1,1) > q(2,2)) {
        double S = sqrt(1.0 + q(1,1) - q(0,0) - q(2,2)) * 2; // S=4*qy
        quat[0] = (q(0,2) - q(2,0)) / S;
        quat[1] = (q(0,1) + q(1,0)) / S;
        quat[2] = 0.25 * S;
        quat[3] = (q(1,2) + q(2,1)) / S;
    } else {
        double S = sqrt(1.0 + q(2,2) - q(0,0) - q(1,1)) * 2; // S=4*qz
        quat[0] = (q(1,0) - q(0,1)) / S;
        quat[1] = (q(1,2) + q(2,1)) / S;
        quat[2] = (q(0,1) + q(1,0)) / S;
        quat[3] = 0.25 * S;
    }
    return quat;
}

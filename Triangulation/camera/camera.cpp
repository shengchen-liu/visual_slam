#include <fstream>
#include <sys/stat.h>

#include <unordered_map>
#include "camera.h"

Camera::Camera(std::string model_path_):model_path(model_path_)
{
    // init extrinsic
    // 把body坐标系朝向旋转一下,得到相机坐标系，好让它看到landmark,  相机坐标系的轴在body坐标系中的表示
    // 相机朝着轨迹里面看， 特征点在轨迹外部， 这里我们采用这个
    R_bc <<    0, 0, -1,
            -1, 0, 0,
            0, -1, 0;

    intrinsic_matrix << fx, 0, cx,
                        0, fy, cy,
                        0, 0, 1;


    std::ifstream f;
    f.open(model_path);

    while(!f.eof())
    {
        std::string s;
        std::getline(f,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double x,y,z;
            ss >> x;
            ss >> y;
            ss >> z;
            Eigen::Vector4d pt0( x, y, z, 1 );
            ss >> x;
            ss >> y;
            ss >> z;
            Eigen::Vector4d pt1( x, y, z, 1 );

            int start_idx = 0;
            int end_idx = 0;

            bool isHistoryPoint = false;
            for (int i = 0; i < ModelPoints.size(); ++i) {
                Eigen::Vector4d pt = ModelPoints[i];
                if(pt == pt0)
                {
                    start_idx = i;
                    isHistoryPoint = true;
                }
            }
            if(!isHistoryPoint){
                ModelPoints.push_back(pt0);
                start_idx = ModelPoints.size() - 1;
            }
                

            isHistoryPoint = false;
            for (int i = 0; i < ModelPoints.size(); ++i) {
                Eigen::Vector4d pt = ModelPoints[i];
                if(pt == pt1)
                {
                    end_idx = i;
                    isHistoryPoint = true;
                }
            }
            if(!isHistoryPoint) {
                ModelPoints.push_back(pt1);
                end_idx = ModelPoints.size() - 1;
            }
            ModelLines.push_back(std::make_pair(start_idx, end_idx));
        }
    }

}

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3d Camera::euler2Rotation( Eigen::Vector3d  eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy = cos(yaw); double sy = sin(yaw);

    Eigen::Matrix3d RIb;
    RIb<< cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr,
            -sp,         cp*sr,           cp*cr;
    return RIb;
}

MotionData Camera::MotionModel(double t) {
    MotionData data;
    // param
    float ellipse_x = 15;
    float ellipse_y = 20;
    float z = 1;           // z轴做sin运动
    float K1 = 10;          // z轴的正弦频率是x，y的k1倍
    float K = M_PI/ 10;    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

    // translation
    // twb:  body frame in world frame
    Eigen::Vector3d position( ellipse_x * cos( K * t) + 5, ellipse_y * sin( K * t) + 5,  z * sin( K1 * K * t ) + 5);
    
    // Rotation
    double k_roll = 0.1;
    double k_pitch = 0.2;

    Eigen::Vector3d eulerAngles(k_roll * cos(t) , k_pitch * sin(t) , K*t );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    
    Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);         // body frame to world frame
    Eigen::Matrix3d Rwc = Rwb * R_bc;

    data.Rwc = Rwc;
    data.twc = position;

    Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();

    Twc.block(0, 0, 3, 3) = Rwc;
    Twc.block(0, 3, 3, 1) = position;

    Twb.block(0, 0, 3, 3) = Rwb;
    Twb.block(0, 3, 3, 1) = position;

    Twcs.push_back(Twc);
    Twbs.push_back(Twb);

    return data;
}

void Camera::GenerateFrames() {
    // generate cam pose and points in camera

    int count_frames = 0;

    for (float t = t_start; t< t_end;) {
        Frame cur_frame;

        MotionData cam = MotionModel(t);
        motion_datas.push_back(cam);
        t += 1.0/cam_frequency;

        cur_frame.pose = cam;
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block(0, 0, 3, 3) = cam.Rwc;
        Twc.block(0, 3, 3, 1) = cam.twc;

        cv::Mat cur_image(image_h,image_w, CV_8UC3, cv::Scalar(255,255,255));

        int count_points = 0;

        // 遍历所有的特征点，看哪些特征点在视野里
        for (int i = 0; i < ModelPoints.size(); ++i) {
            Eigen::Vector4d pw = ModelPoints[i];
            pw[3] = 1;                                    // 改成齐次坐标最后一位
            Eigen::Vector4d pc1 = Twc.inverse() * pw;     // T_wc.inverse() * Pw  -- > point in cam frame

            if(pc1(2) < 0) {
                cur_frame.points.push_back(Eigen::Vector2d(-1, -1));
                continue; // z必须大于０,在摄像机坐标系前方
            }

            Eigen::Vector3d obs(pc1(0)/pc1(2), pc1(1)/pc1(2), 1);
            Eigen::Vector3d proj = intrinsic_matrix * obs;
            
            cur_frame.points.push_back(Eigen::Vector2d(proj(0), proj(1)));
            cv::circle(cur_image, cv::Point(proj(0), proj(1)), 3, cv::Scalar(255, 0, 0));
            

            if( proj(0) < image_w && proj(0) > 0 && proj(1) > 0 && proj(1) < image_h )
                count_points++;
            
        }


        if(count_points == ModelPoints.size()) {
            if(first_frame_has_whole_points < 0) first_frame_has_whole_points = count_frames;
            else if(second_frame_has_whole_points < 0) second_frame_has_whole_points = count_frames;
        }


        for(int i = 0; i < ModelLines.size(); i++){
            auto line = ModelLines[i];
            if(cur_frame.points[line.first][0] < 0 || cur_frame.points[line.second][0] < 0) continue;
            cv::line(cur_image, 
                     cv::Point(cur_frame.points[line.first][0], cur_frame.points[line.first][1]), 
                     cv::Point(cur_frame.points[line.second][0], cur_frame.points[line.second][1]), 
                     cv::Scalar(0, 255, 0), 2);
        }

        // cv::imshow("render", cur_image);
        // cv::waitKey(0);
        
        cur_frame.image_ = cur_image;
        cur_frame.frame_idx = count_frames;
        frames.push_back(cur_frame);
        count_frames ++;
    }
}

Eigen::Matrix3x4d Camera::GetFirstFrameProjectMatrix(){
    Eigen::Matrix4d Tcw = Twcs[first_frame_has_whole_points].inverse();

    Eigen::Matrix3x4d ProjectMat;
    ProjectMat.setZero();
    ProjectMat.block<3, 3>(0, 0) = intrinsic_matrix;
    ProjectMat = ProjectMat * Tcw;
    return ProjectMat;
}
Eigen::Matrix3x4d Camera::GetSecondFrameProjectMatrix(){
    Eigen::Matrix4d Tcw = Twcs[second_frame_has_whole_points].inverse();

    Eigen::Matrix3x4d ProjectMat;
    ProjectMat.setZero();
    ProjectMat.block<3, 3>(0, 0) = intrinsic_matrix;
    ProjectMat = ProjectMat * Tcw;
    return ProjectMat;
}

std::vector<Eigen::Vector2d> Camera::GetFirstFramePoints() {
    cv::imshow("render", frames[first_frame_has_whole_points].image_);
    cv::waitKey(0);
    return frames[first_frame_has_whole_points].points;
}

std::vector<Eigen::Vector2d> Camera::GetSecondFramePoints() {
    cv::imshow("render2", frames[second_frame_has_whole_points].image_);
    cv::waitKey(0);
    return frames[second_frame_has_whole_points].points;
}


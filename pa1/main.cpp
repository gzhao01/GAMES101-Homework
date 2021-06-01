#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    float radian = rotation_angle/180*MY_PI;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f m;
    m << cos(radian),-sin(radian),0,0,
         sin(radian),cos(radian),0,0,
         0,0,1,0,
         0,0,0,1;
    model = m;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float eye_fov_radian = eye_fov/180*MY_PI;
    float A = zNear + zFar;
    float B = -zNear * zFar;
    float n=zNear, f=zFar;
    float t=tan(eye_fov_radian/2)*n, b = -t;
    float r=t*aspect_ratio, l=-r;
    Eigen::Matrix4f m_per2ortho;
    m_per2ortho << zNear,0,0,0,
                    0,zNear,0,0,
                    0,0,A,B,
                    0,0,1,0;
    Eigen::Matrix4f translate, scale;
    translate << 1,0,0,-(r+l)/2,
                 0,1,0,-(t+b)/2,
                 0,0,1,-(n+f)/2,
                 0,0,0,1;
    scale << 2/(r-l),0,0,0,
             0,2/(t-b),0,0,
             0,0,2/(f-n),0,
             0,0,0,1;
    projection = scale * translate * m_per2ortho;

    return projection;
}

//rotate around any axis
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    float radian_angle = angle/180*MY_PI;
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    //store tmp matrix, change it to 4x4
    Eigen::Matrix3f temp = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f m;
    m << 0,-axis[2], axis[1],
        axis[2],0,-axis[0],
        -axis[1],axis[0],0;
    //Rodrigues' rotation formula
    temp = cos(radian_angle)*Eigen::Matrix3f::Identity()
         +(1-cos(radian_angle))*axis*axis.transpose()
         +sin(radian_angle)*m;
    rotation << temp(0,0),temp(0,1),temp(0,2),0,
                temp(1,0),temp(1,1),temp(1,2),0,
                temp(2,0),temp(2,1),temp(2,2),0,
                0,0,0,1;
    return rotation;
}

int main(int argc, const char** argv)
{
    Eigen::Vector3f axis;
    axis<<0,0,1; // rotate around z axis by default
    float angle = 0;
    float angle_inter=0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        //output file name
        if (argc == 4) {
            filename = std::string(argv[3]);            
        }
        //rotate axis
        if(argc == 6){
            command_line = false;
            std::cout << std::stof(argv[3]);
            axis[0] = std::stof(argv[3]);
            axis[1] = std::stof(argv[4]);
            axis[2] = std::stof(argv[5]);
        }
        else{
            return 0;
        }
    }
    angle_inter = angle;

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(axis,angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(axis,angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += angle_inter;
        }
        else if (key == 'd') {
            angle -= angle_inter;
        }
    }

    return 0;
}

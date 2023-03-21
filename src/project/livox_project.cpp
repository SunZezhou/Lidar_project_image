#include"../../include/ros_detection_tracking/project.h"
#include<vector>


using namespace std;

struct PointData{
    float x;
    float y;
    float z;
    int i;
};

vector<PointData> vector_data;

namespace DetectandTract{

    void projector::initParams()
    {
        std::string pkg_loc = ros::package::getPath("ros_detection_tracking");
        std::ifstream infile(pkg_loc + "/cfg/livox_realsense_params.txt");
        infile >> i_params.camera_topic;
        infile >> i_params.lidar_topic;
        infile >> i_params.depth_topic;
        //std::cout<<i_params.camera_topic<<std::endl;
        // double_t camtocam[12];
        double_t cameraIn[16];
        double_t RT[16];
        // for (int i = 0; i < 16; i++){
        //     infile >> camtocam[i];
        // }
        // cv::Mat(4, 4, 6, &camtocam).copyTo(i_params.camtocam_mat);//cameratocamera params

        for (int i = 0; i < 12; i++){
            infile >> cameraIn[i];
        }
        cv::Mat(4, 4, 6, &cameraIn).copyTo(i_params.cameraIn);//cameraIn params

        for (int i = 0; i < 16; i++){
            infile >> RT[i];
        }
        cv::Mat(4, 4, 6, &RT).copyTo(i_params.RT);//T_camera_lidar
        std::cout<<i_params.RT<<std::endl;
    }


    void projector::depth_callback(const sensor_msgs::Image::ConstPtr &img, 
                                    const livox_ros_driver2::CustomMsg::ConstPtr &pc)
        {
            // sensor_msgs::CompressedImage::Ptr image_msg = img;
            cout << img->header.stamp.toSec() << " " << pc->header.stamp.toSec() << endl;
            cv_bridge::CvImagePtr cv_ptr;
            try {
                // cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
                cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1); // for depth image
            }
            catch (cv_bridge::Exception &e) {
                std::cout << "exception" << std::endl;
                return;
            }
            // cv::Mat raw_img = cv_ptr->image;
            cv_ptr->image /= 200.0;
            cv::Mat temp_img;
            cv_ptr->image.convertTo(temp_img, CV_8UC1);
            cv::Mat raw_img;
            cv::applyColorMap(temp_img, raw_img, 1);
            livox_ros_driver2::CustomMsg livox_cloud;
            livox_cloud = *pc;
            // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            // pcl::fromROSMsg(*pc, *cloud);
            
            cv::Mat visImg = raw_img.clone();
            cv::Mat overlay = visImg.clone();
            std::cout<<"get pc and image data"<<std::endl;
            
            cv::Mat X(4,1,cv::DataType<double>::type);
            cv::Mat Y(3,1,cv::DataType<double>::type);

            for(uint i = 0; i < livox_cloud.point_num; ++i) {
                if (livox_cloud.points[i].x <= 0.02)
                    continue;

                X.at<double>(0,0) = livox_cloud.points[i].x;
                X.at<double>(1,0) = livox_cloud.points[i].y;
                X.at<double>(2,0) = livox_cloud.points[i].z;
                X.at<double>(3,0) = 1;
                // std::cout << X.at<double>(0,0) << " " << X.at<double>(1,0) << " " << X.at<double>(2,0) << std::endl;

                Y = i_params.cameraIn * i_params.RT * X;

                cv::Point pt;
                std::cout << Y.at<double>(0,0) << " " << Y.at<double>(1,0) << " " << Y.at<double>(2,0) << std::endl;
                pt.x = Y.at<double>(0,0) / Y.at<double>(2,0);
                pt.y = Y.at<double>(1,0) / Y.at<double>(2,0);

                if(0<= pt.x <= 1280&& 0<=pt.y<=720)
                {
                    float val = livox_cloud.points[i].x;
                    float maxVal = 50.0;
                    int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
                    int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
                    cv::circle(overlay, pt, 3, cv::Scalar(0, green, red), -1);
                }
            }

            // Publish the image projection
            ros::Time time = ros::Time::now();
            cv_ptr->encoding = "bgr8";
            cv_ptr->header.stamp = time;
            cv_ptr->header.frame_id = "livox";
            cv_ptr->image = overlay;
            depth_publisher.publish(cv_ptr->toImageMsg());
            std::cout<<"depth picture is published!"<<std::endl;
        }

    
    void projector::livox_projection_callback(const sensor_msgs::CompressedImage::ConstPtr &img, 
                                    const livox_ros_driver2::CustomMsg::ConstPtr &pc)
        {
            // sensor_msgs::CompressedImage::Ptr image_msg = img;
            cout << img->header.stamp.toSec() << " " << pc->header.stamp.toSec() << endl;
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
            }
            catch (cv_bridge::Exception &e) {
                std::cout << "exception" << std::endl;
                return;
            }
            cv::Mat raw_img = cv_ptr->image;
            livox_ros_driver2::CustomMsg livox_cloud;
            livox_cloud = *pc;
            // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            // pcl::fromROSMsg(*pc, *cloud);
            
            cv::Mat visImg = raw_img.clone();
            cv::Mat overlay = visImg.clone();
            std::cout<<"get pc and image data"<<std::endl;
            
            cv::Mat X(4,1,cv::DataType<double>::type);
            cv::Mat Y(3,1,cv::DataType<double>::type);

            for(uint i = 0; i < livox_cloud.point_num; ++i) {
                if (livox_cloud.points[i].x <= 0.02)
                    continue;

                X.at<double>(0,0) = livox_cloud.points[i].x;
                X.at<double>(1,0) = livox_cloud.points[i].y;
                X.at<double>(2,0) = livox_cloud.points[i].z;
                X.at<double>(3,0) = 1;
                // std::cout << X.at<double>(0,0) << " " << X.at<double>(1,0) << " " << X.at<double>(2,0) << std::endl;

                Y = i_params.cameraIn * i_params.RT * X;

                cv::Point pt;
                std::cout << Y.at<double>(0,0) << " " << Y.at<double>(1,0) << " " << Y.at<double>(2,0) << std::endl;
                pt.x = Y.at<double>(0,0) / Y.at<double>(2,0);
                pt.y = Y.at<double>(1,0) / Y.at<double>(2,0);

                if(0<= pt.x <= 1280&& 0<=pt.y<=720)
                {
                    float val = livox_cloud.points[i].x;
                    float maxVal = 50.0;
                    int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
                    int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
                    cv::circle(overlay, pt, 3, cv::Scalar(0, green, red), -1);
                }
            }

            // Publish the image projection
            ros::Time time = ros::Time::now();
            cv_ptr->encoding = "bgr8";
            cv_ptr->header.stamp = time;
            cv_ptr->header.frame_id = "livox";
            cv_ptr->image = overlay;
            image_publisher.publish(cv_ptr->toImageMsg());
            std::cout<<"project picture is published!"<<std::endl;
        }

    projector::projector() 
    {
        ros::NodeHandle nh("~");
        initParams();

        ros::Publisher project_img_pub;

        message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub(nh, i_params.camera_topic, 5);
        message_filters::Subscriber<livox_ros_driver2::CustomMsg> lidar_sub(nh, i_params.lidar_topic, 5);
        message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, i_params.depth_topic, 5);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, livox_ros_driver2::CustomMsg> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_sub, lidar_sub);
        sync.registerCallback(boost::bind(&projector::livox_projection_callback, this, _1, _2));

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, livox_ros_driver2::CustomMsg> depthPolicy;
        message_filters::Synchronizer<depthPolicy> sync_depth(depthPolicy(5), depth_sub, lidar_sub);
        sync_depth.registerCallback(boost::bind(&projector::depth_callback, this, _1, _2));

        image_transport::ImageTransport imageTransport(nh);
        image_publisher = imageTransport.advertise("/project_pc_image", 20);
        depth_publisher = imageTransport.advertise("/project_pc_depth", 20);

        ros::spin();
    }



}

int main(int argc, char **argv){
    ros::init(argc, argv, "project_pc_to_image");
    DetectandTract::projector projector;
    return 0;
}
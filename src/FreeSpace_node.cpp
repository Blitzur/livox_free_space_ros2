#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <std_msgs/msg/header.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
//#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/msg/grid_cells.hpp"
#include "pcl/conversions.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/filters/voxel_grid.h>
#include <ctime>
#include <memory.h>
#include "pcl/PCLPointCloud2.h"
#include "FreeSpace.hpp"

class LivoxFreeSpace : public rclcpp::Node {
public:
    LivoxFreeSpace() : Node("livox_freespace_ros2_port") {

        height_offset = this->declare_parameter("height_offset", 0.7);

        //////////////////////  Publisher //////////////////////
        /// Visually debugging
        fs_distance_circle_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("fs_marker/circle", 1);
        fs_distance_text_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("fs_marker/dis_text", 1);
        fs_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("fs_pointcloud/pointcloud", 1);
        fs_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fs_pointcloud/free_space", 1);

        //////////////////////  Subscribers //////////////////////
        //currently we use the mpc_deg for speeds under 30km/h and the ftc_deg for speeds 30+ in future we would like to only use the mpc_deg!
        sub_pc = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_3WEDJBW001K6541", 1,std::bind(&LivoxFreeSpace::PointCloudCallback, this, std::placeholders::_1));

        init();
    }

private:
    //general
    unsigned char *pVImg;
    std::string namesp;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fs_distance_circle_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fs_distance_text_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fs_points_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fs_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc;
    std::vector<sensor_msgs::msg::PointCloud2> recieved_pc_msgs;
    sensor_msgs::msg::PointCloud2 this_pc_msg;
    visualization_msgs::msg::MarkerArray circles, texts;
    bool is_background_pub = false, msg_type = true;
    uint8_t colors[19][3] = {
        {244, 67, 54}, {233, 30, 99}, {156, 39, 176}, {103, 58, 183}, {63, 81, 181}, {33, 150, 243}, {3, 169, 244},
        {0, 188, 212}, {0, 150, 136}, {76, 175, 80},
        {139, 195, 74}, {205, 220, 57}, {255, 235, 59}, {255, 193, 7}, {255, 152, 0}, {25, 87, 34}, {121, 85, 72},
        {96, 125, 139}, {255, 100, 200}
    };
    uint8_t colors_bg[10][3] = {
        {83, 134, 139}, {0, 139, 139}, {46, 139, 87}, {84, 139, 84}, {47, 79, 79}, {139, 117, 0}, {139, 10, 80},
        {104, 34, 139}, {16, 78, 139}, {96, 123, 139}
    };

    float height_offset = 0.0;
    std_msgs::msg::Header gheader;

    ////////////// Initialize //////////////
    void init() {
        namesp = this->get_namespace();
        if (namesp != "/") {
            namesp += "/";
        }
        namesp = namesp.substr(1, namesp.size());

        std::cout << "height offset:\t\t\t" << height_offset << std::endl;
        PrepareBackground();
        is_background_pub = true;
        fs_distance_circle_pub->publish(circles);
        fs_distance_text_pub->publish(texts);
    }

    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        pcl::PointCloud<pcl::PointXYZI> pc;

        pcl::fromROSMsg(*cloud_msg, pc);
        for (int i = 0; i < pc.points.size(); i++) {
            pc.points[i].z = pc.points[i].z + height_offset;
        }
        gheader = cloud_msg->header;
        GenerateFreeSpace(pc);

        std::cout << "Recieved pointcloud: secs = " << cloud_msg->header.stamp.sec << ", nsecs = " << cloud_msg->header.stamp.nanosec << std::endl;
        //this_pc_msg = cloud_msg;
        this_pc_msg = *cloud_msg;
    }

    void PrepareBackground() {
        for (int dis = 50; dis < 500; dis = dis + 50) {
            visualization_msgs::msg::Marker circle, text;
            circle.header.frame_id = "lidar/fm";
            circle.header.stamp = rclcpp::Time();
            circle.id = dis;
            circle.action = visualization_msgs::msg::Marker::ADD;
            circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
            circle.color.r = 0.5;
            circle.color.g = 0.5;
            circle.color.b = 0.5;
            circle.color.a = 1;
            circle.scale.x = 0.1;
            for (float i = 0; i <= 2 * 3.2; i = i + 0.1) {
                geometry_msgs::msg::Point p;
                p.x = dis * cos(i);
                p.y = dis * sin(i);
                p.z = -2;
                circle.points.push_back(p);
            }
            circles.markers.push_back(circle);

            text.header.frame_id = "lidar/fm";
            text.header.stamp = rclcpp::Time();
            text.id = dis;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.color.r = 1;
            text.color.g = 1;
            text.color.b = 1;
            text.color.a = 1;
            text.scale.z = 5;
            text.pose.position.x = dis * cos(5.76);
            text.pose.position.y = dis * sin(5.76);
            text.pose.position.z = -1;
            text.pose.orientation.x = 0;
            text.pose.orientation.y = 0;
            text.pose.orientation.z = -0.25;
            text.pose.orientation.w = 0.96;
            text.text = std::to_string(dis) + "m";
            texts.markers.push_back(text);
        }
        visualization_msgs::msg::Marker one_line;
        one_line.header.frame_id = "lidar/fm";
        one_line.header.stamp = rclcpp::Time();
        one_line.id = 500;
        one_line.action = visualization_msgs::msg::Marker::ADD;
        one_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        one_line.color.r = 0.5;
        one_line.color.g = 0.5;
        one_line.color.b = 0.5;
        one_line.color.a = 1;
        one_line.scale.x = 0.1;
        geometry_msgs::msg::Point p1, p2;
        p1.x = 0;
        p1.y = 0;
        p1.z = -2;
        p2.x = 500;
        p2.y = 0;
        p2.z = -2;
        one_line.points.push_back(p1);
        one_line.points.push_back(p2);
        circles.markers.push_back(one_line);
    }

    void ApplyColorToPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pc, const pcl::PointCloud<pcl::PointXYZI> &input) {
        for (int i = 0; i < pc.points.size(); i++) {
            pc.points[i].x = input.points[i].x;
            pc.points[i].y = input.points[i].y;
            pc.points[i].z = input.points[i].z;
            if (input.points[i].intensity < 30.0) {
                int green = input.points[i].intensity * 255.0 / 30.0;
                pc.points[i].r = 0;
                pc.points[i].g = green & 0xff;
                pc.points[i].b = 0xff;
            } else if (input.points[i].intensity < 90.0) {
                int blue = (90 - input.points[i].intensity) * 255.0 / 60.0;
                pc.points[i].r = 0;
                pc.points[i].g = 0xff;
                pc.points[i].b = blue & 0xff;
            } else if (input.points[i].intensity < 150.0) {
                int red = (input.points[i].intensity - 90) * 255.0 / 60.0;
                pc.points[i].r = red & 0xff;
                pc.points[i].g = 0xff;
                pc.points[i].b = 0;
            } else {
                int green = (255 - input.points[i].intensity) * 255.0 / (256 - 150);
                pc.points[i].r = 0xff;
                pc.points[i].g = green & 0xff;
                pc.points[i].b = 0;
            }
        }
    }

    void GenerateFreeSpace(pcl::PointCloud<pcl::PointXYZI> &pc) {
        clock_t t0, t1, t2;
        t0 = clock();

        int dnum = pc.points.size();
        std::cout << "Point cloud size: " << dnum << std::endl;

        float *data = (float *) calloc(dnum * 4, sizeof(float));
        std::vector<float> free_space;
        for (int p = 0; p < dnum; ++p) {
            data[p * 4 + 0] = pc.points[p].x;
            data[p * 4 + 1] = pc.points[p].y;
            data[p * 4 + 2] = pc.points[p].z;
            data[p * 4 + 3] = pc.points[p].intensity;
        }
        GenerateFreeSpace(data, dnum, free_space);
        t1 = clock();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->clear();
        cloud->width = dnum;
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);
        ApplyColorToPointCloud(*cloud, pc);
        sensor_msgs::msg::PointCloud2 msg2;
        pcl::toROSMsg(*cloud, msg2);

        msg2.header.stamp = gheader.stamp;
        msg2.header.frame_id = "lidar/fm";
        fs_points_pub->publish(msg2);

        pcl::PointCloud<pcl::PointXYZI> fs;
        fs.clear();
        for (int i = 0; i < free_space.size(); i += 3) {
            pcl::PointXYZI p;
            p.x = free_space[i];
            p.y = free_space[i + 1];
            p.z = 0;
            p.intensity = free_space[i + 2];
            fs.points.push_back(p);
        }
        sensor_msgs::msg::PointCloud2 msg3;
        pcl::toROSMsg(fs, msg3);

        msg3.header.stamp = gheader.stamp;
        msg3.header.frame_id = "lidar/fm";
        fs_pub->publish(msg3);
        std::vector<float>().swap(free_space);
        t2 = clock();

        printf("\n\n");
        printf("Total Time: %f, FreeSpace: %f, Publish Results: %f\n\n",
               1000.0 * (t2 - t0) / CLOCKS_PER_SEC, 1000.0 * (t1 - t0) / CLOCKS_PER_SEC,
               1000.0 * (t2 - t1) / CLOCKS_PER_SEC);
        printf("---------------------------------------------\n\n");
        free(data);
    }

    int GenerateFreeSpace(float *fPoints1, int pointNum, std::vector<float> &free_space) {
        clock_t t0, t1, t2, t3, t4;
        t0 = clock();
        // down sampling
        float *fPoints2 = (float *) calloc(pointNum * 4, sizeof(float));
        int *idtrans1 = (int *) calloc(pointNum, sizeof(int));
        int *idtransx = (int *) calloc(pointNum, sizeof(int));
        int *idtransy = (int *) calloc(pointNum, sizeof(int));
        int *idtrans2 = (int *) calloc(pointNum, sizeof(int));

        int pntNum = 0;

        pVImg = (unsigned char *) calloc(DN_SAMPLE_IMG_NX * DN_SAMPLE_IMG_NY * DN_SAMPLE_IMG_NZ,
                                                        sizeof(unsigned char));
        std::vector<int> count(DENOISE_IMG_NX * DENOISE_IMG_NY * DENOISE_IMG_NZ, 0);

        for (int pid = 0; pid < pointNum; pid++) {
            int ix = (fPoints1[pid * 4] + DN_SAMPLE_IMG_OFFX) / DENOISE_IMG_DX;
            int iy = (fPoints1[pid * 4 + 1] + DN_SAMPLE_IMG_OFFY) / DENOISE_IMG_DY;
            int iz = (fPoints1[pid * 4 + 2] + DN_SAMPLE_IMG_OFFZ) / DENOISE_IMG_DZ;
            idtrans1[pid] = -1;
            if ((ix >= 0) && (ix < DENOISE_IMG_NX) && (iy >= 0) && (iy < DENOISE_IMG_NY) && (iz >= 0) && (
                    iz < DENOISE_IMG_NZ)) {
                int idx = iz * DENOISE_IMG_NX * DENOISE_IMG_NY + iy * DENOISE_IMG_NX + ix;
                idtrans1[pid] = idx;
                count[idx]++;
            }
        }

        for (int pid = 0; pid < pointNum; pid++) {
            if (idtrans1[pid] > -1 && count[idtrans1[pid]] < 3) {
                fPoints1[pid * 4] = 0;
                fPoints1[pid * 4 + 1] = 0;
                fPoints1[pid * 4 + 2] = 0;
            }
        }

        for (int pid = 0; pid < pointNum; pid++) {
            int ix = (fPoints1[pid * 4] + DN_SAMPLE_IMG_OFFX) / DN_SAMPLE_IMG_DX;
            int iy = (fPoints1[pid * 4 + 1] + DN_SAMPLE_IMG_OFFY) / DN_SAMPLE_IMG_DY;
            int iz = (fPoints1[pid * 4 + 2] + DN_SAMPLE_IMG_OFFZ) / DN_SAMPLE_IMG_DZ;

            idtrans1[pid] = -1;
            if ((ix >= 0) && (ix < DN_SAMPLE_IMG_NX) && (iy >= 0) && (iy < DN_SAMPLE_IMG_NY) && (iz >= 0) && (
                    iz < DN_SAMPLE_IMG_NZ)) {
                idtrans1[pid] = iz * DN_SAMPLE_IMG_NX * DN_SAMPLE_IMG_NY + iy * DN_SAMPLE_IMG_NX + ix;
                idtransx[pid] = ix;
                idtransy[pid] = iy;
                if (pVImg[idtrans1[pid]] == 0) //没有访问过，肯定栅格内会有重复的，所以fPoints2只取第一个
                {
                    fPoints2[pntNum * 4] = fPoints1[pid * 4];
                    fPoints2[pntNum * 4 + 1] = fPoints1[pid * 4 + 1];
                    fPoints2[pntNum * 4 + 2] = fPoints1[pid * 4 + 2];
                    fPoints2[pntNum * 4 + 3] = fPoints1[pid * 4 + 3];
                    idtrans2[pntNum] = pid;
                    pntNum++;
                }

                pVImg[idtrans1[pid]] = 1;
            }
        }

        t1 = clock();

        int *pLabelGnd = (int *) calloc(pntNum, sizeof(int));
        int ground_point_num = GroundSegment(pLabelGnd, fPoints2, pntNum, 1.0);

        t2 = clock();

        int agnum = pntNum - ground_point_num;
        float *fPoints3 = (float *) calloc(agnum * 4, sizeof(float));
        int agcnt = 0;
        for (int ii = 0; ii < pntNum; ii++) {
            if (pLabelGnd[ii] == 0) {
                fPoints3[agcnt * 4] = fPoints2[ii * 4];
                fPoints3[agcnt * 4 + 1] = fPoints2[ii * 4 + 1];
                fPoints3[agcnt * 4 + 2] = fPoints2[ii * 4 + 2];
                fPoints3[agcnt * 4 + 3] = fPoints2[ii * 4 + 3];
                agcnt++;
            }
        }
        float *free_space_small = (float *) calloc(360, sizeof(float));
        this->FreeSpace(fPoints3, agnum, free_space_small, 360);
        this->FreeSpaceFilter(free_space_small, 360, free_space);

        free(fPoints2);
        free(idtrans1);
        free(idtrans2);
        free(idtransx);
        free(idtransy);
        free(fPoints3);
        free(pLabelGnd);
        free(this->pVImg);
        free(free_space_small);
        std::vector<int>().swap(count);
        t3 = clock();
        // printf("FreeSpace total Time: %f, Downsample: %f, Ground Segment: %f, FreeSpace: %f\n\n", 1000.0*(t3 - t0) / CLOCKS_PER_SEC,
        //         1000.0*(t1 - t0) / CLOCKS_PER_SEC, 1000.0*(t2 - t1) / CLOCKS_PER_SEC, 1000.0*(t3 - t2) / CLOCKS_PER_SEC);
    }

    int filter_x[28] = {
        -1, 0, 1, -3, -2, 2, 3, -4, 4, -4, 4, -5, 5, -5, 5, -5, 5, -1, 0, 1, -3, -2, 2, 3, -4, 4, -4, 4
    };
    int filter_y[28] = {
        -5, -5, -5, -4, -4, -4, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 1, 5, 5, 5, 4, 4, 4, 4, 3, 3, 2, 2
    };
    int all_x[89] = {
        -1, 0, 1,
        -3, -2, -1, 0, 1, 2, 3,
        -4, -3, -2, -1, 0, 1, 2, 3, 4,
        -4, -3, -2, -1, 0, 1, 2, 3, 4,
        -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5,
        -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5,
        -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5,
        -1, 0, 1,
        -3, -2 - 1, 0, 1, 2, 3,
        -4, -3, -2, -1, 0, 1, 2, 3, 4,
        -4, -3, -2, -1, 0, 1, 2, 3, 4
    };
    int all_y[89] = {
        -5, -5, -5,
        -4, -4, -4, -4, -4, -4, -4,
        -3, -3, -3, -3, -3, -3, -3, -3, -3,
        -2, -2, -2, -2, -2, -2, -2, -2, -2,
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        5, 5, 5,
        4, 4, 4, 4, 4, 4, 4,
        3, 3, 3, 3, 3, 3, 3, 3, 3,
        2, 2, 2, 2, 2, 2, 2, 2, 2
    };

    void FreeSpaceFilter(float *free_space_small, int n, std::vector<float> &free_space) {
        clock_t t0, t1, t2, t3, t4;
        t0 = clock();
        float pixel_size = 0.2, delta_d_in_r = 0.13, delta_r = 0.15;
        //delta_d_in_r is smaller than pixel_size and delta_r, to make sure all pixels are covered
        Eigen::MatrixXi src = Eigen::MatrixXi::Zero(100 / pixel_size, 100 / pixel_size), dst = Eigen::MatrixXi::Zero(
            100 / pixel_size, 100 / pixel_size);

        std::vector<float> delta_t;
        for (float j = 0.0001; j < 50; j += delta_r) // Prepare the delta theta of different radius
        {
            delta_t.push_back(delta_d_in_r / j);
        }
        for (int i = 0; i < 360; i++) {
            float r = min(free_space_small[i], free_space_small[(i + 1) % n]);
            r = min(r, free_space_small[(i - 1 + n) % n]);
            r = sqrt(r);
            int k = 0;
            for (float j = 0; j < r - 0.5; j += delta_r) {
                float dt = delta_t[k++];
                float theta = (i - 180) * FREE_PI / 180.0;
                for (float t = theta - 0.01; t < theta + 0.01; t += dt) {
                    float x = j * cos(t);
                    float y = j * sin(t);
                    int m = int((50.0 - x) / pixel_size);
                    int nn = int((50.0 - y) / pixel_size);
                    src(m, nn) = 1;
                }
            }
        }

        t1 = clock();
        for (int i = 0; i < 360; i++) {
            for (float j = 0; j < 49; j += delta_r) {
                float x = j * cos((i - 180) * FREE_PI / 180.0);
                float y = j * sin((i - 180) * FREE_PI / 180.0);
                int m = int((50.0 - x) / pixel_size);
                int nn = int((50.0 - y) / pixel_size);
                int theta = int(atan2f(y, x) * 180.0 / FREE_PI + 180.0 + 0.5);
                theta = theta % n;
                float r = min(free_space_small[theta], free_space_small[(theta + 1) % n]);
                r = min(r, free_space_small[(theta - 1 + n) % n]);
                if (r > j * j + 1) {
                    int result = 0;
                    for (int k = 0; k < 28; k++) {
                        result += src(m + filter_x[k], nn + filter_y[k]);
                    }
                    if (result < 28) // check if position (m, nn) is in free space
                        break;
                    for (int k = 0; k < 89; k++) {
                        dst(m + all_x[k], nn + all_y[k]) = max(1, dst(m + all_x[k], nn + all_y[k]));
                    }
                    dst(m, nn) = 2;
                }
            }
        }


        t2 = clock();

        for (int i = 0; i < dst.rows(); i++) {
            for (int j = 0; j < dst.cols(); j++) {
                if (dst(i, j) > 0) {
                    float x = (100.0 - i * pixel_size) - 50.0;
                    float y = (100.0 - j * pixel_size) - 50.0;
                    free_space.push_back(x);
                    free_space.push_back(y);
                    free_space.push_back(255);
                }
            }
        }
        t3 = clock();
        // printf("filter time: %f, generate map: %f, conv: %f, fs generate: %f\n\n", 1000.0*(t3 - t0) / CLOCKS_PER_SEC,
        //         1000.0*(t1 - t0) / CLOCKS_PER_SEC, 1000.0*(t2 - t1) / CLOCKS_PER_SEC, 1000.0*(t3 - t2) / CLOCKS_PER_SEC);
    }

    void FreeSpace(float *fPoints, int n, float *free_space, int free_space_n) {
        int thetaId;
        float distance;

        for (int ii = 0; ii < free_space_n; ii++) {
            free_space[ii] = 2500;
        }

        for (int pid = 0; pid < n; pid++) {
            if (fPoints[pid * 4 + 2] < 3) // points of high tree, buildings are rejected
            {
                if (abs(fPoints[pid * 4 + 1]) < 1.2 && abs(fPoints[pid * 4]) < 2.5) // reject the near points of robot
                    continue;
                distance = fPoints[pid * 4] * fPoints[pid * 4] + fPoints[pid * 4 + 1] * fPoints[pid * 4 + 1];
                thetaId = int((atan2f(fPoints[pid * 4 + 1], fPoints[pid * 4]) + FREE_PI) * 180.0 / FREE_PI + 0.5);
                thetaId = thetaId % free_space_n;
                if (free_space[thetaId] > distance && distance > 1) {
                    free_space[thetaId] = distance;
                }
            }
        }
    }

    /*
    int LivoxFreeSpace::GroundSegment(int* pLabel,float *fPoints,int pointNum,float fSearchRadius)
    Fast ground segmentation using rule-based & plane fitting method
    */
    int GroundSegment(int *pLabel, float *fPoints, int pointNum, float fSearchRadius) {

        int gnum = 0;

        float *pGndImg1 = (float *) calloc(GND_IMG_NX1 * GND_IMG_NY1, sizeof(float));
        int *tmpLabel1 = (int *) calloc(pointNum, sizeof(int));

        for (int ii = 0; ii < GND_IMG_NX1 * GND_IMG_NY1; ii++) {
            pGndImg1[ii] = 100;
        }
        for (int pid = 0; pid < pointNum; pid++) {
            int ix = (fPoints[pid * 4] + GND_IMG_OFFX1) / (GND_IMG_DX1 + 0.000001);
            int iy = (fPoints[pid * 4 + 1] + GND_IMG_OFFY1) / (GND_IMG_DY1 + 0.000001);
            if (ix < 0 || ix >= GND_IMG_NX1 || iy < 0 || iy >= GND_IMG_NY1) {
                tmpLabel1[pid] = -1;
                continue;
            }

            int iid = ix + iy * GND_IMG_NX1;
            tmpLabel1[pid] = iid;

            if (pGndImg1[iid] > fPoints[pid * 4 + 2]) {
                pGndImg1[iid] = fPoints[pid * 4 + 2];
            }
        }

        int pnum = 0;
        for (int pid = 0; pid < pointNum; pid++) {
            if (tmpLabel1[pid] >= 0) {
                if (pGndImg1[tmpLabel1[pid]] + 0.4 > fPoints[pid * 4 + 2]) {
                    pLabel[pid] = 1;
                    pnum++;
                }
            }
        }
        free(pGndImg1);
        free(tmpLabel1);


        for (int pid = 0; pid < pointNum; pid++) {
            if (pLabel[pid] == 1) {
                if (fPoints[pid * 4 + 2] > 1) {
                    pLabel[pid] = 0;
                } else if (fPoints[pid * 4] * fPoints[pid * 4] + fPoints[pid * 4 + 1] * fPoints[pid * 4 + 1] < 100) {
                    if (fPoints[pid * 4 + 2] > 0.5) {
                        pLabel[pid] = 0;
                    }
                }
            } else {
                if (fPoints[pid * 4] * fPoints[pid * 4] + fPoints[pid * 4 + 1] * fPoints[pid * 4 + 1] < 400) {
                    if (fPoints[pid * 4 + 2] < 0.2) {
                        pLabel[pid] = 1;
                    }
                }
            }
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        if (pointNum <= 0) {
            pointNum = 1;
        }
        pointNum = 1;
        std::vector<int> in_zoneVec(pointNum);
        //int in_zone[pointNum] = {0};
        for (int pid = 0; pid < pointNum; pid++) {
            if (fPoints[pid * 4] * fPoints[pid * 4] + fPoints[pid * 4 + 1] * fPoints[pid * 4 + 1] < 400) {
                in_zoneVec[pid] = 1;
                if (pLabel[pid] == 1) {
                    pcl::PointXYZI p;
                    p.x = fPoints[pid * 4];
                    p.y = fPoints[pid * 4 + 1];
                    p.z = fPoints[pid * 4 + 2];
                    p.intensity = fPoints[pid * 4 + 3];
                    cloud->points.push_back(p);
                }
            }
        }

        Eigen::Matrix3f cov;
        Eigen::Vector4f pc_mean;
        Eigen::MatrixXf normal_;

        pcl::computeMeanAndCovarianceMatrix(*cloud, cov, pc_mean);
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
        normal_ = (svd.matrixU().col(2));
        Eigen::Vector3f seeds_mean = pc_mean.head<3>();
        //  normal.T * [x,y,z] = -d
        float d_ = -(normal_.transpose() * seeds_mean)(0, 0);
        float th_dist_d_ = 0.3 - d_;
        Eigen::MatrixXf points(pointNum, 3);
        for (int k = 0; k < pointNum; k++) {
            points.row(k) << fPoints[k * 4], fPoints[k * 4 + 1], fPoints[k * 4 + 2];
        }

        // ground plane model
        Eigen::VectorXf result = points * normal_;

        for (int k = 0; k < pointNum; k++) {
            if (!in_zoneVec[k])
                continue;
            if (result[k] < th_dist_d_) {
                pLabel[k] = 1;
            } else {
                pLabel[k] = 0;
            }
        }


        gnum = 0;
        for (int pid = 0; pid < pointNum; pid++) {
            if (pLabel[pid] == 1) {
                gnum++;
            }
        }

        return gnum;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<LivoxFreeSpace> mpNode = std::make_shared<LivoxFreeSpace>();
    rclcpp::spin(mpNode);
    rclcpp::shutdown();
    return 0;
}

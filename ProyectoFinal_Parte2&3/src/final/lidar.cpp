#include <iostream>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include "Eigen/Dense"

#define DEBUG

using namespace std;
using namespace Eigen;

class Kalman
{

public:

    MatrixXf X;
    MatrixXf F;
    MatrixXf P;
    MatrixXf R;
    MatrixXf H;
    MatrixXf Z;
    MatrixXf I;
    MatrixXf nextX;
    MatrixXf nextP;

    Kalman()
    {
        //Definir X
        X = MatrixXf(4,1);
        X(0,0) = 0;
        X(1,0) = 0;
        X(2,0) = 0;
        X(3,0) = 0;

        //Definir F
        float dt = 0.1f;
        F = MatrixXf(4,4);
        F(0,0) = 1;
        F(1,1) = 1;
        F(2,2) = 1;
        F(3,3) = 1;
        F(0,2) = dt;
        F(1,3) = dt;
        
        //Definir P
        P = MatrixXf(4,4);
        P(0,0) = 2.0f;
        P(1,1) = 2.0f;
        P(2,2) = 2.0f;
        P(3,3) = 2.0f; 

        //Ruido
        R = MatrixXf(2,2);
        R(0,0) = 0.1;
        R(0,1) = 0.1;
        R(1,0) = 0.1;
        R(1,1) = 0.1;

        //Matriz H
        H = MatrixXf(2,4);
        H(0,0) = 1;
        H(1,2) = 1;

        //Matriz Z
        Z = MatrixXf(2,1);

        //Matriz Identidad
        I = MatrixXf(4,4);
        I(0,0) = 1;
        I(1,1) = 1;
        I(2,2) = 1;
        I(3,3) = 1;
    }

    void setZ(float x, float y)
    {
        Z(0,0) = x;
        Z(1,0) = y;
    }

    void Prediccion()
    {
        nextX = (F * X); // Falta U que es cero
        nextP = F * P * F.transpose();
    }


    void Actualizacion()
    {
        MatrixXf Y = Z - (H * X);
        MatrixXf S = (H * P * H.transpose()) + R;
        MatrixXf K = P * H.transpose() * S.inverse();
        X = nextX + (K * Y);
        P = (I - K * H) * nextP;
    }

    void PrintNext()
    {
        cout << nextX << endl << endl;
        cout << nextP << endl;
    }
};

class Lidar
{

public:

    Kalman *k;
    ros::Publisher pubPose;
    ros::Publisher pubVel;
    ros::Publisher pubSteer;

    Lidar()
    {
        this->k = new Kalman;
        ros::NodeHandle nh;
        this->pubPose = nh.advertise<sensor_msgs::LaserScan>("robot/next_pose", 1000);
        pubVel = nh.advertise<std_msgs::Float32>("AutoNOMOS_mini_1/manual_control/velocity", 1000);
        pubSteer = nh.advertise<std_msgs::Float32>("AutoNOMOS_mini_1/manual_control/steering", 1000);
    }

    //Publica un mensaje de Pose con el objetivo que lo lea el controlador de la vez pasada
    void PublicPose()
    {
        tf2_msgs::TFMessage msg;
        msg.transforms = vector<geometry_msgs::TransformStamped>();
        /*msg.transforms[0].transform.translation.x = 10;
        msg.transforms[0].transform.translation.y = 10;
        msg.transforms[0].transform.rotation.z = 0;*/
        pubPose.publish(msg);
    }

    //Publica directamente al modelo de Gazebo
    void PublicVelocityAndSteering(float v, float theta)
    {
        std_msgs::Float32 msgv;
        msgv.data = v;
        std_msgs::Float32 msgs;
        //msgs.data = angles::to_degrees(gamma);
        msgs.data = theta;
        pubVel.publish(msgv);
        pubSteer.publish(msgs);
    }

    void UpdateLidar(const sensor_msgs::LaserScan &msg)
    {
        vector<float> arr_dist;
        vector<float> arr_angles;
        float step = (msg.angle_max - msg.angle_min) / (arr_dist.size() - 1);
        float mean = 0.0f;

    #ifdef DEBUG
        cout << "Angulos: " << msg.angle_min << " , " <<  msg.angle_max << endl;
    #endif

        //Llenar arreglos (angulo, dostancia) descartando lectiras infinitas por el Lidar y calcular media
        for(int i = 0; i < msg.ranges.size(); i++)
        {
            if(isfinite(msg.ranges[i]))
            {
                float angle = msg.angle_min + (i * step);
                mean += msg.ranges[i];
                arr_dist.push_back(msg.ranges[i]);
                arr_angles.push_back(angle);
            }
        }
        mean = mean / arr_angles.size();
        
        //Calcular el punto del centreo del grupo a partir de la media
        float theta = 0.0f;
        for(int i = 1; i < arr_dist.size(); i++)
        {
            if(arr_dist[i-1] <= mean && mean < arr_dist[i+1])
            {
                theta = arr_angles[i-1] + (((arr_angles[i+1] - arr_angles[i-1]) * (mean - arr_dist[i-1])) / (arr_dist[i+1] - arr_dist[i-1]));
            }
        }

    #ifdef DEBUG
        cout << "Media: " << mean << " Angulo: " << theta << endl;
    #endif

        //Calcular "x" y "y" a partir de la distancia y theta
        float x = mean * cosf(theta);
        float y = mean * sinf(theta);

        //Llamar a Kalman
        k->setZ(x,y);
        k->Prediccion();
        float v = (k->nextX(2,0) - k->X(2,0)) / 0.1;

#ifdef DEBUG
        cout << "X: " << x << " Y: " << y << endl;
        cout << "Variable X actualizada del ciclo:" << endl << k->X << endl;
        cout << "Velocidad: " << v << endl;
#endif

        //Mandar el objetivo al control
        PublicVelocityAndSteering(v, theta);

        k->Actualizacion();
    }

};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "lidar_raeder");
    Lidar l;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("AutoNOMOS_mini_1/laser_scan", 1000, &Lidar::UpdateLidar, &l);
    ros::spin();

    return 0;
}
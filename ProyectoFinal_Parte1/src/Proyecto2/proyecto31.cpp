#include <ros/ros.h>
#include <stdlib.h>
#include <iomanip>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32MultiArray.h>
#include <math.h>
#include <string.h>

int matrizPos[8][7]={{0,0,0,0,0,0,0}, 
{1,1,1,1,1,0,0}, 
{0,1,0,1,0,1,0}, 
{0,1,0,1,0,0,0}, 
{0,0,1,1,1,1,1}, 
{0,0,1,0,1,0,0},
{0,0,0,1,0,1,0}, 
{0,0,0,1,0,0,0}};
int steCen[8][7]={{0,0,0,0,0,0,0}, 
{1,1,1,1,1,0,0}, 
{0,1,0,1,0,1,0}, 
{0,1,0,1,0,0,0}, 
{0,0,1,1,1,1,1}, 
{0,0,1,0,1,0,0},
{0,0,0,1,0,1,0}, 
{0,0,0,1,0,0,0}};
int steIzq[8][7]={{0,0,0,0,0,0,0}, 
{1,1,1,1,0,0,0}, 
{1,0,1,0,1,0,0}, 
{1,0,1,0,0,0,0}, 
{0,1,1,1,1,1,0}, 
{0,1,0,1,0,0,0},
{0,0,1,0,1,0,0}, 
{0,0,1,0,0,0,0}};
int steDer[8][7]={{0,0,0,0,0,0,0}, 
{0,1,1,1,1,1,0}, 
{0,0,1,0,1,0,1}, 
{0,0,1,0,1,0,0}, 
{0,0,0,1,1,1,1}, 
{0,0,0,1,0,1,0},
{0,0,0,0,1,0,1}, 
{0,0,0,0,1,0,0}};
int matrizInd[8][3];
double v = 0.14285714;
double probabilidad[7]={v, v, v, v, v, v, v};
double probabilidad1[7];
double probabilidad2[7];
double x[7];
double y[12];
double h[7];

int i=0;
int d=0;
int c=0;
int m=0;

double eta=1;
double steAng=0;
using namespace std;

void histDerecha(const std_msgs::Int32MultiArray& msg){
    d=1;
}

void histIzquierda(const std_msgs::Int32MultiArray& msg){
    i=1;
}

void histCentro(const std_msgs::Int32MultiArray& msg){
    c=1;
}

void ste(const std_msgs::Int16& msg){
	steAng=double(msg.data);
	/*ROS_INFO_STREAM("Steering Angle " 
            << " steAng=" << steAng);*/
}

int main(int argc, char **argv){

    ros::init(argc, argv, "filtro_de_histograma");
    ros::NodeHandle nh;

    ros::Subscriber lectorDerecha = nh.subscribe("points/right", 10, &histDerecha);
    ros::Subscriber lectorIzquierda = nh.subscribe("points/left", 10, &histIzquierda);
    ros::Subscriber lectorCentro = nh.subscribe("points/center", 10, &histCentro);
    ros::Subscriber lectorste = nh.subscribe("manual_control/steering", 10, &ste);

    ros::Rate rate(10);

    while(ros::ok()){

        if(i==0){
            if(c==0){
                if(d==0)
                    m=0;
                else
                    m=1;
            }
            else{
                if(d==0)
                    m=2;
                else
                    m=3;
            }
        }
        else{
            if(c==0){
                if(d==0)
                    m=4;
                else
                    m=5;
            }
            else{
                if(d==0)
                    m=6;
                else
                    m=7;
            }
        }

        i=0;
        d=0;
        c=0;
        probabilidad1[0]=probabilidad[0]+matrizPos[m][0];
        probabilidad1[1]=probabilidad[1]+matrizPos[m][1];
        probabilidad1[2]=probabilidad[2]+matrizPos[m][2];
        probabilidad1[3]=probabilidad[3]+matrizPos[m][3];
        probabilidad1[4]=probabilidad[4]+matrizPos[m][4];
        probabilidad1[5]=probabilidad[5]+matrizPos[m][5];
        probabilidad1[6]=probabilidad[6]+matrizPos[m][6];
        eta=probabilidad1[0]+probabilidad1[1]+probabilidad1[2]+probabilidad1[3]+probabilidad1[4]+probabilidad1[5]+probabilidad1[6];
        probabilidad1[0]=probabilidad1[0]/eta;
        probabilidad1[1]=probabilidad1[1]/eta;
        probabilidad1[2]=probabilidad1[2]/eta;
        probabilidad1[3]=probabilidad1[3]/eta;
        probabilidad1[4]=probabilidad1[4]/eta;
        probabilidad1[5]=probabilidad1[5]/eta;
        probabilidad1[6]=probabilidad1[6]/eta;

        if(steAng<45.0){
        	probabilidad2[0]=probabilidad[0]+steIzq[m][0];
        	probabilidad2[1]=probabilidad[1]+steIzq[m][1];
        	probabilidad2[2]=probabilidad[2]+steIzq[m][2];
        	probabilidad2[3]=probabilidad[3]+steIzq[m][3];
        	probabilidad2[4]=probabilidad[4]+steIzq[m][4];
        	probabilidad2[5]=probabilidad[5]+steIzq[m][5];
        	probabilidad2[6]=probabilidad[6]+steIzq[m][6];
        } else if(steAng>120.0){
        	probabilidad2[0]=probabilidad[0]+steDer[m][0];
        	probabilidad2[1]=probabilidad[1]+steDer[m][1];
        	probabilidad2[2]=probabilidad[2]+steDer[m][2];
        	probabilidad2[3]=probabilidad[3]+steDer[m][3];
        	probabilidad2[4]=probabilidad[4]+steDer[m][4];
        	probabilidad2[5]=probabilidad[5]+steDer[m][5];
        	probabilidad2[6]=probabilidad[6]+steDer[m][6];
        }else{
        	probabilidad2[0]=probabilidad[0]+steCen[m][0];
        	probabilidad2[1]=probabilidad[1]+steCen[m][1];
        	probabilidad2[2]=probabilidad[2]+steCen[m][2];
        	probabilidad2[3]=probabilidad[3]+steCen[m][3];
        	probabilidad2[4]=probabilidad[4]+steCen[m][4];
        	probabilidad2[5]=probabilidad[5]+steCen[m][5];
        	probabilidad2[6]=probabilidad[6]+steCen[m][6];
        }
        eta=probabilidad2[0]+probabilidad2[1]+probabilidad2[2]+probabilidad2[3]+probabilidad2[4]+probabilidad2[5]+probabilidad2[6];
        probabilidad2[0]=probabilidad2[0]/eta;
        probabilidad2[1]=probabilidad2[1]/eta;
        probabilidad2[2]=probabilidad2[2]/eta;
        probabilidad2[3]=probabilidad2[3]/eta;
        probabilidad2[4]=probabilidad2[4]/eta;
        probabilidad2[5]=probabilidad2[5]/eta;
        probabilidad2[6]=probabilidad2[6]/eta;

        x[0] = probabilidad1[0];
        x[1] = probabilidad1[1];
        x[2] = probabilidad1[2];
        x[3] = probabilidad1[3];
        x[4] = probabilidad1[4];
        x[5] = probabilidad1[5];
        x[6] = probabilidad1[6];

        h[0] = probabilidad2[0];
        h[1] = probabilidad2[1];
        h[2] = probabilidad2[2];
        h[3] = probabilidad2[3];
        h[4] = probabilidad2[4];
        h[5] = probabilidad2[5];
        h[6] = probabilidad2[6];

        y[3]=x[0]*h[3]+x[1]*h[2]+x[2]*h[1]+x[3]*h[0];
		y[4]=x[0]*h[4]+x[1]*h[3]+x[2]*h[2]+x[3]*h[1]+x[4]*h[0];
		y[5]=x[0]*h[5]+x[1]*h[4]+x[2]*h[3]+x[3]*h[2]+x[4]*h[1]+x[5]*h[0];
		y[6]=x[0]*h[6]+x[1]*h[5]+x[2]*h[4]+x[3]*h[3]+x[4]*h[2]+x[5]*h[1]+x[6]*h[0];
		y[7]=x[1]*h[6]+x[2]*h[5]+x[3]*h[4]+x[4]*h[3]+x[5]*h[2]+x[6]*h[1];
		y[8]=x[2]*h[6]+x[3]*h[5]+x[4]*h[4]+x[5]*h[3]+x[6]*h[2];
		y[9]=x[3]*h[6]+x[4]*h[5]+x[5]*h[4]+x[5]*h[3];

		/*if(probabilidad[0]!=y[3] || probabilidad[1]!=y[4] || probabilidad[2]!=y[5] || probabilidad[3]!=y[6] || probabilidad[4]!=y[7] || probabilidad[5]!=y[8] || probabilidad[6]!=y[9]){
			ROS_INFO_STREAM("Probabilidad de estados: "
        	<< " Mensajito=" << "Sí cambió");
		}
		else{
			ROS_INFO_STREAM("Probabilidad de estados: "
        	<< " Mensajito=" << "No cambió");
		}*/
		probabilidad[0]=y[3];
		probabilidad[1]=y[4];
		probabilidad[2]=y[5];
		probabilidad[3]=y[6];
		probabilidad[4]=y[7];
		probabilidad[5]=y[8];
		probabilidad[6]=y[9];

        ROS_INFO_STREAM("Probabilidad de estados: "
        	<< " steAng=" << steAng 
            << " probAI=" << probabilidad[0]
            << " probLI=" << probabilidad[1]
            << " probCI=" << probabilidad[2]
            << " probC =" << probabilidad[3]
            << " probCD=" << probabilidad[4]
            << " probLD=" << probabilidad[5]
            << " probAD=" << probabilidad[6]);
        ros::spinOnce();
    }
    
}
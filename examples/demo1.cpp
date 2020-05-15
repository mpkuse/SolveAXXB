// Here I show a simple demo on how to use this library
#include <iostream>
#include <vector>
#include <map>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;


//axxb
#include "../axxb/axxb_utils.h"

#include "../axxb/axxbsolver.h"
#include "../axxb/conventionalaxxbsvdsolver.h"
#include "../axxb/extendedaxxbelilambdasvdsolver.h"
#include "../axxb/andreffextendedaxxbsolver.h"


// Creates trajectory for the 2 sensors
void create_sensor_trajectories(
    vector<Matrix4d>& sensor1_trajectory,
    vector<Matrix4d>& sensor2_trajectory,
    Matrix4d sensor1_T_sensor2
)
{
    sensor1_trajectory.clear();
    sensor2_trajectory.clear();


    // trajectory start and ends
    const double _PI_ = 3.14159265359;
    Vector3d rpy_start = Vector3d(10,20,-10)/180.*_PI_; //Vector3d::Random();
    Vector3d rpy_end   = Vector3d( -10,-5,20 )/180.*_PI_;  // ; Vector3d::Random();
    Vector3d t_start   = Vector3d::Random()*10;
    Vector3d t_end     = Vector3d::Random()*10;
    cout << "rpy(rads): " << rpy_start.transpose() << "--->" << rpy_end.transpose() << endl;
    cout << "rpy(deg) : " << rads2deg(rpy_start).transpose() << "--->" << rads2deg(rpy_end).transpose() << endl;
    cout << "t        : " << t_start.transpose() << "--->" << t_end.transpose() << endl;



    for( double f=0 ; f<=1.0 ; f+=0.1 )
    {
        Vector3d rpy = rpy_start * (1.0-f) + rpy_end * f;
        Vector3d t   = t_start * (1.0-f) + t_end*f;

        Matrix4d w_T_i__sensor1 = rpyt_to_transform( rpy, t );
        Matrix4d w_T_i__sensor2 =  w_T_i__sensor1 * sensor1_T_sensor2;

        cout << "---\n";
        cout << "\tw_T_i__sensor1:" << prettyPrint( w_T_i__sensor1 ) << endl;
        cout << "\tw_T_i__sensor2:" << prettyPrint( w_T_i__sensor2 ) << endl;

        sensor1_trajectory.push_back( w_T_i__sensor1 );
        sensor2_trajectory.push_back( w_T_i__sensor2 );
    }
}

// given as input trajectories creates relative poses samples
void multiple_relative_poses_from_trajectories(
    const vector<Matrix4d>& sensor1_trajectory,
    const vector<Matrix4d>& sensor2_trajectory,

    vector<Matrix4d>& random_relative_poses_sensor1,
    vector<Matrix4d>& random_relative_poses_sensor2

)
{
    random_relative_poses_sensor1.clear();
    random_relative_poses_sensor2.clear();
    assert( sensor1_trajectory.size() == sensor2_trajectory.size() &&  sensor2_trajectory.size() >= 3 );
    int N = (int) sensor1_trajectory.size();


    #if 0
    // 0---1
    {
        int a = 0;
        int b = 1;
        Matrix4d rel_sensor1 = sensor1_trajectory[a].inverse() * sensor1_trajectory[b].inverse();
        Matrix4d rel_sensor2 = sensor2_trajectory[a].inverse() * sensor2_trajectory[b].inverse();

        random_relative_poses_sensor1.push_back( rel_sensor1 );
        random_relative_poses_sensor2.push_back( rel_sensor2 );
    }


    // 3---2
    {
        int a = 3;
        int b = 2;
        Matrix4d rel_sensor1 = sensor1_trajectory[a].inverse() * sensor1_trajectory[b].inverse();
        Matrix4d rel_sensor2 = sensor2_trajectory[a].inverse() * sensor2_trajectory[b].inverse();

        random_relative_poses_sensor1.push_back( rel_sensor1 );
        random_relative_poses_sensor2.push_back( rel_sensor2 );
    }
    #endif


    std::map< std::pair<int,int>, bool  > hit_map; //< to keep track of which pairs have been seen and not repeat those
    for( int s=0 ; s<20 ; s++ )
    {
        int a = rand()%N;
        int b = rand()%N;
        cout << "#" << s << "\ta=" <<a << "\tb=" << b << "\t";
        if( a==b ) {
            cout << "\t a n b are equal....skip\n";
            continue;
        }

        if( hit_map.count( make_pair(a,b) ) > 0 || hit_map.count( make_pair(b,a) ) > 0  ) {
            cout << "\t seen this pair before....skip\n";
            continue;
        }

        cout << "\t OK!\n";
        hit_map[ make_pair(a,b) ] = true;
        Matrix4d rel_sensor1 = sensor1_trajectory[a].inverse() * sensor1_trajectory[b].inverse();
        Matrix4d rel_sensor2 = sensor2_trajectory[a].inverse() * sensor2_trajectory[b].inverse();

        random_relative_poses_sensor1.push_back( rel_sensor1 );
        random_relative_poses_sensor2.push_back( rel_sensor2 );
    }


}

int main()
{
    cout << "demo1\n";


    cout << "++++ set a relative pose between sensors\n";
    // relative transform between the sensors
    // The algorithm does not know this.
    Matrix4d sensor1_T_sensor2 = Matrix4d::Identity();
    Vector3d sensor1_rpy_deg_sensor2 = Vector3d( 0,0,10 );
    Vector3d sensor1_t_sensor2 = Vector3d( 0, 0.0, 0 );
    sensor1_T_sensor2 = rpyt_to_transform( deg2rads(sensor1_rpy_deg_sensor2),  sensor1_t_sensor2 );
    cout << "sensor1_T_sensor2: " << prettyPrint( sensor1_T_sensor2 ) << endl;



    cout << "+++++ Create trajectories for both sensors.\n";
    vector<Matrix4d> sensor1_trajectory, sensor2_trajectory;
    create_sensor_trajectories( sensor1_trajectory, sensor2_trajectory, sensor1_T_sensor2 );
    cout << "\tsensor1_trajectory.size() = " << sensor1_trajectory.size() << endl;


    cout << "+++++ pick random poses (a,b) and compute relative poses for each poses from the trajectory." << endl;
    cout << "repeat this multiple times\n";
    vector<Matrix4d> random_relative_poses_sensor1, random_relative_poses_sensor2;
    multiple_relative_poses_from_trajectories(
        sensor1_trajectory, sensor2_trajectory,
        random_relative_poses_sensor1, random_relative_poses_sensor2
    );
    cout << "\trandom_relative_poses_sensor1.size() = " << random_relative_poses_sensor1.size() << endl;


    cout << "++++++ ConventionalAXXBSVDSolver\n";
    auto obj = ConventionalAXXBSVDSolver( random_relative_poses_sensor1, random_relative_poses_sensor2 );
    // auto obj = ExtendedAXXBEliLambdaSVDSolver( random_relative_poses_sensor1, random_relative_poses_sensor2);
    // auto obj = AndreffExtendedAXXBSolver( random_relative_poses_sensor1, random_relative_poses_sensor2);
    Matrix4d estimated_X = obj.SolveX();
    cout << "estimated_X:\n" << estimated_X << endl;
    cout << "estimated_X:\n" << prettyPrint(estimated_X) << endl;

    Matrix4d estimated_X_inverse = estimated_X.inverse();
    cout << "estimated_X_inverse:\n" << prettyPrint(estimated_X_inverse) << endl;


    return 0;
}

#include <iostream>
using namespace std;


#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

#include "../axxb/axxb_utils.h"


int main()
{
    cout << "++++ start with a random rpy, t\n";
    Vector3d rpy = Vector3d::Random();
    Vector3d t = Vector3d::Random();
    cout << "rpy (rads):\t" << rpy.transpose() << endl;
    cout << "rpy (deg):\t" << rads2deg(rpy).transpose() << endl;
    cout << "t:\t" << t.transpose() << endl;


    //
    cout << "++++ Convert this to a transformation matrix (4x4)\n";
    Matrix4d w_T_c = rpyt_to_transform( rpy, t );
    cout << "w_T_c:\n" << w_T_c << endl;
    cout << "w_T_c:" << prettyPrint(w_T_c) << endl;


    //
    cout << "++++ Convert the 4x4 matrix back to rpy t\n";
    Vector3d rpy_cap, t_cap;
    transform_to_rpyt( w_T_c, rpy_cap, t_cap );
    cout << "rpy_cap (rads):\t" << rpy_cap.transpose() << endl;
    cout << "rpy_cap (deg):\t" << rads2deg(rpy_cap).transpose() << endl;
    cout << "t_cap:\t" << t_cap.transpose() << endl;
    return 0;
}

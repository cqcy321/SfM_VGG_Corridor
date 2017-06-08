#define CERES_FOUND 1
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

using namespace std;
using namespace cv;
using namespace cv::sfm;

bool loadPoints3D(const std::string &filename, vector<Vec3d>& points3d)
{
    ifstream f;
    f.open(filename.c_str());

    if(f.eof())
    return false;

    int count = 0;
    double a[6];
    while(!f.eof())
    {
        count ++;
        string snode;
        getline(f,snode);
        stringstream ssnode;
        ssnode << snode;

        for(int i=0;i<6;i++){
            ssnode >> a[i];
        }
        if(count==40)continue;
        points3d.push_back(-Vec3d(a[0],a[1],a[2]));
        points3d.push_back(-Vec3d(a[3],a[4],a[5]));

    }
    cout<<points3d.size()<<endl;
    return true;
}

bool loadCamera(const std::string &filename, vector<Matx33d> &Rs_est, vector<Vec3d> &ts_est)
{
    ifstream f;
    f.open(filename.c_str());

    if(f.eof())
    return false;

    int count = 0;
    double a[12];
    while(!f.eof())
    {
        count ++;
        int j=0;
        string snode;
        stringstream ssnode;
        getline(f,snode);
        ssnode << snode;
        for(int i=0;i<4;i++){
            ssnode >> a[j++];
        }
        ssnode.clear();
        getline(f,snode);
        ssnode << snode;
        for(int i=0;i<4;i++){
            ssnode >> a[j++];
        }
        getline(f,snode);
        ssnode.clear();
        ssnode << snode;
        for(int i=0;i<4;i++){
            ssnode >> a[j++];
        }
        Rs_est.push_back(Matx33d(-a[0],-a[1],-a[2],a[4],a[5],a[6],-a[8],-a[9],-a[10]));
        ts_est.push_back(Vec3d(a[3],a[7],a[11]));

    }
    return true;
}

static void help() {
  cout
      << "\n------------------------------------------------------------------\n"
      << " This program shows the camera trajectory reconstruction capabilities\n"
      << " in the OpenCV Structure From Motion (SFM) module.\n"
      << " \n"
      << " Usage:\n"
      << "        example_sfm_trajectory_reconstruction"
      << "------------------------------------------------------------------\n\n"
      << endl;
}


/* Keyboard callback to control 3D visualization
 */

bool camera_pov = false;

void keyboard_callback(const viz::KeyboardEvent &event, void* cookie)
{
    if ( event.action == 0 &&!event.symbol.compare("s") )
        camera_pov = !camera_pov;
}


/* Sample main code
 */

int main(int argc, char** argv)
{
    //definitions of variables
    bool is_projective = true;
    vector<Matx33d> Rs_est;
    vector<Vec3d> ts_est;
    cv::Matx33d K(517.8237 , 0, 358.8203, 0 -528.3985, 251.1046, 0, 0, 1);

    for (int num =0;num<11;num++)
    {
        char camfile[40];
        sprintf(camfile,"camera/cameras%.3d.txt",num);
        loadCamera(camfile,Rs_est,ts_est);
    }

    vector<Vec3d> point_cloud_est;
    loadPoints3D("Corridor/bt.l3d",point_cloud_est);

    // Print output
    cout << "\n----------------------------\n" << endl;
    cout << "Reconstruction: " << endl;
    cout << "============================" << endl;
    cout << "3D Visualization: " << endl;
    cout << "============================" << endl;


    /// Create 3D windows
    viz::Viz3d window_est("Estimation Coordinate Frame");
    window_est.setBackgroundColor(viz::Color::white()); // black by default
    window_est.registerKeyboardCallback(&keyboard_callback);

    vector<Affine3d> path_est;
    for (size_t i = 0; i < Rs_est.size(); ++i)
        path_est.push_back(Affine3d(Rs_est[i],ts_est[i]));

    cout << "[DONE]" << endl;

    /// Add cameras
    cout << "Rendering Trajectory  ... ";

    /// Wait for key 'q' to close the window
    cout << endl << "Press:                       " << endl;
    cout <<         " 's' to switch the camera pov" << endl;
    cout <<         " 'q' to close the windows    " << endl;


    if (1)// path_est.size() > 0 )
    {
        // animated trajectory
        int idx = 0, forw = -1, n = static_cast<int>(path_est.size());

        while(!window_est.wasStopped())
        {
            int N=point_cloud_est.size();
            /// Render points as 3D cubes
            for (size_t i = 0; i < N; i+=2)
            {
                char buffer[50];
                sprintf(buffer, "%d", static_cast<int>(i));
                window_est.showWidget(buffer,viz::WLine(point_cloud_est[i],point_cloud_est[i+1],viz::Color::black()));
            }
            window_est.showWidget("coos", viz::WCoordinateSystem());
            Affine3d cam_pose = path_est[idx];
            viz::WCameraPosition cpw; // Coordinate axes
            viz::WCameraPosition cpw_frustum(K, 0.3, viz::Color::yellow()); // Camera frustum

            if ( camera_pov )
                window_est.setViewerPose(cam_pose);
            else
            {
            // render complete trajectory
                window_est.showWidget("cameras_frames_and_lines_est", viz::WTrajectory(path_est, viz::WTrajectory::PATH, 1.0, viz::Color::green()));

                window_est.showWidget("CPW", cpw, cam_pose);
                window_est.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
            }

            // update trajectory index (spring effect)
            forw *= (idx==n || idx==0) ? -1: 1; idx += forw;

            // frame rate 1s
            window_est.spinOnce(1, true);
            window_est.removeAllWidgets();
        }
    }
//    Mat a=imread("/home/cy/Documents/workspace/QT_Pros/build-SfM-Desktop_Qt_5_5_1_GCC_64bit-Debug/images/B21.jpg");
//a.data;
//    waitKey();
//    waitKey();
//    cout<<"eada"<<endl;

    return 0;
}

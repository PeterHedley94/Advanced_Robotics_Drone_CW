#include <memory>
#include <unistd.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include<arp/Frontend.hpp>
#include <arp/Autopilot.hpp>
#include <arp/VisualInertialTracker.hpp>
#include <arp/StatePublisher.hpp>

class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  arp::VisualInertialTracker *vit;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    // -- for later use
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    vit->addImage(timeMicroseconds, lastImage_);
  }

  bool getLastImage(cv::Mat& image)
  {
    std::lock_guard<std::mutex> l(imageMutex_);
    if (lastImage_.empty())
      return false;
    image = lastImage_.clone();
    lastImage_ = cv::Mat();  // clear, only get same image once.
    return true;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    // -- for later use

    Eigen::Vector3d omega_S(msg-> angular_velocity.x, msg-> angular_velocity.y, msg-> angular_velocity.z);
    Eigen::Vector3d acc_S(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    vit->addImuMeasurement(timeMicroseconds, omega_S, acc_S);
  }

  //FRANCOIS
/*  visualInertialTracker->setVisualisationCallback(std::bind(
  &arp::StatePublisher::publish, &publisher,
  std::placeholders::_1, std::placeholders::_2));*/

  void setCallBack

 private:
  cv::Mat lastImage_;
  std::mutex imageMutex_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arp_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  //HERE CW2
  ros::Publisher pubShow = nh.advertise<geometry_msgs::PoseStamped>("ardrone/camera_pose", 1);
  arp::Frontend frontend;
  double k1, k2, p1, p2, cu, cv, fu, fv;
  nh.getParam("arp_node/fu", fu);
  nh.getParam("arp_node/fv", fv);
  nh.getParam("arp_node/cu", cu);
  nh.getParam("arp_node/cv", cv);
  nh.getParam("arp_node/p1", p1);
  nh.getParam("arp_node/p2", p2);
  frontend.setCameraParameters(640, 360, fu, fv, cu, cv, k1, k2, p1, p2);
  frontend.setTarget(0, 0.16);

  // setup inputs
  Subscriber subscriber;
  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50,
                                        &Subscriber::imuCallback, &subscriber);

  // set up autopilot
  arp::Autopilot autopilot(nh);
  arp::StatePublisher publish(nh);

  // setup rendering
  SDL_Event event;
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window * window = SDL_CreateWindow("Hello AR Drone", SDL_WINDOWPOS_UNDEFINED,
                                         SDL_WINDOWPOS_UNDEFINED, 640, 360, 0);
  SDL_Renderer * renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);
  SDL_Texture * texture;

  // enter main event loop
  std::cout << "===== Hello AR Drone ====" << std::endl;
  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration dur(0.04);
    dur.sleep();
    SDL_PollEvent(&event);
    if (event.type == SDL_QUIT) {
      break;
    }

    // render image, if there is a new one available
    cv::Mat image;
    if (subscriber.getLastImage(image)) {

      // TODO: add overlays to the cv::Mat image, e.g. text

      // http://stackoverflow.com/questions/22702630/converting-cvmat-to-sdl-texture
      //Convert to SDL_Surface
      IplImage opencvimg2 = (IplImage) image;
      IplImage* opencvimg = &opencvimg2;
      auto frameSurface = SDL_CreateRGBSurfaceFrom(
          (void*) opencvimg->imageData, opencvimg->width, opencvimg->height,
          opencvimg->depth * opencvimg->nChannels, opencvimg->widthStep,
          0xff0000, 0x00ff00, 0x0000ff, 0);
      if (frameSurface == NULL) {
        std::cout << "Couldn't convert Mat to Surface." << std::endl;
      } else {
        texture = SDL_CreateTextureFromSurface(renderer, frameSurface);
        SDL_FreeSurface(frameSurface);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
      }

      //HERE
      arp::VisualInertialTracker VITracker;
      std::shared_ptr<arp::Frontend> spfrontend(&frontend);
      VITracker.setFrontend(spfrontend);

      arp::ViEkf viekf;
      std::shared_ptr<arp::ViEkf> spviEkf(&viekf);
      VITracker.setEstimator(spviEkf);
      Eigen::Matrix4d T_WT;
      T_WT << 1, 0, 0, 0,
              0, 0, -1, 0,
              0, 1, 0, 0,
              0, 0, 0, 1;
      arp::kinematics::Transformation T_wt(T_WT);
      viekf.setTarget(0, T_wt, 0.16);

      Eigen::Matrix4d T_SC_mat;
      T_SC_mat << -0.00195087, -0.03257782,  0.99946730, 0.17409445,
                  -0.99962338, -0.02729525, -0.00284087, 0.02255834,
                   0.02737326, -0.99909642, -0.03251230, 0.00174723,
                   0.00000000,  0.00000000,  0.00000000, 1.00000000;
      arp::kinematics::Transformation T_SC(T_SC_mat);

      viekf.setCameraExtrinsics(T_SC);
      viekf.setCameraIntrinsics(frontend.undistortedCameraModel());
      
      VITracker.setVisualisationCallback(std::bind(
        &arp::StatePublisher::publish, &publish,
        std::placeholders::_1, std::placeholders::_2));



      arp::Frontend::DetectionVec detections;
      int noDetect = frontend.detect(image, detections);
      geometry_msgs::PoseStamped viz;
      for(int i = 0; i < noDetect; i++){
        std::cout << "success" << std::endl;
        Eigen::Matrix<double, 7, 1> pq = detections[i].T_CT.inverse().coeffs();
        viz.pose.position.x = pq[0]; viz.pose.position.y = pq[1]; viz.pose.position.z = pq[2];
        viz.pose.orientation.x = pq[3]; viz.pose.orientation.y = pq[4]; viz.pose.orientation.z = pq[5]; viz.pose.orientation.w = pq[6];
        viz.header.frame_id = "target"; //other headers ?
        pubShow.publish(viz);

      }
    }

    //Multiple Key Capture Begins
    const Uint8 *state = SDL_GetKeyboardState(NULL);



    // check states!
    auto droneStatus = autopilot.droneStatus();



    // command
    if (state[SDL_SCANCODE_ESCAPE]) {
      std::cout << "ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=" << droneStatus;
      bool success = autopilot.estopReset();
      if(success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_T]) {
      std::cout << "Taking off...                          status=" << droneStatus;
      bool success = autopilot.takeoff();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

	int moves[4] = {0,0,0,0};
	bool move = false;

    if (state[SDL_SCANCODE_UP]){
      std::cout << "Moving Forward               status=" << droneStatus;
      moves[0]++;move = true;
    }

    if (state[SDL_SCANCODE_DOWN]){
      std::cout << "Moving Down               status=" << droneStatus;
		moves[0]--;move = true;
    }
    if (state[SDL_SCANCODE_LEFT]){
      std::cout << "Moving Left               status=" << droneStatus;
      moves[1]++;move = true;

    }
    if (state[SDL_SCANCODE_RIGHT]){
      std::cout << "Moving Right               status=" << droneStatus;
      moves[1]--;move = true;
    }

    if (state[SDL_SCANCODE_W]){
      std::cout << "Moving Up               status=" << droneStatus;
      moves[2]++;move = true;
    }

    if (state[SDL_SCANCODE_S]){
      std::cout << "Moving Down              status=" << droneStatus;
      moves[2]--;move = true;
    }

    if (state[SDL_SCANCODE_A]){
      std::cout << "Yaw Left              status=" << droneStatus;
      moves[3]++;move = true;
    }

	if (state[SDL_SCANCODE_D]){
      std::cout << "Yaw Right            status=" << droneStatus;
      moves[3]--;move = true;
    }

	bool success = autopilot.manualMove(moves[0], moves[1], moves[2], moves[3]);
	if (success && move) {
	std::cout << " [ MOVE OK ]" << std::endl;
	} else {
	std::cout << " [MOVE FAIL]" << std::endl;
	}

    if (state[SDL_SCANCODE_L]) {
      std::cout << "Going to land...                       status=" << droneStatus;
      bool success = autopilot.land();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_C]) {
      std::cout << "Requesting flattrim calibration...     status=" << droneStatus;
      bool success = autopilot.flattrimCalibrate();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    // TODO: process moving commands when in state 3,4, or 7
  }

  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

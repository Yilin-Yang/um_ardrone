#include <ros/ros.h>
#include "um_ardrone/navdata_altitude_rebroadcaster.h"
#include "um_ardrone/imu_rebroadcaster.h"
#include "um_ardrone/odometry_rebroadcaster.h"
#include "um_ardrone/tum_ekf_rebroadcaster.h"
  using um_ardrone::ImuRebroadcaster;
  using um_ardrone::NavdataAltitudeRebroadcaster;
  using um_ardrone::OdometryRebroadcaster;
  using um_ardrone::Rebroadcaster;
  using um_ardrone::TumEkfRebroadcaster;

#include <array>
  using std::array;
#include <memory>
  using std::make_unique;
  using std::unique_ptr;
#include <stdexcept>
  using std::runtime_error;
#include <string>
  using std::string;
#include <vector>
  using std::vector;

struct RebroadcasterParams
{
  explicit RebroadcasterParams(const string& name_in)
  : name{name_in}
  {}

  string paramEnable()       const { return string{"enable_"} + name;             }
  string paramSubTopic()     const { return name + string{"_sub_topic"};          }
  string paramPubTopic()     const { return name + string{"_pub_topic"};          }
  string paramTfFrame()      const { return name + string{"_tf_frame"};           }
  string paramSubQueueSize() const { return name + string{"_sub_queue_size"};     }
  string paramPubQueueSize() const { return name + string{"_pub_queue_size"};     }

  // standardized parameter names that don't have associated member vars
  string paramChildTfFrame() const { return name + string{"_child_tf_frame"};     }
  string paramEulerCovar()   const { return name + string{"_euler_covariance"};   }
  string paramAngVelCovar()  const { return name + string{"_ang_vel_covariance"}; }
  string paramLinAccCovar()  const { return name + string{"_lin_acc_covariance"}; }
  string paramPoseCovar()    const { return name + string{"_pose_covariance"};    }
  string paramTwistCovar()   const { return name + string{"_twist_covariance"};   }

  string name;

  bool should_rebroadcast;
  string sub_topic;
  string pub_topic;
  string tf_frame;
  int max_sub_queue_size;
  int max_pub_queue_size;
}; // struct RebroadcasterParams

unique_ptr<Rebroadcaster> makeImuRebroadcaster(
  const RebroadcasterParams& params,
  ros::NodeHandle& node_handle
)
{
  vector<double> euler_covar, ang_vel_covar, lin_acc_covar;
  node_handle.getParam(params.paramEulerCovar(),  euler_covar);
  node_handle.getParam(params.paramAngVelCovar(), ang_vel_covar);
  node_handle.getParam(params.paramLinAccCovar(), lin_acc_covar);

  static constexpr size_t COVAR_MAT_SIZE = 9;
  if      (euler_covar.size() != COVAR_MAT_SIZE
      or ang_vel_covar.size() != COVAR_MAT_SIZE
      or lin_acc_covar.size() != COVAR_MAT_SIZE)
  {
    throw runtime_error{
      "One or more badly sized IMU covariance matrices! "
      "(All should have 9 elements!)"
    };
  }

  array<double, COVAR_MAT_SIZE> euler_covar_arr,
                                ang_vel_covar_arr,
                                lin_acc_covar_arr;

  static constexpr size_t NUM_MATRIX_CHARS = sizeof(double) * COVAR_MAT_SIZE;
  memcpy(euler_covar_arr.data(),   euler_covar.data(),   NUM_MATRIX_CHARS);
  memcpy(ang_vel_covar_arr.data(), ang_vel_covar.data(), NUM_MATRIX_CHARS);
  memcpy(lin_acc_covar_arr.data(), lin_acc_covar.data(), NUM_MATRIX_CHARS);

  return make_unique<ImuRebroadcaster>(
    params.sub_topic,
    params.pub_topic,
    params.tf_frame,
    euler_covar_arr,
    ang_vel_covar_arr,
    lin_acc_covar_arr
  );
}

template <typename T>
unique_ptr<T> makeOdometryRebroadcaster(
  const RebroadcasterParams& params,
  ros::NodeHandle& node_handle
)
{
  vector<double> pose_covar, twist_covar;
  node_handle.getParam(params.paramPoseCovar(),  pose_covar);
  node_handle.getParam(params.paramTwistCovar(), twist_covar);

  string child_tf_frame_id;
  node_handle.getParam(params.paramChildTfFrame(), child_tf_frame_id);

  static constexpr size_t COVAR_MAT_SIZE = 36;
  if     (pose_covar.size() != COVAR_MAT_SIZE
      or twist_covar.size() != COVAR_MAT_SIZE)
  {
    throw runtime_error{
      string{"One or more badly sized covariance matrices for "}
      + params.name
      + string{" (All should have 36 elements!)"}
    };
  }

  array<double, COVAR_MAT_SIZE> pose_covar_arr, twist_covar_arr;

  static constexpr size_t NUM_MATRIX_CHARS = sizeof(double) * COVAR_MAT_SIZE;

  memcpy(pose_covar_arr.data(),  pose_covar.data(),  NUM_MATRIX_CHARS);
  memcpy(twist_covar_arr.data(), twist_covar.data(), NUM_MATRIX_CHARS);

  return make_unique<T>(
    params.sub_topic,
    params.pub_topic,
    params.tf_frame,
    child_tf_frame_id,
    pose_covar_arr,
    twist_covar_arr,
    params.max_sub_queue_size,
    params.max_pub_queue_size
  );
}

unique_ptr<Rebroadcaster> makeSonarRebroadcaster(
  const RebroadcasterParams& params,
  ros::NodeHandle& node_handle
)
{
  vector<double> pose_covar;
  node_handle.getParam(params.paramPoseCovar(),  pose_covar);

  static constexpr size_t COVAR_MAT_SIZE = 36;
  if (pose_covar.size() != COVAR_MAT_SIZE)
  {
    throw runtime_error{
      string{"Badly sized SONAR altimeter covariance matrice! Should have "}
      + std::to_string(COVAR_MAT_SIZE) + string{" elements, but instead has "}
      + std::to_string(pose_covar.size())
    };
  }

  array<double, COVAR_MAT_SIZE> pose_covar_arr;
  static constexpr size_t NUM_MATRIX_CHARS = sizeof(double) * COVAR_MAT_SIZE;
  memcpy(pose_covar_arr.data(), pose_covar.data(), NUM_MATRIX_CHARS);

  return make_unique<NavdataAltitudeRebroadcaster>(
    params.sub_topic,
    params.pub_topic,
    params.tf_frame,
    pose_covar_arr,
    params.max_sub_queue_size,
    params.max_pub_queue_size
  );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "um_rebroadcast");
  ros::NodeHandle node_handle{"~"};

  vector<RebroadcasterParams> rebroadcaster_params{
    RebroadcasterParams("imu"),
    RebroadcasterParams("odometry"),
    RebroadcasterParams("sonar"),
    RebroadcasterParams("tum"),
  };

  vector<unique_ptr<Rebroadcaster>> rebroadcasters;

  for (auto& r : rebroadcaster_params)
  {
    node_handle.getParam(r.paramEnable(), r.should_rebroadcast);
    if (not r.should_rebroadcast) continue;
    node_handle.getParam(r.paramSubTopic(),     r.sub_topic);
    node_handle.getParam(r.paramPubTopic(),     r.pub_topic);
    node_handle.getParam(r.paramTfFrame(),      r.tf_frame);
    node_handle.getParam(r.paramSubQueueSize(), r.max_sub_queue_size);
    node_handle.getParam(r.paramPubQueueSize(), r.max_pub_queue_size);

    if (r.name == "imu")
    {
      rebroadcasters.emplace_back(makeImuRebroadcaster(r, node_handle));
    }
    else if (r.name == "odometry")
    {
      rebroadcasters.emplace_back(
        makeOdometryRebroadcaster<OdometryRebroadcaster>(r, node_handle)
      );
    }
    else if (r.name == "sonar")
    {
      rebroadcasters.emplace_back(makeSonarRebroadcaster(r, node_handle));
    }
    else if (r.name == "tum")
    {
      rebroadcasters.emplace_back(
        makeOdometryRebroadcaster<TumEkfRebroadcaster>(r, node_handle)
      );
    }
  }

  if (rebroadcasters.empty())
  {
    ROS_ERROR("ERROR: No rebroadcaster objects initialized! Exiting.");
    return 1;
  }

  ros::spin();

  ROS_INFO("Initializing um_rebroadcast node.");
  ROS_INFO("Loading parameters...");
}

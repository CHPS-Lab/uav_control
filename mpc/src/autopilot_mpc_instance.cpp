#include <mpc/mpc_controller.h>
#include <mpc/mpc_params.h>

#include "autopilot/autopilot.h"

template class autopilot::AutoPilot<mpc::MpcController<float>,
                          mpc::MpcParams<float>>;
template class autopilot::AutoPilot<mpc::MpcController<double>,
                          mpc::MpcParams<double>>;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autopilot");
  autopilot::AutoPilot<mpc::MpcController<float>,
                       mpc::MpcParams<float>> autopilot;

  ros::spin();

  return 0;
}

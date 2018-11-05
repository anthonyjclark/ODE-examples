#include <ode/ode.h>
#include <iostream>
#include <logger.hpp>
#include <cxxopts.hpp>


void handle_collisions(void *sim, dGeomID geom1, dGeomID geom2);


struct UGV {

  dWorldID world_;
  dSpaceID space_;
  dJointGroupID contact_group_;

  dBodyID chassis_;
  dGeomID chassis_geom_;

  dBodyID wheels_[4];
  dGeomID wheel_geoms_[4];

  dJointID motors_[4];

  dGeomID ground_geom_;

  dReal time_;
  dReal time_step_;
  dReal vis_step_;
  dReal next_vis_output_;

  review::logger rl_;

  UGV(dReal gravity, dReal density, dReal offset[3], dReal chassis_size[3], dReal wheel_radius,
      dReal time_step, double vis_step) :
      time_(0), time_step_(time_step), vis_step_(vis_step), rl_(vis_step), next_vis_output_(0)
  {
    // Initialize ODE, create the world, and create the collision space

    dInitODE2(0);

    world_ = dWorldCreate();
    dWorldSetGravity(world_, 0.0, gravity, 0.0);

    space_ = dSimpleSpaceCreate(nullptr);
    contact_group_ = dJointGroupCreate(0);


    // Chassis

    chassis_ = dBodyCreate(world_);
    dBodySetPosition(chassis_, offset[0], offset[1] + chassis_size[1]/2, offset[2]);

    dMass mass;
    dMassSetBox(&mass, density, chassis_size[0], chassis_size[1], chassis_size[2]);
    dBodySetMass(chassis_, &mass);

    chassis_geom_ = dCreateBox(space_, chassis_size[0], chassis_size[1], chassis_size[2]);
    dGeomSetBody(chassis_geom_, chassis_);

    rl_.add_box("chassis", chassis_size[0], chassis_size[1], chassis_size[2]);


    // Wheels

    for (int wheel = 0; wheel < 4; ++wheel) {

      // First two wheel are front wheels (x-axis is forward)
      dReal wheel_x = offset[0] + ((wheel < 2) ? 1 : -1) * chassis_size[0]/2;

      dReal wheel_y = offset[1] + chassis_size[1]/2;

      // First and third wheels are the left wheels (negative z-axis)
      dReal wheel_z = offset[2] + ((wheel % 2 != 0) ? 1 : -1) * chassis_size[2]/2;

      wheels_[wheel] = dBodyCreate(world_);
      dBodySetPosition(wheels_[wheel], wheel_x, wheel_y, wheel_z);

      dMassSetSphere(&mass, density, wheel_radius);
      dBodySetMass(wheels_[wheel], &mass);

      wheel_geoms_[wheel] = dCreateSphere(space_, wheel_radius);
      dGeomSetBody(wheel_geoms_[wheel], wheels_[wheel]);

      motors_[wheel] = dJointCreateHinge(world_, nullptr);
      dJointAttach(motors_[wheel], chassis_, wheels_[wheel]);
      dJointSetHingeAnchor(motors_[wheel], wheel_x, wheel_y, wheel_z);
      dJointSetHingeAxis(motors_[wheel], 0, 0, 1);
      dJointSetHingeParam(motors_[wheel], dParamFMax, 1);

      rl_.add_sphere("wheel" + std::to_string(wheel), wheel_radius);
    }


    // Create the ground plane

    ground_geom_ = dCreatePlane(space_, 0, 1, 0, 0);


    // Add first visualization frame

    add_vis_frame();

  }

  bool step(dReal time_stop)
  {
    dSpaceCollide(space_, this, &handle_collisions);
    dWorldStep(world_, time_step_);
    dJointGroupEmpty(contact_group_);

    time_ += time_step_;
    if (time_ >= next_vis_output_) {
      add_vis_frame();
      next_vis_output_ += vis_step_;
    }

    return time_ < time_stop;
  }

  void add_vis_frame()
  {
    rl_.new_frame();

    const auto pos = dBodyGetPosition(chassis_);
    const auto quat = dBodyGetQuaternion(chassis_);
    rl_.add_to_frame("chassis",
        pos[0], pos[1], pos[2],
        quat[3], quat[0], quat[1], quat[2]);

    for (int wheel = 0; wheel < 4; ++wheel) {
      const auto pos = dBodyGetPosition(wheels_[wheel]);
      const auto quat = dBodyGetQuaternion(wheels_[wheel]);
      rl_.add_to_frame("wheel" + std::to_string(wheel),
          pos[0], pos[1], pos[2],
          quat[3], quat[0], quat[1], quat[2]);
    }
  }
};


void handle_collisions(void *sim, dGeomID geom1, dGeomID geom2)
{
  auto sim_data = static_cast<UGV*>(sim);

  // Only handle collisions involving the ground
  if (!(geom1 == sim_data->ground_geom_ || geom2 == sim_data->ground_geom_)) return;

  // Get the rigid bodies associated with the geometries
  dBodyID body1 = dGeomGetBody(geom1);
  dBodyID body2 = dGeomGetBody(geom2);

  // Maximum number of contacts to create between bodies (see ODE documentation)
  const int MAX_NUM_CONTACTS = 8;
  dContact contacts[MAX_NUM_CONTACTS];

  // Add collision joints
  int numc = dCollide(geom1, geom2, MAX_NUM_CONTACTS, &contacts[0].geom, sizeof(dContact));
  for (int i = 0; i < numc; ++i) {

    contacts[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 |
        dContactSlip1 | dContactSlip2;

    contacts[i].surface.mu = 1.5;
    contacts[i].surface.slip1 = 0.1;
    contacts[i].surface.slip2 = 0.1;
    contacts[i].surface.soft_erp = 0.5;
    contacts[i].surface.soft_cfm = 0.3;

    // struct dSurfaceParameters {
    //      int mode;
    //      dReal mu;
    //      dReal mu2;
    //      dReal rho;
    //      dReal rho2;
    //      dReal rhoN;
    //      dReal bounce;
    //      dReal bounce_vel;
    //      dReal soft_erp;
    //      dReal soft_cfm;
    //      dReal motion1, motion2, motionN;
    //      dReal slip1, slip2;
    // };

    dJointID contact = dJointCreateContact(sim_data->world_, sim_data->contact_group_, &contacts[i]);
    dJointAttach(contact, body1, body2);
  }
}


int main(int argc, char* argv[])
{
  cxxopts::Options options("UGV Simulation", "Simulate a simple UGV.");
  options.add_options()
      ("radius", "Wheel radius", cxxopts::value<dReal>())
      ;

  auto result = options.parse(argc, argv);

  constexpr dReal density = 200;
  constexpr dReal gravity_y = -9.81f;

  dReal ugv1_wheel_radius =   result["radius"].as<dReal>();
  dReal ugv1_size[] = {0.25, 0.1, 0.25};
  dReal ugv1_position[] = {0, ugv1_wheel_radius * 1.1f, 0};

  constexpr dReal TIME_STOP = 10;
  constexpr dReal TIME_STEP = 0.001;
  constexpr double VIS_STEP = 1.0 / 60.0;

  UGV ugv1(gravity_y, density, ugv1_position, ugv1_size, ugv1_wheel_radius, TIME_STEP, VIS_STEP);

  do {
    // Do stuff...
    for (int wheel = 0; wheel < 4; ++wheel) {
      dReal ang_vel = wheel % 2 == 0 ? 4 : 3.75f;
      dJointSetHingeParam(ugv1.motors_[wheel], dParamVel, ang_vel);
    }
  } while (ugv1.step(TIME_STOP));

  std::cout << ugv1.rl_.to_string(true) << std::endl;

  // Cleanup (destroy and close) <-- I don't feel like it

  return EXIT_SUCCESS;

}

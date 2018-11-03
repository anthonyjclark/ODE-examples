#include <ode/ode.h>
#include <iostream>

//TODO:AJC gimpact? vs opcode


struct CollisionData {
  dWorldID world;
  dJointGroupID contact_group;
};


void handle_collisions(void *data, dGeomID geom1, dGeomID geom2)
{
  auto collision_data = static_cast<CollisionData*>(data);

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

    contacts[i].surface.mu = 50.0;
    contacts[i].surface.soft_erp = 0.96;
    contacts[i].surface.soft_cfm = 2.00;

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

    dJointID contact = dJointCreateContact(collision_data->world,
        collision_data->contact_group, &contacts[i]);

    dJointAttach(contact, body1, body2);
  }
}

int main()
{
  constexpr dReal density = 1.0;
  constexpr dReal radius = 0.3;
  constexpr double restitution = 0.9;
  constexpr double damping = 0.1;
  constexpr dReal starting_height = 10.0;
  constexpr dReal gravity_y = -9.81;
  const std::string name{"sphere1"};

  //
  // Initialize ODE, create the world, and create the collision space
  //   This must be done first.
  //

  dInitODE2(0);

  dWorldID world = dWorldCreate();
  dWorldSetGravity(world, 0.0, gravity_y, 0.0);

  dSpaceID space = dSimpleSpaceCreate(0);
  dJointGroupID collision_contact_group = dJointGroupCreate(0);
  CollisionData collision_data {world, collision_contact_group};


  //
  // Create the sphere object
  //


  // Rigid body for dynamics
  dBodyID sphere = dBodyCreate(world);
  dBodySetPosition(sphere, 0.0, starting_height, 0.0);

  // Mass
  dMass sphere_mass;
  dMassSetSphere(&sphere_mass, density, radius);
  dBodySetMass(sphere, &sphere_mass);

  // Geometry for collisions
  dGeomID sphere_geom = dCreateSphere(space, radius);
  dGeomSetBody(sphere_geom, sphere);

  //
  // Create the ground plane
  //

  dGeomID ground_geom = dCreatePlane(space, 0, 1, 0, 0);


  //
  // Simulate the world for some amount of time
  //

  constexpr dReal TIME_STOP = 10;
  constexpr dReal TIME_STEP = 0.001;
  constexpr dReal OUTPUT_STEP = 0.05;

  std::cout << "Time \"Height (R=" << restitution << ")\"\n";
  std::cout << 0 << " " << starting_height << std::endl;

  dReal next_output_time = OUTPUT_STEP;
  for (dReal time = 0.0; time < TIME_STOP + TIME_STEP/2.0; time += TIME_STEP) {

    dSpaceCollide(space, &collision_data, &handle_collisions);
    dWorldStep(world, static_cast<dReal>(TIME_STEP));
    dJointGroupEmpty(collision_contact_group);

    if (time > next_output_time) {
      const auto sphere_position = dBodyGetPosition(sphere);
      std::cout << time << " " << sphere_position[1] << std::endl;
      next_output_time += OUTPUT_STEP;
    }
  }

  //
  // Cleanup
  //

  // TODO:AJC other destroy
  dSpaceDestroy(space);
  dWorldDestroy(world);
  dCloseODE();

  return EXIT_SUCCESS;

}

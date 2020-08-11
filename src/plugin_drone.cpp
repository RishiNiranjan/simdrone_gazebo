#include "plugin_drone.h"

#include <cmath>
#include <stdlib.h>
#include <iostream>

namespace gazebo
{

    DroneSimpleController::DroneSimpleController()
    {
        m_posCtrl = false;
        m_velMode = false;
        navi_state = TAKINGOFF_MODEL;
    }

    DroneSimpleController::~DroneSimpleController()
    {
        // Deprecated since Gazebo 8.
        //event::Events::DisconnectWorldUpdateBegin(updateConnection);

        // node->shutdown();
        // delete node;
    }

    void DroneSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        world = _model->GetWorld();

        link = _model->GetLink();
        link_name_ = link->GetName();                 //base_link

        if (!link)
        {
            gzerr << "gazebo_ros_baro plugin error: bodyName does not exist\n";
            return;
        }

        if (!_sdf->HasElement("maxForce"))
            max_force_ = -1;
        else
            max_force_ = _sdf->GetElement("maxForce")->Get<double>();

        if (!_sdf->HasElement("motionSmallNoise"))
            motion_small_noise_ = 0;
        else
            motion_small_noise_ = _sdf->GetElement("motionSmallNoise")->Get<double>();

        if (!_sdf->HasElement("motionDriftNoise"))
            motion_drift_noise_ = 0;
        else
            motion_drift_noise_ = _sdf->GetElement("motionDriftNoise")->Get<double>();

        if (!_sdf->HasElement("motionDriftNoiseTime"))
            motion_drift_noise_time_ = 1.0;
        else
            motion_drift_noise_time_ = _sdf->GetElement("motionDriftNoiseTime")->Get<double>();

        // get inertia and mass of quadrotor body
        inertia = link->GetInertial()->PrincipalMoments();
        mass = link->GetInertial()->Mass();

        gzdbg <<inertia << " " <<mass << "\n";

        this->node = transport::NodePtr(new transport::Node());
        this->node->Init();

        // subscribe imu
        this->imu_subscriber_ = this->node->Subscribe("/drone/imu", &DroneSimpleController::ImuCallback, this);
        if (this->imu_subscriber_)
            gzdbg << "subscribed to /drone/imu\n";

        // subscribe keyboard control
        this->cmd_subscriber_ = this->node->Subscribe("/cmd/val", &DroneSimpleController::CmdCallback, this);
        if (this->cmd_subscriber_)
            gzdbg << "subscribed to /cmd/val\n";

        LoadControllerSettings(_model, _sdf);

        Reset();

        updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DroneSimpleController::Update, this));

        gzdbg << "rishi plugin_drone is loaded\n";
    }

    void DroneSimpleController::ImuCallback(ConstIMUPtr &imu)
    {
        //directly read the quternion from the IMU data
        pose.Rot().Set(imu->orientation().w(), imu->orientation().x(), imu->orientation().y(), imu->orientation().z());
        euler = pose.Rot().Euler();
        angular_velocity = pose.Rot().RotateVector(ignition::math::Vector3d(imu->angular_velocity().x(), imu->angular_velocity().y(), imu->angular_velocity().z()));
    }


    void DroneSimpleController::CmdCallback(ConstTwistPtr &cmd)
    {
        cmd_val = *cmd;
        static common::Time last_sim_time = world->SimTime();
        static double time_counter_for_drift_noise = 0;
        static double drift_noise[4] ={ 0.0, 0.0, 0.0, 0.0 };
        // Get simulator time
        common::Time cur_sim_time = world->SimTime();
        double dt = (cur_sim_time - last_sim_time).Double();
        // save last time stamp
        last_sim_time = cur_sim_time;

        // generate noise
        if (time_counter_for_drift_noise > motion_drift_noise_time_)
        {
            drift_noise[0] = 2*motion_drift_noise_*(drand48()-0.5);
            drift_noise[1] = 2*motion_drift_noise_*(drand48()-0.5);
            drift_noise[2] = 2*motion_drift_noise_*(drand48()-0.5);
            drift_noise[3] = 2*motion_drift_noise_*(drand48()-0.5);
            time_counter_for_drift_noise = 0.0;
        }
        time_counter_for_drift_noise += dt;
        //TODO
        // cmd_val.angular.x += drift_noise[0] + 2*motion_small_noise_*(drand48()-0.5);
        // cmd_val.angular.y += drift_noise[1] + 2*motion_small_noise_*(drand48()-0.5);
        // cmd_val.angular.z += drift_noise[3] + 2*motion_small_noise_*(drand48()-0.5);
        // cmd_val.linear.z += drift_noise[2] + 2*motion_small_noise_*(drand48()-0.5);
        // msgs::Set(twist_msg.mutable_linear(), ignition::math::Vector3d(0, 0, 0.0));
        // msgs::Set(twist_msg.mutable_angular(), ignition::math::Vector3d(0.0,speed,0.0));
    }

    void DroneSimpleController::Update()
    {
        // Get new commands/state
        // callback_queue_.callAvailable();

        // Get simulator time
        common::Time sim_time = world->SimTime();
        double dt = (sim_time - last_time).Double();
        if (dt == 0.0)
            return;

        UpdateState(dt);
        UpdateDynamics(dt);

        // save last time stamp
        last_time = sim_time;
    }

    void DroneSimpleController::UpdateState(double dt)
    {
        if (navi_state == TAKINGOFF_MODEL)
        {
            m_timeAfterCmd += dt;
            if (m_timeAfterCmd > 0.5)
            {
                navi_state = FLYING_MODEL;
                std::cout << "Entering flying model!" << std::endl;
            }
        }
        else if (navi_state == LANDING_MODEL)
        {
            m_timeAfterCmd += dt;
            if (m_timeAfterCmd > 1.0)
            {
                navi_state = LANDED_MODEL;
                std::cout << "Landed!" << std::endl;
            }
        }
        else
            m_timeAfterCmd = 0;
    }

    void DroneSimpleController::UpdateDynamics(double dt)
    {
        ignition::math::Vector3d force, torque;

        acceleration = (link->WorldLinearVel() - velocity) / dt;
        velocity = link->WorldLinearVel();

        //convert the acceleration and velocity into the body frame
        ignition::math::Vector3d body_vel = pose.Rot().RotateVector(velocity);
        ignition::math::Vector3d body_acc = pose.Rot().RotateVector(acceleration);

        ignition::math::Vector3d poschange = pose.Pos() - position;
        position = pose.Pos();

        // Get gravity
        ignition::math::Vector3d gravity_body = pose.Rot().RotateVector(world->Gravity());
        double gravity = gravity_body.Length();
        double load_factor = gravity * gravity / world->Gravity().Dot(gravity_body); // Get gravity

        // Rotate vectors to coordinate frames relevant for control
        ignition::math::Quaterniond heading_quaternion(cos(euler.Z() / 2), 0, 0, sin(euler.Z() / 2));
        ignition::math::Vector3d velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
        ignition::math::Vector3d acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
        ignition::math::Vector3d angular_velocity_body = pose.Rot().RotateVectorReverse(angular_velocity);

        // update controllers
        force.Set(0.0, 0.0, 0.0);
        torque.Set(0.0, 0.0, 0.0);

        //normal control
        if (navi_state == FLYING_MODEL) //&& cmd_val.linear.x >= 0 && cmd_val.linear.y >= 0)
        {
            //hovering
            double pitch_command = controllers_.velocity_x.update(cmd_val.linear().x(), velocity_xy.X(), acceleration_xy.X(), dt) / gravity;
            double roll_command = -controllers_.velocity_y.update(cmd_val.linear().y(), velocity_xy.Y(), acceleration_xy.Y(), dt) / gravity;
            torque.X() = inertia.X() * controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
            torque.Y() = inertia.Y() * controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);
        }
        else
        {
            //control by velocity
            if (m_velMode)
            {
                double pitch_command = controllers_.velocity_x.update(cmd_val.angular().x(), velocity_xy.X(), velocity_xy.X(), dt) / gravity;
                double roll_command = -controllers_.velocity_y.update(cmd_val.angular().y(), velocity_xy.Y(), velocity_xy.Y(), dt) / gravity;
                torque.X() = inertia.X() * controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
                torque.Y() = inertia.Y() * controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);
            }
            else
            {
                //control by tilting
                torque.X() = inertia.X() * controllers_.roll.update(cmd_val.angular().x(), euler.X(), angular_velocity_body.X(), dt);
                torque.Y() = inertia.Y() * controllers_.pitch.update(cmd_val.angular().y(), euler.Y(), angular_velocity_body.Y(), dt);
            }
        }

        torque.Z() = inertia.Z() *  controllers_.yaw.update(cmd_val.angular().z(), angular_velocity.Z(), 0, dt);
        force.Z()  = mass      * (controllers_.velocity_z.update(cmd_val.linear().z(), velocity.Z(), acceleration.Z(), dt) + load_factor * gravity);

        if (max_force_ > 0.0 && force.Z() > max_force_)
            force.Z() = max_force_;
        if (force.Z() < 0.0)
            force.Z() = 0.0;

        // process robot state information
        if (navi_state == LANDED_MODEL)
        {
        }
        else if (navi_state == FLYING_MODEL)
        {
            link->AddRelativeForce(force);
            link->AddRelativeTorque(torque);
            // gzdbg <<force <<"--"<<torque << "\n";
        }
        else if (navi_state == TAKINGOFF_MODEL)
        {
            link->AddRelativeForce(force * 1.5);
            link->AddRelativeTorque(torque * 1.5);
        }
        else if (navi_state == LANDING_MODEL)
        {
            link->AddRelativeForce(force * 0.8);
            link->AddRelativeTorque(torque * 0.8);
        }
    }
    ////////////////////////////////////////////////////////////////////////////////
    // Reset the controller
    void DroneSimpleController::Reset()
    {
        controllers_.roll.reset();
        controllers_.pitch.reset();
        controllers_.yaw.reset();
        controllers_.velocity_x.reset();
        controllers_.velocity_y.reset();
        controllers_.velocity_z.reset();

        link->SetForce(ignition::math::Vector3d(0, 0, 0));
        link->SetTorque(ignition::math::Vector3d(0, 0, 0));

        // reset state
        pose.Reset();
        velocity.Set();
        angular_velocity.Set();
        acceleration.Set();
        euler.Set();
        // state_stamp = ros::Time();
    }

    void DroneSimpleController::LoadControllerSettings(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        controllers_.roll.Load(_sdf, "rollpitch");
        controllers_.pitch.Load(_sdf, "rollpitch");
        controllers_.yaw.Load(_sdf, "yaw");
        controllers_.velocity_x.Load(_sdf, "velocityXY");
        controllers_.velocity_y.Load(_sdf, "velocityXY");
        controllers_.velocity_z.Load(_sdf, "velocityZ");

        controllers_.pos_x.Load(_sdf, "positionXY");
        controllers_.pos_y.Load(_sdf, "positionXY");
        controllers_.pos_z.Load(_sdf, "positionZ");
    }
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(DroneSimpleController)

} // namespace gazebo

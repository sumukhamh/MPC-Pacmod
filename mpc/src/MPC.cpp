#include <MPC.h>

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

MPC::MPC(){
}

MPC::MPC(ros::NodeHandle n) {  
  current_state << 0, 0, 0, 0, 0, 0;
  trajectory_sub = n.subscribe("/reference_trajectory", 20, &MPC::callback_trajectory, this);
  pose_sub = n.subscribe("/pose", 100, &MPC::callback_pose, this);
  velocity_sub = n.subscribe("/current_velocity", 100, &MPC::callback_velocity, this);
  control_cmd_pub = n.advertise<mpc_msgs::Ctrl>("/mpc_control", 1000);
}

MPC::~MPC() {}

void MPC::update_trajectory(){
  vector<double> ptsx = trajectory_x;
  vector<double> ptsy = trajectory_y;
  double v = current_velocity;
  double px = current_pose.x;
  double py = current_pose.y;
  double psi = current_pose.yaw;        
  double cos_phi = cos(psi);
  double sin_phi = sin(psi);

  // Reference line
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // Convert from map coordinates to vehicle coordinates
  for (unsigned int i = 0; i < ptsx.size(); i++) {
    double delta_x = ptsx[i] - px;
    double delta_y = ptsy[i] - py;
    next_x_vals.push_back(cos_phi * delta_x + sin_phi * delta_y);
    next_y_vals.push_back(cos_phi * delta_y - sin_phi * delta_x);
  }

  // Polynomial Fit
  Eigen::VectorXd ptsx_wrt_car = Eigen::VectorXd::Map(next_x_vals.data(), next_x_vals.size());
  Eigen::VectorXd ptsy_wrt_car = Eigen::VectorXd::Map(next_y_vals.data(), next_y_vals.size());
  coefficients = polyfit(ptsx_wrt_car, ptsy_wrt_car, 3);

  // errors in the current car position
  double cte = polyeval(coefficients, 0);
  double epsi = -atan(coefficients[1]);

  current_state << 0, 0, 0, v, cte, epsi; 
}

void MPC::callback_velocity(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& velocity_msg){
  current_velocity = velocity_msg->vehicle_speed;
}

void MPC::callback_pose(const geometry_msgs::Pose::ConstPtr& pose_msg){
  current_pose.x = pose_msg->position.x;
  current_pose.y = pose_msg->position.y;
  current_pose.z = pose_msg->position.z;
  
  tf::Quaternion quat;
  tf::quaternionMsgToTF(pose_msg->orientation, quat);
  double temp_roll, temp_pitch, temp_yaw;
  tf::Matrix3x3(quat).getRPY(temp_roll, temp_pitch, temp_yaw);

  current_pose.roll = temp_roll;
  current_pose.pitch = temp_pitch;
  current_pose.yaw = temp_yaw;
}

void MPC::callback_trajectory(const geometry_msgs::PoseArray::ConstPtr& trajectory_msg){
  trajectory_x.clear();
  trajectory_y.clear();
  for(unsigned int i = 0; i < trajectory_msg->poses.size(); i++){
    trajectory_x.push_back(trajectory_msg->poses[i].position.x);
    trajectory_y.push_back(trajectory_msg->poses[i].position.y);
  }
  update_trajectory();
}

void MPC::publish_commands(double& velocity, double& steer_value, double& accel_value) {
  mpc_msgs::Ctrl control_msg;
  control_msg.velocity = velocity;
  control_msg.steer_cmd = steer_value;
  control_msg.accel_cmd = accel_value;
  control_cmd_pub.publish(control_msg);
}

void MPC::Solve() {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    double x = current_state[0];
    double y = current_state[1];
    double psi = current_state[2];
    double v = current_state[3];
    double cte = current_state[4];
    double epsi = current_state[5];

    // Set the number of model variables (includes both states and inputs).
    size_t n_vars = 6 * N + 2 * (N - 1);

    // Set the number of constraints
    size_t n_constraints = 6 * N;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (unsigned int i = 0; i < n_vars; i++) {
      vars[i] = 0;
    }

    // initial state
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Non-actuators upper and lower limits
    for (unsigned int i = 0; i < delta_start; i++) {
      vars_lowerbound[i] = -1.0e19;
      vars_upperbound[i] = 1.0e19;
    }

    // delta in radians [-25 deg, +25 deg]
    for (unsigned int i = delta_start; i < a_start; i++) {
      vars_lowerbound[i] = -0.436332;
      vars_upperbound[i] = 0.436332;
    }
    // Acceleration bounds [-1, 1]
    for (unsigned int i = a_start; i < n_vars; i++) {
      vars_lowerbound[i] = -1.0;
      vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (unsigned int i = 0; i < n_constraints; i++) {
      constraints_lowerbound[i] = 0;
      constraints_upperbound[i] = 0;
    }

    // initial state: lower and upper value same as initial value
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coefficients);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    // CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    // CppAD::ipopt::solve<Dvector, FG_eval>(
    //   options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
    //   constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    // ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // steer_value = -solution.x[delta_start] / deg2rad(25); // normalize
    // accel_value = solution.x[a_start];

    publish_commands(v, steer_value, accel_value);
}

void MPC::run(){
  while(ros::ok()){
    MPC::Solve();
    ros::spinOnce();
  }
}

#include <MPC_Util.h>

Pose::Pose(): x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0) {
}

FG_eval::FG_eval(Eigen::VectorXd coeffs) { 
    this->coeffs = coeffs; 
}

void FG_eval::operator()(ADvector &fg, const ADvector &vars) {
    fg[0] = 0;

    // Minimize error
    for (unsigned int t = 0; t < N; t++) {
        fg[0] += 1500 * CppAD::pow(vars[cte_start + t], 2);
        fg[0] += 4000 * CppAD::pow(vars[epsi_start + t], 2);
        fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Penalize use of actuators
    for (unsigned int t = 0; t < N - 1; t++) {
        fg[0] += 8000 * CppAD::pow(vars[delta_start + t], 2);
        fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // Penalize rate of change in actuation
    for (unsigned int t = 0; t < N - 2; t++) {
        fg[0] += 320000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += 1000* CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Initial Constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // Other Constraints
    for (unsigned int t = 0; t < N - 1; t++) {
        // The state at time t.
        AD<double> x0 = vars[x_start + t];
        AD<double> y0 = vars[y_start + t];
        AD<double> psi0 = vars[psi_start + t];
        AD<double> v0 = vars[v_start + t];
        AD<double> cte0 = vars[cte_start + t];
        AD<double> epsi0 = vars[epsi_start + t];

        // The state at time t+1 .
        AD<double> x1 = vars[x_start + t + 1];
        AD<double> y1 = vars[y_start + t + 1];
        AD<double> psi1 = vars[psi_start + t + 1];
        AD<double> v1 = vars[v_start + t + 1];
        AD<double> cte1 = vars[cte_start + t + 1];
        AD<double> epsi1 = vars[epsi_start + t + 1];

        // Actuation at time t.
        AD<double> delta0 = vars[delta_start + t];
        AD<double> a0 = vars[a_start + t];

        // 3rd Order Polynomial
        AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
        AD<double> df0 = coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0;
        AD<double> psides0 = CppAD::atan(df0);


        // Constraints as per the model equations
        fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[2 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
        fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
        fg[2 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[2 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}
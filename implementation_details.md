
###The Model

The state of the model includes `x, y, psi, v, cte, dpsi`, and the acturators of the model include `delta, a`, where

* `x, y` are the position of the vehicle
* `psi` is the orientation
* `v` is the velocity
* `cte` is the cross track error
* `dpsi` is the difference between the current orientation from the desired orientation
* `delta` is the steering angle
* `a` is the acceleration

Update equations include

* `x1 = x0 + v0 * cos(psi0) * dt`
* `y1 = y0 + v0 * sin(psi0) * dt`
* `psi1 = psi0 - v0 * delta0 / Lf* dt`
* `v1 = v0 + a0 * dt`
* `cte1 = f0 - y0 + v0 * sin(dpsi0) * dt`
* `epsi1 = psi0 - psides0 - v0 * delta0 / Lf * dt`

with `x1` denotes the x position at the first time point, `x0` the x position at the zeroth time point, and similar for other variables, and `f0` being the desired position, `psides0` being the desired orientation.

      
      AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2.0*coeffs[2]*x0 + 3.0*coeffs[3]*x0*x0);
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
      
The cost function includes

* The term to ensure desired trajectory:
* `3000 * \sum_t cte[t]^2 + 3000 * \sum_t dpsi[t]^2 + \sum_t (v[t] - v_desired)^2` 
* The term to minimize control inputs:
* `10 * \sum_t delta[t]^2 + 10 * \sum_t a[t]^2`
* The term to smooth control inputs:
* `300 * \sum_t (delta[t] - delta[t-1])^2 + 10 * \sum_t (a[t] - a[t-1])^2`

###Timestep Length and Elapsed Duration (`N` & `dt`)

`N` is set as 10, and `dt` is set as 0.125. I started with `N = 25` and `dt = 0.05`. I tune the parameter by varying `N` and set the corresponding `dt` to make their product `N * dt` in the similar range.

Experiment showed that a larger `dt` improves performance in the turn, since the vehicle is able to project into the entire turn. A smaller `dt` enables the vehicle to stay in the center of the track when first entering into the turn. The final values are chosen with a trade-off in these two aspects.

###Polynomial Fitting and MPC Preprocessing

Waypoints are first transformed from map's coordinate into vehicle's coordinate. This enables more accurate estimation of the polynomial, and the exact representation of cross track error.

      vector<double> ptsx_in_car;
      vector<double> ptsy_in_car;
      for(int i = 0; i < ptsx.size(); ++i) {
        ptsx_in_car.push_back( cos(psi) * (ptsx[i]-px) + sin(psi) * (ptsy[i]-py) );
        ptsy_in_car.push_back(-sin(psi) * (ptsx[i]-px) + cos(psi) * (ptsy[i]-py) );
      }

A polynomial is fitted to the transformed waypoints.

###Model Predictive Control with Latency

For later time steps (`t>1`), the actuations of the previous time step is being used to compensate for the 100ms latency. Moreover, the time step is set as 125ms, in the same magnitude of the latency.

      if(t > 1) {
        delta0 = vars[delta_start + t - 2];
        a0 = vars[a_start + t - 2];
      }
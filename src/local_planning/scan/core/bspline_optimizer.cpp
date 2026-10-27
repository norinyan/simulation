#include "bspline_opt/bspline_optimizer.h"
#include <cstdio>
#include <cstring>
#include <limits>
#include <algorithm>
#include <cmath>
// using namespace std;

namespace scan_planner
{
  namespace
  {
    // A uniform cubic's derivative is contained in its derivative-control
    // polygon. Bound distance between samples without relying on end-to-end
    // distance (which is zero for a closed or returning trajectory).
    int collisionSamples(const Eigen::MatrixXd & points, double interval, double resolution)
    {
      double speed = 0;
      for (int i = 1; i < points.cols(); ++i)
        speed = std::max(speed, (points.col(i)-points.col(i-1)).norm()/interval);
      const double spans = points.cols()-3;
      const double count = std::ceil(std::max(spans, spans*interval*speed/(resolution*0.5)));
      if (!std::isfinite(count) || count > std::numeric_limits<int>::max())
        throw std::runtime_error("SCAN_INVALID_COLLISION_SAMPLING");
      return std::max(1, static_cast<int>(count));
    }
  }

  void BsplineOptimizer::setParam(const Options &options)
  {
    if (options.fixed_boundary_points < 3 || options.max_iterations < 1 ||
        options.clearance <= 0 || options.max_velocity <= 0 || options.max_acceleration <= 0)
      throw std::invalid_argument("SCAN_INVALID_OPTIMIZER_OPTIONS");
    options_ = options;
    lambda1_ = options.lambda_smooth;
    lambda2_ = options.lambda_collision;
    lambda3_ = options.lambda_feasibility;
    lambda4_ = options.lambda_fitness;
    dist0_ = options.clearance;
    cps_.clearance = dist0_;
    max_vel_ = options.max_velocity;
    max_acc_ = options.max_acceleration;
    order_ = 3;
  }

  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &env)
  {
    this->grid_map_ = env;
  }

  double BsplineOptimizer::estimateSegmentYaw(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(diff(1), diff(0));
  }

  double BsplineOptimizer::estimateControlPointYaw(const Eigen::MatrixXd &q, int id) const
  {
    if (q.cols() <= 1)
      return 0.0;

    int prev_id = std::max(0, id - 1);
    int next_id = std::min(static_cast<int>(q.cols()) - 1, id + 1);
    return estimateSegmentYaw(q.col(prev_id), q.col(next_id));
  }

  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.resize(points.cols());
    cps_.points = points;
  }

  void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

  /* This function is very similar to check_collision_and_rebound().
   * It was written separately, just because I did it once and it has been running stably since March 2020.
   * But I will merge then someday.*/
  std::vector<std::vector<Eigen::Vector3d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
  {

    if (flag_first_init)
    {
      cps_.clearance = dist0_;
      cps_.resize(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    constexpr int ENOUGH_INTERVAL = 2;
    int in_id = -1, out_id = -1;
    vector<std::pair<int, int>> segment_ids;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    // Cover every free control point and its adjacent fixed boundary. The
    // entire published trajectory, including its stopping tail, is executable.
    const int i_end = static_cast<int>(init_points.cols())-order_;
    for (int i = order_; i <= i_end; ++i)
    {
      checkDeadline();
      const double length = (init_points.col(i)-init_points.col(i-1)).norm();
      const int samples = std::max(1, static_cast<int>(std::ceil(length/(grid_map_->getResolution()*0.5))));
      for (int sample = 0; sample <= samples; ++sample)
      {
        checkDeadline();
        const double a = 1.0-static_cast<double>(sample)/samples;
        Eigen::Vector3d sample_pt = a * init_points.col(i - 1) + (1 - a) * init_points.col(i);
        double sample_yaw = estimateSegmentYaw(init_points.col(i - 1), init_points.col(i));
        occ = grid_map_->getInflateOccupancy(sample_pt, sample_yaw);
        // cout << setprecision(5);
        // cout << (a * init_points.col(i-1) + (1-a) * init_points.col(i)).transpose() << " occ1=" << occ << endl;

        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
          {
            in_id = i - 1;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        else if (!occ && last_occ)
        {
          out_id = i;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;
        }

        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          if (in_id >= 0 && out_id >= in_id && out_id < init_points.cols())
            segment_ids.push_back(std::pair<int, int>(in_id, out_id));
          else
            std::fprintf(stderr, "SCAN: Skip invalid collision segment: in_id=%d, out_id=%d.", in_id, out_id);
        }
      }
    }

    if (flag_got_start && last_occ)
      throw std::runtime_error("SCAN_TERMINAL_CONTROL_COLLISION");

    /*** a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_paths;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      checkDeadline();
      //cout << "in=" << in.transpose() << " out=" << out.transpose() << endl;
      Eigen::Vector3d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
      ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
      if (ret == ASTAR_RET::SUCCESS)
      {
        vector<Eigen::Vector3d> path = a_star_->getPath();
        if (path.size() < 2)
        {
          std::fprintf(stderr, "SCAN: a star path has less than 2 points, force return!");
          return a_star_paths;
        }
        a_star_paths.push_back(path);
      }
      else
      {
        std::fprintf(stderr, "SCAN: a star error, force return!");
        return a_star_paths;
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      checkDeadline();

      if (i == 0) // first segment
      {
        id_low_bound = order_;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - order_ - 1;
        }
      }
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - order_ - 1;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    // cout << "+++++++++" << endl;
    // for ( int j=0; j<bounds.size(); ++j )
    // {
    //   cout << bounds[j].first << "  " << bounds[j].second << endl;
    // }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> final_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient thrust
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      checkDeadline();
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        final_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

        final_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
      }
      else
      {
        final_segment_ids[i].first = segment_ids[i].first;
        final_segment_ids[i].second = segment_ids[i].second;
      }

      //cout << "final:" << "i = " << i << " first = " << final_segment_ids[i].first << " second = " << final_segment_ids[i].second << endl;
    }

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      checkDeadline();
      if (i >= a_star_paths.size() || a_star_paths[i].size() < 2)
      {
        std::fprintf(stderr, "SCAN: Skip invalid A-star path while assigning control point directions.");
        continue;
      }

      // step 1
      for (int j = final_segment_ids[i].first; j <= final_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        checkDeadline();
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
        int Astar_id = a_star_paths[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_paths[i].size())
        {
          checkDeadline();
          last_Astar_id = Astar_id;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          if (Astar_id < 0 || Astar_id >= (int)a_star_paths[i].size())
            break;

          val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            const double denom = ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]);
            if (std::abs(denom) < 1e-8)
              break;

            intersection_point =
                a_star_paths[i][Astar_id] +
                ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(cps_.points.col(j) - a_star_paths[i][Astar_id]) / denom) // = t
                );

            //cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0)
        {
          cps_.flag_temp[j] = true;
          double length = (intersection_point - cps_.points.col(j)).norm();
          if (length > 1e-5)
          {
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              checkDeadline();
              Eigen::Vector3d sample_pt = (a / length) * intersection_point + (1 - a / length) * cps_.points.col(j);
              double sample_yaw = estimateControlPointYaw(cps_.points, j);
              occ = grid_map_->getInflateOccupancy(sample_pt, sample_yaw);

              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                break;
              }
            }
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(segment_ids[i].second) - cps_.points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (cps_.points.col(segment_ids[i].second) + cps_.points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_paths[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_paths[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_paths[i].size())
        {
          checkDeadline();
          last_Astar_id = Astar_id;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          if (Astar_id < 0 || Astar_id >= (int)a_star_paths[i].size())
            break;

          val = (a_star_paths[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            const double denom = ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]);
            if (std::abs(denom) < 1e-8)
              break;

            intersection_point =
                a_star_paths[i][Astar_id] +
                ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_paths[i][Astar_id]) / denom) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(cps_.points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //step 3
      if (got_intersection_id >= 0)
      {
        for (int j = got_intersection_id + 1; j <= final_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        for (int j = got_intersection_id - 1; j >= final_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    return a_star_paths;
  }

  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // cout << "k=" << k << endl;
    // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
    return (opt->grid_map_->expired() || opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->checkDeadline();
    opt->combineCostRebound(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->checkDeadline();
    opt->combineCostRefine(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    double demarcation = cps_.clearance;
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    force_stop_type_ = DONT_STOP;
    if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
    {
      check_collision_and_rebound();
    }

    /*** calculate distance cost and gradient ***/
    for (auto i = order_; i < end_idx; ++i)
    {
      checkDeadline();
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        checkDeadline();
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        double dist_err = cps_.clearance - dist;
        Eigen::Vector3d dist_grad = cps_.direction[i][j];

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else if (dist_err < demarcation)
        {
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
  }

  void BsplineOptimizer::addCurveDistanceCost(const Eigen::MatrixXd &q, double &cost,
                                               Eigen::MatrixXd &gradient)
  {
    // Optimize the same cubic curve that is published and executed. A safe
    // control polygon can still cut inside an obstacle at a rounded corner.
    // Keep parameter samples fixed throughout each line search, so the cost
    // and its chain-rule gradient describe the same objective function.
    constexpr int subdivisions = 4;
    const double clearance = cps_.clearance;
    for (int span = 0; span+3 < q.cols(); ++span)
    {
      for (int sample = 0; sample <= subdivisions; ++sample)
      {
        checkDeadline();
        const double u = static_cast<double>(sample)/subdivisions;
        const double u2 = u*u, u3 = u2*u;
        const double basis[4] = {
          (1-3*u+3*u2-u3)/6, (4-6*u2+3*u3)/6,
          (1+3*u+3*u2-3*u3)/6, u3/6};
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        for (int j = 0; j < 4; ++j) position += basis[j]*q.col(span+j);
        Eigen::Vector3d distance_gradient;
        const double distance = grid_map_->signedDistance(position, distance_gradient);
        if (!std::isfinite(distance) || !distance_gradient.allFinite())
          throw std::runtime_error("SCAN_INVALID_DISTANCE_GRADIENT");
        const double error = clearance-distance;
        if (error <= 0) continue;
        // The continued cubic penalty has units of distance cubed, whereas
        // smoothness uses squared distances. Normalize by the configured
        // clearance so equal weights do not suppress collision corrections
        // simply because the map is expressed in metres.
        const double weight = (sample == 0 || sample == subdivisions ? 0.5 : 1.0)/
          (subdivisions*clearance);
        double value, derivative;
        if (error < clearance)
        {
          value = error*error*error;
          derivative = 3*error*error;
        }
        else
        {
          value = 3*clearance*error*error-3*clearance*clearance*error+clearance*clearance*clearance;
          derivative = 6*clearance*error-3*clearance*clearance;
        }
        cost += weight*value;
        for (int j = 0; j < 4; ++j)
          gradient.col(span+j) -= weight*derivative*basis[j]*distance_gradient;
      }
    }
  }

  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {

    cost = 0.0;

    int end_idx = q.cols() - order_;

    // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
    double b2 = 1;
    for (auto i = order_ - 1; i < end_idx + 1; ++i)
    {
      checkDeadline();
      Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
      Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();
      // Sliding along an inclined reference while freezing Z would no longer
      // follow the terrain. Only flat spans permit the upstream tangent slack.
      const double a2 = std::abs(v.z())>1e-6 ? 1.0 : 25.0;

      double xdotv = x.dot(v);
      Eigen::Vector3d xcrossv = x.cross(v);

      double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
      cost += f;

      Eigen::Matrix3d m;
      m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
      Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

      gradient.col(i - 1) += df_dx / 6;
      gradient.col(i) += 4 * df_dx / 6;
      gradient.col(i + 1) += df_dx / 6;
    }
  }

  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
  {

    cost = 0.0;

    if (falg_use_jerk)
    {
      Eigen::Vector3d jerk, temp_j;

      for (int i = 0; i < q.cols() - 3; i++)
      {
        checkDeadline();
        /* evaluate jerk */
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        cost += jerk.squaredNorm();
        temp_j = 2.0 * jerk;
        /* jerk gradient */
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
      }
    }
    else
    {
      Eigen::Vector3d acc, temp_acc;

      for (int i = 0; i < q.cols() - 2; i++)
      {
        checkDeadline();
        /* evaluate acc */
        acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
        cost += acc.squaredNorm();
        temp_acc = 2.0 * acc;
        /* acc gradient */
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
      }
    }
  }

  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {

    //#define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS

    cost = 0.0;
    double demarcation = 1.0; // 1m/s, 1m/s/s
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
    double al = ar, bl = -br, cl = cr;

    /* abbreviation */
    double ts, ts_inv2, ts_inv3;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    ts_inv3 = 1 / ts / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      checkDeadline();
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      for (int j = 0; j < 3; j++)
      {
        checkDeadline();
        if (vi(j) > max_vel_ + demarcation)
        {
          double diff = vi(j) - max_vel_;
          cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

          double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) > max_vel_)
        {
          double diff = vi(j) - max_vel_;
          cost += pow(diff, 3) * ts_inv3;
          ;

          double grad = 3 * diff * diff / ts * ts_inv3;
          ;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -(max_vel_ + demarcation))
        {
          double diff = vi(j) + max_vel_;
          cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

          double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -max_vel_)
        {
          double diff = vi(j) + max_vel_;
          cost += -pow(diff, 3) * ts_inv3;

          double grad = -3 * diff * diff / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      checkDeadline();
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 3; j++)
      {
        checkDeadline();
        if (ai(j) > max_acc_ + demarcation)
        {
          double diff = ai(j) - max_acc_;
          cost += ar * diff * diff + br * diff + cr;

          double grad = (2.0 * ar * diff + br) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) > max_acc_)
        {
          double diff = ai(j) - max_acc_;
          cost += pow(diff, 3);

          double grad = 3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -(max_acc_ + demarcation))
        {
          double diff = ai(j) + max_acc_;
          cost += al * diff * diff + bl * diff + cl;

          double grad = (2.0 * al * diff + bl) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -max_acc_)
        {
          double diff = ai(j) + max_acc_;
          cost += -pow(diff, 3);

          double grad = -3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

#else

    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      checkDeadline();
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      //cout << "temp_v * vi=" ;
      for (int j = 0; j < 3; j++)
      {
        checkDeadline();
        if (vi(j) > max_vel_)
        {
          // cout << "fuck VEL" << endl;
          // cout << vi(j) << endl;
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      checkDeadline();
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      //cout << "temp_a * ai=" ;
      for (int j = 0; j < 3; j++)
      {
        checkDeadline();
        if (ai(j) > max_acc_)
        {
          // cout << "fuck ACC" << endl;
          // cout << ai(j) << endl;
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
      //cout << endl;
    }

#endif
  }

  bool BsplineOptimizer::check_collision_and_rebound(void)
  {
    bool added_direction = false;
    int end_idx = cps_.size - order_;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    const int i_end = end_idx;
    for (int i = order_ - 1; i <= i_end; ++i)
    {
      checkDeadline();

      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i), estimateControlPointYaw(cps_.points, i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          checkDeadline();
          cout.precision(2);
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false; // Not really takes effect, just for better hunman understanding.
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          checkDeadline();
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j), estimateControlPointYaw(cps_.points, j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          std::fprintf(stderr, "SCAN: ERROR! the drone is in obstacle. This should not happen.");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.size; ++j)
        {
          checkDeadline();
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j), estimateControlPointYaw(cps_.points, j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.size) // fail to get the obs free point
        {
          std::fprintf(stderr, "SCAN: WARN! terminal point of the current trajectory is in obstacle, skip this planning.");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_paths;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        checkDeadline();
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
        ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
        if (ret == ASTAR_RET::SUCCESS)
        {
          vector<Eigen::Vector3d> path = a_star_->getPath();
          if (path.size() < 2)
          {
            std::fprintf(stderr, "SCAN: A-star path has less than 2 points, drop this collision segment.");
            segment_ids.erase(segment_ids.begin() + i);
            i--;
            continue;
          }
          a_star_paths.push_back(path);
        }
        else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size())
        {
          segment_ids[i].second = segment_ids[i + 1].second;
          segment_ids.erase(segment_ids.begin() + i + 1);
          --i;
          std::fprintf(stderr, "SCAN: A-star failed on one collision segment, merge it with the next segment.");
        }
        else
        {
          std::fprintf(stderr, "SCAN: a star error");
          segment_ids.erase(segment_ids.begin() + i);
          i--;
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        checkDeadline();
        if (i >= a_star_paths.size() || a_star_paths[i].size() < 2)
        {
          std::fprintf(stderr, "SCAN: Skip invalid A-star path while assigning rebound directions.");
          continue;
        }

        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          checkDeadline();
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_paths[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
          while (Astar_id >= 0 && Astar_id < (int)a_star_paths[i].size())
          {
            checkDeadline();
            last_Astar_id = Astar_id;

            if (val >= 0)
              --Astar_id;
            else
              ++Astar_id;

            if (Astar_id < 0 || Astar_id >= (int)a_star_paths[i].size())
              break;

            val = (a_star_paths[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            // cout << val << endl;

            if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
            {
              const double denom = ctrl_pts_law.dot(a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]);
              if (std::abs(denom) < 1e-8)
                break;

              intersection_point =
                  a_star_paths[i][Astar_id] +
                  ((a_star_paths[i][Astar_id] - a_star_paths[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_paths[i][Astar_id]) / denom) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            cps_.flag_temp[j] = true;
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                checkDeadline();
                Eigen::Vector3d sample_pt = (a / length) * intersection_point + (1 - a / length) * cps_.points.col(j);
                double sample_yaw = estimateControlPointYaw(cps_.points, j);
                bool occ = grid_map_->getInflateOccupancy(sample_pt, sample_yaw);

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  added_direction = true;
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          std::fprintf(stderr, "SCAN: Failed to generate direction. It doesn't matter.");
      }

      // Cancel L-BFGS only when its objective actually changed. A virtual
      // control-point collision may yield no usable A* direction; restarting
      // the identical objective would just repeat the same cancellation.
      // Continue optimizing the curve; full-curve safety is still checked.
      if (added_direction)
      {
        force_stop_type_ = STOP_FOR_REBOUND;
        return true;
      }
    }

    return false;
  }

  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
  {
    setBsplineInterval(ts);

    bool flag_success = rebound_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
  {

    setControlPoints(init_points);
    setBsplineInterval(ts);

    bool flag_success = refine_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  void BsplineOptimizer::copyToSolver(double *variables) const
  {
    for (int i = 0; i < variable_num_/3; ++i)
      for (int axis = 0; axis < 3; ++axis)
        variables[3*i+axis] = cps_.points(axis, options_.fixed_boundary_points+i)-solver_origin_[axis];
  }

  void BsplineOptimizer::copyFromSolver(const double *variables)
  {
    for (int i = 0; i < variable_num_/3; ++i)
      for (int axis = 0; axis < 3; ++axis)
        cps_.points(axis, options_.fixed_boundary_points+i) = variables[3*i+axis]+solver_origin_[axis];
  }

  bool BsplineOptimizer::rebound_optimize()
  {
    iter_num_ = 0;
    int start_id = options_.fixed_boundary_points;
    int end_id = this->cps_.size - options_.fixed_boundary_points;
    variable_num_ = 3 * (end_id - start_id);
    if (variable_num_ <= 0) throw std::invalid_argument("SCAN_TOO_FEW_CONTROL_POINTS");
    // L-BFGS normalizes its gradient tolerance by ||x||. Use coordinates
    // relative to the fixed initial pose so changing the world origin cannot
    // change convergence. Costs still query the actual world geometry.
    solver_origin_ = (cps_.points.col(0)+4*cps_.points.col(1)+cps_.points.col(2))/6;
    double final_cost;

    int restart_nums = 0, rebound_times = 0;
    ;
    bool flag_force_return, flag_occ, success;
    new_lambda2_ = lambda2_;
    constexpr int MAX_RESART_NUMS_SET = 3;
    do
    {
      checkDeadline();
      /* ---------- prepare ---------- */
      min_cost_ = std::numeric_limits<double>::max();
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;

      std::vector<double> storage(variable_num_);
      double *q = storage.data();
      copyToSolver(q);

      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = options_.max_iterations;
      // Keep the library default gradient precision (1e-5).

      /* ---------- optimize ---------- */

      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      checkDeadline();
      copyFromSolver(q);



      /* ---------- success temporary, check collision again ---------- */
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
        flag_force_return = false;

        UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
        double tm, tmp;
        traj.getTimeSpan(tm, tmp);
        const int samples = collisionSamples(cps_.points, bspline_interval_, grid_map_->getResolution());
        const double t_step = (tmp-tm)/samples;
        for (int sample = 0; sample <= samples; ++sample)
        {
          checkDeadline();
          const double t = tm+(tmp-tm)*sample/samples;
          Eigen::Vector3d pos = traj.evaluateDeBoorT(t);
          Eigen::Vector3d pos_next = traj.evaluateDeBoorT(std::min(t + t_step, tmp));
          flag_occ = grid_map_->getInflateOccupancy(pos, estimateSegmentYaw(pos, pos_next));
          if (flag_occ)
          {
            //cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

            if (t <= bspline_interval_) // First 3 control points in obstacles!
            {
              cout << cps_.points.col(1).transpose() << "\n"
                   << cps_.points.col(2).transpose() << "\n"
                   << cps_.points.col(3).transpose() << "\n"
                   << cps_.points.col(4).transpose() << endl;
              std::fprintf(stderr, "SCAN: First 3 control points in obstacles! return false, t=%f", t);
              return false;
            }

            break;
          }
        }

        if (!flag_occ)
        {

          success = true;
        }
        else // restart
        {
          restart_nums++;
          initControlPoints(cps_.points, false);
          new_lambda2_ *= 2;

        }
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++;

      }
      else
      {
        std::fprintf(stderr, "SCAN: Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        // while (ros::ok());
      }

    } while ((flag_occ && restart_nums < MAX_RESART_NUMS_SET) ||
             (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    return success;
  }

  bool BsplineOptimizer::refine_optimize()
  {
    iter_num_ = 0;
    int start_id = options_.fixed_boundary_points;
    int end_id = this->cps_.points.cols() - options_.fixed_boundary_points;
    variable_num_ = 3 * (end_id - start_id);
    if (variable_num_ <= 0) throw std::invalid_argument("SCAN_TOO_FEW_CONTROL_POINTS");
    solver_origin_ = (cps_.points.col(0)+4*cps_.points.col(1)+cps_.points.col(2))/6;

    std::vector<double> storage(variable_num_);
    double *q = storage.data();
    double final_cost;

    copyToSolver(q);

    const double origin_lambda2 = lambda2_;
    bool flag_safe = true;
    int iter_count = 0;
    do
    {
      checkDeadline();
      flag_safe = true;
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = options_.max_iterations;
      // Same default precision and local-coordinate contract as rebound.

      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      checkDeadline();
      copyFromSolver(q);
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //pass
      }
      else
      {
        std::fprintf(stderr, "SCAN: Solver error in refining!, return = %d, %s", result, lbfgs::lbfgs_strerror(result));
        lambda2_ = origin_lambda2;
        return false;
      }

      UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
      double tm, tmp;
      traj.getTimeSpan(tm, tmp);
      const int samples = collisionSamples(cps_.points, bspline_interval_, grid_map_->getResolution());
      const double t_step = (tmp-tm)/samples;
      for (int sample = 0; sample <= samples; ++sample)
      {
        checkDeadline();
        const double t = tm+(tmp-tm)*sample/samples;
        Eigen::Vector3d pos = traj.evaluateDeBoorT(t);
        Eigen::Vector3d pos_next = traj.evaluateDeBoorT(std::min(t + t_step, tmp));
        if (grid_map_->getInflateOccupancy(pos, estimateSegmentYaw(pos, pos_next)))
        {
          // cout << "Refined traj hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

          Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
          for (size_t i = 0; i < ref_pts_.size(); i++)
          {
            checkDeadline();
            ref_pts.row(i) = ref_pts_[i].transpose();
          }

          flag_safe = false;
          break;
        }
      }

      if (!flag_safe)
        lambda2_ *= 2;

      iter_count++;
    } while (!flag_safe && iter_count < 3);

    lambda2_ = origin_lambda2;

    //cout << "iter_num_=" << iter_num_ << endl;

    return flag_safe;
  }

  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {

    copyFromSolver(x);

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_distance, f_feasibility, f_fitness=0;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.size);

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    addCurveDistanceCost(cps_.points, f_distance, g_distance);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);
    if (!ref_pts_.empty()) {calcFitnessCost(cps_.points, f_fitness, g_fitness);}

    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + lambda4_ * f_fitness;
    //printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance +
                              lambda3_ * g_feasibility + lambda4_ * g_fitness;
    grad_3D.row(2).setZero();
    memcpy(grad, grad_3D.data() + 3 * options_.fixed_boundary_points, n * sizeof(grad[0]));
  }

  void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
  {

    copyFromSolver(x);

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_fitness, f_feasibility, f_distance = 0;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    //time_satrt = ros::Time::now();

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcFitnessCost(cps_.points, f_fitness, g_fitness);
    addCurveDistanceCost(cps_.points, f_distance, g_distance);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    /* ---------- convert to solver format...---------- */
    f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility + lambda2_ * f_distance;
    // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness +
                              lambda3_ * g_feasibility + lambda2_ * g_distance;
    grad_3D.row(2).setZero();
    memcpy(grad, grad_3D.data() + 3 * options_.fixed_boundary_points, n * sizeof(grad[0]));
  }

} // namespace scan_planner


#include <csm/csm_all.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

namespace sm=sensor_msgs;

void prepare_data(LDP &ldp, double theta) {
    ldp = ld_alloc_new(24);
    int n = 24;
    for (unsigned int i = 0; i < n; i++) {
        // calculate position in laser frame

        // valid data
        ldp->valid[i] = 1;
        ldp->readings[i] = 3;

        // invalid data
        /* ldp->valid[i] = 0
         * ldp->readings[i] = -1;  // for invalid range
         * */



        ldp->theta[i] = i / 30.0 + theta;

        ldp->cluster[i] = -1;
    }

    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[n - 1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}

void prepare_csm(ros::NodeHandle nh_private_, sm_params &input_) {
    // **** csm
    // Maximum angular displacement between scans
    if (!nh_private_.getParam("max_angular_correction_deg", input_.max_angular_correction_deg))
        input_.max_angular_correction_deg = 45.0;

    // Maximum translation between scans (m)
    if (!nh_private_.getParam("max_linear_correction", input_.max_linear_correction))
        input_.max_linear_correction = 0.50;

    // Maximum ICP cycle iterations
    if (!nh_private_.getParam("max_iterations", input_.max_iterations))
        input_.max_iterations = 10;

    // A threshold for stopping (m)
    if (!nh_private_.getParam("epsilon_xy", input_.epsilon_xy))
        input_.epsilon_xy = 0.000001;

    // A threshold for stopping (rad)
    if (!nh_private_.getParam("epsilon_theta", input_.epsilon_theta))
        input_.epsilon_theta = 0.000001;

    // Maximum distance for a correspondence to be valid
    if (!nh_private_.getParam("max_correspondence_dist", input_.max_correspondence_dist))
        input_.max_correspondence_dist = 0.3;

    // Noise in the scan (m)
    if (!nh_private_.getParam("sigma", input_.sigma))
        input_.sigma = 0.010;

    // Use smart tricks for finding correspondences.
    if (!nh_private_.getParam("use_corr_tricks", input_.use_corr_tricks))
        input_.use_corr_tricks = 1;

    // Restart: Restart if error is over threshold
    if (!nh_private_.getParam("restart", input_.restart))
        input_.restart = 0;

    // Restart: Threshold for restarting
    if (!nh_private_.getParam("restart_threshold_mean_error", input_.restart_threshold_mean_error))
        input_.restart_threshold_mean_error = 0.01;

    // Restart: displacement for restarting. (m)
    if (!nh_private_.getParam("restart_dt", input_.restart_dt))
        input_.restart_dt = 1.0;

    // Restart: displacement for restarting. (rad)
    if (!nh_private_.getParam("restart_dtheta", input_.restart_dtheta))
        input_.restart_dtheta = 0.1;

    // Max distance for staying in the same clustering
    if (!nh_private_.getParam("clustering_threshold", input_.clustering_threshold))
        input_.clustering_threshold = 0.25;

    // Number of neighbour rays used to estimate the orientation
    if (!nh_private_.getParam("orientation_neighbourhood", input_.orientation_neighbourhood))
        input_.orientation_neighbourhood = 20;

    // If 0, it's vanilla ICP
    if (!nh_private_.getParam("use_point_to_line_distance", input_.use_point_to_line_distance))
        input_.use_point_to_line_distance = 1;

    // Discard correspondences based on the angles
    if (!nh_private_.getParam("do_alpha_test", input_.do_alpha_test))
        input_.do_alpha_test = 0;

    // Discard correspondences based on the angles - threshold angle, in degrees
    if (!nh_private_.getParam("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
        input_.do_alpha_test_thresholdDeg = 20.0;

    // Percentage of correspondences to consider: if 0.9,
    // always discard the top 10% of correspondences with more error
    if (!nh_private_.getParam("outliers_maxPerc", input_.outliers_maxPerc))
        input_.outliers_maxPerc = 0.90;

    // Parameters describing a simple adaptive algorithm for discarding.
    //  1) Order the errors.
    //  2) Choose the percentile according to outliers_adaptive_order.
    //     (if it is 0.7, get the 70% percentile)
    //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
    //     with the value of the error at the chosen percentile.
    //  4) Discard correspondences over the threshold.
    //  This is useful to be conservative; yet remove the biggest errors.
    if (!nh_private_.getParam("outliers_adaptive_order", input_.outliers_adaptive_order))
        input_.outliers_adaptive_order = 0.7;

    if (!nh_private_.getParam("outliers_adaptive_mult", input_.outliers_adaptive_mult))
        input_.outliers_adaptive_mult = 2.0;

    // If you already have a guess of the solution, you can compute the polar angle
    // of the points of one scan in the new position. If the polar angle is not a monotone
    // function of the readings index, it means that the surface is not visible in the
    // next position. If it is not visible, then we don't use it for matching.
    if (!nh_private_.getParam("do_visibility_test", input_.do_visibility_test))
        input_.do_visibility_test = 0;

    // no two points in laser_sens can have the same corr.
    if (!nh_private_.getParam("outliers_remove_doubles", input_.outliers_remove_doubles))
        input_.outliers_remove_doubles = 1;

    // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
    if (!nh_private_.getParam("do_compute_covariance", input_.do_compute_covariance))
        input_.do_compute_covariance = 0;

    // Checks that find_correspondences_tricks gives the right answer
    if (!nh_private_.getParam("debug_verify_tricks", input_.debug_verify_tricks))
        input_.debug_verify_tricks = 0;

    // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
    // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
    if (!nh_private_.getParam("use_ml_weights", input_.use_ml_weights))
        input_.use_ml_weights = 0;

    // If 1, the field 'readings_sigma' in the second scan is used to weight the
    // correspondence by 1/sigma^2
    if (!nh_private_.getParam("use_sigma_weights", input_.use_sigma_weights))
        input_.use_sigma_weights = 0;

    input_.min_reading = 0;
    input_.max_reading = 30;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "csm_test");
    ros::NodeHandle nh_private_("~");

    // **** input feed to csm
    sm_params input_;
    sm_result output_;
    // **** csm data structure
    LDP ldp_in, ldp_out;

    // **** prepare data and parameters
    prepare_data(ldp_in, 0.1);
    prepare_data(ldp_out, 0.32);
    prepare_csm(nh_private_, input_);

    // **** fill data in input
    input_.laser_ref = ldp_in;
    input_.laser_sens = ldp_out;

    // **** pose init guess
    input_.first_guess[0] = 0.0;
    input_.first_guess[1] = 0.0;
    input_.first_guess[2] = 0.0;

    // Initialize output_ vectors as Null for error-checking
    output_.cov_x_m = 0;
    output_.dx_dy1_m = 0;
    output_.dx_dy2_m = 0;

    // If they are non-Null, free covariance gsl matrices to avoid leaking memory
    if (output_.cov_x_m) {
        gsl_matrix_free(output_.cov_x_m);
        output_.cov_x_m = 0;
    }
    if (output_.dx_dy1_m) {
        gsl_matrix_free(output_.dx_dy1_m);
        output_.dx_dy1_m = 0;
    }
    if (output_.dx_dy2_m) {
        gsl_matrix_free(output_.dx_dy2_m);
        output_.dx_dy2_m = 0;
    }


    sm_icp(&input_, &output_);
    if (output_.valid) {

        printf("get res: [%f,%f,%f]", output_.x[0], output_.x[1], output_.x[2]);
    } else {
        printf("failure");
    }


    return 0;
}

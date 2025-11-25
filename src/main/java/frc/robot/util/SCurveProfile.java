// package frc.robot.util;

// import java.util.ArrayList;
// import java.util.List;

// public class SCurveProfile {
//     /**Returns an s-curve profile with the given constraints.
// Returns:
// t_rec -- list of timestamps
// x_rec -- list of positions at each timestep
// v_rec -- list of velocities at each timestep
// a_rec -- list of accelerations at each timestep
// Keyword arguments:
// max_v -- maximum velocity of profile
// max_a -- maximum acceleration of profile
// time_to_max_a -- time from rest to maximum acceleration
// dt -- timestep
// goal -- final position when the profile is at rest
// */
// public ArrayList<List<Double>> generate_s_curve_profile(double max_v,double max_a,double
// time_to_max_a,double dt,double goal) {

// List<Double> t_rec = new ArrayList<>();
// List<Double> x_rec = new ArrayList<>();
// List<Double> v_rec = new ArrayList<>();
// List<Double> a_rec = new ArrayList<>();
// double j = max_a / time_to_max_a;
// boolean short_profile = max_v * (time_to_max_a + max_v / max_a) > goal;
// double profile_max_v;

// if (short_profile) {
//     profile_max_v = max_a * (
//     Math.sqrt(goal / max_a - 0.75 * Math.pow(time_to_max_a, 2)) - 0.5 * time_to_max_a);
// }
// else {
//     profile_max_v = max_v;
// }
// // Find times at critical points
// double t2 = profile_max_v / max_a;
// double t3 = t2 + time_to_max_a;
// double t4;
// if (short_profile) {
//     t4 = t3;
// } else {
//     t4 = goal / profile_max_v;
// }
// double t5 = t4 + time_to_max_a;
// double t6 = t4 + t2;
// double t7 = t6 + time_to_max_a;
// double time_total = t7;
// while (t_rec.get(t_rec.size() - 1) < time_total) {}
//     double t = t_rec.get(t_rec.size() - 1) + dt;
//     t_rec.add(t);
//     if (t < time_to_max_a) {
//         // Ramp up acceleration
//         a_rec.add(j * t);
//         v_rec.add(0.5 * j * Math.pow(t,2));
//     } else if (t < t2) {
//         // Increase speed at max acceleration
//         a_rec.add(max_a);
//         v_rec.add(max_a * (t - 0.5 * time_to_max_a));
//     } else if (t < t3) {
//         // Ramp down acceleration
//         a_rec.add(max_a - j * (t - t2));
//         v_rec.add(max_a * (t - 0.5 * time_to_max_a) - 0.5 * j * Math.pow(t - t2, 2));
//     } else if (t < t4) {
//         // Maintain max velocity
//         a_rec.add(0.0);
//         v_rec.add(profile_max_v);
//     } else if (t < t5) {
//         // Ramp down acceleration
//         a_rec.add(-j * (t - t4));
//         v_rec.add(profile_max_v - 0.5 * j * Math.pow(t - t4, 2));
//     } else if (t < t6) {
//         // Decrease speed at max acceleration
//         a_rec.add(-max_a);
//         v_rec.add(max_a * (t2 + t5 - t - 0.5 * time_to_max_a));
//     }  else if (t < t7) {
//         // Ramp up acceleration
//         a_rec.add(-max_a + j * (t - t6));
//         v_rec.add(max_a * (t2 + t5 - t - 0.5 * time_to_max_a) + 0.5 * j * Math.pow(t - t6, 2));
//     } else {
//         a_rec.add(0.0);
//         v_rec.add(0.0);
//         x_rec.add(x_rec.get(t_rec.size() - 1) + v_rec.get(t_rec.size() - 1) * dt);
//     }
// return new ArrayList<List<Double>>().add(t_rec).add(x_rec).add(v_rec).add(a_rec);
// }
// }

// #include "strategy/univector.h"

// /**
//  * @brief composses univec repulsion fields by all obstacles
//  * 
//  * @param moving_robot_id 
//  * @param my_robots 
//  * @param enemy_robots 
//  * @return std::pair<double, double> returns min distance phi and min distance
//  */
// std::pair<double, double> univec::repulsion_field(unsigned int moving_robot_id, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots)
// {
//     double repulsion_phi;
//     std::vector<double> phis;
//     double dist;
//     std::vector<double> distances;
//     ctrl::vec2 obstacle;

//     for (size_t i = 0; i < my_robots.size(); ++i)
//     {
//         if (i != moving_robot_id)
//         {
//             obstacle =  ctrl::future_position(my_robots[i], my_robots[moving_robot_id]);
//             repulsion_phi = (ctrl::vec2(my_robots[moving_robot_id]) - obstacle).theta();
//             phis.push_back(repulsion_phi);
//             dist = ctrl::vec2(my_robots[moving_robot_id]).distance(obstacle);
//             distances.push_back(dist);
//         }
//     }

//     for (size_t i = 0; i < enemy_robots.size(); ++i)
//     {
//         obstacle =  ctrl::future_position(enemy_robots[i], my_robots[moving_robot_id]);
//         repulsion_phi = (ctrl::vec2(my_robots[moving_robot_id]) - obstacle).theta();
//         phis.push_back(repulsion_phi);
//         dist = ctrl::vec2(my_robots[moving_robot_id]).distance(obstacle);
//         distances.push_back(dist);
//     }

//     auto min_dist_it = std::min_element(distances.begin(), distances.end());
//     return std::make_pair(phis[min_dist_it - distances.begin()], *min_dist_it);
// }

// //  ----------> BALL FIELD <-----------
// /**
//  * @brief Calculates a spiral field arround (0, 0)
//  * 
//  * @param pos 
//  * @param cw char indicating spiral orientation ('+' to counterclockwise and '-' to clockwise)
//  * @return double 
//  */
// double spiral_field(ctrl::vec2 pos, char cw)
// {
//     double phi, dist, theta;
//     double sgn = (cw == '-') ? -1.0 : 1.0;
//     dist = pos.abs();
//     theta = pos.theta();

//     if (dist > univec::RADIUS)
//         phi = theta + (sgn) * HALF_PI * (2.0 - ((univec::RADIUS + univec::K_SPIRAL) / (dist + univec::K_SPIRAL)));
//     else
//         phi = theta + (sgn) * HALF_PI * std::sqrt(dist / univec::RADIUS);

//     return phi;
// }

// double univec::spiral_field_cw(ctrl::vec2 pos)
// {
//     return spiral_field(pos, '-');
// }

// double univec::spiral_field_ccw(ctrl::vec2 pos)
// {
//     return spiral_field(pos, '+');
// }

// /**
//  * @brief 
//  * 
//  * @param pos 
//  * @param target 
//  * @return double 
//  */
// double univec::spiral_field_to_target(ctrl::vec2 pos, ctrl::vec2 target)
// {
//     double phi;
//     // univec always operate over translated points
//     ctrl::vec2 translated = pos - target;
//     double yl = translated.y + univec::RADIUS;
//     double yr = translated.y - univec::RADIUS;
//     ctrl::vec2 posl = translated - ctrl::vec2(0, univec::RADIUS);
//     ctrl::vec2 posr = translated + ctrl::vec2(0, univec::RADIUS);

//     if (-univec::RADIUS <= translated.y && translated.y < univec::RADIUS)
//     {
//         double sin_phi, cos_phi;
//         double phicw = univec::spiral_field_cw(posr);
//         double phiccw = univec::spiral_field_ccw(posl);
//         sincos(phicw, &sin_phi, &cos_phi); // Get sin & cosin at once
//         ctrl::vec2 Ncw = ctrl::vec2(cos_phi, sin_phi);
//         sincos(phiccw, &sin_phi, &cos_phi);
//         ctrl::vec2 Nccw = ctrl::vec2(cos_phi, sin_phi);
//         ctrl::vec2 tmp = (yl * Nccw - yr * Ncw) * (1.0 / (2.0 * univec::RADIUS));
//         phi = tmp.theta();
//     }
//     else if (translated.y < -univec::RADIUS)
//         phi = univec::spiral_field_cw(posl);
//     else
//         phi = univec::spiral_field_ccw(posr);

//     return phi;
// }

// /**
//  * @brief composes repulsion and move to goal fields with gaussian compound ratio
//  * 
//  * @param repulsion_vec 
//  * @param spiral_vec 
//  * @return double 
//  */
// double univec::composite_field(double repulsion_phi, double spiral_phi)
// {
//     double gauss = math::gaussian(univec::RADIUS - univec::D_MIN, univec::SIGMA);
//     if (univec::RADIUS <= univec::D_MIN)
//         return repulsion_phi;
//     else
//         return (repulsion_phi * gauss + spiral_phi * (1-gauss));
// }

// /**
//  * @brief Field that converge to target with final orientation in vertical
//  * 
//  * @param pos 
//  * @param target 
//  * @return double 
//  */
// double univec::vertical_line_field(ctrl::vec2 pos, ctrl::vec2 target)
// {
//     double phi;
//     // univec always operate over translated points
//     ctrl::vec2 translated = pos - target;

//     if (translated.y <= 0.0)
//         phi = PI * (1.0 / (1.0 + std::exp(-1.0 * univec::K_SPIRAL * translated.x)));
//     else
//         phi = PI * (-1.0 / (1.0 + std::exp(-1.0 * univec::K_SPIRAL * translated.x)));

//     return phi;
// }

// /**
//  * @brief Field that converge to target with final orientation in horizontal
//  * 
//  * @param pos 
//  * @param target 
//  * @return double 
//  */
// double univec::horizontal_line_field(ctrl::vec2 pos, ctrl::vec2 target)
// {
//     double phi;
//     // univec always operate over translated points
//     ctrl::vec2 translated = pos - target;
//     double e = std::exp(univec::K_SPIRAL * translated.y);

//     if (translated.x <= 0.0)
//         phi = HALF_PI * ((1 - e) / (1 + e));
//     else
//         phi = HALF_PI * ((1 + 3 * e) / (1 + e));

//     return phi;
// }
/****************************************************************************
 * Copyright ©2017 Brian Curless.  All rights reserved.  Permission is hereby
 * granted to students registered for University of Washington CSE 457 or CSE
 * 557 for use solely during Autumn Quarter 2017 for purposes of the course.
 * No other use, copying, distribution, or modification is permitted without
 * prior written consent. Copyrights for third-party components of this work
 * must be honored.  Instructors interested in reusing these course materials
 * should contact the author.
 ****************************************************************************/
#include "beziercurveevaluator.h"

std::vector<glm::vec2> BezierCurveEvaluator::EvaluateCurve(const std::vector<glm::vec2> &ctrl_pts, int density) const {
    std::vector<glm::vec2> evaluated_pts;

    // REQUIREMENT:
    // Currently this function returns points for a Linear Evaluation.
    // Replace this code with code that returns evaluated points for a Bezier
    // Spline curve. Be sure to respect the extend_x_ and wrap_ flags in a
    // a reasonable way.

    if (density == 0) density = 100;
//    for (size_t i = 0; i < ctrl_pts.size()-1; i++) {
//        for (int j = 0; j < density; j++) {
//            float t = j/(float) density;
//            glm::vec2 p = t*ctrl_pts[i+1] + (1-t)*ctrl_pts[i];
//            evaluated_pts.push_back(p);
//        }
//    }
//    evaluated_pts.push_back(ctrl_pts.back());


    if (ctrl_pts.size() <= 2) {
        evaluated_pts.assign(ctrl_pts.begin(), ctrl_pts.end());
        if (extend_x_) ExtendX(evaluated_pts, ctrl_pts);
        return evaluated_pts;
    }
    std::vector<glm::vec2> new_ctrl_pts;
    new_ctrl_pts.assign(ctrl_pts.begin(), ctrl_pts.end());
    if (wrap_y_) {
        new_ctrl_pts.push_back(glm::vec2(ctrl_pts[0].x + max_x_, ctrl_pts[0].y));
    }
    const int total_group = (new_ctrl_pts.size() - 1) / 3;
    const int remain_pts = new_ctrl_pts.size() - total_group * 3;

    for (int i = 0; i < total_group; i++) {
        for (int j = 0; j <= density; j++) {
            float u = j / (float) density;
            glm::vec2 Q_u = deCasteljau(new_ctrl_pts[3*i], new_ctrl_pts[3*i+1],
                    new_ctrl_pts[3*i+2], new_ctrl_pts[3*i+3], u);
            if (wrap_y_ || Q_u.x <= max_x_) {
                evaluated_pts.push_back(Q_u);
            }
        }
    }
    // linear interpolation for remain points that not part of bezier group
    if (remain_pts != 0) {
        for (int i = new_ctrl_pts.size() - remain_pts; i < new_ctrl_pts.size(); i++) {
            evaluated_pts.push_back(new_ctrl_pts[i]);
        }
    }

    if (extend_x_) ExtendX(evaluated_pts, ctrl_pts);
    return evaluated_pts;


}

/****************************************************************************
 * Copyright ©2017 Brian Curless.  All rights reserved.  Permission is hereby
 * granted to students registered for University of Washington CSE 457 or CSE
 * 557 for use solely during Autumn Quarter 2017 for purposes of the course.
 * No other use, copying, distribution, or modification is permitted without
 * prior written consent. Copyrights for third-party components of this work
 * must be honored.  Instructors interested in reusing these course materials
 * should contact the author.
 ****************************************************************************/
#include "bsplinecurveevaluator.h"

std::vector<glm::vec2> BSplineCurveEvaluator::EvaluateCurve(const std::vector<glm::vec2> &ctrl_pts, int density) const {
    std::vector<glm::vec2> evaluated_pts;

    // REQUIREMENT:
    // Currently this function returns points for a Linear Evaluation.
    // Replace this code with code that returns evaluated points for a B-Spline
    // curve. Be sure to respect the extend_x_ and wrap_ flags.

    if (density == 0) density = 100;
//    for (size_t i = 0; i < ctrl_pts.size()-1; i++) {
//        for (int j = 0; j < density; j++) {
//            float t = j/(float) density;
//            glm::vec2 p = t*ctrl_pts[i+1] + (1-t)*ctrl_pts[i];
//            evaluated_pts.push_back(p);
//        }
//    }
//    evaluated_pts.push_back(ctrl_pts.back());
//    if (extend_x_) ExtendX(evaluated_pts, ctrl_pts);

    if (ctrl_pts.size() <= 2) {
        evaluated_pts.assign(ctrl_pts.begin(), ctrl_pts.end());
        if (extend_x_) ExtendX(evaluated_pts, ctrl_pts);
        return evaluated_pts;
    }

    // triple end point
    std::vector<glm::vec2> ctrl_pts_copy;
        if (wrap_y_) {
            ctrl_pts_copy.push_back(glm::vec2(ctrl_pts[ctrl_pts.size()-1].x - max_x_, ctrl_pts[ctrl_pts.size()-1].y));
            ctrl_pts_copy.push_back(glm::vec2(ctrl_pts[ctrl_pts.size()-1].x - max_x_, ctrl_pts[ctrl_pts.size()-1].y));
            ctrl_pts_copy.insert(ctrl_pts_copy.end(), ctrl_pts.begin(), ctrl_pts.end());
            ctrl_pts_copy.push_back(glm::vec2(ctrl_pts[0].x + max_x_, ctrl_pts[0].y));
            ctrl_pts_copy.push_back(glm::vec2(ctrl_pts[0].x + max_x_, ctrl_pts[0].y));
        } else {
            ctrl_pts_copy.push_back(ctrl_pts[0]);
            ctrl_pts_copy.push_back(ctrl_pts[0]);
            ctrl_pts_copy.insert(ctrl_pts_copy.end(), ctrl_pts.begin(), ctrl_pts.end());
            ctrl_pts_copy.push_back(ctrl_pts.back());
            ctrl_pts_copy.push_back(ctrl_pts.back());
        }

    for (int i = 1; i <= ctrl_pts.size()+1; i++) {
        glm::vec2 v0 = ctrl_pts_copy[i-1]/6.0f + 2.0f * ctrl_pts_copy[i]/3.0f + ctrl_pts_copy[i+1]/6.0f;
        glm::vec2 v1 = 2.0f * ctrl_pts_copy[i]/3.0f + ctrl_pts_copy[i+1]/3.0f;
        glm::vec2 v2 = ctrl_pts_copy[i]/3.0f + 2.0f * ctrl_pts_copy[i+1]/3.0f;
        glm::vec2 v3 = ctrl_pts_copy[i]/6.0f + 2.0f * ctrl_pts_copy[i+1]/3.0f + ctrl_pts_copy[i+2]/6.0f;
        if (v1.x > v3.x) v1.x = v3.x;
        if (v0.x > v2.x) v2.x = v0.x;
            for (int j = 0; j <= density; j++) {
                float u = j / (float) density;
                glm::vec2 Q_u = deCasteljau(v0, v1, v2, v3, u);
                if (wrap_y_ || Q_u.x <= max_x_) {
                    evaluated_pts.push_back(Q_u);
                }
            }
        }
    if (extend_x_) ExtendX(evaluated_pts, ctrl_pts);
    return evaluated_pts;
    }









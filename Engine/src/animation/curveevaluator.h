/****************************************************************************
 * Copyright ©2017 Brian Curless.  All rights reserved.  Permission is hereby
 * granted to students registered for University of Washington CSE 457 or CSE
 * 557 for use solely during Autumn Quarter 2017 for purposes of the course.
 * No other use, copying, distribution, or modification is permitted without
 * prior written consent. Copyrights for third-party components of this work
 * must be honored.  Instructors interested in reusing these course materials
 * should contact the author.
 ****************************************************************************/
#ifndef CURVEEVALUATOR_H
#define CURVEEVALUATOR_H

#include <vectors.h>

class CurveEvaluator {
public:
    CurveEvaluator() : wrap_y_(false), extend_x_(false), max_x_(0.f) {}
	CurveEvaluator(float animation_length, bool wrap) : wrap_y_(wrap), extend_x_(true), max_x_(animation_length) {}
    virtual std::vector<glm::vec2> EvaluateCurve(const std::vector<glm::vec2>& ctrl_pts, int density) const = 0;
    virtual ~CurveEvaluator() {}
    void Wrap(bool wrap = true) { wrap_y_ = wrap; }
protected:
    void ExtendX(std::vector<glm::vec2>& out_pts, const std::vector<glm::vec2>& in_pts) const {
        if (wrap_y_) {
            // if wrapping is on, linearly interpolate the y value at xmin and
            // xmax so that the slopes of the lines adjacent to the
            // wraparound are equal.
            float dx1 = in_pts[0].x;
            float dx2 = max_x_ - in_pts.back().x;
            float t = dx2/(dx1 + dx2);
            float y = t*in_pts[0].y + (1-t)*in_pts.back().y;
            out_pts.insert(out_pts.begin(), glm::vec2(0, y));
            out_pts.push_back(glm::vec2(max_x_, y));
        } else {
	        if (in_pts.back().x < max_x_)
	            out_pts.push_back(glm::vec2(max_x_, in_pts.back().y));
	        if (in_pts[0].x > 0)
                out_pts.insert(out_pts.begin(), glm::vec2(0, in_pts[0].y));
        }
    }
    bool wrap_y_;
    bool extend_x_;
    float max_x_;

    const glm::vec2 deCasteljau(const glm::vec2 v0,const glm::vec2 v1, const glm::vec2 v2, const glm::vec2 v3, float u) const {
        float x = (1-u)*(1-u)*(1-u)*v0.x + 3*u*(1-u)*(1-u)*v1.x + 3*u*u*(1-u)*v2.x + u*u*u*v3.x;
        float y = (1-u)*(1-u)*(1-u)*v0.y + 3*u*(1-u)*(1-u)*v1.y + 3*u*u*(1-u)*v2.y + u*u*u*v3.y;
        return glm::vec2(x,y);
    }


};

#endif // CURVEEVALUATOR_H

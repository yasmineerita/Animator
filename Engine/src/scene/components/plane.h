/****************************************************************************
 * Copyright ©2017 Brian Curless.  All rights reserved.  Permission is hereby
 * granted to students registered for University of Washington CSE 457 or CSE
 * 557 for use solely during Autumn Quarter 2017 for purposes of the course.
 * No other use, copying, distribution, or modification is permitted without
 * prior written consent. Copyrights for third-party components of this work
 * must be honored.  Instructors interested in reusing these course materials
 * should contact the author.
 ****************************************************************************/
#ifndef PLANE_H
#define PLANE_H

#include <properties.h>
#include <scene/components/geometry.h>

class Plane : public Geometry {
public:
    ChoiceProperty Subdivisions;

    Plane();

    virtual Mesh* GetRenderMesh() { return mesh_.get(); }

protected:
    void OnSubdivisionsSet(int subdivisions);
    std::unique_ptr<Mesh> CreateMesh(unsigned int subdivisions);
    std::unique_ptr<Mesh> mesh_;
};

#endif // PLANE_H

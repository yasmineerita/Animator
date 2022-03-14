/****************************************************************************
 * Copyright ©2017 Brian Curless.  All rights reserved.  Permission is hereby
 * granted to students registered for University of Washington CSE 457 or CSE
 * 557 for use solely during Autumn Quarter 2017 for purposes of the course.
 * No other use, copying, distribution, or modification is permitted without
 * prior written consent. Copyrights for third-party components of this work
 * must be honored.  Instructors interested in reusing these course materials
 * should contact the author.
 ****************************************************************************/
#include "particlesystem.h"
#include <scene/sceneobject.h>

#include <QDebug>

REGISTER_COMPONENT(ParticleSystem, ParticleSystem)

ParticleSystem::ParticleSystem() :
    ParticleGeometry({"Sphere"}, 0),
    ParticleMaterial(AssetType::Material),
    InitialVelocity(glm::vec3(5.0f, 5.0f, 0.0f)),
    Mass(0.1f, 0.0f, 10.0f, 0.1f),
    Period(0.5f, 0.0f, 1.0f, 0.01f),
    ConstantF(glm::vec3(0.0f, -9.8f, 0.0f)),
    DragF(0.0f, 0.0f, 10.0f, 0.01f),
    constant_force_(ConstantF.Get()),
    // REQUIREMENT:
    // init drag force with DragF -- refer to how we deal with constant_force_
    // remember  (f = -k_d * v), where DragF represents k_d
    drag(DragF.Get()),
    num_particles_(0),
    particle_index_(0),
    simulating_(false)
{
    AddProperty("Geometry", &ParticleGeometry);
    AddProperty("Material", &ParticleMaterial);
    AddProperty("Initial Velocity", &InitialVelocity);
    AddProperty("Mass", &Mass);
    AddProperty("Period (s)", &Period);
    AddProperty("Constant Force", &ConstantF);
    AddProperty("Drag Coefficient", &DragF);

    ParticleGeometry.ValueSet.Connect(this, &ParticleSystem::OnGeometrySet);

    forces_.push_back(std::shared_ptr<Force>(&constant_force_));

    // REQUIREMENT: 
    //    add viscous drag force into forces_ array if your drag force also inherit from class Force
    //    If not, you could use your own way to prepare your drag force
//    forces_.push_back(std::shared_ptr<Force>(&drag));

}

void ParticleSystem::UpdateModelMatrix(glm::mat4 model_matrix) {
   model_matrix_ = model_matrix;
}

void ParticleSystem::EmitParticles() {
    if (!simulating_) return;

    // REQUIREMENT:
    // Create some particles!
    //    - We have designed a class Particle for you
    //    - We've provided some UI controls for you
    //          -- Mass.Get() defines particle mass, and
    //          -- InitialVelocity.Get() defines particle init velocity in local object space
    //    - Notice particles should be created in world space. (use model_matrix_ to transform particles from local object space to world space)
    // Store particles in the member variable particles_
    // For performance reasons, limit the amount of particles that exist at the same time
    // to some finite amount (MAX_PARTICLES). Either delete or recycle old particles as needed.

    // Reset the time
    glm::vec4 localVec = glm::fvec4(InitialVelocity.Get().x,InitialVelocity.Get().y,InitialVelocity.Get().z,0);
    localVec = model_matrix_ * localVec;
    glm::vec3 worldVec = glm::vec3(localVec.x,localVec.y,localVec.z);

    glm::vec4 tmpVec = glm::fvec4(0,0,0,1);
    tmpVec = model_matrix_ * tmpVec;
    glm::vec3 tmpP = glm::vec3(tmpVec.x,tmpVec.y,tmpVec.z);

    std::unique_ptr<Particle> p(
                new Particle( Mass.Get(), tmpP, worldVec,glm::vec3(0,0,0)));

    if (num_particles_ == MAX_PARTICLES){
        particles_.erase(particles_.begin());
        num_particles_ = num_particles_ - 1;
    }

    particles_.push_back(std::move(p));
    num_particles_ = num_particles_ + 1;

    time_to_emit_ = Period.Get();
}

std::vector<Particle*> ParticleSystem::GetParticles() {
    // Return a vector of particles (used by renderer to draw them)
    std::vector<Particle*> particles;
    for (auto& particle : particles_) particles.push_back(particle.get());
    return particles;
}

void ParticleSystem::StartSimulation() {
    simulating_ = true;
    constant_force_.SetForce(ConstantF.Get());
    // REQUIREMENT:
    // Set your added drag force as DragF.Get() -- Refer to what we did on constact_force_

    drag = DragF.Get();

    ResetSimulation();
}

void ParticleSystem::UpdateSimulation(float delta_t, const std::vector<std::pair<SceneObject*, glm::mat4>>& colliders) {
    if (!simulating_) return;

    // Emit Particles
    time_to_emit_ -= delta_t;
    if (time_to_emit_ <= 0.0) EmitParticles();

   // REQUIREMENT:
   // For each particle ...
   //      Calculate forces
   //      Solve the system of forces using Euler's method
   //      Update the particle
   //      Check for and handle collisions

   for (auto& p : particles_) {
//       p->Velocity;
//       p->Mass;
//       p->Position;

//       model_matrix_

//       glm::vec3 gravity_f = forces_.front()->GetForce(*p);
//       glm::vec3 drag_f = forces_.back()->GetForce(*p);
       glm::vec3 gravity_f = constant_force_.GetForce(*p);

       glm::vec3 total_f = gravity_f - (float)drag * p->Velocity;
       glm::vec3 a = total_f/p->Mass;

       p->Position = p->Velocity * (float)delta_t  + p->Position;
       p->Velocity = a * (float)delta_t            + p->Velocity;



      // Collision code might look something like this:
      for (auto& kv : colliders) {

          SceneObject* collider_object = kv.first;
          glm::mat4 collider_model_matrix = kv.second;

          glm::vec4 worldPosition = glm::vec4(p->Position.x,p->Position.y,p->Position.z,1);


          worldPosition =  glm::inverse(collider_model_matrix) * worldPosition;

          glm::vec3 localP = glm::vec3(worldPosition.x, worldPosition.y, worldPosition.z);


          glm::vec3 worldVelocity = glm::vec3(p->Velocity.x,p->Velocity.y,p->Velocity.z);

          glm::mat3 test = glm::transpose(collider_model_matrix);

          worldVelocity =  glm::inverse(test) * worldVelocity;


          glm::dvec3 localV = glm::dvec3(worldVelocity.x, worldVelocity.y, worldVelocity.z);
          static const double EPSILON = 0.1;
          float particle_radius = 0.5f;

          // When checking collisions, remember to bring particles from world space to collider local object space
          // The trasformation matrix can be derived by taking invese of collider_model_matrix
          if (SphereCollider* sphere_collider = collider_object->GetComponent<SphereCollider>()) {
              // Check for Sphere Collision
              double r = sphere_collider->Radius.Get();
              if (sqrt(worldPosition.x * worldPosition.x + worldPosition.y * worldPosition.y +
                      worldPosition.z * worldPosition.z) < (particle_radius + r + EPSILON)){

                  glm::dvec3 n = glm::normalize(localP);
                  if ( glm::dot( (localV), n ) <  0.0f){
                  glm::dvec3 Vn = glm::dot(localV, n ) * n;
                  glm::dvec3 Vt = localV - Vn;
                  double rs = sphere_collider->Restitution.Get();
                  localV = Vt -  rs * Vn;
                  }
              }
          } else if (PlaneCollider* plane_collider = collider_object->GetComponent<PlaneCollider>()) {
              // Check for Plane Collision

              double w = plane_collider->Width.Get();
              double h = plane_collider->Height.Get();

              glm::vec3 a = glm::vec3(w/2, h/2, 0);
              glm::vec3 b = glm::vec3(w/2, -h/2, 0);
              glm::vec3 c = glm::vec3(-w/2, -h/2, 0);
              glm::vec3 d = glm::vec3(-w/2, h/2, 0);
              glm::vec3 n = glm::vec3(0,0,1);

//              glm::dot( (localP - a),n ) <=  EPSILON
//                                    && glm::dot( (localP - b),n ) <=  EPSILON
//                                    && glm::dot( (localP - c),n ) <=  EPSILON
//                                    && glm::dot( (localP - d),n ) <=  EPSILON
              if (
                      glm::dot( (localV), glm::dvec3(0,0,1.f) ) >  0.0f &&
                      abs(localP.y) < (h/2+  particle_radius + EPSILON)
                      && abs(localP.x) < (w/2 + particle_radius + EPSILON)
                      && localP.z < (particle_radius + EPSILON)
                      && localP.z >= -EPSILON
                      ){
                  glm::dvec3 n = glm::dvec3(0,0,1.f);

                  glm::dvec3 Vn = glm::dot(localV, n ) * n;
                  glm::dvec3 Vt = localV - Vn;
                  double rs = plane_collider->Restitution.Get();
                  localV = Vt -  rs * Vn;
//                   qDebug("hihi %f", rs);


              }
          }
          // When updating particle velocity, remember it's in the worls space.


          worldVelocity = glm::vec3(localV.x,localV.y,localV.z);

          worldVelocity =  test * worldVelocity;
          localV = glm::vec3(worldVelocity.x, worldVelocity.y, worldVelocity.z);
          p->Velocity = localV;
      }
   }
}

void ParticleSystem::StopSimulation() {
    simulating_ = false;
}

void ParticleSystem::ResetSimulation() {
    // Clear all particles
    time_to_emit_ = Period.Get();
}

bool ParticleSystem::IsSimulating() {
    return simulating_;
}


void ParticleSystem::OnGeometrySet(int c) {
    GeomChanged.Emit(ParticleGeometry.GetChoices()[c]);
}

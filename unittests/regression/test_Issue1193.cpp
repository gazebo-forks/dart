/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include "dart/dart.hpp"

#include "TestHelpers.hpp"

#include "dart/utils/sdf/SdfParser.hpp"

using namespace dart::math;
using namespace dart::collision;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;

//==============================================================================
TEST(Issue1193, AngularVelAdd)
{
  WorldPtr world = World::create();
  EXPECT_TRUE(world != nullptr);
  world->setGravity(Eigen::Vector3d(0.0, -10.0, 0.0));
  const double dt = 0.001;
  world->setTimeStep(dt);

  SkeletonPtr sphereSkel = createSphere(0.05, Vector3d(0.0, 1.0, 0.0));
  BodyNode* sphere = sphereSkel->getBodyNode(0);
  Joint* sphereJoint = sphere->getParentJoint();
  sphereJoint->setVelocity(0, 10.0);  // ang_x -> Affect lz and ly
  sphereJoint->setVelocity(1, 10.0);  // ang_y -> No effect
  sphereJoint->setVelocity(2, 10.0);  // ang_z -> Affect lx and ly
  world->addSkeleton(sphereSkel);

  Eigen::Vector3d linearVelBefore = sphere->getLinearVelocity();
  EXPECT_EQ(Vector3d::Zero(), linearVelBefore);

  int maxSteps = 500;
  for (int i = 0; i < maxSteps; i++)
  {
    world->step();
  }

  Vector3d linearVelAfter = sphere->getLinearVelocity();
  double lx = linearVelAfter[0];
  double ly = linearVelAfter[1];
  double lz = linearVelAfter[2];

  EXPECT_NEAR(0.0, lx, 1e-8);
  EXPECT_NEAR(maxSteps * world->getGravity().y() * dt, ly, 1e-8);
  EXPECT_NEAR(0.0, lz, 1e-8);
}

const double tol = 1e-5;
const int g_iters = 100000;

Eigen::Vector3d computeWorldAngularMomentum(const SkeletonPtr skel)
{
  Eigen::Vector3d angMomentum = Eigen::Vector3d::Zero();
  for(auto *bn: skel->getBodyNodes())
  {
    angMomentum += dart::math::dAdInvT(
                       bn->getWorldTransform(),
                       bn->getSpatialInertia() * bn->getSpatialVelocity())
                       .head<3>();
    // Eigen::Isometry3d pose = bn->getWorldTransform();
    // Eigen::Matrix3d Icom = bn->getInertia().getMoment();
    // Eigen::Matrix3d Io = pose.rotation() * Icom * pose.rotation().transpose();

    // Eigen::Vector3d angVelWorld = bn->getAngularVelocity();
    // Eigen::Vector3d linVelWorld = bn->getLinearVelocity();
    // angMomentum += Io * angVelWorld
    //                + pose.translation().cross(bn->getMass() * linVelWorld);
  }
  return angMomentum;
}


TEST(Issue1193, SingleBody)
{
  WorldPtr world = World::create();
  const double dt = 0.001;
  world->setTimeStep(dt);
  world->setGravity(Vector3d::Zero());
  SkeletonPtr skel = createBox({1, 1, 1});
  skel->disableSelfCollisionCheck();
  world->addSkeleton(skel);
  auto rootBn = skel->getRootBodyNode();

  Vector6d vels = compose({0, 25, 0}, {0, 0, -100});

  rootBn->getParentJoint()->setVelocities(vels);

  // rootBn->addExtTorque({0, 50, 0});
  for (int i = 0; i < g_iters; ++i)
  {
    world->step();
    auto pose = rootBn->getWorldTransform();
    std::cout << i << " " << pose.translation().transpose() << std::endl;
  }
  auto position = rootBn->getWorldTransform().translation();
  EXPECT_NEAR(0.0, position.x(), tol);
  EXPECT_NEAR(0.0, position.y(), tol);
  EXPECT_NEAR(g_iters * dt * vels[5], position.z(), tol);
}

TEST(Issue1193, SingleBodyWithOffDiagonalMoi)
{
  WorldPtr world = World::create();
  const double dt = 0.001;
  world->setTimeStep(dt);
  world->setGravity(Vector3d::Zero());
  SkeletonPtr skel = createBox({1, 1, 1});
  skel->disableSelfCollisionCheck();
  world->addSkeleton(skel);
  auto rootBn = skel->getRootBodyNode();
  rootBn->setMomentOfInertia(1.1, 1.1, 0.7, 0.1, 0, 0);

  Vector6d vels = compose({0, 25, 0}, {0, 0, -100});

  rootBn->getParentJoint()->setVelocities(vels);

  rootBn->addExtTorque({0, 50, 0});
  //
  for (int i = 0; i < g_iters; ++i)
  {
    world->step();
    auto pose = rootBn->getWorldTransform();
    std::cout << i << " " << pose.translation().transpose() << std::endl;
  }
  auto position = rootBn->getWorldTransform().translation();
  EXPECT_NEAR(0.0, position.x(), tol);
  EXPECT_NEAR(0.0, position.y(), tol);
  EXPECT_NEAR(g_iters * dt * vels[5], position.z(), tol);
}

TEST(Issue1193, SingleBodyWithJointOffset)
{
  WorldPtr world = World::create();
  const double dt = 0.0001;
  world->setTimeStep(dt);
  world->setGravity(Vector3d::Zero());
  SkeletonPtr skel = createBox({1, 1, 1});
  skel->disableSelfCollisionCheck();
  world->addSkeleton(skel);
  auto rootBn = skel->getRootBodyNode();
  rootBn->setMomentOfInertia(1.1, 1.1, 0.7, 0.1, 0, 0);

  Vector6d vels = compose({0, 25, 0}, {0, 0, -100});

  auto *freeJoint = dynamic_cast<FreeJoint *>(rootBn->getParentJoint());
  freeJoint->setVelocities(vels);

  Eigen::Isometry3d jointPoseInParent = Eigen::Isometry3d::Identity();
  jointPoseInParent.translate(Eigen::Vector3d(0.0, 4.0, 0));
  freeJoint->setTransformFromParentBodyNode(jointPoseInParent);

  // rootBn->addExtTorque({0, 50, 0});
  for (int i = 0; i < g_iters; ++i)
  {
    world->step();
    auto pose = rootBn->getWorldTransform();
    std::cout << i << " " << pose.translation().transpose() << std::endl;
  }

  auto position = rootBn->getWorldTransform().translation();
  EXPECT_NEAR(0.0, position.x(), tol);
  EXPECT_NEAR(4.0, position.y(), tol);
  EXPECT_NEAR(g_iters * dt * vels[5], position.z(), tol);
}

TEST(Issue1193, SingleBodyWithCOMOffset)
{
  WorldPtr world = World::create();
  const double dt = 0.001;
  world->setTimeStep(dt);
  world->setGravity(Vector3d::Zero());
  SkeletonPtr skel = createBox({1, 1, 1});
  world->addSkeleton(skel);
  auto rootBn = skel->getRootBodyNode();
  rootBn->setMomentOfInertia(1.0/6, 1.0/6.0, 1.0/6.0, 0, 0, 0);
  // rootBn->setLocalCOM({1, 5, 8});
  rootBn->setLocalCOM({0, 0, 8});


  rootBn->addExtForce({10, 0, 0});
  for (int i = 0; i < g_iters; ++i)
  {
    world->step();
    auto pose = rootBn->getWorldTransform();
    auto comPos = rootBn->getCOM();
    auto comLinearVel = rootBn->getCOMLinearVelocity();
    std::cout << i << " " << comPos.transpose() << " " << comLinearVel.transpose() << std::endl;
  }
}

//==============================================================================
TEST(Issue1193, WithFixedJoint)
{
  WorldPtr world = World::create();
  const double dt = 0.001;
  world->setTimeStep(dt);
  world->setGravity(Vector3d::Zero());
  SkeletonPtr skel = createBox({1, 1, 1}, {0, 0, 2});
  skel->disableSelfCollisionCheck();
  world->addSkeleton(skel);
  auto rootBn = skel->getRootBodyNode();

  Eigen::Isometry3d comPose;
  comPose = Eigen::Translation3d(0, 0, -2);
  auto comFrame = SimpleFrame::createShared(rootBn, "CombinedCOM", comPose);

  GenericJoint<R1Space>::Properties joint2Prop(std::string("joint2"));
  BodyNode::Properties link2Prop(
      BodyNode::AspectProperties(std::string("link2")));
  link2Prop.mInertia.setMass(1.0);

  auto pair = rootBn->createChildJointAndBodyNodePair<WeldJoint>(
      WeldJoint::Properties(joint2Prop), link2Prop);
  auto *joint = pair.first;

  Eigen::Isometry3d jointPoseInParent = Eigen::Isometry3d::Identity();
  jointPoseInParent.translate(Eigen::Vector3d(0.0, 0.0, -4));
  joint->setTransformFromParentBodyNode(jointPoseInParent);

  // Vector6d vels = compose({0, 0, 0}, {0, 0, -100});
  // Vector6d vels = compose({0, 25, 0}, {0, 0, -100});
  // Vector6d vels = compose({0, 25, 0}, {0, 0, 0});
  // rootBn->getParentJoint()->setVelocities(vels);

  pair.second->addExtTorque({0, 250000, 0});
  pair.second->addExtForce({0,0, -200000});
  for (int i = 0; i < g_iters; ++i)
  {
    auto pose = comFrame->getWorldTransform();
    std::cout << i << " " << pose.translation().transpose() << std::endl;
    world->step();
  }
  auto position = comFrame->getWorldTransform().translation();
  EXPECT_NEAR(0.0, position.x(), tol);
  EXPECT_NEAR(0.0, position.y(), tol);
  // EXPECT_NEAR(g_iters * dt * vels[5], position.z(), tol);
}

TEST(Issue1193, WithRevoluteJoint)
{
  WorldPtr world = World::create();
  const double dt = 0.001;
  world->setTimeStep(dt);
  world->setGravity(Vector3d::Zero());
  SkeletonPtr skel = createBox({1, 1, 1}, {0, 0, 2});
  skel->disableSelfCollisionCheck();
  world->addSkeleton(skel);
  auto rootBn = skel->getRootBodyNode();

  Eigen::Isometry3d comPose;
  comPose = Eigen::Translation3d(0, 0, -2);
  auto comFrame = SimpleFrame::createShared(rootBn, "CombinedCOM", comPose);

  GenericJoint<R1Space>::Properties joint2Prop(std::string("joint2"));
  BodyNode::Properties link2Prop(
      BodyNode::AspectProperties(std::string("link2")));
  link2Prop.mInertia.setMass(1.0);

  auto pair = rootBn->createChildJointAndBodyNodePair<RevoluteJoint>(
      RevoluteJoint::Properties(joint2Prop, Vector3d(1, 0, 0)), link2Prop);

  auto *joint = pair.first;
  (void) joint;

  Eigen::Isometry3d jointPoseInParent = Eigen::Isometry3d::Identity();
  jointPoseInParent.translate(Eigen::Vector3d(0.0, 0.0, -4));
  joint->setTransformFromParentBodyNode(jointPoseInParent);

  // Vector6d vels = compose({0, 25, 0}, {0, 0, -100});

  // rootBn->getParentJoint()->setVelocities(vels);

  // pair.second->addExtTorque({0, 250000, 0});
  // pair.second->addExtForce({0,0, -200000});
  for (int i = 0; i < g_iters; ++i)
  {
    joint->setCommand(0, 0.1);
    // joint->setCommand(0, 10.0);
    world->step();
    auto pose = rootBn->getWorldTransform();
    auto poseCOM = comFrame->getWorldTransform();
    std::cout << i << " " << pose.translation().transpose() << "\t"
      << poseCOM.translation().transpose() << std::endl;
  }

  auto position = comFrame->getWorldTransform().translation();
  EXPECT_NEAR(0.0, position.x(), tol);
  EXPECT_NEAR(0.0, position.y(), tol);
  // EXPECT_NEAR(g_iters * dt * vels[5], position.z(), tol);
}

TEST(Issue1193, WithRevoluteJointSdf)
{
  auto world = SdfParser::readWorld(
      "dart://sample/sdf/test/issue1193_revolute_test.sdf");
  ASSERT_TRUE(world != nullptr);
  const double dt = 0.001;
  world->setTimeStep(dt);

  SkeletonPtr skel = world->getSkeleton(0);
  auto rootBn = skel->getRootBodyNode();

  Eigen::Isometry3d comPose;
  comPose = Eigen::Translation3d(0, 0, -2);
  auto comFrame = SimpleFrame::createShared(rootBn, "CombinedCOM", comPose);

  auto *joint = skel->getJoint("revJoint");
  ASSERT_NE(nullptr, joint);

  for (int i = 0; i < g_iters; ++i)
  {
    // joint->setCommand(0, 0.1);
    joint->setCommand(0, 10.0);
    world->step();
    auto pose = rootBn->getWorldTransform();
    auto poseCOM = comFrame->getWorldTransform();
    std::cout << i << " " << pose.translation().transpose() << "\t"
      << poseCOM.translation().transpose() << std::endl;
  }

  auto position = comFrame->getWorldTransform().translation();
  EXPECT_NEAR(0.0, position.x(), tol);
  EXPECT_NEAR(0.0, position.y(), tol);
}

TEST(Issue1193, WithRevoluteJointWithOffsetSdf)
{
  auto world = SdfParser::readWorld(
      "dart://sample/sdf/test/issue1193_revolute_with_offset_test.sdf");
  ASSERT_TRUE(world != nullptr);
  const double dt = 0.0001;
  world->setTimeStep(dt);
  world->setGravity(Vector3d::Zero());

  SkeletonPtr skel = world->getSkeleton(0);
  auto link1 = skel->getBodyNode("link1");

  Eigen::Isometry3d comPose;
  comPose = Eigen::Translation3d(0, 0, -2);
  auto comFrame = SimpleFrame::createShared(link1, "CombinedCOM", comPose);

  auto *joint = skel->getJoint("revJoint");
  ASSERT_NE(nullptr, joint);

  Eigen::Vector3d maxDeviationFromOrigin = Eigen::Vector3d::Zero();
  // link1->getParentJoint()->setVelocities(compose({0, 25, 0}, {0, 0, -100}));

  joint->setForce(0, 100.0);
  world->step();
  Eigen::Vector3d h0 = computeWorldAngularMomentum(skel);
  double h0Mag = h0.norm();
  std::cout <<  "H0: " << h0.transpose() << std::endl;
  for (int i = 1; i < 1000*g_iters; ++i)
  {
    // joint->setCommand(0, 10.0);
    world->step();
    auto pose = link1->getWorldTransform();
    auto poseCOM = comFrame->getWorldTransform();
    // std::cout << i << " " << poseCOM.translation().transpose() << std::endl;
    maxDeviationFromOrigin
        = poseCOM.translation().cwiseAbs().cwiseMax(maxDeviationFromOrigin);
    // std::cout << i << " " << maxDeviationFromOrigin.transpose() << "\n";

    Eigen::Vector3d hNext = computeWorldAngularMomentum(skel);
    std::cout << i << " " << (hNext - h0).transpose() / h0Mag << std::endl;
  }

  auto position = comFrame->getWorldTransform().translation();
  EXPECT_NEAR(0.0, maxDeviationFromOrigin.x(), tol);
  EXPECT_NEAR(2.0, maxDeviationFromOrigin.y(), tol);
  EXPECT_NEAR(0.0, maxDeviationFromOrigin.z(), tol);
}

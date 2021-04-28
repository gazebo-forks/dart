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

using namespace dart::math;
using namespace dart::collision;
using namespace dart::dynamics;
using namespace dart::simulation;

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


TEST(Issue000, SingleBody)
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

  rootBn->addExtTorque({0, 50, 0});
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

TEST(Issue000, SingleBodyWithOffDiagonalMoi)
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

TEST(Issue000, SingleBodyWithJointOffset)
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

//==============================================================================
TEST(Issue000, WithFixedJoint)
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
  // auto position = comFrame->getWorldTransform().translation();
  // EXPECT_NEAR(0.0, position.x(), tol);
  // EXPECT_NEAR(0.0, position.y(), tol);
  // EXPECT_NEAR(g_iters * dt * vels[5], position.z(), tol);
}

TEST(Issue000, WithRevoluteJoint)
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
    joint->setCommand(0, 1000.0);
    // joint->setCommand(0, 10.0);
    world->step();
    auto pose = rootBn->getWorldTransform();
    auto poseCOM = comFrame->getWorldTransform();
    std::cout << i << " " << pose.translation().transpose() << "\t"
      << poseCOM.translation().transpose() << std::endl;
  }

  // auto position = comFrame->getWorldTransform().translation();
  // EXPECT_NEAR(0.0, position.x(), tol);
  // EXPECT_NEAR(0.0, position.y(), tol);
  // EXPECT_NEAR(g_iters * dt * vels[5], position.z(), tol);
}

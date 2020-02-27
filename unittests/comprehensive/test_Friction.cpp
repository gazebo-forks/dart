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

#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/math/Random.hpp"

#include "TestHelpers.hpp"

using namespace dart;
using namespace dynamics;

//==============================================================================
dynamics::SkeletonPtr createFloor()
{
  auto floor = dynamics::Skeleton::create("floor");

  // Give the floor a body
  auto body
      = floor->createJointAndBodyNodePair<dynamics::WeldJoint>(nullptr).second;

  // Give the body a shape
  double floorWidth = 100.0;
  double floorHeight = 0.01;
  auto box = std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d(floorWidth, floorWidth, floorHeight));
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::LightGray());

  // Put the body into position
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -0.5 - floorHeight / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

//==============================================================================
TEST(Friction, FrictionPerShapeNode)
{
  auto skeleton1
      = createBox(Eigen::Vector3d(0.3, 0.3, 0.3), Eigen::Vector3d(-0.5, 0, 0));
  skeleton1->setName("Skeleton1");
  auto skeleton2
      = createBox(Eigen::Vector3d(0.3, 0.3, 0.3), Eigen::Vector3d(+0.5, 0, 0));
  skeleton1->setName("Skeleton2");

  auto body1 = skeleton1->getRootBodyNode();
  EXPECT_DOUBLE_EQ(
      body1->getShapeNode(0)->getDynamicsAspect()->getFrictionCoeff(), 1.0);

  auto body2 = skeleton2->getRootBodyNode();
  EXPECT_DOUBLE_EQ(
      body2->getShapeNode(0)->getDynamicsAspect()->getFrictionCoeff(), 1.0);
  body2->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(0.0);
  EXPECT_DOUBLE_EQ(
      body2->getShapeNode(0)->getDynamicsAspect()->getFrictionCoeff(), 0.0);

  // Create a world and add the rigid body
  auto world = simulation::World::create();
  EXPECT_TRUE(equals(world->getGravity(), ::Eigen::Vector3d(0, 0, -9.81)));
  world->setGravity(Eigen::Vector3d(0.0, -5.0, -9.81));
  EXPECT_TRUE(equals(world->getGravity(), ::Eigen::Vector3d(0.0, -5.0, -9.81)));

  world->addSkeleton(createFloor());
  world->addSkeleton(skeleton1);
  world->addSkeleton(skeleton2);

  const auto numSteps = 500;
  for (auto i = 0u; i < numSteps; ++i)
  {
    world->step();

    // Wait until the first box settle-in on the ground
    if (i > 300)
    {
      const auto y1 = body1->getTransform().translation()[1];
      EXPECT_NEAR(y1, -0.17889, 0.001);

      // The second box still moves even after landed on the ground because its
      // friction is zero.
      const auto y2 = body2->getTransform().translation()[1];
      EXPECT_LE(y2, -0.17889);
    }
  }
}

//==============================================================================
TEST(Friction, SlipVelocity)
{
  auto skeleton1 = createSphere(0.5);
  auto body1 = skeleton1->getRootBodyNode();
  body1->setMass(10);
  // Set the COM to the bottom of the sphere so it doesn't roll.
  body1->setLocalCOM({0, 0, -0.5});
  auto shapeNode1 = body1->getShapeNode(0);
  EXPECT_DOUBLE_EQ(shapeNode1->getDynamicsAspect()->getFrictionCoeff(), 1.0);

  shapeNode1->getDynamicsAspect()->setFirstFrictionDirection({1, 0, 0});

  auto world = simulation::World::create();

  auto floor = createFloor();
  EXPECT_DOUBLE_EQ(
      floor->getShapeNode(0)->getDynamicsAspect()->getFrictionCoeff(), 1.0);
  world->addSkeleton(floor);
  world->addSkeleton(skeleton1);

  const double slipParam = 0.001;
  const Eigen::Vector3d extForce{body1->getMass() * 5, 0, 0};
  const auto numSteps = 2000;
  for (auto i = 0u; i < numSteps; ++i)
  {
    body1->addExtForce(extForce, body1->getLocalCOM(), false);
    world->step();
    const auto linVel = body1->getLinearVelocity();
    const auto angVel = body1->getAngularVelocity();
    const auto x1 = linVel[0];

    EXPECT_NEAR(0.0, angVel.norm(), 0.001);

    // Wait until the first box settle-in on the ground
    if (i == 300)
    {
      // Expect that the friction is high enough to prevent motion
      EXPECT_NEAR(x1, 0.0, 0.001);

      // Now set slip
      body1->getShapeNode(0)->getDynamicsAspect()->setSlipCompliance(slipParam);
    }
    // Wait for the ball to accelerate to its slip velocity
    else if (i > 1000)
    {
      const double expVel = slipParam * (extForce.x());
      EXPECT_NEAR(expVel, x1, 0.001);
    }
  }
}

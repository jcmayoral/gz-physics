/*
 * Copyright (C) 2024 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <memory>
#include <mutex>
#include <unordered_map>
#include <optional>
#include <utility>
#include <limits>
#include <btBulletCollisionCommon.h>
#include <unordered_set>
#include <btBulletDynamicsCommon.h>
#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/ode/OdeCollisionGroup.hpp>
#include <dart/collision/bullet/BulletCollisionGroup.hpp>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

#include <gz/common/Console.hh>

#include "GzCollisionDetector.hh"

using namespace dart;
using namespace collision;

/////////////////////////////////////////////////
GzCollisionDetector::GzCollisionDetector()
{
}

/////////////////////////////////////////////////
void GzCollisionDetector::SetCollisionPairMaxContacts(
    std::size_t _maxContacts)
{
  this->maxCollisionPairContacts = _maxContacts;
}

/////////////////////////////////////////////////
std::size_t GzCollisionDetector::GetCollisionPairMaxContacts() const
{
  return this->maxCollisionPairContacts;
}

/////////////////////////////////////////////////
void GzCollisionDetector::LimitCollisionPairMaxContacts(
    CollisionResult *_result)
{
  if (this->maxCollisionPairContacts ==
    std::numeric_limits<std::size_t>::max())
    return;

  auto allContacts = _result->getContacts();
  _result->clear();


  if (this->maxCollisionPairContacts == 0u)
    return;

  // A map of collision pairs and their contact info
  // Contact info is stored in std::pair. The elements are:
  // <contact count, index of last contact point (in _result)>
  std::unordered_map<dart::collision::CollisionObject *,
      std::unordered_map<dart::collision::CollisionObject *,
      std::pair<std::size_t, std::size_t>>>
      contactMap;

  for (auto &contact : allContacts)
  {
    auto &[count, lastContactIdx] =
        contactMap[contact.collisionObject1][contact.collisionObject2];
    count++;
    auto &[otherCount, otherLastContactIdx] =
      contactMap[contact.collisionObject2][contact.collisionObject1];

    std::size_t total =  count + otherCount;
    if (total <= this->maxCollisionPairContacts)
    {
      if (total == this->maxCollisionPairContacts)
      {
        lastContactIdx = _result->getNumContacts();
        otherLastContactIdx = lastContactIdx;
      }
      _result->addContact(contact);
    }
    else
    {
      // If too many contacts were generated, replace the last contact point
      // of the collision pair with one that has a larger penetration depth
      auto &c = _result->getContact(lastContactIdx);
      if (contact.penetrationDepth > c.penetrationDepth)
      {
        c = contact;
      }
    }
  }
}

/////////////////////////////////////////////////
std::optional<std::vector<GzRayResult>> GzCollisionDetector::BatchRaycast(
    CollisionGroup */*_group*/,
    const std::vector<GzRay> &/*_rays*/) const
{
  static bool warned = false;
  if (!warned)
  {
    warned = true;
    gzwarn << "BatchRaycast: collision detector does not support batch "
           << "raycasting. All ray results will be NaN." << std::endl;
  }
  return std::nullopt;
}

/////////////////////////////////////////////////
GzOdeCollisionDetector::GzOdeCollisionDetector()
  : OdeCollisionDetector(), GzCollisionDetector()
{
}

/////////////////////////////////////////////////
GzOdeCollisionDetector::Registrar<GzOdeCollisionDetector>
    GzOdeCollisionDetector::mRegistrar{
        GzOdeCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<GzOdeCollisionDetector> {
          return GzOdeCollisionDetector::create();
        }};

/////////////////////////////////////////////////
std::shared_ptr<GzOdeCollisionDetector> GzOdeCollisionDetector::create()
{
  // GzOdeCollisionDetector constructor calls the OdeCollisionDetector
  // constructor, that calls the non-thread safe dInitODE2(0).
  // To mitigate this problem, we use a static mutex to ensure that
  // each GzOdeCollisionDetector constructor is called not at the same time.
  // See https://github.com/gazebosim/gz-sim/issues/18 for more info.
  static std::mutex odeInitMutex;
  std::unique_lock<std::mutex> lock(odeInitMutex);
  return std::shared_ptr<GzOdeCollisionDetector>(new GzOdeCollisionDetector());
}

/////////////////////////////////////////////////
bool GzOdeCollisionDetector::collide(
    CollisionGroup *_group,
    const CollisionOption &_option,
    CollisionResult *_result)
{
  bool ret = OdeCollisionDetector::collide(_group, _option, _result);
  this->LimitCollisionPairMaxContacts(_result);
  return ret;
}

/////////////////////////////////////////////////
bool GzOdeCollisionDetector::collide(
    CollisionGroup *_group1,
    CollisionGroup *_group2,
    const CollisionOption &_option,
    CollisionResult *_result)
{
  bool ret = OdeCollisionDetector::collide(_group1, _group2, _option, _result);
  this->LimitCollisionPairMaxContacts(_result);
  return ret;
}

class GzOdeCollisionGroup : public dart::collision::OdeCollisionGroup
{
public:
explicit GzOdeCollisionGroup(
  const dart::collision::CollisionDetectorPtr &_detector)
  : dart::collision::OdeCollisionGroup(_detector)
  {}

  public: dSpaceID Space() const
  {
    return this->mSpaceId;
  }
};

/////////////////////////////////////////////////
std::optional<std::vector<GzRayResult>> GzOdeCollisionDetector::BatchRaycast(
    CollisionGroup *_group,
    const std::vector<GzRay> &_rays) const
{
  auto *gzGroup = dynamic_cast<GzOdeCollisionGroup *>(_group);
  if (!gzGroup)
    return std::nullopt;

  dSpaceID worldSpace = gzGroup->Space();
  if (!worldSpace)
    return std::nullopt;

  std::vector<GzRayResult> results;
  results.reserve(_rays.size());

  static const Eigen::Vector3d kNaN =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

  for (const auto &ray : _rays)
  {
    GzRayResult out;
    out.point = kNaN;
    out.normal = kNaN;
    out.fraction = std::numeric_limits<double>::infinity();

    Eigen::Vector3d origin = ray.origin;
    Eigen::Vector3d target = ray.target;
    Eigen::Vector3d dir = target - origin;
    double len = dir.norm();

    if (len < 1e-9)
    {
      results.push_back(out);
      continue;
    }

    dir /= len;
    double paddedLen = len + 0.20;   // Más robusto en movimiento

    // Geom RAY sin space intermedio (rápido)
    dGeomID rayGeom = dCreateRay(0, paddedLen);
    dGeomRaySet(rayGeom,
                origin.x(), origin.y(), origin.z(),
                dir.x(), dir.y(), dir.z());

    dGeomRaySetParams(rayGeom, 0, 0);
    dGeomRaySetClosestHit(rayGeom, 1);

    struct HitData
    {
      double bestT = std::numeric_limits<double>::infinity();
      Eigen::Vector3d point;
      Eigen::Vector3d normal;
      Eigen::Vector3d origin;
      Eigen::Vector3d dir;
      double len;
    } hit;

    hit.origin = origin;
    hit.dir = dir;
    hit.len = len;

    auto callback = [](void *data, dGeomID o1, dGeomID o2)
    {
      HitData *h = static_cast<HitData *>(data);

      dContactGeom contacts[16];
      int n = dCollide(o1, o2, 16, contacts, sizeof(dContactGeom));

      for (int i = 0; i < n; ++i)
      {
        const auto &c = contacts[i];

        Eigen::Vector3d p(c.pos[0], c.pos[1], c.pos[2]);
        Eigen::Vector3d rel = p - h->origin;
        double t = rel.dot(h->dir);

        // 🔥 Anti-huecos y anti-movimiento
        const double eps = 0.05;  // 5 cm de tolerancia
        if (t < -eps || t > h->len + eps)
          continue;

        if (t < h->bestT)
        {
          h->bestT = t;
          h->point = p;
          h->normal = Eigen::Vector3d(
              c.normal[0], c.normal[1], c.normal[2]);
        }
      }
    };

    dSpaceCollide2(rayGeom, (dGeomID)worldSpace, &hit, callback);

    dGeomDestroy(rayGeom);

    if (hit.bestT < std::numeric_limits<double>::infinity())
    {
      out.point = hit.point;
      out.normal = hit.normal;
      out.fraction = hit.bestT / len;
    }

    results.push_back(out);
  }

  return results;
}


std::unique_ptr<dart::collision::CollisionGroup>
GzOdeCollisionDetector::createCollisionGroup()
{
  return std::make_unique<GzOdeCollisionGroup>(this->shared_from_this());
}



/// \brief Exposes BulletCollisionGroup::getBulletCollisionWorld() which
/// is protected in the base class.
class GzBulletCollisionGroup : public dart::collision::BulletCollisionGroup
{
  public: explicit GzBulletCollisionGroup(
      const dart::collision::CollisionDetectorPtr &_detector);

  /// \brief Return the underlying btCollisionWorld
  public: const btCollisionWorld *getCollisionWorld() const;
};

/////////////////////////////////////////////////
GzBulletCollisionGroup::GzBulletCollisionGroup(
    const dart::collision::CollisionDetectorPtr &_detector)
  : dart::collision::BulletCollisionGroup(_detector)
{
}

/////////////////////////////////////////////////
const btCollisionWorld *GzBulletCollisionGroup::getCollisionWorld() const
{
  // getBulletCollisionWorld() is protected in BulletCollisionGroup.
  return this->getBulletCollisionWorld();
}

/////////////////////////////////////////////////
GzBulletCollisionDetector::GzBulletCollisionDetector()
  : BulletCollisionDetector(), GzCollisionDetector()
{
}

/////////////////////////////////////////////////
std::unique_ptr<dart::collision::CollisionGroup>
GzBulletCollisionDetector::createCollisionGroup()
{
  return std::make_unique<GzBulletCollisionGroup>(this->shared_from_this());
}


/////////////////////////////////////////////////
GzBulletCollisionDetector::Registrar<GzBulletCollisionDetector>
    GzBulletCollisionDetector::mRegistrar{
        GzBulletCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<GzBulletCollisionDetector> {
          return GzBulletCollisionDetector::create();
        }};

/////////////////////////////////////////////////
std::shared_ptr<GzBulletCollisionDetector> GzBulletCollisionDetector::create()
{
  return std::shared_ptr<GzBulletCollisionDetector>(
      new GzBulletCollisionDetector());
}

/////////////////////////////////////////////////
bool GzBulletCollisionDetector::collide(
    CollisionGroup *_group,
    const CollisionOption &_option,
    CollisionResult *_result)
{
  bool ret = BulletCollisionDetector::collide(_group, _option, _result);
  this->LimitCollisionPairMaxContacts(_result);
  return ret;
}

/////////////////////////////////////////////////
bool GzBulletCollisionDetector::collide(
    CollisionGroup *_group1,
    CollisionGroup *_group2,
    const CollisionOption &_option,
    CollisionResult *_result)
{
  bool ret = BulletCollisionDetector::collide(
      _group1, _group2, _option, _result);
  this->LimitCollisionPairMaxContacts(_result);
  return ret;
}

/////////////////////////////////////////////////
std::optional<std::vector<GzRayResult>> GzBulletCollisionDetector::BatchRaycast(
    CollisionGroup *_group,
    const std::vector<GzRay> &_rays) const
{
  std::vector<GzRayResult> results;
  results.reserve(_rays.size());

  auto *gzGroup = dynamic_cast<GzBulletCollisionGroup *>(_group);
  if (!gzGroup)
    return std::nullopt;

  const btCollisionWorld *btWorld = gzGroup->getCollisionWorld();
  if (!btWorld)
    return std::nullopt;

  for (const auto &ray : _rays)
  {
    const btVector3 btFrom(
      static_cast<btScalar>(ray.origin.x()),
      static_cast<btScalar>(ray.origin.y()),
      static_cast<btScalar>(ray.origin.z()));
    const btVector3 btTo(
      static_cast<btScalar>(ray.target.x()),
      static_cast<btScalar>(ray.target.y()),
      static_cast<btScalar>(ray.target.z()));

    btCollisionWorld::ClosestRayResultCallback rayCallback(btFrom, btTo);
    btWorld->rayTest(btFrom, btTo, rayCallback);

    GzRayResult result;
    if (rayCallback.hasHit())
    {
      const btVector3 &hp = rayCallback.m_hitPointWorld;
      const btVector3 &hn = rayCallback.m_hitNormalWorld;
      result.point << hp.x(), hp.y(), hp.z();
      result.normal << hn.x(), hn.y(), hn.z();
      result.fraction = static_cast<double>(rayCallback.m_closestHitFraction);
    }
    else
    {
      // No object in range: fraction is +INF per REP-117.
      // point and normal are undefined (NaN) when there is no hit.
      constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
      result.point = Eigen::Vector3d::Constant(kNaN);
      result.fraction = std::numeric_limits<double>::infinity();
      result.normal = Eigen::Vector3d::Constant(kNaN);
    }
    results.push_back(std::move(result));
  }

  return results;
}

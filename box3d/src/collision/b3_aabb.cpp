
#include "collision/b3_aabb.hpp"


b3AABB::b3AABB(const b3Vector3d &lower_bound, const b3Vector3d &upper_bound) {
  m_min = lower_bound;
  m_max = upper_bound;
}


<<<<<<< HEAD
bool b3AABB::overlapped(const b3AABB &A, const b3AABB &B)
{
    if (A.m_max.x() < B.m_min.x() || A.m_min.x() > B.m_max.x()) return false;
    if (A.m_max.y() < B.m_min.y() || A.m_min.y() > B.m_max.y()) return false;
    if (A.m_max.z() < B.m_min.z() || A.m_min.z() > B.m_max.z()) return false;
=======
bool b3AABB::overlapped(const b3AABB &A, const b3AABB &B) {
  bool result = true;

  if (A.m_max.x() < B.m_min.x() || A.m_min.x() > B.m_max.x()) return false;
  if (A.m_max.y() < B.m_min.y() || A.m_min.y() > B.m_max.y()) return false;
  if (A.m_max.z() < B.m_min.z() || A.m_min.z() > B.m_max.z()) return false;
>>>>>>> origin/zhb-dev

  return true;
}



using System;
using System.Collections.Generic;
using stupid.Maths;

namespace stupid.Colliders
{
    public static class CollisionMath
    {
        private static Vector3S[] _axes = new Vector3S[15];
        private static List<Vector3S> _contactPoints = new List<Vector3S>();

        public static int SphereVSphere(Vector3S positionA, Vector3S positionB, f32 radA, f32 radB, ref ContactS contact)
        {
            f32 squaredDistance = Vector3S.DistanceSquared(positionA, positionB);
            f32 combinedRadius = radA + radB;
            f32 squaredCombinedRadius = combinedRadius * combinedRadius;

            if (squaredDistance > squaredCombinedRadius)
            {
                return -1; // No intersection
            }

            Vector3S direction = (positionA - positionB).NormalizeWithMagnitude(out f32 distance);
            Vector3S point = positionB + direction * radB;
            Vector3S normal = direction;
            f32 penetrationDepth = combinedRadius - distance;

            contact.point = point;
            contact.normal = normal;
            contact.penetrationDepth = penetrationDepth;
            return 1;
        }

        //Contact point on box, normal pointing towards box
        public static int BoxVsSphere(BoxColliderS box, SphereColliderS sphere, ref ContactS contact)
        {
            var boxTrans = box.attachedCollidable.transform;
            var sphereTrans = sphere.attachedCollidable.transform;

            // Transform the sphere center into the box's local space
            var inverseBoxRotation = boxTrans.rotationMatrix.Transpose();
            var localSpherePos = inverseBoxRotation * (sphereTrans.position - boxTrans.position);
            var halfSize = box.halfSize;

            // Find the closest point on the box to the sphere center
            var closestPoint = new Vector3S(
                MathS.Clamp(localSpherePos.x, -halfSize.x, halfSize.x),
                MathS.Clamp(localSpherePos.y, -halfSize.y, halfSize.y),
                MathS.Clamp(localSpherePos.z, -halfSize.z, halfSize.z)
            );

            // Calculate the vector from the closest point to the sphere center
            var distanceVector = localSpherePos - closestPoint;
            var distanceSquared = distanceVector.sqrMagnitude;

            // If the distance is greater than the sphere's radius, there's no intersection
            if (distanceSquared > sphere.radius * sphere.radius)
                return 0;

            // Calculate the distance and the normal vector pointing from the closest point to the sphere center
            var distance = MathS.Sqrt(distanceSquared);
            var normal = distance > f32.epsilon ? distanceVector / distance : new Vector3S(1, 0, 0); // Default normal

            // Transform the closest point back to world space
            var worldClosestPoint = boxTrans.position + (boxTrans.rotationMatrix * closestPoint);

            // Transform the normal back to world space
            var worldNormal = (boxTrans.rotationMatrix * normal).Normalize();

            // Calculate penetration depth
            var penetrationDepth = sphere.radius - distance;
            if (penetrationDepth < f32.zero)
                penetrationDepth = f32.zero; // Ensure non-negative depth

            contact.point = worldClosestPoint;
            contact.normal = -worldNormal;
            contact.penetrationDepth = penetrationDepth;

            return 1;
        }


        //Contact point on sphere, normal points towards sphere
        public static int SphereVsBox(SphereColliderS sphere, BoxColliderS box, ref ContactS contact)
        {
            var boxTrans = box.attachedCollidable.transform;
            var sphereTrans = sphere.attachedCollidable.transform;

            // Transform the sphere center into the box's local space
            var inverseBoxRotation = boxTrans.rotationMatrix.Transpose();
            var localSpherePos = inverseBoxRotation * (sphereTrans.position - boxTrans.position);
            var halfSize = box.halfSize;

            // Find the closest point on the box to the sphere center
            var closestPoint = new Vector3S(
                MathS.Clamp(localSpherePos.x, -halfSize.x, halfSize.x),
                MathS.Clamp(localSpherePos.y, -halfSize.y, halfSize.y),
                MathS.Clamp(localSpherePos.z, -halfSize.z, halfSize.z)
            );

            // Calculate the vector from the closest point to the sphere center
            var distanceVector = localSpherePos - closestPoint;
            var distanceSquared = distanceVector.sqrMagnitude;

            // If the distance is greater than the sphere's radius, there's no intersection
            if (distanceSquared > sphere.radius * sphere.radius)
                return 0;

            // Calculate the distance and the normal vector pointing from the closest point to the sphere center
            var distance = MathS.Sqrt(distanceSquared);
            var normal = distance > f32.epsilon ? distanceVector / distance : new Vector3S(1, 0, 0); // Default normal

            // Transform the normal back to world space
            var worldNormal = (boxTrans.rotationMatrix * normal).Normalize();

            // Calculate the contact point on the sphere surface
            var worldContactPoint = sphereTrans.position - worldNormal * sphere.radius;

            // Calculate penetration depth
            var penetrationDepth = sphere.radius - distance;
            if (penetrationDepth < f32.zero)
                penetrationDepth = f32.zero; // Ensure non-negative depth

            contact.point = worldContactPoint;
            contact.normal = worldNormal;
            contact.penetrationDepth = penetrationDepth;
            return 1;
        }


        public static int BoxVsBox(BoxColliderS a, BoxColliderS b, ref ContactS contact)
        {
            Vector3S relativePosition = b.attachedCollidable.transform.position - a.attachedCollidable.transform.position;

            f32 minOverlap = f32.maxValue;
            Vector3S minAxis = Vector3S.zero;

            // Check for overlaps on the primary axes of both boxes
            if (!CheckOverlapOnAxis(relativePosition, a.axes[0], a, b, ref minOverlap, ref minAxis)) return 0;
            if (!CheckOverlapOnAxis(relativePosition, a.axes[1], a, b, ref minOverlap, ref minAxis)) return 0;
            if (!CheckOverlapOnAxis(relativePosition, a.axes[2], a, b, ref minOverlap, ref minAxis)) return 0;
            if (!CheckOverlapOnAxis(relativePosition, b.axes[0], a, b, ref minOverlap, ref minAxis)) return 0;
            if (!CheckOverlapOnAxis(relativePosition, b.axes[1], a, b, ref minOverlap, ref minAxis)) return 0;
            if (!CheckOverlapOnAxis(relativePosition, b.axes[2], a, b, ref minOverlap, ref minAxis)) return 0;

            // Check for overlaps on the cross product of axes pairs
            if (!CheckOverlapOnAxis(relativePosition, Vector3S.Cross(a.axes[0], b.axes[0]).Normalize(), a, b, ref minOverlap, ref minAxis)) return 0;
            if (!CheckOverlapOnAxis(relativePosition, Vector3S.Cross(a.axes[0], b.axes[1]).Normalize(), a, b, ref minOverlap, ref minAxis)) return 0;
            if (!CheckOverlapOnAxis(relativePosition, Vector3S.Cross(a.axes[0], b.axes[2]).Normalize(), a, b, ref minOverlap, ref minAxis)) return 0;
            if (!CheckOverlapOnAxis(relativePosition, Vector3S.Cross(a.axes[1], b.axes[0]).Normalize(), a, b, ref minOverlap, ref minAxis)) return 0;
            if (!CheckOverlapOnAxis(relativePosition, Vector3S.Cross(a.axes[1], b.axes[1]).Normalize(), a, b, ref minOverlap, ref minAxis)) return 0;
            if (!CheckOverlapOnAxis(relativePosition, Vector3S.Cross(a.axes[1], b.axes[2]).Normalize(), a, b, ref minOverlap, ref minAxis)) return 0;
            if (!CheckOverlapOnAxis(relativePosition, Vector3S.Cross(a.axes[2], b.axes[0]).Normalize(), a, b, ref minOverlap, ref minAxis)) return 0;
            if (!CheckOverlapOnAxis(relativePosition, Vector3S.Cross(a.axes[2], b.axes[1]).Normalize(), a, b, ref minOverlap, ref minAxis)) return 0;
            if (!CheckOverlapOnAxis(relativePosition, Vector3S.Cross(a.axes[2], b.axes[2]).Normalize(), a, b, ref minOverlap, ref minAxis)) return 0;

            Vector3S normal = minAxis.Normalize();

            // Flip the normal if it's pointing in the wrong direction
            if (Vector3S.Dot(normal, relativePosition) > f32.zero)
            {
                normal = -normal;
            }

            Vector3S contactPoint = FindContactPoint(a, b);
            f32 penetrationDepth = minOverlap;

            if (contactPoint != Vector3S.zero)
            {
                contact.point = contactPoint;
                contact.normal = normal;
                contact.penetrationDepth = penetrationDepth;
                return 1;
            }

            return 0;
        }

        private static bool CheckOverlapOnAxis(Vector3S relativePosition, Vector3S axis, BoxColliderS a, BoxColliderS b, ref f32 minOverlap, ref Vector3S minAxis)
        {
            if (axis.sqrMagnitude <= f32.epsilon) return true; // Skip zero-length axes

            if (!OverlapOnAxis(relativePosition, axis, a, b, out f32 overlap))
            {
                return false; // No overlap on this axis, no collision
            }

            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                minAxis = axis;
            }

            return true;
        }

        private static bool OverlapOnAxis(Vector3S relativePosition, Vector3S axis, BoxColliderS a, BoxColliderS b, out f32 overlap)
        {
            f32 projectionA = ProjectBox(axis, a);
            f32 projectionB = ProjectBox(axis, b);
            f32 distance = MathS.Abs(Vector3S.Dot(relativePosition, axis));
            overlap = projectionA + projectionB - distance;
            return overlap > f32.zero;
        }

        private static f32 ProjectBox(Vector3S axis, BoxColliderS box)
        {
            Vector3S halfSize = box.halfSize;
            var rotMat = box.attachedCollidable.transform.rotationMatrix;
            return
                halfSize.x * MathS.Abs(Vector3S.Dot(rotMat.GetColumn(0), axis)) +
                halfSize.y * MathS.Abs(Vector3S.Dot(rotMat.GetColumn(1), axis)) +
                halfSize.z * MathS.Abs(Vector3S.Dot(rotMat.GetColumn(2), axis));
        }

        private static Vector3S FindContactPoint(BoxColliderS a, BoxColliderS b)
        {
            _contactPoints.Clear();

            AddContactPointsIfInsideOBB(a.vertices, b, b.attachedCollidable.transform.position, b.halfSize, b.attachedCollidable.transform.rotationMatrix);
            AddContactPointsIfInsideOBB(b.vertices, a, a.attachedCollidable.transform.position, a.halfSize, a.attachedCollidable.transform.rotationMatrix);

            if (_contactPoints.Count == 0)
            {
                return Vector3S.zero;
            }

            return CalculateAverageContactPoint();
        }

        private static void AddContactPointsIfInsideOBB(Vector3S[] vertices, BoxColliderS box, Vector3S position, Vector3S halfSize, Matrix3S rotation)
        {
            for (int i = 0; i < vertices.Length; i++)
            {
                if (IsPointInsideOBB(vertices[i], position, halfSize, rotation))
                {
                    _contactPoints.Add(vertices[i]);
                }
            }
        }

        private static Vector3S CalculateAverageContactPoint()
        {
            Vector3S sum = Vector3S.zero;
            for (int i = 0; i < _contactPoints.Count; i++)
            {
                sum += _contactPoints[i];
            }

            return sum / (f32)_contactPoints.Count;
        }

        private static bool IsPointInsideOBB(Vector3S point, Vector3S position, Vector3S halfSize, Matrix3S rotation)
        {
            Vector3S localPoint = rotation.Transpose() * (point - position);
            return MathS.Abs(localPoint.x) <= halfSize.x && MathS.Abs(localPoint.y) <= halfSize.y && MathS.Abs(localPoint.z) <= halfSize.z;
        }
    }
}

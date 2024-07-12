using System;
using System.Collections.Generic;
using stupid.Maths;

namespace stupid.Colliders
{
    public static class CollisionMath
    {
        public static bool SphereVSphere(Vector3S positionA, Vector3S positionB, f32 radA, f32 radB, out ContactS contact)
        {
            // Calculate the squared distance between the two positions
            var squaredDistance = Vector3S.DistanceSquared(positionA, positionB);

            // Calculate the combined radius of both spheres
            var combinedRadius = radA + radB;
            var squaredCombinedRadius = combinedRadius * combinedRadius;

            // If the squared distance is greater than the squared combined radius, there is no intersection
            if (squaredDistance > squaredCombinedRadius)
            {
                contact = new ContactS();
                return false;
            }

            // Calculate direction and distance between the spheres
            var direction = (positionB - positionA).NormalizeWithMagnitude(out var distance);

            // Set contact information
            contact.point = positionA + direction * radA;
            contact.normal = direction;
            contact.penetrationDepth = combinedRadius - distance;

            return true;
        }

        public static bool BoxVsSphere(Vector3S boxPosition, QuaternionS boxRotation, Vector3S boxSize, Vector3S spherePosition, f32 sphereRadius, out ContactS contact)
        {
            contact = new ContactS();

            // Transform the sphere center into the box's local space
            Vector3S localSpherePosition = QuaternionS.Inverse(boxRotation) * (spherePosition - boxPosition);
            Vector3S halfSize = boxSize * f32.half;

            // Clamp the sphere's local position to the box's extents to find the closest point
            Vector3S closestPoint = new Vector3S(
                MathS.Clamp(localSpherePosition.x, -halfSize.x, halfSize.x),
                MathS.Clamp(localSpherePosition.y, -halfSize.y, halfSize.y),
                MathS.Clamp(localSpherePosition.z, -halfSize.z, halfSize.z)
            );

            // Calculate the distance between the sphere's center and this closest point
            Vector3S distanceVector = localSpherePosition - closestPoint;
            f32 distanceSquared = distanceVector.SqrMagnitude;

            // If the distance is greater than the sphere's radius, there's no intersection
            if (distanceSquared > sphereRadius * sphereRadius)
            {
                return false;
            }

            // Calculate the actual distance and normalize the distance vector
            f32 distance = MathS.Sqrt(distanceSquared);
            Vector3S normal = distance > f32.epsilon ? distanceVector / distance : Vector3S.zero;

            // Transform the closest point and normal back to world space
            contact.point = boxPosition + (boxRotation * closestPoint);
            contact.normal = boxRotation * normal;
            contact.penetrationDepth = sphereRadius - distance;

            return true;
        }

        static Vector3S[] _axes = new Vector3S[6];
        static List<Vector3S> _testAxes = new List<Vector3S>();
        public static bool BoxVsBox(BoxColliderS a, BoxColliderS b, out ContactS contact)
        {
            contact = new ContactS();

            var at = a.attachedCollidable.transform;
            var bt = b.attachedCollidable.transform;

            Vector3S halfSizeA = a.size * f32.half;
            Vector3S halfSizeB = b.size * f32.half;

            Matrix3S rotA = at.rotationMatrix;
            Matrix3S rotB = bt.rotationMatrix;

            Vector3S relativePosition = bt.position - at.position;

            _axes[0] = rotA.GetColumn(0);
            _axes[1] = rotA.GetColumn(1);
            _axes[2] = rotA.GetColumn(2);
            _axes[3] = rotB.GetColumn(0);
            _axes[4] = rotB.GetColumn(1);
            _axes[5] = rotB.GetColumn(2);

            _testAxes.Clear();
            _testAxes.AddRange(_axes);

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    Vector3S cross = Vector3S.Cross(_axes[i], _axes[3 + j]);
                    if (cross.SqrMagnitude > f32.epsilon)
                    {
                        _testAxes.Add(cross.Normalize());
                    }
                }
            }

            f32 minOverlap = f32.maxValue;
            Vector3S minAxis = Vector3S.zero;

            foreach (Vector3S axis in _testAxes)
            {
                if (!OverlapOnAxis(relativePosition, axis, halfSizeA, halfSizeB, rotA, rotB, out f32 overlap))
                {
                    return false; // No overlap on this axis, no collision
                }
                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    minAxis = axis;
                }
            }

            Vector3S normal = minAxis;
            if (Vector3S.Dot(minAxis, relativePosition) < f32.zero)
            {
                normal = -minAxis;
            }

            contact.normal = normal;
            contact.penetrationDepth = minOverlap;
            contact.point = FindContactPoints(at.position, halfSizeA, rotA, bt.position, halfSizeB, rotB, a.vertices, b.vertices);

            return true;
        }

        private static bool OverlapOnAxis(Vector3S relativePosition, Vector3S axis, Vector3S halfSizeA, Vector3S halfSizeB, Matrix3S rotA, Matrix3S rotB, out f32 overlap)
        {
            var projectionA = ProjectBox(halfSizeA, axis, rotA);
            var projectionB = ProjectBox(halfSizeB, axis, rotB);
            var distance = MathS.Abs(Vector3S.Dot(relativePosition, axis));
            overlap = projectionA + projectionB - distance;
            return overlap > f32.zero;
        }

        private static f32 ProjectBox(Vector3S halfSize, Vector3S axis, Matrix3S rotation)
        {
            return
                halfSize.x * MathS.Abs(Vector3S.Dot(rotation.GetColumn(0), axis)) +
                halfSize.y * MathS.Abs(Vector3S.Dot(rotation.GetColumn(1), axis)) +
                halfSize.z * MathS.Abs(Vector3S.Dot(rotation.GetColumn(2), axis));
        }


        static List<Vector3S> _contactPoints = new List<Vector3S>();
        private static Vector3S FindContactPoints(Vector3S positionA, Vector3S halfSizeA, Matrix3S rotationA, Vector3S positionB, Vector3S halfSizeB, Matrix3S rotationB, Vector3S[] verticesA, Vector3S[] verticesB)
        {
            _contactPoints.Clear();

            foreach (var vertexA in verticesA)
            {
                if (IsPointInsideOBB(vertexA, positionB, halfSizeB, rotationB))
                {
                    _contactPoints.Add(vertexA);
                }
            }

            foreach (var vertexB in verticesB)
            {
                if (IsPointInsideOBB(vertexB, positionA, halfSizeA, rotationA))
                {
                    _contactPoints.Add(vertexB);
                }
            }

            if (_contactPoints.Count == 0)
            {
                return Vector3S.zero;
            }

            Vector3S averageContactPoint = Vector3S.zero;
            foreach (var point in _contactPoints)
            {
                averageContactPoint += point;
            }

            return averageContactPoint / (f32)_contactPoints.Count;
        }

        private static bool IsPointInsideOBB(Vector3S point, Vector3S position, Vector3S halfSize, Matrix3S rotation)
        {
            Vector3S localPoint = rotation.Transpose() * (point - position);
            return MathS.Abs(localPoint.x) <= halfSize.x && MathS.Abs(localPoint.y) <= halfSize.y && MathS.Abs(localPoint.z) <= halfSize.z;
        }


    }
}

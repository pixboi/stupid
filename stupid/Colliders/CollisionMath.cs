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

        public static bool BoxVsBox(Vector3S positionA, QuaternionS rotationA, Vector3S sizeA, Vector3S positionB, QuaternionS rotationB, Vector3S sizeB, out ContactS contact)
        {
            contact = new ContactS();

            Vector3S halfSizeA = sizeA * f32.half;
            Vector3S halfSizeB = sizeB * f32.half;

            Matrix3S rotA = Matrix3S.Rotate(rotationA);
            Matrix3S rotB = Matrix3S.Rotate(rotationB);

            Vector3S relativePosition = positionB - positionA;

            List<Vector3S> axes = new List<Vector3S>
    {
        rotA.GetColumn(0),
        rotA.GetColumn(1),
        rotA.GetColumn(2),
        rotB.GetColumn(0),
        rotB.GetColumn(1),
        rotB.GetColumn(2)
    };

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    Vector3S cross = Vector3S.Cross(axes[i], axes[3 + j]);
                    if (cross.SqrMagnitude > f32.epsilon)
                    {
                        axes.Add(cross.Normalize());
                    }
                }
            }

            f32 minOverlap = f32.maxValue;
            Vector3S minAxis = Vector3S.zero;

            foreach (Vector3S axis in axes)
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
            contact.point = FindContactPoints(positionA, halfSizeA, rotA, positionB, halfSizeB, rotB, normal);

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

        private static Vector3S FindContactPoints(Vector3S positionA, Vector3S halfSizeA, Matrix3S rotationA, Vector3S positionB, Vector3S halfSizeB, Matrix3S rotationB, Vector3S normal)
        {
            Vector3S[] verticesA = GetBoxVertices(positionA, halfSizeA, rotationA);
            Vector3S[] verticesB = GetBoxVertices(positionB, halfSizeB, rotationB);

            List<Vector3S> contactPoints = new List<Vector3S>();

            foreach (var vertexA in verticesA)
            {
                if (IsPointInsideOBB(vertexA, positionB, halfSizeB, rotationB))
                {
                    contactPoints.Add(vertexA);
                }
            }

            foreach (var vertexB in verticesB)
            {
                if (IsPointInsideOBB(vertexB, positionA, halfSizeA, rotationA))
                {
                    contactPoints.Add(vertexB);
                }
            }

            if (contactPoints.Count == 0)
            {
                return Vector3S.zero;
            }

            Vector3S averageContactPoint = Vector3S.zero;
            foreach (var point in contactPoints)
            {
                averageContactPoint += point;
            }

            return averageContactPoint / (f32)contactPoints.Count;
        }

        private static Vector3S[] GetBoxVertices(Vector3S position, Vector3S halfSize, Matrix3S rotation)
        {
            Vector3S[] vertices = new Vector3S[8];

            Vector3S right = rotation.GetColumn(0) * halfSize.x;
            Vector3S up = rotation.GetColumn(1) * halfSize.y;
            Vector3S forward = rotation.GetColumn(2) * halfSize.z;

            vertices[0] = position + right + up + forward;
            vertices[1] = position + right + up - forward;
            vertices[2] = position + right - up + forward;
            vertices[3] = position + right - up - forward;
            vertices[4] = position - right + up + forward;
            vertices[5] = position - right + up - forward;
            vertices[6] = position - right - up + forward;
            vertices[7] = position - right - up - forward;

            return vertices;
        }

        private static bool IsPointInsideOBB(Vector3S point, Vector3S position, Vector3S halfSize, Matrix3S rotation)
        {
            Vector3S localPoint = rotation.Transpose() * (point - position);
            return MathS.Abs(localPoint.x) <= halfSize.x && MathS.Abs(localPoint.y) <= halfSize.y && MathS.Abs(localPoint.z) <= halfSize.z;
        }


    }
}

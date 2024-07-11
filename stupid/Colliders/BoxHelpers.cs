using System;
using System.Collections.Generic;
using stupid.Maths;

namespace stupid.Colliders
{
    public static class BoxHelpers
    {
        public static bool IntersectBoxSphere(Vector3S boxPosition, QuaternionS boxRotation, Vector3S boxSize, Vector3S spherePosition, f32 sphereRadius, out ContactS contact)
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

        public static bool IntersectBox(Vector3S positionA, QuaternionS rotationA, Vector3S sizeA, Vector3S positionB, QuaternionS rotationB, Vector3S sizeB, out ContactS contact)
        {
            contact = new ContactS();

            Vector3S halfSizeA = sizeA * f32.half;
            Vector3S halfSizeB = sizeB * f32.half;

            Matrix3S rotA = Matrix3S.Rotate(rotationA);
            Matrix3S rotB = Matrix3S.Rotate(rotationB);

            Vector3S relativePosition = positionB - positionA;
            //Vector3S t = rotA.Transpose() * relativePosition;

            List<Vector3S> axes = new List<Vector3S>
            {
                new Vector3S(rotA.m00, rotA.m01, rotA.m02),
                new Vector3S(rotA.m10, rotA.m11, rotA.m12),
                new Vector3S(rotA.m20, rotA.m21, rotA.m22),
                new Vector3S(rotB.m00, rotB.m01, rotB.m02),
                new Vector3S(rotB.m10, rotB.m11, rotB.m12),
                new Vector3S(rotB.m20, rotB.m21, rotB.m22)
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
                if (!OverlapOnAxis(relativePosition, axis, halfSizeA, halfSizeB, rotA, rotB, out f32 penetration))
                {
                    return false;
                }
                if (penetration < minOverlap)
                {
                    minOverlap = penetration;
                    minAxis = axis;
                }
            }

            contact.normal = minAxis;
            contact.penetrationDepth = minOverlap;
            contact.point = CalculateContactPoint(positionA, halfSizeA, rotA, positionB, halfSizeB, rotB, minAxis);

            return true;
        }

        private static bool OverlapOnAxis(Vector3S relativePosition, Vector3S axis, Vector3S halfSizeA, Vector3S halfSizeB, Matrix3S rotA, Matrix3S rotB, out f32 penetration)
        {
            f32 projectionA = ProjectBox(halfSizeA, axis, rotA);
            f32 projectionB = ProjectBox(halfSizeB, axis, rotB);
            f32 distance = MathS.Abs(Vector3S.Dot(relativePosition, axis));
            f32 overlap = projectionA + projectionB - distance;

            penetration = overlap;
            return overlap > f32.zero;
        }

        private static f32 ProjectBox(Vector3S halfSize, Vector3S axis, Matrix3S rotation)
        {
            Vector3S absAxis = new Vector3S(
                MathS.Abs(Vector3S.Dot(new Vector3S(rotation.m00, rotation.m01, rotation.m02), axis)),
                MathS.Abs(Vector3S.Dot(new Vector3S(rotation.m10, rotation.m11, rotation.m12), axis)),
                MathS.Abs(Vector3S.Dot(new Vector3S(rotation.m20, rotation.m21, rotation.m22), axis))
            );

            return absAxis.x * halfSize.x + absAxis.y * halfSize.y + absAxis.z * halfSize.z;
        }

        private static Vector3S CalculateContactPoint(Vector3S positionA, Vector3S halfSizeA, Matrix3S rotationA, Vector3S positionB, Vector3S halfSizeB, Matrix3S rotationB, Vector3S normal)
        {
            Vector3S pointA = FindClosestPointOnBox(positionA, halfSizeA, rotationA, positionB, normal);
            Vector3S pointB = FindClosestPointOnBox(positionB, halfSizeB, rotationB, positionA, -normal);

            return (pointA + pointB) * f32.half;
        }

        private static Vector3S FindClosestPointOnBox(Vector3S boxPosition, Vector3S boxHalfSize, Matrix3S boxRotation, Vector3S point, Vector3S direction)
        {
            Vector3S localPoint = boxRotation.Transpose() * (point - boxPosition);

            Vector3S closestPoint = new Vector3S(
                MathS.Clamp(localPoint.x, -boxHalfSize.x, boxHalfSize.x),
                MathS.Clamp(localPoint.y, -boxHalfSize.y, boxHalfSize.y),
                MathS.Clamp(localPoint.z, -boxHalfSize.z, boxHalfSize.z)
            );

            return boxPosition + (boxRotation * closestPoint);
        }
    }
}

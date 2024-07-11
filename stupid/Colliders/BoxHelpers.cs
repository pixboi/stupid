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

            // Calculate the relative position
            Vector3S relativePosition = positionB - positionA;

            // Calculate the half sizes
            Vector3S halfSizeA = sizeA * f32.half;
            Vector3S halfSizeB = sizeB * f32.half;

            // Calculate the axes to test
            List<Vector3S> axes = new List<Vector3S>
            {
                rotationA * Vector3S.right,
                rotationA * Vector3S.up,
                rotationA * Vector3S.forward,
                rotationB * Vector3S.right,
                rotationB * Vector3S.up,
                rotationB * Vector3S.forward
            };

            // Add the cross products of the axes of both boxes to the axis list
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    Vector3S cross = Vector3S.Cross(axes[i], axes[3 + j]);
                    if (cross.SqrMagnitude > f32.epsilon) // Avoid near-zero length vectors
                    {
                        axes.Add(cross.Normalize());
                    }
                }
            }

            // Check for separation on each axis and collect contact points and normals
            foreach (Vector3S axis in axes)
            {
                if (!OverlapOnAxis(relativePosition, axis, halfSizeA, halfSizeB, rotationA, rotationB, out f32 penetration))
                {
                    return false; // Separation found
                }
                contact.normal = axis;
                contact.penetrationDepth = penetration;
                contact.point = CalculateContactPoint(positionA, halfSizeA, rotationA, positionB, halfSizeB, rotationB, axis);
            }

            return true;
        }

        private static bool OverlapOnAxis(Vector3S relativePosition, Vector3S axis, Vector3S halfSizeA, Vector3S halfSizeB, QuaternionS rotationA, QuaternionS rotationB, out f32 penetration)
        {
            f32 projectionA = ProjectBox(halfSizeA, axis, rotationA);
            f32 projectionB = ProjectBox(halfSizeB, axis, rotationB);
            f32 distance = MathS.Abs(Vector3S.Dot(relativePosition, axis));
            f32 overlap = projectionA + projectionB - distance;

            penetration = overlap;
            return overlap > f32.zero;
        }

        private static f32 ProjectBox(Vector3S halfSize, Vector3S axis, QuaternionS rotation)
        {
            Vector3S absAxis = new Vector3S(
                MathS.Abs(Vector3S.Dot(rotation * Vector3S.right, axis)),
                MathS.Abs(Vector3S.Dot(rotation * Vector3S.up, axis)),
                MathS.Abs(Vector3S.Dot(rotation * Vector3S.forward, axis))
            );

            return absAxis.x * halfSize.x + absAxis.y * halfSize.y + absAxis.z * halfSize.z;
        }

        private static Vector3S CalculateContactPoint(Vector3S positionA, Vector3S halfSizeA, QuaternionS rotationA, Vector3S positionB, Vector3S halfSizeB, QuaternionS rotationB, Vector3S normal)
        {
            // Find the closest points on each box to the other box along the collision normal
            Vector3S pointA = FindClosestPointOnBox(positionA, halfSizeA, rotationA, positionB, normal);
            Vector3S pointB = FindClosestPointOnBox(positionB, halfSizeB, rotationB, positionA, -normal);

            // Return the midpoint of the closest points as the contact point
            return (pointA + pointB) * f32.half;
        }

        private static Vector3S FindClosestPointOnBox(Vector3S boxPosition, Vector3S boxHalfSize, QuaternionS boxRotation, Vector3S point, Vector3S direction)
        {
            // Transform the point and direction into the box's local space
            Vector3S localPoint = QuaternionS.Inverse(boxRotation) * (point - boxPosition);

            // Calculate the closest point in local space
            Vector3S closestPoint = new Vector3S(
                MathS.Clamp(localPoint.x, -boxHalfSize.x, boxHalfSize.x),
                MathS.Clamp(localPoint.y, -boxHalfSize.y, boxHalfSize.y),
                MathS.Clamp(localPoint.z, -boxHalfSize.z, boxHalfSize.z)
            );

            // Transform the closest point back to world space
            return boxPosition + (boxRotation * closestPoint);
        }
    }
}

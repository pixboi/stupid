using stupid.Maths;
using System.Collections.Generic;

namespace stupid.Colliders
{
    public class BoxColliderS : BaseShape
    {
        public Vector3S size { get; private set; }
        public Vector3S halfSize { get; private set; }
        public BoxColliderS(Vector3S size)
        {
            this.size = size;
            this.halfSize = size * f32.half;
        }

        public override BoundsS CalculateBounds(Vector3S position)
        {
            // Get the rotation matrix from the quaternion
            var rotationMatrix = Matrix3S.Rotate(attachedCollidable.transform.rotation);

            // Calculate the extents of the rotated box along each axis
            Vector3S rotatedHalfSize = new Vector3S(
                MathS.Abs(rotationMatrix.m00) * halfSize.x + MathS.Abs(rotationMatrix.m01) * halfSize.y + MathS.Abs(rotationMatrix.m02) * halfSize.z,
                MathS.Abs(rotationMatrix.m10) * halfSize.x + MathS.Abs(rotationMatrix.m11) * halfSize.y + MathS.Abs(rotationMatrix.m12) * halfSize.z,
                MathS.Abs(rotationMatrix.m20) * halfSize.x + MathS.Abs(rotationMatrix.m21) * halfSize.y + MathS.Abs(rotationMatrix.m22) * halfSize.z
            );

            var min = position - rotatedHalfSize;
            var max = position + rotatedHalfSize;
            _bounds = new BoundsS(min, max);
            return _bounds;
        }

        public override bool Intersects(Collidable other, out ContactS contact)
        {
            contact = new ContactS();

            if (other.collider is BoxColliderS otherBox)
            {
                return IntersectBox(
                    this.attachedCollidable.transform.position,
                    this.attachedCollidable.transform.rotation,
                    this.size,
                    other.transform.position,
                    other.transform.rotation,
                    otherBox.size,
                    out contact);
            }
            else if (other.collider is SphereColliderS otherSphere)
            {
                return IntersectBoxSphere(
                    this.attachedCollidable.transform.position,
                    this.attachedCollidable.transform.rotation,
                    this.size,
                    other.transform.position,
                    otherSphere.radius,
                    out contact);
            }

            return false;
        }

        static readonly f32 boxInertia = (f32)1 / (f32)12;
        public override Matrix3S CalculateInertiaTensor(f32 mass)
        {
            // For a solid box: I = 1/12 * m * (h^2 + d^2) for each axis
            var h = size.x;
            var d = size.y;
            var w = size.z;
            var inertiaX = boxInertia * mass * (d * d + w * w);
            var inertiaY = boxInertia * mass * (h * h + w * w);
            var inertiaZ = boxInertia * mass * (h * h + d * d);

            return new Matrix3S(
                new Vector3S(inertiaX, f32.zero, f32.zero),
                new Vector3S(f32.zero, inertiaY, f32.zero),
                new Vector3S(f32.zero, f32.zero, inertiaZ)
            );
        }

        public static bool IntersectBoxSphere(Vector3S boxPosition, QuaternionS boxRotation, Vector3S boxSize, Vector3S spherePosition, f32 sphereRadius, out ContactS contact)
        {
            contact = new ContactS();

            // Transform the sphere center into the box's local space
            var localSpherePosition = QuaternionS.Inverse(boxRotation) * (spherePosition - boxPosition);
            var halfSize = boxSize * f32.half;

            // Clamp the sphere's local position to the box's extents to find the closest point
            var closestPoint = new Vector3S(
                MathS.Clamp(localSpherePosition.x, -halfSize.x, halfSize.x),
                MathS.Clamp(localSpherePosition.y, -halfSize.y, halfSize.y),
                MathS.Clamp(localSpherePosition.z, -halfSize.z, halfSize.z)
            );

            // Calculate the distance between the sphere's center and this closest point
            var distanceVector = localSpherePosition - closestPoint;
            var distanceSquared = distanceVector.SqrMagnitude;

            // If the distance is greater than the sphere's radius, there's no intersection
            if (distanceSquared > sphereRadius * sphereRadius)
            {
                return false;
            }

            // Calculate the actual distance and normalize the distance vector
            var distance = MathS.Sqrt(distanceSquared);
            var normal = distance > f32.epsilon ? distanceVector / distance : Vector3S.zero;

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
            var relativePosition = positionB - positionA;

            // Calculate the half sizes
            var halfSizeA = sizeA * f32.half;
            var halfSizeB = sizeB * f32.half;

            // Calculate the axes to test
            var axes = new List<Vector3S>
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
                    var cross = Vector3S.Cross(axes[i], axes[3 + j]);
                    if (cross.SqrMagnitude > f32.epsilon) // Avoid near-zero length vectors
                    {
                        axes.Add(cross.Normalize());
                    }
                }
            }

            // Check for separation on each axis
            f32 minPenetration = f32.maxValue;
            Vector3S minPenetrationAxis = Vector3S.zero;
            bool intersect = true;

            foreach (var axis in axes)
            {
                if (!OverlapOnAxis(relativePosition, axis, halfSizeA, halfSizeB, rotationA, rotationB, out var penetration))
                {
                    intersect = false;
                    break; // Separation found
                }

                if (penetration < minPenetration)
                {
                    minPenetration = penetration;
                    minPenetrationAxis = axis;
                }
            }

            if (!intersect)
            {
                return false;
            }

            // Calculate contact information
            contact.point = CalculateContactPoint(positionA, halfSizeA, rotationA, positionB, halfSizeB, rotationB, minPenetrationAxis);
            contact.normal = minPenetrationAxis;
            contact.penetrationDepth = minPenetration;

            return true;
        }

        private static bool OverlapOnAxis(Vector3S relativePosition, Vector3S axis, Vector3S halfSizeA, Vector3S halfSizeB, QuaternionS rotationA, QuaternionS rotationB, out f32 penetration)
        {
            var projectionA = ProjectBox(halfSizeA, axis, rotationA);
            var projectionB = ProjectBox(halfSizeB, axis, rotationB);
            var distance = MathS.Abs(Vector3S.Dot(relativePosition, axis));
            var overlap = projectionA + projectionB - distance;

            penetration = overlap;
            return overlap > f32.zero;
        }

        private static f32 ProjectBox(Vector3S halfSize, Vector3S axis, QuaternionS rotation)
        {
            var absAxis = new Vector3S(
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
            Vector3S localDirection = QuaternionS.Inverse(boxRotation) * direction;

            // Calculate the closest point in local space
            Vector3S closestPoint = localPoint;

            // Project the local point onto the box's surfaces along the local direction
            if (localDirection.x != f32.zero)
            {
                f32 tx = (localDirection.x > f32.zero ? boxHalfSize.x - localPoint.x : -boxHalfSize.x - localPoint.x) / localDirection.x;
                closestPoint.x = localPoint.x + tx * localDirection.x;
            }
            if (localDirection.y != f32.zero)
            {
                f32 ty = (localDirection.y > f32.zero ? boxHalfSize.y - localPoint.y : -boxHalfSize.y - localPoint.y) / localDirection.y;
                closestPoint.y = localPoint.y + ty * localDirection.y;
            }
            if (localDirection.z != f32.zero)
            {
                f32 tz = (localDirection.z > f32.zero ? boxHalfSize.z - localPoint.z : -boxHalfSize.z - localPoint.z) / localDirection.z;
                closestPoint.z = localPoint.z + tz * localDirection.z;
            }

            // Clamp the closest point to the box's extents
            closestPoint.x = MathS.Clamp(closestPoint.x, -boxHalfSize.x, boxHalfSize.x);
            closestPoint.y = MathS.Clamp(closestPoint.y, -boxHalfSize.y, boxHalfSize.y);
            closestPoint.z = MathS.Clamp(closestPoint.z, -boxHalfSize.z, boxHalfSize.z);

            // Transform the closest point back to world space
            return boxPosition + (boxRotation * closestPoint);
        }
    }
}

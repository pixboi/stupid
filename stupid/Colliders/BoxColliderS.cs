using stupid.Maths;

namespace stupid.Colliders
{
    public class BoxColliderS : BaseShape
    {
        public Vector3S size { get; private set; }

        public BoxColliderS(Vector3S size)
        {
            this.size = size;
        }

        public override BoundsS CalculateBounds(Vector3S position)
        {
            // Calculate the world-space AABB of the box collider
            var halfSize = size * (f32)0.5f;

            var rotatedHalfSize = halfSize.Rotate(attachedCollidable.transform.rotation);
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

        public override Matrix3S CalculateInertiaTensor(f32 mass)
        {
            // For a solid box: I = 1/12 * m * (h^2 + d^2) for each axis
            var h = size.x;
            var d = size.y;
            var w = size.z;
            var inertiaX = ((f32)1 / (f32)12) * mass * (d * d + w * w);
            var inertiaY = ((f32)1 / (f32)12) * mass * (h * h + w * w);
            var inertiaZ = ((f32)1 / (f32)12) * mass * (h * h + d * d);

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
            var halfSize = boxSize * (f32)0.5f;

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
            var epsilon = (f32)1e-6f; // Small value to prevent division by zero
            var normal = distance > epsilon ? distanceVector / distance : Vector3S.zero;

            // Transform the closest point and normal back to world space
            contact.point = boxPosition + (boxRotation * closestPoint);
            contact.normal = boxRotation * normal;
            contact.penetrationDepth = sphereRadius - distance;

            return true;
        }

        static readonly Vector3S[] DIRS =
        {
                Vector3S.right,
               Vector3S.up,
               Vector3S.forward,
             Vector3S.right,
             Vector3S.up,
                Vector3S.forward
        };

        public static bool IntersectBox(Vector3S positionA, QuaternionS rotationA, Vector3S sizeA, Vector3S positionB, QuaternionS rotationB, Vector3S sizeB, out ContactS contact)
        {
            contact = new ContactS();

            // Calculate the relative position and orientation
            var relativePosition = positionB - positionA;
            var relativeRotation = rotationB * rotationA.Conjugate();

            // Calculate the half sizes
            var halfSizeA = sizeA * (f32)0.5f;
            var halfSizeB = sizeB * (f32)0.5f;

            // Calculate the axes
            var axes = new Vector3S[]
            {
                rotationA * Vector3S.right,
                rotationA * Vector3S.up,
                rotationA * Vector3S.forward,
                rotationB * Vector3S.right,
                rotationB * Vector3S.up,
                rotationB * Vector3S.forward
            };

            // Check for separation on each axis
            foreach (var axis in axes)
            {
                if (!OverlapOnAxis(relativePosition, axis, halfSizeA, halfSizeB, relativeRotation))
                {
                    return false;
                }
            }

            // Calculate contact information (simple version)
            contact.point = positionA + halfSizeA; // Approximate contact point
            contact.normal = (relativePosition).Normalize(); // Approximate normal
            contact.penetrationDepth = (halfSizeA + halfSizeB).Magnitude() - relativePosition.Magnitude(); // Approximate penetration depth

            return true;
        }

        private static bool OverlapOnAxis(Vector3S relativePosition, Vector3S axis, Vector3S halfSizeA, Vector3S halfSizeB, QuaternionS relativeRotation)
        {
            var projectionA = ProjectBox(halfSizeA, axis);
            var projectionB = ProjectBox(halfSizeB, relativeRotation * axis);
            var distance = MathS.Abs(Vector3S.Dot(relativePosition, axis));

            return distance <= projectionA + projectionB;
        }

        private static f32 ProjectBox(Vector3S halfSize, Vector3S axis)
        {
            return MathS.Abs(Vector3S.Dot(axis, Vector3S.right) * halfSize.x) +
                   MathS.Abs(Vector3S.Dot(axis, Vector3S.up) * halfSize.y) +
                   MathS.Abs(Vector3S.Dot(axis, Vector3S.forward) * halfSize.z);
        }

        public override int Intersects(Collidable other, ref ContactS[] contactCache)
        {
            throw new System.NotImplementedException();
        }
    }
}
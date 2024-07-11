using stupid.Maths;

namespace stupid.Colliders
{
    public class SphereColliderS : BaseShape
    {
        public f32 radius { get; private set; }
        public SphereColliderS(f32 radius)
        {
            this.radius = radius;
        }

        public override BoundsS CalculateBounds(Vector3S position)
        {
            var size = new Vector3S(radius, radius, radius);
            _bounds = new BoundsS(position - size, position + size);
            return _bounds;
        }

        public override bool Intersects(Collidable other, out ContactS contact)
        {
            contact = new ContactS();

            if (other.collider is SphereColliderS otherSphere)
            {
                return IntersectSphere(
                    this.attachedCollidable.transform.position,
                    otherSphere.attachedCollidable.transform.position,
                    this.radius,
                    otherSphere.radius,
                    out contact);
            }

            if (other.collider is BoxColliderS otherBox)
            {
                return otherBox.Intersects(this.attachedCollidable, out contact);
            }

            return false;
        }

        static readonly f32 sphereInertia = ((f32)2f / (f32)5f);
        public override Matrix3S CalculateInertiaTensor(f32 mass)
        {
            // For a solid sphere: I = 2/5 * m * r^2
            f32 inertia = sphereInertia * mass * radius * radius;

            return new Matrix3S(
                new Vector3S(inertia, f32.zero, f32.zero),
                new Vector3S(f32.zero, inertia, f32.zero),
                new Vector3S(f32.zero, f32.zero, inertia)
            );
        }


        public static bool IntersectSphere(Vector3S positionA, Vector3S positionB, f32 radA, f32 radB, out ContactS contact)
        {
            contact = new ContactS();

            // Calculate the squared distance between the two positions
            var squaredDistance = Vector3S.DistanceSquared(positionA, positionB);

            // Calculate the combined radius of both spheres
            var combinedRadius = radA + radB;
            var squaredCombinedRadius = combinedRadius * combinedRadius;

            // If the squared distance is greater than the squared combined radius, there is no intersection
            if (squaredDistance > squaredCombinedRadius)
            {
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
    }
}

using stupid.Maths;

namespace stupid.Colliders
{
    public class SSphereCollider : BaseCollider
    {
        public f32 Radius { get; private set; }

        public SSphereCollider(f32 radius)
        {
            Radius = radius;
        }

        public override SBounds CalculateBounds(Vector3S position)
        {
            var size = new Vector3S(Radius, Radius, Radius);
            _bounds = new SBounds(position - size, position + size);
            return _bounds;
        }

        public override SBounds GetBounds() => _bounds;

        public override bool Intersects(Vector3S positionA, Vector3S positionB, ICollider other, out Contact contact)
        {
            contact = new Contact();

            if (other is SSphereCollider otherSphere)
            {
                return IntersectsSphere(positionA, positionB, otherSphere, out contact);
            }
            else if (other is SBoxCollider otherBox)
            {
                return otherBox.Intersects(positionB, positionA, this, out contact);
            }

            return false;
        }

        public override Matrix3S CalculateInertiaTensor(f32 mass)
        {
            // For a solid sphere: I = 2/5 * m * r^2
            f32 inertia = (f32.FromRaw(2) / f32.FromRaw(5)) * mass * Radius * Radius;
            return new Matrix3S(
                new Vector3S(inertia, f32.zero, f32.zero),
                new Vector3S(f32.zero, inertia, f32.zero),
                new Vector3S(f32.zero, f32.zero, inertia)
            );
        }

        private bool IntersectsSphere(Vector3S positionA, Vector3S positionB, SSphereCollider otherSphere, out Contact contact)
        {
            contact = new Contact();

            // Calculate the squared distance between the two positions
            var squaredDistance = Vector3S.DistanceSquared(positionA, positionB);

            // Calculate the combined radius of both spheres
            var combinedRadius = Radius + otherSphere.Radius;
            var squaredCombinedRadius = combinedRadius * combinedRadius;

            // If the squared distance is greater than the squared combined radius, there is no intersection
            if (squaredDistance > squaredCombinedRadius)
            {
                return false;
            }

            // Calculate direction and distance between the spheres
            var direction = (positionB - positionA).NormalizeWithMagnitude(out var distance);

            // Set contact information
            contact.point = positionA + direction * Radius;
            contact.normal = direction;
            contact.penetrationDepth = combinedRadius - distance;

            return true;
        }
    }
}

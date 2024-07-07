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

        private bool IntersectsSphere(Vector3S positionA, Vector3S positionB, SSphereCollider otherSphere, out Contact contact)
        {
            contact = new Contact();

            f32 squaredDistance = Vector3S.DistanceSquared(positionA, positionB);
            f32 combinedRadius = Radius + otherSphere.Radius;
            f32 squaredCombinedRadius = combinedRadius * combinedRadius;

            if (squaredDistance <= squaredCombinedRadius)
            {
                Vector3S direction = (positionB - positionA).NormalizeWithMagnitude(out var distance);
                //f32 distance = Vector3S.Distance(positionA, positionB); // We calculate the distance only when necessary
                contact.point = positionA + direction * Radius;
                contact.normal = direction;
                contact.penetrationDepth = combinedRadius - distance;
                return true;
            }

            return false;
        }

    }
}

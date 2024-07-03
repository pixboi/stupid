using SoftFloat;
using stupid.Maths;

namespace stupid.Colliders
{
    public class SSphereCollider : ICollider
    {
        public sfloat radius { get; private set; }
        public SRigidbody AttachedRigidbody { get; private set; }
        public SRigidbody GetRigidbody() { return AttachedRigidbody; }

        public SSphereCollider(sfloat radius)
        {
            this.radius = radius;
        }

        public void Attach(SRigidbody rigidbody)
        {
            AttachedRigidbody = rigidbody;
            CalculateBounds(rigidbody.position);
        }

        public Vector3S _size { get; private set; }
        public SBounds _bounds { get; private set; }

        public SBounds CalculateBounds(Vector3S position)
        {
            _size = new Vector3S(radius, radius, radius);
            _bounds = new SBounds(position - _size, position + _size);
            return _bounds;
        }

        public SBounds GetBounds()
        {
            return _bounds;
        }

        public bool Intersects(Vector3S positionA, Vector3S positionB, ICollider other, out Contact contact)
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

            sfloat distance = Vector3S.Distance(positionA, positionB);
            sfloat combinedRadius = radius + otherSphere.radius;

            if (distance <= combinedRadius)
            {
                // Calculate collision information
                contact.point = positionA + (positionB - positionA) * (radius / combinedRadius);
                contact.normal = (positionB - positionA).Normalize();
                contact.penetrationDepth = combinedRadius - distance;
                return true;
            }

            return false;
        }
    }
}

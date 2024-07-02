using SoftFloat;

namespace stupid
{
    public class SphereCollider : ICollider
    {
        public sfloat radius { get; private set; }
        public Rigidbody attachedRigidbody { get; private set; }
        public Rigidbody GetRigidbody() { return attachedRigidbody; }
        public SphereCollider(sfloat radius)
        {
            this.radius = radius;
        }

        public void Attach(Rigidbody rigidbody)
        {
            attachedRigidbody = rigidbody;
            CalculateBounds(rigidbody.position);
        }

        public Vector3S _size { get; private set; }
        public Bounds _bounds { get; private set; }
        public Bounds CalculateBounds(Vector3S position)
        {
            _size = new Vector3S(radius, radius, radius);
            var bounds = new Bounds(position - _size, position + _size);
            _bounds = bounds;
            return _bounds;
        }

        public Bounds GetBounds()
        {
            return _bounds;
        }

        public bool Intersects(Vector3S positionA, Vector3S positionB, ICollider other, out Contact contact)
        {
            contact = new Contact();

            if (other is SphereCollider otherSphere)
            {
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
            }
            return false;
        }
    }
}

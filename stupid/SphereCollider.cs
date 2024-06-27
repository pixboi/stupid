using SoftFloat;
using System.Drawing;

namespace stupid
{
    public class SphereCollider : ICollider
    {
        public sfloat radius;
        public Rigidbody attachedRigidbody {  get; private set; }

        Vector3S _size;

        public SphereCollider(sfloat radius)
        {
            this.radius = radius;
            _size = new Vector3S(radius, radius, radius);
        }

        public void Attach(Rigidbody rigidbody)
        {
            attachedRigidbody = rigidbody;
        }

        public Bounds GetBounds(Vector3S position)
        {
            return new Bounds(position - _size, position + _size);
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

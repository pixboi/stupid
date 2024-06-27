using SoftFloat;

namespace stupid
{
    public class SphereCollider : ICollider
    {
        public sfloat radius;
        public Rigidbody attachedRigidbody;

        public SphereCollider(sfloat radius)
        {
            this.radius = radius;
        }

        public void Attach(Rigidbody rigidbody)
        {
            attachedRigidbody = rigidbody;
        }

        public Bounds GetBounds(Vector3S position)
        {
            Vector3S size = new Vector3S(radius, radius, radius);
            return new Bounds(position - size, position + size);
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

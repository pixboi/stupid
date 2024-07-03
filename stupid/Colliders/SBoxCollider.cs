using SoftFloat;
using stupid.Maths;

namespace stupid.Colliders
{
    public class SBoxCollider : ICollider
    {
        public Vector3S Size { get; private set; }
        public SRigidbody AttachedRigidbody { get; private set; }
        public SRigidbody GetRigidbody() { return AttachedRigidbody; }

        public SBoxCollider(Vector3S size)
        {
            Size = size;
        }

        public void Attach(SRigidbody rigidbody)
        {
            AttachedRigidbody = rigidbody;
            CalculateBounds(rigidbody.position);
        }

        public SBounds _bounds { get; private set; }

        public SBounds CalculateBounds(Vector3S position)
        {
            var halfSize = Size * (sfloat)0.5f;
            _bounds = new SBounds(position - halfSize, position + halfSize);
            return _bounds;
        }

        public SBounds GetBounds()
        {
            return _bounds;
        }

        public bool Intersects(Vector3S positionA, Vector3S positionB, ICollider other, out Contact contact)
        {
            contact = new Contact();

            if (other is SBoxCollider otherBox)
            {
                return IntersectsBox(positionA, positionB, otherBox, out contact);
            }
            else if (other is SSphereCollider otherSphere)
            {
                return IntersectsSphere(positionA, positionB, otherSphere, out contact);
            }

            return false;
        }

        private bool IntersectsBox(Vector3S positionA, Vector3S positionB, SBoxCollider other, out Contact contact)
        {
            contact = new Contact();

            Vector3S aMin = positionA - Size * (sfloat)0.5f;
            Vector3S aMax = positionA + Size * (sfloat)0.5f;
            Vector3S bMin = positionB - other.Size * (sfloat)0.5f;
            Vector3S bMax = positionB + other.Size * (sfloat)0.5f;

            if (aMin.x <= bMax.x && aMax.x >= bMin.x &&
                aMin.y <= bMax.y && aMax.y >= bMin.y &&
                aMin.z <= bMax.z && aMax.z >= bMin.z)
            {
                // Calculate penetration depth and collision normal
                sfloat dx = MathS.Min(aMax.x - bMin.x, bMax.x - aMin.x);
                sfloat dy = MathS.Min(aMax.y - bMin.y, bMax.y - aMin.y);
                sfloat dz = MathS.Min(aMax.z - bMin.z, bMax.z - aMin.z);

                if (dx < dy && dx < dz)
                {
                    contact.penetrationDepth = dx;
                    contact.normal = (positionB.x > positionA.x) ? Vector3S.right : Vector3S.left;
                }
                else if (dy < dx && dy < dz)
                {
                    contact.penetrationDepth = dy;
                    contact.normal = (positionB.y > positionA.y) ? Vector3S.up : Vector3S.down;
                }
                else
                {
                    contact.penetrationDepth = dz;
                    contact.normal = (positionB.z > positionA.z) ? Vector3S.forward : Vector3S.back;
                }

                contact.point = positionA + (contact.normal * (contact.penetrationDepth * (sfloat)0.5f));
                return true;
            }

            return false;
        }

        private bool IntersectsSphere(Vector3S positionA, Vector3S positionB, SSphereCollider sphere, out Contact contact)
        {
            contact = new Contact();

            Vector3S halfSize = Size * (sfloat)0.5f;
            Vector3S boxCenter = positionA;
            Vector3S sphereCenter = positionB;
            sfloat radius = sphere.radius;

            Vector3S closestPoint = new Vector3S(
                MathS.Max(boxCenter.x - halfSize.x, MathS.Min(sphereCenter.x, boxCenter.x + halfSize.x)),
                MathS.Max(boxCenter.y - halfSize.y, MathS.Min(sphereCenter.y, boxCenter.y + halfSize.y)),
                MathS.Max(boxCenter.z - halfSize.z, MathS.Min(sphereCenter.z, boxCenter.z + halfSize.z))
            );

            sfloat distance = Vector3S.Distance(closestPoint, sphereCenter);

            if (distance <= radius)
            {
                contact.point = closestPoint;
                contact.normal = (sphereCenter - closestPoint).Normalize();
                contact.penetrationDepth = radius - distance;
                return true;
            }

            return false;
        }
    }
}

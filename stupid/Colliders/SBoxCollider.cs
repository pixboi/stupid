using SoftFloat;
using stupid.Maths;

namespace stupid.Colliders
{
    public class SBoxCollider : BaseCollider
    {
        public Vector3S Size { get; private set; }
        public SBoxCollider(Vector3S size)
        {
            Size = size;
        }

        public override SBounds CalculateBounds(Vector3S position)
        {
            Vector3S halfSize = Size * (sfloat)0.5f;
            Vector3S[] vertices = new Vector3S[8];

            vertices[0] = position + attachedRigidbody.rotation * new Vector3S(-halfSize.x, -halfSize.y, -halfSize.z);
            vertices[1] = position + attachedRigidbody.rotation * new Vector3S(halfSize.x, -halfSize.y, -halfSize.z);
            vertices[2] = position + attachedRigidbody.rotation * new Vector3S(-halfSize.x, halfSize.y, -halfSize.z);
            vertices[3] = position + attachedRigidbody.rotation * new Vector3S(halfSize.x, halfSize.y, -halfSize.z);
            vertices[4] = position + attachedRigidbody.rotation * new Vector3S(-halfSize.x, -halfSize.y, halfSize.z);
            vertices[5] = position + attachedRigidbody.rotation * new Vector3S(halfSize.x, -halfSize.y, halfSize.z);
            vertices[6] = position + attachedRigidbody.rotation * new Vector3S(-halfSize.x, halfSize.y, halfSize.z);
            vertices[7] = position + attachedRigidbody.rotation * new Vector3S(halfSize.x, halfSize.y, halfSize.z);

            Vector3S min = vertices[0];
            Vector3S max = vertices[0];

            for (int i = 1; i < 8; i++)
            {
                min = Vector3S.Min(min, vertices[i]);
                max = Vector3S.Max(max, vertices[i]);
            }

            _bounds = new SBounds(min, max);
            return _bounds;
        }

        public override SBounds GetBounds() => _bounds;

        public override bool Intersects(Vector3S positionA, Vector3S positionB, ICollider other, out Contact contact)
        {
            contact = new Contact();

            if (other is SBoxCollider otherBox)
            {
                return IntersectsBox(positionA, positionB, otherBox, out contact);
            }
            else if (other is SSphereCollider sphere)
            {
                return IntersectsSphere(positionA, positionB, sphere, out contact);
            }

            return false;
        }

        private bool IntersectsBox(Vector3S positionA, Vector3S positionB, SBoxCollider other, out Contact contact)
        {
            contact = new Contact();

            Vector3S[] axes = GetAxes(this.attachedRigidbody.rotation, other.attachedRigidbody.rotation);

            foreach (var axis in axes)
            {
                if (!OverlapOnAxis(positionA, this.Size * (sfloat)0.5f, positionB, other.Size * (sfloat)0.5f, axis))
                {
                    return false;
                }
            }

            Vector3S direction = positionB - positionA;
            sfloat overlap = sfloat.MaxValue;
            Vector3S smallestAxis = Vector3S.zero;

            foreach (var axis in axes)
            {
                sfloat o = CalculateOverlap(positionA, this.Size * (sfloat)0.5f, positionB, other.Size * (sfloat)0.5f, axis);
                if (o < overlap)
                {
                    overlap = o;
                    smallestAxis = axis;
                }
            }

            contact.normal = smallestAxis;
            contact.penetrationDepth = overlap;
            contact.point = (positionA + positionB) * (sfloat)0.5f;

            return true;
        }

        private Vector3S[] GetAxes(SQuaternion rotationA, SQuaternion rotationB)
        {
            return new Vector3S[]
            {
                rotationA * Vector3S.right,
                rotationA * Vector3S.up,
                rotationA * Vector3S.forward,
                rotationB * Vector3S.right,
                rotationB * Vector3S.up,
                rotationB * Vector3S.forward,
            };
        }

        private bool OverlapOnAxis(Vector3S posA, Vector3S halfSizeA, Vector3S posB, Vector3S halfSizeB, Vector3S axis)
        {
            sfloat aMin, aMax, bMin, bMax;

            ProjectBox(posA, halfSizeA, axis, out aMin, out aMax);
            ProjectBox(posB, halfSizeB, axis, out bMin, out bMax);

            return aMax >= bMin && bMax >= aMin;
        }

        private void ProjectBox(Vector3S pos, Vector3S halfSize, Vector3S axis, out sfloat min, out sfloat max)
        {
            Vector3S[] vertices = new Vector3S[8]
            {
                pos + attachedRigidbody.rotation * new Vector3S(-halfSize.x, -halfSize.y, -halfSize.z),
                pos + attachedRigidbody.rotation * new Vector3S(halfSize.x, -halfSize.y, -halfSize.z),
                pos + attachedRigidbody.rotation * new Vector3S(-halfSize.x, halfSize.y, -halfSize.z),
                pos + attachedRigidbody.rotation * new Vector3S(halfSize.x, halfSize.y, -halfSize.z),
                pos + attachedRigidbody.rotation * new Vector3S(-halfSize.x, -halfSize.y, halfSize.z),
                pos + attachedRigidbody.rotation * new Vector3S(halfSize.x, -halfSize.y, halfSize.z),
                pos + attachedRigidbody.rotation * new Vector3S(-halfSize.x, halfSize.y, halfSize.z),
                pos + attachedRigidbody.rotation * new Vector3S(halfSize.x, halfSize.y, halfSize.z)
            };

            min = max = Vector3S.Dot(vertices[0], axis);
            for (int i = 1; i < 8; i++)
            {
                sfloat projection = Vector3S.Dot(vertices[i], axis);
                if (projection < min)
                    min = projection;
                if (projection > max)
                    max = projection;
            }
        }

        private sfloat CalculateOverlap(Vector3S posA, Vector3S halfSizeA, Vector3S posB, Vector3S halfSizeB, Vector3S axis)
        {
            sfloat aMin, aMax, bMin, bMax;

            ProjectBox(posA, halfSizeA, axis, out aMin, out aMax);
            ProjectBox(posB, halfSizeB, axis, out bMin, out bMax);

            return MathS.Min(aMax - bMin, bMax - aMin);
        }

        private bool IntersectsSphere(Vector3S positionA, Vector3S positionB, SSphereCollider sphere, out Contact contact)
        {
            contact = new Contact();

            Vector3S halfSize = Size * (sfloat)0.5f;
            Vector3S closestPoint = new Vector3S(
                MathS.Max(positionA.x - halfSize.x, MathS.Min(positionB.x, positionA.x + halfSize.x)),
                MathS.Max(positionA.y - halfSize.y, MathS.Min(positionB.y, positionA.y + halfSize.y)),
                MathS.Max(positionA.z - halfSize.z, MathS.Min(positionB.z, positionA.z + halfSize.z))
            );

            Vector3S diff = positionB - closestPoint;
            sfloat distanceSquared = diff.MagnitudeSquared();

            if (distanceSquared <= sphere.Radius * sphere.Radius)
            {
                sfloat distance = libm.sqrtf(distanceSquared);
                contact.point = closestPoint;
                contact.normal = diff.Normalize();
                contact.penetrationDepth = sphere.Radius - distance;
                return true;
            }

            return false;
        }
    }
}

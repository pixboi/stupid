using stupid.Maths;
using System.Diagnostics.Contracts;

namespace stupid.Colliders
{
    public class InsideAABBColliderS : BaseShape
    {
        public Vector3S min { get; private set; }
        public Vector3S max { get; private set; }

        public InsideAABBColliderS(Vector3S min, Vector3S max)
        {
            this.min = min;
            this.max = max;
        }

        public override BoundsS CalculateBounds(Vector3S position)
        {
            _bounds = new BoundsS(min, max);
            return _bounds;
        }

        public override int Intersects(Collidable other, ref ContactS[] cache)
        {
            var bounds = other.GetBounds();
            var position = other.transform.position;
            int count = 0;

            if (bounds.min.x < min.x && count < cache.Length)
            {
                var c = new ContactS
                {
                    point = new Vector3S(min.x, position.y, position.z),
                    normal = Vector3S.right,
                    penetrationDepth = MathS.Abs(min.x - bounds.min.x)
                };
                cache[count++] = c;
            }
            if (bounds.max.x > max.x && count < cache.Length)
            {
                var c = new ContactS
                {
                    point = new Vector3S(max.x, position.y, position.z),
                    normal = Vector3S.left,
                    penetrationDepth = MathS.Abs(bounds.max.x - max.x)
                };
                cache[count++] = c;
            }
            if (bounds.min.y < min.y && count < cache.Length)
            {
                var c = new ContactS
                {
                    point = new Vector3S(position.x, min.y, position.z),
                    normal = Vector3S.up,
                    penetrationDepth = MathS.Abs(min.y - bounds.min.y)
                };
                cache[count++] = c;
            }
            if (bounds.max.y > max.y && count < cache.Length)
            {
                var c = new ContactS
                {
                    point = new Vector3S(position.x, max.y, position.z),
                    normal = Vector3S.down,
                    penetrationDepth = MathS.Abs(bounds.max.y - max.y)
                };
                cache[count++] = c;
            }
            if (bounds.min.z < min.z && count < cache.Length)
            {
                var c = new ContactS
                {
                    point = new Vector3S(position.x, position.y, min.z),
                    normal = Vector3S.forward,
                    penetrationDepth = MathS.Abs(min.z - bounds.min.z)
                };
                cache[count++] = c;
            }
            if (bounds.max.z > max.z && count < cache.Length)
            {
                var c = new ContactS
                {
                    point = new Vector3S(position.x, position.y, max.z),
                    normal = Vector3S.back,
                    penetrationDepth = MathS.Abs(bounds.max.z - max.z)
                };
                cache[count++] = c;
            }

            return count;
        }

        public override bool Intersects(Collidable other, out ContactS contact)
        {
            if (other.collider is SphereColliderS)
            {
                return IntersectsSphere(other, out contact);
            }

            contact = new ContactS();
            var bounds = other.GetBounds();
            var position = other.transform.position;

            int contactCount = 0;
            Vector3S averagePoint = Vector3S.zero;
            Vector3S averageNormal = Vector3S.zero;
            f32 maxPenetration = f32.zero;

            void AddContact(Vector3S point, Vector3S normal, f32 penetrationDepth)
            {
                averagePoint += point;
                averageNormal += normal;
                if (penetrationDepth > maxPenetration)
                {
                    maxPenetration = penetrationDepth;
                }
                contactCount++;
            }

            if (bounds.min.x < min.x)
            {
                AddContact(new Vector3S(min.x, position.y, position.z), Vector3S.right, MathS.Abs(min.x - bounds.min.x));
            }
            if (bounds.max.x > max.x)
            {
                AddContact(new Vector3S(max.x, position.y, position.z), Vector3S.left, MathS.Abs(bounds.max.x - max.x));
            }
            if (bounds.min.y < min.y)
            {
                AddContact(new Vector3S(position.x, min.y, position.z), Vector3S.up, MathS.Abs(min.y - bounds.min.y));
            }
            if (bounds.max.y > max.y)
            {
                AddContact(new Vector3S(position.x, max.y, position.z), Vector3S.down, MathS.Abs(bounds.max.y - max.y));
            }
            if (bounds.min.z < min.z)
            {
                AddContact(new Vector3S(position.x, position.y, min.z), Vector3S.forward, MathS.Abs(min.z - bounds.min.z));
            }
            if (bounds.max.z > max.z)
            {
                AddContact(new Vector3S(position.x, position.y, max.z), Vector3S.back, MathS.Abs(bounds.max.z - max.z));
            }

            if (contactCount == 0)
            {
                return false;
            }

            // Calculate average contact point and normal
            averagePoint /= (f32)contactCount;
            averageNormal = averageNormal.Normalize();

            contact.point = averagePoint;
            contact.normal = averageNormal;
            contact.penetrationDepth = maxPenetration;

            return true;
        }

        public bool IntersectsSphere(Collidable other, out ContactS contact)
        {
            contact = new ContactS();

            if (!(other.collider is SphereColliderS sphereCollider))
            {
                return false;
            }

            var position = other.transform.position;
            f32 radius = sphereCollider.radius;

            int contactCount = 0;
            Vector3S averagePoint = Vector3S.zero;
            Vector3S averageNormal = Vector3S.zero;
            f32 maxPenetration = f32.zero;

            void AddContact(Vector3S point, Vector3S normal, f32 penetrationDepth)
            {
                averagePoint += point;
                averageNormal += normal;
                if (penetrationDepth > maxPenetration)
                {
                    maxPenetration = penetrationDepth;
                }
                contactCount++;
            }

            if (position.x - radius < min.x)
            {
                AddContact(new Vector3S(min.x, position.y, position.z), Vector3S.right, MathS.Abs(min.x - (position.x - radius)));
            }
            if (position.x + radius > max.x)
            {
                AddContact(new Vector3S(max.x, position.y, position.z), Vector3S.left, MathS.Abs((position.x + radius) - max.x));
            }
            if (position.y - radius < min.y)
            {
                AddContact(new Vector3S(position.x, min.y, position.z), Vector3S.up, MathS.Abs(min.y - (position.y - radius)));
            }
            if (position.y + radius > max.y)
            {
                AddContact(new Vector3S(position.x, max.y, position.z), Vector3S.down, MathS.Abs((position.y + radius) - max.y));
            }
            if (position.z - radius < min.z)
            {
                AddContact(new Vector3S(position.x, position.y, min.z), Vector3S.forward, MathS.Abs(min.z - (position.z - radius)));
            }
            if (position.z + radius > max.z)
            {
                AddContact(new Vector3S(position.x, position.y, max.z), Vector3S.back, MathS.Abs((position.z + radius) - max.z));
            }

            if (contactCount == 0)
            {
                return false;
            }

            // Calculate average contact point and normal
            averagePoint /= (f32)contactCount;
            averageNormal = averageNormal.Normalize();

            contact.point = averagePoint;
            contact.normal = averageNormal;
            contact.penetrationDepth = maxPenetration;

            return true;
        }

        public override Matrix3S CalculateInertiaTensor(f32 mass)
        {
            // This collider doesn't have its own inertia tensor since it's used for bounds checking
            return new Matrix3S(
                new Vector3S(f32.zero, f32.zero, f32.zero),
                new Vector3S(f32.zero, f32.zero, f32.zero),
                new Vector3S(f32.zero, f32.zero, f32.zero)
            );
        }
    }
}

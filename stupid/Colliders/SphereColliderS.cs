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

        public override BoundsS CalculateAABB(Vector3S position, QuaternionS rotation)
        {
            var size = new Vector3S(radius, radius, radius);
            _bounds = new BoundsS(position - size, position + size);
            return _bounds;
        }

        public override int Intersects(Collidable other, ref ContactS contact)
        {
            if (other.collider is SphereColliderS otherSphere)
            {
                return CollisionMath.SphereVSphere(
                    this.collidable.transform.position,
                    otherSphere.collidable.transform.position,
                    this.radius,
                    otherSphere.radius,
                    ref contact);
            }

            if (other.collider is BoxColliderS otherBox)
            {
                return CollisionMath.SphereVsBox(this, otherBox, ref contact);
            }

            return 0;
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
    }
}

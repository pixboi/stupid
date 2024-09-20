using stupid;
using stupid.Maths;

namespace stupid.Colliders
{
    public class SphereColliderS : Shape
    {
        public readonly f32 radius;
        public override bool NeedsRotationUpdate => false;

        public override void OnRotationUpdate()
        {
            throw new System.NotImplementedException();
        }

        public SphereColliderS(f32 radius)
        {
            this.radius = radius;
        }

        public override BoundsS GetBounds(TransformS transform)
        {
            var size = new Vector3S(radius, radius, radius);
            return new BoundsS(transform.position - size, transform.position + size);
        }

        public override int Intersects(Collidable other, ref ContactData[] contacts)
        {
            if (other.collider is SphereColliderS otherSphere)
            {
                return CollisionMath.SphereVSphere(this, otherSphere, ref contacts);
            }

            if (other.collider is BoxColliderS otherBox)
            {
                return CollisionMath.SphereVsBox(this, otherBox, ref contacts);
            }

            return 0;
        }

        static readonly f32 sphereInertia = (f32)(2f / 5f);
        public override Matrix3S CalculateInertiaTensor(in f32 mass)
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

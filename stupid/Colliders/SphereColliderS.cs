using stupid.Constraints;
using stupid.Maths;

namespace stupid.Colliders
{
    public struct SphereColliderS : IShape
    {
        public readonly f32 radius;
        public bool NeedsRotationUpdate => false;
        public void OnRotationUpdate()
        {
            throw new System.NotImplementedException();
        }

        public Collidable collidable => _collidable;
        Collidable _collidable;
        public void Attach(in Collidable collidable) => this._collidable = collidable;
        public SphereColliderS(f32 radius)
        {
            this.radius = radius;
            this._collidable = null;
        }

        public BoundsS GetBounds(in TransformS t)
        {
            Vector3S size;
            size.x = radius;
            size.y = radius;
            size.z = radius;

            return new BoundsS(t.position - size, t.position + size);
        }

        public int Intersects(Collidable other, ref ContactData[] contacts)
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
        public Matrix3S CalculateInertiaTensor(in f32 mass)
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

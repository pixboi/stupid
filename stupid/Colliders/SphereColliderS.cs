using stupid.Maths;

namespace stupid.Colliders
{
    public struct SphereColliderS : IShape
    {
        public f32 radius { get; private set; }

        Collidable _collidable;
        public Collidable collidable => _collidable;

        BoundsS _bounds;
        public BoundsS bounds => _bounds;

        public bool NeedsRotationUpdate => false;

        public SphereColliderS(f32 radius)
        {
            this.radius = radius;
            this._bounds = new BoundsS();
            this._collidable = null;
        }

        public void Attach(Collidable body)
        {
            _collidable = body;
        }

        public void OnRotationUpdate()
        {
            return;
        }

        public BoundsS CalculateAABB(in Vector3S position, in QuaternionS rotation)
        {
            var size = new Vector3S(radius, radius, radius);
            _bounds = new BoundsS(position - size, position + size);
            return _bounds;
        }

        public int Intersects(Collidable other, ref ContactS contact)
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
        public Matrix3S CalculateInertiaTensor(f32 mass)
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

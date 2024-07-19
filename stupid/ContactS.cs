using stupid.Maths;

namespace stupid.Colliders
{
    public struct ContactS
    {
        public Collidable a, b;
        public Vector3S point;
        public Vector3S normal;
        public f32 penetrationDepth;

        // Cached impulses for warm starting
        public f32 cachedNormalImpulse;
        public f32 cachedFrictionImpulse;
        public Vector3S cachedImpulse;

        public ContactS(Collidable a, Collidable b, Vector3S point, Vector3S normal, f32 penetrationDepth)
        {
            this.a = a;
            this.b = b;
            this.point = point;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;

            // Initialize cached impulses to zero
            this.cachedNormalImpulse = f32.zero;
            this.cachedFrictionImpulse = f32.zero;
            this.cachedImpulse = Vector3S.zero;
        }

        public void ResetCachedImpulses()
        {
            // Reset the cached impulses to zero
            cachedNormalImpulse = f32.zero;
            cachedFrictionImpulse = f32.zero;
            cachedImpulse = Vector3S.zero;
        }
    }
}

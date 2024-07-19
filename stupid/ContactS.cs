using stupid.Maths;

namespace stupid
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

        public void ResetCachedImpulses()
        {
            // Reset the cached impulses to zero
            cachedNormalImpulse = f32.zero;
            cachedFrictionImpulse = f32.zero;
            cachedImpulse = Vector3S.zero;
        }
    }
}

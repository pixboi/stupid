using stupid.Maths;

namespace stupid
{
    public struct ContactS
    {
        public readonly Vector3S point;
        public readonly Vector3S normal;
        public readonly f32 penetrationDepth;

        // Cached impulses for warm starting
        public Vector3S cachedImpulse;
        public f32 cachedNormalImpulse;
        public f32 cachedFrictionImpulse;

        public ContactS(Vector3S point, Vector3S normal, f32 penetrationDepth)
        {
            this.point = point;
            this.normal = normal;
            this.penetrationDepth = penetrationDepth;
            this.cachedImpulse = Vector3S.zero;
            this.cachedNormalImpulse = f32.zero;
            this.cachedFrictionImpulse = f32.zero;
        }

        public ContactS(ContactS fresh, ContactS old)
        {
            this.point = fresh.point;
            this.normal = fresh.normal;
            this.penetrationDepth = fresh.penetrationDepth;
            this.cachedImpulse = old.cachedImpulse;
            this.cachedNormalImpulse = old.cachedNormalImpulse;
            this.cachedFrictionImpulse = old.cachedFrictionImpulse;
        }
    }
}

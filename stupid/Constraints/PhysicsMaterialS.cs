using stupid.Maths;

namespace stupid.Constraints
{
    public readonly struct PhysicsMaterialS
    {
        public readonly f32 staticFriction;
        public PhysicsMaterialS(f32 staticFriction, f32 dynamicFriction, f32 bounciness)
        {
            this.staticFriction = staticFriction;
        }

        public static readonly PhysicsMaterialS DEFAULT_MATERIAL = new PhysicsMaterialS(f32.one, f32.one, f32.zero);
        public static readonly PhysicsMaterialS DEFAULT_MATERIAL1 = new PhysicsMaterialS((f32)0.6, (f32)0.6, f32.zero);
    }
}

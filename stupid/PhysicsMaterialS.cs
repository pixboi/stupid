using stupid.Maths;

namespace stupid
{
    public readonly struct PhysicsMaterialS
    {
        public readonly f32 staticFriction, dynamicFriction, restitution;

        public PhysicsMaterialS(f32 staticFriction, f32 dynamicFriction, f32 bounciness)
        {
            this.staticFriction = staticFriction;
            this.dynamicFriction = dynamicFriction;
            this.restitution = bounciness;
        }

        public static readonly PhysicsMaterialS DEFAULT_MATERIAL1 = new PhysicsMaterialS(f32.two, f32.two, f32.zero);
        public static readonly PhysicsMaterialS DEFAULT_MATERIAL = new PhysicsMaterialS((f32)0.6, (f32)0.6, f32.zero);
    }
}

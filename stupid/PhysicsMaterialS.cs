using stupid.Maths;

namespace stupid
{
    public struct PhysicsMaterialS
    {
        public f32 staticFriction, dynamicFriction;
        public f32 bounciness;

        public PhysicsMaterialS(f32 staticFriction, f32 dynamicFriction, f32 bounciness)
        {
            this.staticFriction = staticFriction;
            this.dynamicFriction = dynamicFriction;
            this.bounciness = bounciness;
        }

        public static readonly PhysicsMaterialS DEFAULT_MATERIAL = new PhysicsMaterialS((f32)0.6, (f32)0.6, (f32)0.6);
    }
}

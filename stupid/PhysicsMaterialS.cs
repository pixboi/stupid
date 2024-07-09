using stupid.Maths;

namespace stupid
{
    public struct PhysicsMaterialS
    {
        public f32 friction;
        public f32 bounciness;

        public PhysicsMaterialS(f32 friction, f32 bounciness)
        {
            this.friction = friction;
            this.bounciness = bounciness;
        }

        public static readonly PhysicsMaterialS DEFAULT_MATERIAL = new PhysicsMaterialS(f32.half, f32.half);
    }
}

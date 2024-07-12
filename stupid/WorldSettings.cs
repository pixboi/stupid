using stupid.Maths;
using stupid.Collections;
using stupid.Colliders;

namespace stupid
{
    public struct WorldSettings
    {
        public Vector3S Gravity;
        public PhysicsMaterialS DefaultMaterial;
        public f32 BounceThreshold;
        public f32 DefaultMaxDepenetrationVelocity;
        public f32 SleepTreshold;
        public f32 DefaultContactOffset;
        public f32 DefaultSolverIterations;
        public f32 DefaultSolverVelocityIterations;
        public BoundsS WorldBounds;
        public f32 DefaultMaxAngularSpeed;

        public static WorldSettings Default()
        {
            return new WorldSettings
            {
                Gravity = new Vector3S(0, -20, 0),
                DefaultMaterial = PhysicsMaterialS.DEFAULT_MATERIAL,
                BounceThreshold = f32.two,
                DefaultMaxDepenetrationVelocity = (f32)10,
                SleepTreshold = (f32)0.005,
                DefaultContactOffset = (f32)0.01,
                DefaultSolverIterations = f32.one,
                DefaultSolverVelocityIterations = f32.one,
                WorldBounds = new BoundsS(new Vector3S(0, 0, 0), new Vector3S(32, 32, 32)),
                DefaultMaxAngularSpeed = (f32)16,
            };
        }
    }
}

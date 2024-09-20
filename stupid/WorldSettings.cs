﻿using stupid.Colliders;
using stupid.Constraints;
using stupid.Maths;

namespace stupid
{
    public class WorldSettings
    {
        public Vector3S Gravity;
        public PhysicsMaterialS DefaultMaterial;
        public f32 BounceThreshold;
        public f32 DefaultMaxDepenetrationVelocity;
        public f32 SleepTreshold;
        public f32 DefaultContactOffset;
        public int DefaultSolverIterations;
        public int DefaultSolverVelocityIterations;
        public BoundsS WorldBounds;
        public f32 DefaultMaxAngularSpeed;
        public bool Presolve, Relaxation, Warmup;
        public f32 PositionCorrection;
        public f32 Baumgartner;
        public f32 FastMotionThreshold;

        public static WorldSettings Default()
        {
            return new WorldSettings
            {
                Gravity = new Vector3S(0, -10, 0),
                DefaultMaterial = PhysicsMaterialS.DEFAULT_MATERIAL,
                BounceThreshold = f32.two,
                DefaultMaxDepenetrationVelocity = (f32)10,
                SleepTreshold = (f32)0.005,
                DefaultContactOffset = (f32)0.01,
                DefaultSolverIterations = 6,
                DefaultSolverVelocityIterations = 1,
                WorldBounds = new BoundsS(new Vector3S(0, 0, 0), new Vector3S(32, 32, 32)),
                DefaultMaxAngularSpeed = (f32)14,
                Presolve = true,
                Relaxation = true,
                Warmup = true,
                PositionCorrection = (f32)0.2,
                Baumgartner = (f32)0.2,
                FastMotionThreshold = (f32)512,
            };
        }
    }
}

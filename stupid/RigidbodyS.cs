using stupid.Colliders;
using stupid.Maths;
using System;

namespace stupid
{
    public enum ForceModeS
    {
        Force,
        Acceleration,
        Impulse,
        VelocityChange
    }

    public class RigidbodyS : Collidable
    {
        // Integration
        public Vector3S velocity;
        public Vector3S angularVelocity;

        public f32 maxAngularVelocitySqr = (f32)49;

        //Runtime
        public Vector3S forceBucket { get; private set; }
        public Vector3S torqueBucket { get; private set; }
        public Tensor tensor { get; private set; }

        // Settings
        public f32 mass = f32.one;
        public f32 drag = f32.zero;
        public f32 angularDrag = f32.one;
        public f32 inverseMass => f32.one / mass;
        public bool useGravity = true;
        public bool isKinematic = false;

        public RigidbodyS(int index, IShape collider, bool isDynamic = true, TransformS transform = default,
            Vector3S velocity = default,
            Vector3S angularVelocity = default,
            f32 mass = default,
            bool useGravity = true,
            bool isKinematic = false) : base(index, collider, isDynamic, transform)
        {

            this.mass = mass;
            if (this.mass == f32.zero) this.mass = f32.one;

            if (collider != null)
            {
                var inertia = collider.CalculateInertiaTensor(this.mass);
                this.tensor = new Tensor(inertia);
            }

            this.velocity = velocity;
            this.angularVelocity = angularVelocity;

            this.useGravity = useGravity;
            this.isKinematic = isKinematic;
        }

        public void AddForce(Vector3S force, ForceModeS mode = ForceModeS.Force)
        {
            switch (mode)
            {
                case ForceModeS.Force:
                    // Continuous force
                    this.forceBucket += force * World.DeltaTime;
                    break;

                case ForceModeS.Impulse:
                    // Instantaneous force
                    this.velocity += force / this.mass;
                    break;

                case ForceModeS.Acceleration:
                    // Continuous acceleration
                    this.velocity += force * World.DeltaTime;
                    break;

                case ForceModeS.VelocityChange:
                    // Instantaneous change in velocity
                    this.velocity += force;
                    break;
            }
        }

        public void AddTorque(Vector3S force, ForceModeS mode = ForceModeS.Force)
        {
            switch (mode)
            {
                case ForceModeS.Force:
                    // Continuous force
                    this.torqueBucket += force * World.DeltaTime;
                    break;

                case ForceModeS.Impulse:
                    // Instantaneous force
                    this.angularVelocity += force / this.mass;
                    break;

                case ForceModeS.Acceleration:
                    // Continuous acceleration
                    this.angularVelocity += force * World.DeltaTime;
                    break;

                case ForceModeS.VelocityChange:
                    // Instantaneous change in velocity
                    this.angularVelocity += force;
                    break;
            }
        }

        public void ClearBuckets()
        {
            forceBucket = Vector3S.zero;
            torqueBucket = Vector3S.zero;
        }

    }
}

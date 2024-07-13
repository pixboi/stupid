using stupid.Colliders;
using stupid.Maths;
using System;
using System.Runtime;

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

        //Runtime
        public Vector3S forceBucket { get; private set; }
        public Vector3S torqueBucket { get; private set; }
        public Tensor tensor { get; private set; }

        // Settings
        public f32 mass = f32.one;
        public f32 inverseMass { get; private set; }

        public f32 drag = f32.zero;
        public f32 angularDrag = (f32)0.05;
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
            if (this.mass <= f32.zero) this.mass = f32.one;
            inverseMass = f32.one / this.mass;

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

        public void Integrate(f32 deltaTime, WorldSettings settings)
        {
            if (isKinematic) return;

            if (useGravity)
            {
                velocity += settings.Gravity * deltaTime;
            }

            // Update linear velocity with accumulated forces
            if (forceBucket != Vector3S.zero)
            {
                velocity += forceBucket / mass * deltaTime;
            }

            // Update position
            transform.position += velocity * deltaTime;

            // Update angular velocity with accumulated torques
            if (torqueBucket != Vector3S.zero)
            {
                angularVelocity += tensor.inertiaWorld * torqueBucket / mass * deltaTime;
            }

            // Apply drags
            if (drag != f32.zero)
                velocity *= MathS.Clamp(f32.one - drag * deltaTime, f32.zero, f32.one);

            if (angularDrag != f32.zero)
                angularVelocity *= MathS.Clamp(f32.one - angularDrag * deltaTime, f32.zero, f32.one);

            // Clamp the angular velocity
            if (angularVelocity.SqrMagnitude > settings.DefaultMaxAngularSpeed * settings.DefaultMaxAngularSpeed)
            {
                angularVelocity = angularVelocity.ClampMagnitude(f32.zero, settings.DefaultMaxAngularSpeed);
            }

            // Update rotation
            if (angularVelocity.SqrMagnitude > f32.epsilon)
            {
                Vector3S angDelta = angularVelocity * deltaTime * f32.half;
                //var nrmAng = angDelta.NormalizeWithMagnitude(out var mag);
                var dq = new QuaternionS(angDelta.x, angDelta.y, angDelta.z, f32.one);
                //QuaternionS deltaRot = QuaternionS.FromAxisAngle(nrmAng, mag);
                transform.rotation = (dq * transform.rotation).Normalize();
            }

            // Clear accumulated forces and torques
            ClearBuckets();
        }

        public void AddForce(Vector3S force, ForceModeS mode = ForceModeS.Force)
        {
            switch (mode)
            {
                case ForceModeS.Force:
                    // Continuous force
                    this.forceBucket += force;
                    break;

                case ForceModeS.Impulse:
                    // Instantaneous force
                    this.velocity += force / this.mass;
                    break;

                case ForceModeS.Acceleration:
                    // Continuous acceleration
                    this.forceBucket += force * this.mass;
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
                    this.torqueBucket += force;
                    break;

                case ForceModeS.Impulse:
                    // Instantaneous force
                    this.angularVelocity += force / this.mass;
                    break;

                case ForceModeS.Acceleration:
                    // Continuous acceleration
                    this.torqueBucket += force * this.mass;
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

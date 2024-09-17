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

        public Tensor tensor;

        // Settings
        public f32 mass = f32.one;
        public f32 inverseMass;

        public f32 drag = f32.zero;
        public f32 angularDrag = (f32)0.05;
        public bool useGravity = true;
        public bool isKinematic = false;

        public RigidbodyS(int index, IShape collider, bool isDynamic = true, TransformS transform = default,
            Vector3S velocity = default,
            Vector3S angularVelocity = default,
            f32 mass = default,
            bool useGravity = true,
            bool isKinematic = false) : base(index, collider, transform, isDynamic)
        {

            this.mass = mass;
            if (this.mass <= f32.zero) this.mass = f32.one;
            this.inverseMass = f32.one / this.mass;

            if (collider != null)
            {
                var inertia = collider.CalculateInertiaTensor(this.mass);
                this.tensor = new Tensor(inertia, transform);
            }

            this.velocity = velocity;
            this.angularVelocity = angularVelocity;

            this.useGravity = useGravity;
            this.isKinematic = isKinematic;
        }


        public void IntegrateForces(in f32 dt, in WorldSettings settings)
        {
            // Exit early if the object is kinematic, as no integration is needed.
            if (isKinematic) return;

            // Apply gravity to the velocity if gravity is enabled.
            if (useGravity) velocity += settings.Gravity * dt;

            // Apply accumulated forces to the velocity.
            if (forceBucket != Vector3S.zero) velocity += (forceBucket / mass) * dt;

            // Apply linear drag, ensuring it doesn't invert the velocity direction.
            if (drag > f32.zero) velocity *= MathS.Clamp(f32.one - (drag * dt), f32.zero, f32.one);

            // Apply accumulated torques to the angular velocity.
            if (torqueBucket != Vector3S.zero) angularVelocity += tensor.inertiaWorld * (torqueBucket / mass) * dt;

            // Apply angular drag, ensuring it doesn't invert the angular velocity direction.
            if (angularDrag > f32.zero) angularVelocity *= MathS.Clamp(f32.one - angularDrag * dt, f32.zero, f32.one);

            // Clear the accumulated forces and torques after applying them.
            ClearBuckets();
        }

        public void IntegrateVelocity(in f32 dt, in WorldSettings settings)
        {
            // Exit early if the object is kinematic, as no integration is needed.
            if (isKinematic) return;

            var delta = velocity * dt;
            transform.AddDelta(delta);

            // Clamp the angular velocity to avoid excessive rotational speeds.
            if (angularVelocity.Magnitude() > settings.DefaultMaxAngularSpeed)
                angularVelocity = angularVelocity.ClampMagnitude(-settings.DefaultMaxAngularSpeed, settings.DefaultMaxAngularSpeed);


            //This seems to work pretty well, even without the > f32.zero
            var halfAngle = angularVelocity * dt * f32.half;
            var dq = new QuaternionS(halfAngle.x, halfAngle.y, halfAngle.z, f32.one);
            transform.rotation = (dq * transform.rotation).Normalize();

            transform.UpdateRotationMatrix();
            this.tensor.UpdateInertiaTensor(this.transform);
        }

        public void FinalizePosition()
        {
            transform.ActuateDelta();
            //  this.transform.position += this.transform.deltaPosition;
            //  this.transform.deltaPosition.Reset();
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

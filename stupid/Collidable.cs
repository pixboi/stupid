using stupid.Colliders;
using stupid.Constraints;
using stupid.Maths;
using System;
using System.Runtime.CompilerServices;

namespace stupid
{
    public class Collidable : IEquatable<Collidable>
    {
        public readonly IShape collider;       // 64+ bytes (assumed)
        public TransformS transform;           // 64+ bytes (assumed)
        public readonly Tensor tensor;                  // 2 Matrices, big
        public BoundsS bounds;                 // 48 bytes

        // Reorder fields based on size (larger first to smaller)
        public Vector3S velocity;              // 24 bytes
        public Vector3S angularVelocity;       // 24 bytes
        public Vector3S forceBucket;           // 24 bytes
        public Vector3S torqueBucket;          // 24 bytes

        public readonly f32 mass = f32.one;             // 8 bytes
        public readonly f32 inverseMass = f32.one;      // 8 bytes
        public readonly f32 angularDrag = (f32)0.05;    // 8 bytes
        public readonly PhysicsMaterialS material = PhysicsMaterialS.DEFAULT_MATERIAL; // 8 bytes, for now

        public int index;                      // 4 bytes
        public readonly bool isDynamic;                 // 1 byte (group booleans and smaller types)
        public readonly bool useGravity = true;         // 1 byte
        public readonly bool isKinematic = false;       // 1 byte

        public void Register(int index) => this.index = index;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Apply(in RigidbodyData data)
        {
            this.velocity = data.velocity;
            this.angularVelocity = data.angularVelocity;
        }

        public Collidable(int index, IShape collider, TransformS transform, bool isDynamic,
            Vector3S velocity = default,
            Vector3S angularVelocity = default,
            f32 mass = default,
            f32 angularDrag = default,
            bool useGravity = true,
            bool isKinematic = false)
        {
            this.index = index;
            this.collider = collider;
            this.transform = transform;
            this.isDynamic = isDynamic;

            collider?.Attach(this);
            material = PhysicsMaterialS.DEFAULT_MATERIAL;

            this.mass = mass;
            if (this.mass <= f32.zero) this.mass = f32.one;
            this.inverseMass = f32.one / this.mass;

            this.angularDrag = angularDrag;

            //HUge statics like 256x3 can easily get an problematic tensor that cant be inverted
            //HUOM HUOM
            if (collider != null && isDynamic)
            {
                var inertia = collider.CalculateInertiaTensor(this.mass);
                tensor = new Tensor(inertia, transform);
            }

            this.velocity = velocity;
            this.angularVelocity = angularVelocity;
            this.useGravity = useGravity;
            this.isKinematic = isKinematic;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BoundsS CalculateBounds()
        {
            bounds = collider.GetBounds(transform);
            return bounds;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IntegrateForces(in f32 dt, in WorldSettings settings)
        {
            // Exit early if the object is kinematic, as no integration is needed.
            if (isKinematic || !isDynamic) return;

            // Apply gravity to the velocity if gravity is enabled.
            if (useGravity) velocity += settings.Gravity * dt;

            // Apply accumulated forces to the velocity.
            if (forceBucket != Vector3S.zero) velocity += forceBucket / mass * dt;

            // Apply linear drag, ensuring it doesn't invert the velocity direction.
            //if (drag > f32.zero) velocity *= MathS.Clamp(f32.one - drag * dt, f32.zero, f32.one);

            // Apply accumulated torques to the angular velocity.
            if (torqueBucket != Vector3S.zero) angularVelocity += tensor.inertiaWorld * (torqueBucket / mass) * dt;

            // Apply angular drag, ensuring it doesn't invert the angular velocity direction.
            if (angularDrag > f32.zero) angularVelocity *= MathS.Clamp(f32.one - angularDrag * dt, f32.zero, f32.one);

            forceBucket = Vector3S.zero;
            torqueBucket = Vector3S.zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IntegrateVelocity(in f32 dt, in WorldSettings settings)
        {
            if (isKinematic || !isDynamic) return;

            transform.AddDelta(velocity * dt);

            // Clamp the angular velocity to avoid excessive rotational speeds.
            if (angularVelocity.Magnitude() > settings.DefaultMaxAngularSpeed)
                angularVelocity = Vector3S.ClampMagnitude(angularVelocity, -settings.DefaultMaxAngularSpeed, settings.DefaultMaxAngularSpeed);

            //This seems to work pretty well, even without the > f32.zero 
            //In fact, it increased stack stability, wonder why...

            if (angularVelocity.Magnitude() > f32.zero)
            {
                var halfAngle = angularVelocity * dt * f32.half;
                var dq = new QuaternionS(halfAngle.x, halfAngle.y, halfAngle.z, f32.one);
                transform.rotation = (dq * transform.rotation).Normalize();
                /*
                transform.UpdateRotationMatrix();
                tensor.UpdateInertiaTensor(transform);
                */
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void FinalizePosition()
        {
            if (isKinematic || !isDynamic) return;

            transform.FinalizePosition();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddForce(Vector3S force, ForceModeS mode = ForceModeS.Force)
        {
            if (!isDynamic) return;

            switch (mode)
            {
                case ForceModeS.Force:
                    // Continuous force
                    forceBucket += force;
                    break;

                case ForceModeS.Impulse:
                    // Instantaneous force
                    velocity += force / mass;
                    break;

                case ForceModeS.Acceleration:
                    // Continuous acceleration
                    forceBucket += force * mass;
                    break;

                case ForceModeS.VelocityChange:
                    // Instantaneous change in velocity
                    velocity += force;
                    break;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddTorque(Vector3S force, ForceModeS mode = ForceModeS.Force)
        {
            if (!isDynamic) return;

            switch (mode)
            {
                case ForceModeS.Force:
                    // Continuous force
                    torqueBucket += force;
                    break;

                case ForceModeS.Impulse:
                    // Instantaneous force
                    angularVelocity += force / mass;
                    break;

                case ForceModeS.Acceleration:
                    // Continuous acceleration
                    torqueBucket += force * mass;
                    break;

                case ForceModeS.VelocityChange:
                    // Instantaneous change in velocity
                    angularVelocity += force;
                    break;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object? obj)
        {
            return Equals(obj as Collidable);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Collidable? other)
        {
            return !(other is null) &&
                   index == other.index;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            return index;
        }
    }
}

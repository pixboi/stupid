﻿using stupid.Colliders;
using stupid.Constraints;
using stupid.Maths;
using System;
using System.Runtime.CompilerServices;

namespace stupid
{
    public enum ForceModeS
    {
        Force,
        Acceleration,
        Impulse,
        VelocityChange
    }

    public class Collidable : IEquatable<Collidable>
    {
        //Globals, for all
        public int index;
        public void Register(int index) => this.index = index;

        public Shape collider;

        public TransformS transform;

        public PhysicsMaterialS material = PhysicsMaterialS.DEFAULT_MATERIAL;

        public bool isDynamic;

        //Rigidbody data
        public Vector3S velocity;
        public Vector3S angularVelocity;
        public f32 mass = f32.one;
        public f32 inverseMass = f32.one;
        public f32 drag = f32.zero;
        public f32 angularDrag = (f32)0.05;
        public bool useGravity = true;
        public bool isKinematic = false;

        public Vector3S forceBucket;
        public Vector3S torqueBucket;
        public Tensor tensor;

        public Collidable(int index, Shape collider, TransformS transform, bool isDynamic,
            Vector3S velocity = default,
            Vector3S angularVelocity = default,
            f32 mass = default,
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
            inverseMass = f32.one / this.mass;

            if (collider != null)
            {
                var inertia = collider.CalculateInertiaTensor(this.mass);
                tensor = new Tensor(inertia, transform);
            }

            this.velocity = velocity;
            this.angularVelocity = angularVelocity;

            this.useGravity = useGravity;
            this.isKinematic = isKinematic;
        }

        public BoundsS _bounds;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BoundsS CalculateBounds()
        {
            _bounds = collider.GetBounds(transform);
            return _bounds;
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
            if (drag > f32.zero) velocity *= MathS.Clamp(f32.one - drag * dt, f32.zero, f32.one);

            // Apply accumulated torques to the angular velocity.
            if (torqueBucket != Vector3S.zero) angularVelocity += tensor.inertiaWorld * (torqueBucket / mass) * dt;

            // Apply angular drag, ensuring it doesn't invert the angular velocity direction.
            if (angularDrag > f32.zero) angularVelocity *= MathS.Clamp(f32.one - angularDrag * dt, f32.zero, f32.one);

            // Clear the accumulated forces and torques after applying them.
            ClearBuckets();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IntegrateVelocity(in f32 dt, in WorldSettings settings)
        {
            // Exit early if the object is kinematic, as no integration is needed.
            if (isKinematic || !isDynamic) return;

            var delta = velocity * dt;
            transform.AddDelta(delta);

            // Clamp the angular velocity to avoid excessive rotational speeds.
            if (angularVelocity.Magnitude() > settings.DefaultMaxAngularSpeed)
                angularVelocity = Vector3S.ClampMagnitude(angularVelocity, -settings.DefaultMaxAngularSpeed, settings.DefaultMaxAngularSpeed);


            //This seems to work pretty well, even without the > f32.zero 
            //In fact, it increased stack stability, wonder why...
            var halfAngle = angularVelocity * dt * f32.half;
            var dq = new QuaternionS(halfAngle.x, halfAngle.y, halfAngle.z, f32.one);
            transform.rotation = (dq * transform.rotation).Normalize();

            transform.UpdateRotationMatrix();
            tensor.UpdateInertiaTensor(transform);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void FinalizePosition()
        {
            if (!isDynamic) return;

            transform.ActuateDelta();
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
        public void ClearBuckets()
        {
            forceBucket = Vector3S.zero;
            torqueBucket = Vector3S.zero;
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

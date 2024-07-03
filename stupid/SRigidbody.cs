using SoftFloat;
using stupid.Maths;
using stupid.Colliders;

namespace stupid
{
    public class SRigidbody
    {
        // Runtime
        public readonly int index;
        public override int GetHashCode() => index;

        public Vector3S position;
        public Vector3S velocity;
        public Vector3S angularVelocity; // New property for angular velocity
        public SQuaternion rotation;  // New property for orientation

        public ICollider collider;

        // Settings
        public sfloat mass = sfloat.one;
        public sfloat inertia = sfloat.one; // New property for inertia

        public bool useGravity = true;
        public bool isKinematic = false;

        public readonly sfloat sleepThreshold = (sfloat)0.001f;
        public bool isSleeping { get; private set; }

        public void WakeUp() => isSleeping = false;

        public void Sleep()
        {
            isSleeping = true;
            this.velocity = Vector3S.zero;
            this.angularVelocity = Vector3S.zero; // Stop rotation when sleeping
        }

        public SRigidbody(int index)
        {
            this.index = index;
            this.rotation = SQuaternion.Identity;
        }

        public SRigidbody(int index, Vector3S position = default, Vector3S velocity = default, Vector3S angularVelocity = default)
        {
            this.index = index;
            this.position = position;
            this.velocity = velocity;
            this.angularVelocity = angularVelocity; // Initialize angular velocity
            this.rotation = SQuaternion.Identity; // Initialize orientation
        }

        public void Attach(ICollider collider)
        {
            this.collider = collider;
            collider.Attach(this);
        }

        public void ApplyTorque(Vector3S torque)
        {
            if (!isKinematic)
            {
                angularVelocity += torque / inertia;
            }
        }
    }
}

using SoftFloat;
using stupid.Colliders;
using stupid.Maths;

namespace stupid
{
    public class SRigidbody
    {
        // Runtime
        public readonly int index;
        public override int GetHashCode() => index;

        // Transform
        public Vector3S position;
        public SQuaternion rotation;
        public Matrix3S inertiaTensor;

        public Vector3S velocity;
        public Vector3S angularVelocity;
        public ICollider collider;

        // Settings
        public sfloat mass = sfloat.one;
        public bool useGravity = true;
        public bool isKinematic = false;

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
            this.angularVelocity = angularVelocity;
            this.rotation = SQuaternion.Identity;
        }

        public void Attach(ICollider collider)
        {
            this.collider = collider;
            collider.Attach(this);
            if (collider is SSphereCollider sphereCollider)
            {
                CalculateInertiaTensor(sphereCollider.Radius);
            }
        }

        private void CalculateInertiaTensor(sfloat radius)
        {
            // For a solid sphere: I = 2/5 * m * r^2
            sfloat inertia = (sfloat)(0.4f) * mass * radius * radius;
            inertiaTensor = new Matrix3S(
                new Vector3S(inertia, sfloat.zero, sfloat.zero),
                new Vector3S(sfloat.zero, inertia, sfloat.zero),
                new Vector3S(sfloat.zero, sfloat.zero, inertia)
            );
        }

        public Matrix3S GetInverseInertiaTensorWorld()
        {
            // Compute the world space inertia tensor
            Matrix3S rotationMatrix = Matrix3S.Rotate(rotation);
            Matrix3S inverseInertiaTensorLocal = inertiaTensor.Inverse();
            return rotationMatrix * inverseInertiaTensorLocal * rotationMatrix.Transpose();
        }
    }
}

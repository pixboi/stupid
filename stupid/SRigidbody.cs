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
        public Matrix3S inertiaTensorInverse;

        public Vector3S velocity;
        public Vector3S angularVelocity;
        public ICollider collider;

        // Settings
        public f32 mass = f32.one;
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

        private void CalculateInertiaTensor(f32 radius)
        {
            // For a solid sphere: I = 2/5 * m * r^2
            f32 inertia = (f32)(0.4f) * mass * radius * radius;
            inertiaTensor = new Matrix3S(
                new Vector3S(inertia, f32.zero, f32.zero),
                new Vector3S(f32.zero, inertia, f32.zero),
                new Vector3S(f32.zero, f32.zero, inertia)
            );
        }

        public Matrix3S GetInverseInertiaTensorWorld()
        {
            // Compute the world space inertia tensor
            Matrix3S rotationMatrix = Matrix3S.Rotate(rotation);
            Matrix3S inverseInertiaTensorLocal = inertiaTensor.Inverse();
            return rotationMatrix * inverseInertiaTensorLocal * rotationMatrix.Transpose();
        }

        public void CalculateInverseInertiaTensor()
        {
            inertiaTensorInverse = GetInverseInertiaTensorWorld();
        }
    }
}

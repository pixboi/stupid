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

        //Tensors
        public Matrix3S inertiaTensor;
        public Matrix3S inertiaTensorInverse;
        public Matrix3S inverseInertiaTensorLocal; // Precalculated local inverse inertia tensor

        //Integration
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

            switch (collider)
            {
                case SSphereCollider sphereCollider:
                    CalculateSphereInertiaTensor(sphereCollider.Radius);
                    break;
            }
        }

        private void CalculateSphereInertiaTensor(f32 radius)
        {
            // For a solid sphere: I = 2/5 * m * r^2
            f32 inertia = f32.FromRaw(2) / f32.FromRaw(5) * mass * radius * radius;
            SetInertiaTensor(new Matrix3S(
                new Vector3S(inertia, f32.zero, f32.zero),
                new Vector3S(f32.zero, inertia, f32.zero),
                new Vector3S(f32.zero, f32.zero, inertia)
            ));
        }

        private void SetInertiaTensor(Matrix3S inertiaTensor)
        {
            this.inertiaTensor = inertiaTensor;
            this.inverseInertiaTensorLocal = inertiaTensor.Inverse();
        }

        public Matrix3S GetInverseInertiaTensorWorld()
        {
            // Compute the world space inertia tensor using cached and precomputed values
            Matrix3S rotationMatrix = Matrix3S.Rotate(rotation);

            // Perform in-place combined matrix multiplication
            return MultiplyTransposed(rotationMatrix, inverseInertiaTensorLocal);
        }

        private Matrix3S MultiplyTransposed(Matrix3S rotationMatrix, Matrix3S inertiaTensorLocal)
        {
            // Optimize combined multiplication without explicitly calculating the transpose
            f32 r11 = rotationMatrix.M11, r12 = rotationMatrix.M12, r13 = rotationMatrix.M13;
            f32 r21 = rotationMatrix.M21, r22 = rotationMatrix.M22, r23 = rotationMatrix.M23;
            f32 r31 = rotationMatrix.M31, r32 = rotationMatrix.M32, r33 = rotationMatrix.M33;

            f32 i11 = inertiaTensorLocal.M11, i12 = inertiaTensorLocal.M12, i13 = inertiaTensorLocal.M13;
            f32 i21 = inertiaTensorLocal.M21, i22 = inertiaTensorLocal.M22, i23 = inertiaTensorLocal.M23;
            f32 i31 = inertiaTensorLocal.M31, i32 = inertiaTensorLocal.M32, i33 = inertiaTensorLocal.M33;

            f32 m11 = r11 * i11 + r12 * i21 + r13 * i31;
            f32 m12 = r11 * i12 + r12 * i22 + r13 * i32;
            f32 m13 = r11 * i13 + r12 * i23 + r13 * i33;

            f32 m21 = r21 * i11 + r22 * i21 + r23 * i31;
            f32 m22 = r21 * i12 + r22 * i22 + r23 * i32;
            f32 m23 = r21 * i13 + r22 * i23 + r23 * i33;

            f32 m31 = r31 * i11 + r32 * i21 + r33 * i31;
            f32 m32 = r31 * i12 + r32 * i22 + r33 * i32;
            f32 m33 = r31 * i13 + r32 * i23 + r33 * i33;

            return new Matrix3S(
                new Vector3S(m11, m21, m31),
                new Vector3S(m12, m22, m32),
                new Vector3S(m13, m23, m33)
            );
        }

        public void CalculateInverseInertiaTensor()
        {
            this.inertiaTensorInverse = GetInverseInertiaTensorWorld();
        }
    }
}

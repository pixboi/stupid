namespace stupid.Maths
{
    public struct Tensor
    {
        // Tensors
        public Matrix3S inertia;
        public Matrix3S inertiaWorld;
        public Matrix3S inertiaInverseLocal; // Precalculated local inverse inertia tensor

        public void CalculateInverseInertiaTensor(QuaternionS rotation)
        {
            // Compute the world space inertia tensor using cached and precomputed values
            Matrix3S rotationMatrix = Matrix3S.Rotate(rotation);

            // Perform in-place combined matrix multiplication
            inertiaWorld = MultiplyTransposed(rotationMatrix, inertiaInverseLocal);
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
                new Vector3S(m11, m12, m13),
                new Vector3S(m21, m22, m23),
                new Vector3S(m31, m32, m33)
            );
        }
    }
}

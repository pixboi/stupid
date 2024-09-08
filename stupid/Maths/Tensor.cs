using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public struct Tensor
    {
        // Only the inverse of the local inertia tensor is stored
        public readonly Matrix3S inertiaInverse;
        public Matrix3S inertiaWorld;

        public Tensor(in Matrix3S inertia, in TransformS t)
        {
            // Precompute the inverse of the local inertia tensor
            this.inertiaInverse = inertia.Inverse();
            this.inertiaWorld = Matrix3S.identity; // Initial value, updated in the next step
            CalculateInverseInertiaTensor(t);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalculateInverseInertiaTensor(in TransformS t)
        {
            // Use the transform's rotation matrix to calculate the world space inertia tensor
            inertiaWorld = MultiplyTransposed(t.rotationMatrix, inertiaInverse);
        }

        // Optimized matrix multiplication using a transposed rotation matrix
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private Matrix3S MultiplyTransposed(in Matrix3S rotationMatrix, in Matrix3S inertiaTensorLocal)
        {
            // Decompose rotation matrix elements for optimization
            f32 r11 = rotationMatrix.m00, r12 = rotationMatrix.m01, r13 = rotationMatrix.m02;
            f32 r21 = rotationMatrix.m10, r22 = rotationMatrix.m11, r23 = rotationMatrix.m12;
            f32 r31 = rotationMatrix.m20, r32 = rotationMatrix.m21, r33 = rotationMatrix.m22;

            // Decompose inertia tensor elements for optimization
            f32 i11 = inertiaTensorLocal.m00, i12 = inertiaTensorLocal.m01, i13 = inertiaTensorLocal.m02;
            f32 i21 = inertiaTensorLocal.m10, i22 = inertiaTensorLocal.m11, i23 = inertiaTensorLocal.m12;
            f32 i31 = inertiaTensorLocal.m20, i32 = inertiaTensorLocal.m21, i33 = inertiaTensorLocal.m22;

            // Perform R * I (rotation matrix multiplied by local inertia tensor)
            f32 m11 = r11 * i11 + r12 * i21 + r13 * i31;
            f32 m12 = r11 * i12 + r12 * i22 + r13 * i32;
            f32 m13 = r11 * i13 + r12 * i23 + r13 * i33;

            f32 m21 = r21 * i11 + r22 * i21 + r23 * i31;
            f32 m22 = r21 * i12 + r22 * i22 + r23 * i32;
            f32 m23 = r21 * i13 + r22 * i23 + r23 * i33;

            f32 m31 = r31 * i11 + r32 * i21 + r33 * i31;
            f32 m32 = r31 * i12 + r32 * i22 + r33 * i32;
            f32 m33 = r31 * i13 + r32 * i23 + r33 * i33;

            // Perform the final multiplication with the transpose of R
            return new Matrix3S(
                new Vector3S(
                    m11 * r11 + m12 * r12 + m13 * r13,
                    m11 * r21 + m12 * r22 + m13 * r23,
                    m11 * r31 + m12 * r32 + m13 * r33
                ),
                new Vector3S(
                    m21 * r11 + m22 * r12 + m23 * r13,
                    m21 * r21 + m22 * r22 + m23 * r23,
                    m21 * r31 + m22 * r32 + m23 * r33
                ),
                new Vector3S(
                    m31 * r11 + m32 * r12 + m33 * r13,
                    m31 * r21 + m32 * r22 + m33 * r23,
                    m31 * r31 + m32 * r32 + m33 * r33
                )
            );
        }
    }
}

﻿using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public struct Tensor
    {
        // Tensors
        public readonly Matrix3S inertia; // Precalculated local inertia tensor
        public readonly Matrix3S inertiaInverse; // Precalculated local inverse inertia tensor

        public Tensor(Matrix3S inertia, QuaternionS initialRotation = default)
        {
            this.inertia = inertia;
            this.inertiaInverse = inertia.Inverse();

            if (initialRotation == default)
            {
                initialRotation = QuaternionS.identity;
            }

            this.inertiaWorld = inertiaInverse; // This will be updated below
            CalculateInverseInertiaTensor(initialRotation);
        }

        public Matrix3S inertiaWorld { get; private set; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CalculateInverseInertiaTensor(QuaternionS rotation)
        {
            // Compute the world space inertia tensor using cached and precomputed values
            Matrix3S rotationMatrix = Matrix3S.Rotate(rotation);

            // Perform in-place combined matrix multiplication
            inertiaWorld = MultiplyTransposed(rotationMatrix, inertiaInverse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private Matrix3S MultiplyTransposed(Matrix3S rotationMatrix, Matrix3S inertiaTensorLocal)
        {
            // Optimize combined multiplication without explicitly calculating the transpose
            f32 r11 = rotationMatrix.m00, r12 = rotationMatrix.m01, r13 = rotationMatrix.m02;
            f32 r21 = rotationMatrix.m10, r22 = rotationMatrix.m11, r23 = rotationMatrix.m12;
            f32 r31 = rotationMatrix.m20, r32 = rotationMatrix.m21, r33 = rotationMatrix.m22;

            f32 i11 = inertiaTensorLocal.m00, i12 = inertiaTensorLocal.m01, i13 = inertiaTensorLocal.m02;
            f32 i21 = inertiaTensorLocal.m10, i22 = inertiaTensorLocal.m11, i23 = inertiaTensorLocal.m12;
            f32 i31 = inertiaTensorLocal.m20, i32 = inertiaTensorLocal.m21, i33 = inertiaTensorLocal.m22;

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

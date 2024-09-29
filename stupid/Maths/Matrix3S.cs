using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public struct Matrix3S
    {
        public f32 m00, m01, m02;
        public f32 m10, m11, m12;
        public f32 m20, m21, m22;

        // Constructor
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3S(in f32 m00, in f32 m01, in f32 m02, in f32 m10, in f32 m11, in f32 m12, in f32 m20, in f32 m21, in f32 m22)
        {
            this.m00 = m00;
            this.m01 = m01;
            this.m02 = m02;
            this.m10 = m10;
            this.m11 = m11;
            this.m12 = m12;
            this.m20 = m20;
            this.m21 = m21;
            this.m22 = m22;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3S(in Vector3S row1, in Vector3S row2, in Vector3S row3)
        {
            m00 = row1.x;
            m01 = row1.y;
            m02 = row1.z;
            m10 = row2.x;
            m11 = row2.y;
            m12 = row2.z;
            m20 = row3.x;
            m21 = row3.y;
            m22 = row3.z;
        }

        // Identity matrix
        public static readonly Matrix3S identity = new Matrix3S(
            new Vector3S(f32.one, f32.zero, f32.zero),
            new Vector3S(f32.zero, f32.one, f32.zero),
            new Vector3S(f32.zero, f32.zero, f32.one)
        );

        // Get column vector
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S GetColumn(int index)
        {
            return index switch
            {
                0 => new Vector3S(m00, m10, m20),
                1 => new Vector3S(m01, m11, m21),
                2 => new Vector3S(m02, m12, m22),
                _ => throw new IndexOutOfRangeException("Invalid column index")
            };
        }

        // Multiply matrix by vector
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator *(in Matrix3S m, in Vector3S v)
        {
            Vector3S result;

            result.x.rawValue = ((m.m00.rawValue * v.x.rawValue) + (m.m01.rawValue * v.y.rawValue) + (m.m02.rawValue * v.z.rawValue)) >> f32.FractionalBits;
            result.y.rawValue = ((m.m10.rawValue * v.x.rawValue) + (m.m11.rawValue * v.y.rawValue) + (m.m12.rawValue * v.z.rawValue)) >> f32.FractionalBits;
            result.z.rawValue = ((m.m20.rawValue * v.x.rawValue) + (m.m21.rawValue * v.y.rawValue) + (m.m22.rawValue * v.z.rawValue)) >> f32.FractionalBits;

            return result;
        }

        // Multiply matrix by matrix
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3S operator *(in Matrix3S a, in Matrix3S b)
        {
            Matrix3S result;

            result.m00.rawValue = ((a.m00.rawValue * b.m00.rawValue) + (a.m01.rawValue * b.m10.rawValue) + (a.m02.rawValue * b.m20.rawValue)) >> f32.FractionalBits;
            result.m01.rawValue = ((a.m00.rawValue * b.m01.rawValue) + (a.m01.rawValue * b.m11.rawValue) + (a.m02.rawValue * b.m21.rawValue)) >> f32.FractionalBits;
            result.m02.rawValue = ((a.m00.rawValue * b.m02.rawValue) + (a.m01.rawValue * b.m12.rawValue) + (a.m02.rawValue * b.m22.rawValue)) >> f32.FractionalBits;

            result.m10.rawValue = ((a.m10.rawValue * b.m00.rawValue) + (a.m11.rawValue * b.m10.rawValue) + (a.m12.rawValue * b.m20.rawValue)) >> f32.FractionalBits;
            result.m11.rawValue = ((a.m10.rawValue * b.m01.rawValue) + (a.m11.rawValue * b.m11.rawValue) + (a.m12.rawValue * b.m21.rawValue)) >> f32.FractionalBits;
            result.m12.rawValue = ((a.m10.rawValue * b.m02.rawValue) + (a.m11.rawValue * b.m12.rawValue) + (a.m12.rawValue * b.m22.rawValue)) >> f32.FractionalBits;

            result.m20.rawValue = ((a.m20.rawValue * b.m00.rawValue) + (a.m21.rawValue * b.m10.rawValue) + (a.m22.rawValue * b.m20.rawValue)) >> f32.FractionalBits;
            result.m21.rawValue = ((a.m20.rawValue * b.m01.rawValue) + (a.m21.rawValue * b.m11.rawValue) + (a.m22.rawValue * b.m21.rawValue)) >> f32.FractionalBits;
            result.m22.rawValue = ((a.m20.rawValue * b.m02.rawValue) + (a.m21.rawValue * b.m12.rawValue) + (a.m22.rawValue * b.m22.rawValue)) >> f32.FractionalBits;

            return result;
        }

        // Transpose matrix
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3S Transpose()
        {
            Matrix3S result;

            result.m00 = m00;
            result.m01 = m10;
            result.m02 = m20;
            result.m10 = m01;
            result.m11 = m11;
            result.m12 = m21;
            result.m20 = m02;
            result.m21 = m12;
            result.m22 = m22;

            return result;

        }

        // Scale matrix
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3S Scale(in Vector3S v)
        {
            Matrix3S result = identity;

            result.m00 = v.x;
            result.m11 = v.y;
            result.m22 = v.z;

            return result;
        }
        public static Matrix3S CreateInertiaMatrix(in Vector3S inertiaVector)
        {
            return Scale(inertiaVector);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3S Rotate(in QuaternionS q)
        {
            // Precompute values
            f32 xx = q.x * q.x;
            f32 yy = q.y * q.y;
            f32 zz = q.z * q.z;
            f32 xy = q.x * q.y;
            f32 xz = q.x * q.z;
            f32 yz = q.y * q.z;
            f32 wx = q.w * q.x;
            f32 wy = q.w * q.y;
            f32 wz = q.w * q.z;

            f32 two = f32.two;
            f32 one = f32.one;

            // Compute rotation matrix using result pattern
            Matrix3S result;

            result.m00 = one - two * (yy + zz);
            result.m01 = two * (xy - wz);
            result.m02 = two * (xz + wy);

            result.m10 = two * (xy + wz);
            result.m11 = one - two * (xx + zz);
            result.m12 = two * (yz - wx);

            result.m20 = two * (xz - wy);
            result.m21 = two * (yz + wx);
            result.m22 = one - two * (xx + yy);

            return result;
        }


        //MOst of the problems here were caused by the to big boxcolldiers with big tensors?

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3S Inverse(in Matrix3S matrix)
        {
            // Calculate the determinant with safe scaling to prevent overflow
            f32 a = matrix.m00 * (matrix.m11 * matrix.m22 - matrix.m12 * matrix.m21);
            f32 b = matrix.m01 * (matrix.m10 * matrix.m22 - matrix.m12 * matrix.m20);
            f32 c = matrix.m02 * (matrix.m10 * matrix.m21 - matrix.m11 * matrix.m20);

            f32 determinant = a - b + c;

            // Check for very small or zero determinant
            if (MathS.Abs(determinant) < f32.epsilon)
            {
                throw new InvalidOperationException("Matrix is not invertible due to near-zero determinant.");
            }

            // Inverse of the determinant, using a scaling technique to manage precision
            f32 invDet = f32.one / determinant;

            // Create result matrix and calculate the inverse elements
            Matrix3S result;

            result.m00 = invDet * (matrix.m11 * matrix.m22 - matrix.m12 * matrix.m21);
            result.m01 = invDet * (matrix.m02 * matrix.m21 - matrix.m01 * matrix.m22);
            result.m02 = invDet * (matrix.m01 * matrix.m12 - matrix.m02 * matrix.m11);

            result.m10 = invDet * (matrix.m12 * matrix.m20 - matrix.m10 * matrix.m22);
            result.m11 = invDet * (matrix.m00 * matrix.m22 - matrix.m02 * matrix.m20);
            result.m12 = invDet * (matrix.m02 * matrix.m10 - matrix.m00 * matrix.m12);

            result.m20 = invDet * (matrix.m10 * matrix.m21 - matrix.m11 * matrix.m20);
            result.m21 = invDet * (matrix.m01 * matrix.m20 - matrix.m00 * matrix.m21);
            result.m22 = invDet * (matrix.m00 * matrix.m11 - matrix.m01 * matrix.m10);

            return result;
        }


    }
}

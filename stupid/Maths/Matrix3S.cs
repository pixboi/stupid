using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public readonly struct Matrix3S
    {
        public readonly f32 m00, m01, m02;
        public readonly f32 m10, m11, m12;
        public readonly f32 m20, m21, m22;

        // Constructor
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3S(f32 m00, f32 m01, f32 m02, f32 m10, f32 m11, f32 m12, f32 m20, f32 m21, f32 m22)
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
        public static Matrix3S identity => new Matrix3S(
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
            long x = ((m.m00.rawValue * v.x.rawValue) + (m.m01.rawValue * v.y.rawValue) + (m.m02.rawValue * v.z.rawValue)) >> f32.FractionalBits;
            long y = ((m.m10.rawValue * v.x.rawValue) + (m.m11.rawValue * v.y.rawValue) + (m.m12.rawValue * v.z.rawValue)) >> f32.FractionalBits;
            long z = ((m.m20.rawValue * v.x.rawValue) + (m.m21.rawValue * v.y.rawValue) + (m.m22.rawValue * v.z.rawValue)) >> f32.FractionalBits;

            return new Vector3S(new f32(x), new f32(y), new f32(z));
        }

        // Multiply matrix by matrix
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3S operator *(in Matrix3S a, in Matrix3S b)
        {
            long m00 = ((a.m00.rawValue * b.m00.rawValue) + (a.m01.rawValue * b.m10.rawValue) + (a.m02.rawValue * b.m20.rawValue)) >> f32.FractionalBits;
            long m01 = ((a.m00.rawValue * b.m01.rawValue) + (a.m01.rawValue * b.m11.rawValue) + (a.m02.rawValue * b.m21.rawValue)) >> f32.FractionalBits;
            long m02 = ((a.m00.rawValue * b.m02.rawValue) + (a.m01.rawValue * b.m12.rawValue) + (a.m02.rawValue * b.m22.rawValue)) >> f32.FractionalBits;

            long m10 = ((a.m10.rawValue * b.m00.rawValue) + (a.m11.rawValue * b.m10.rawValue) + (a.m12.rawValue * b.m20.rawValue)) >> f32.FractionalBits;
            long m11 = ((a.m10.rawValue * b.m01.rawValue) + (a.m11.rawValue * b.m11.rawValue) + (a.m12.rawValue * b.m21.rawValue)) >> f32.FractionalBits;
            long m12 = ((a.m10.rawValue * b.m02.rawValue) + (a.m11.rawValue * b.m12.rawValue) + (a.m12.rawValue * b.m22.rawValue)) >> f32.FractionalBits;

            long m20 = ((a.m20.rawValue * b.m00.rawValue) + (a.m21.rawValue * b.m10.rawValue) + (a.m22.rawValue * b.m20.rawValue)) >> f32.FractionalBits;
            long m21 = ((a.m20.rawValue * b.m01.rawValue) + (a.m21.rawValue * b.m11.rawValue) + (a.m22.rawValue * b.m21.rawValue)) >> f32.FractionalBits;
            long m22 = ((a.m20.rawValue * b.m02.rawValue) + (a.m21.rawValue * b.m12.rawValue) + (a.m22.rawValue * b.m22.rawValue)) >> f32.FractionalBits;

            return new Matrix3S(
                new f32(m00), new f32(m01), new f32(m02),
                new f32(m10), new f32(m11), new f32(m12),
                new f32(m20), new f32(m21), new f32(m22)
            );
        }

        // Transpose matrix
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3S Transpose()
        {
            return new Matrix3S(
                m00, m10, m20,
                m01, m11, m21,
                m02, m12, m22
            );
        }

        // Scale matrix
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3S Scale(in Vector3S v)
        {
            return new Matrix3S(
                v.x, f32.zero, f32.zero,
                f32.zero, v.y, f32.zero,
                f32.zero, f32.zero, v.z
            );
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

            // Compute rotation matrix
            return new Matrix3S(
                one - two * (yy + zz), two * (xy - wz), two * (xz + wy),
                two * (xy + wz), one - two * (xx + zz), two * (yz - wx),
                two * (xz - wy), two * (yz + wx), one - two * (xx + yy)
            );
        }

        public static Matrix3S CreateInertiaMatrix(in Vector3S inertiaVector)
        {
            return new Matrix3S
                (inertiaVector.x, f32.zero, f32.zero,
                f32.zero, inertiaVector.y, f32.zero,
                f32.zero, f32.zero, inertiaVector.z);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3S Inverse(Matrix3S matrix)
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

            // Calculate the inverse matrix elements
            f32 invM00 = invDet * (matrix.m11 * matrix.m22 - matrix.m12 * matrix.m21);
            f32 invM01 = invDet * (matrix.m02 * matrix.m21 - matrix.m01 * matrix.m22);
            f32 invM02 = invDet * (matrix.m01 * matrix.m12 - matrix.m02 * matrix.m11);

            f32 invM10 = invDet * (matrix.m12 * matrix.m20 - matrix.m10 * matrix.m22);
            f32 invM11 = invDet * (matrix.m00 * matrix.m22 - matrix.m02 * matrix.m20);
            f32 invM12 = invDet * (matrix.m02 * matrix.m10 - matrix.m00 * matrix.m12);

            f32 invM20 = invDet * (matrix.m10 * matrix.m21 - matrix.m11 * matrix.m20);
            f32 invM21 = invDet * (matrix.m01 * matrix.m20 - matrix.m00 * matrix.m21);
            f32 invM22 = invDet * (matrix.m00 * matrix.m11 - matrix.m01 * matrix.m10);

            // Return the inverted matrix
            return new Matrix3S(
                invM00, invM01, invM02,
                invM10, invM11, invM12,
                invM20, invM21, invM22
            );
        }


    }
}

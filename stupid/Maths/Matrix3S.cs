using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public readonly struct Matrix3S
    {
        public readonly f32 m00, m01, m02;
        public readonly f32 m10, m11, m12;
        public readonly f32 m20, m21, m22;

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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S GetColumn(int index)
        {
            return index switch
            {
                0 => new Vector3S(m00, m10, m20),
                1 => new Vector3S(m01, m11, m21),
                2 => new Vector3S(m02, m12, m22),
                _ => throw new IndexOutOfRangeException("Invalid column index"),
            };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator *(in Matrix3S m, in Vector3S v)
        {
            long x = ((m.m00._value * v.x._value) + (m.m01._value * v.y._value) + (m.m02._value * v.z._value)) >> f32.FractionalBits;
            long y = ((m.m10._value * v.x._value) + (m.m11._value * v.y._value) + (m.m12._value * v.z._value)) >> f32.FractionalBits;
            long z = ((m.m20._value * v.x._value) + (m.m21._value * v.y._value) + (m.m22._value * v.z._value)) >> f32.FractionalBits;

            return new Vector3S(new f32(x), new f32(y), new f32(z));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3S operator *(in Matrix3S a, in Matrix3S b)
        {
            long m00 = ((a.m00._value * b.m00._value) + (a.m01._value * b.m10._value) + (a.m02._value * b.m20._value)) >> f32.FractionalBits;
            long m01 = ((a.m00._value * b.m01._value) + (a.m01._value * b.m11._value) + (a.m02._value * b.m21._value)) >> f32.FractionalBits;
            long m02 = ((a.m00._value * b.m02._value) + (a.m01._value * b.m12._value) + (a.m02._value * b.m22._value)) >> f32.FractionalBits;

            long m10 = ((a.m10._value * b.m00._value) + (a.m11._value * b.m10._value) + (a.m12._value * b.m20._value)) >> f32.FractionalBits;
            long m11 = ((a.m10._value * b.m01._value) + (a.m11._value * b.m11._value) + (a.m12._value * b.m21._value)) >> f32.FractionalBits;
            long m12 = ((a.m10._value * b.m02._value) + (a.m11._value * b.m12._value) + (a.m12._value * b.m22._value)) >> f32.FractionalBits;

            long m20 = ((a.m20._value * b.m00._value) + (a.m21._value * b.m10._value) + (a.m22._value * b.m20._value)) >> f32.FractionalBits;
            long m21 = ((a.m20._value * b.m01._value) + (a.m21._value * b.m11._value) + (a.m22._value * b.m21._value)) >> f32.FractionalBits;
            long m22 = ((a.m20._value * b.m02._value) + (a.m21._value * b.m12._value) + (a.m22._value * b.m22._value)) >> f32.FractionalBits;

            return new Matrix3S(
                new f32(m00), new f32(m01), new f32(m02),
                new f32(m10), new f32(m11), new f32(m12),
                new f32(m20), new f32(m21), new f32(m22)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3S Transpose()
        {
            return new Matrix3S(
                m00, m10, m20,
                m01, m11, m21,
                m02, m12, m22
            );
        }

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
            f32 xx = q.x * q.x;
            f32 yy = q.y * q.y;
            f32 zz = q.z * q.z;
            f32 xy = q.x * q.y;
            f32 xz = q.x * q.z;
            f32 yz = q.y * q.z;
            f32 wx = q.w * q.x;
            f32 wy = q.w * q.y;
            f32 wz = q.w * q.z;

            return new Matrix3S(
                f32.one - f32.two * (yy + zz), f32.two * (xy - wz), f32.two * (xz + wy),
                f32.two * (xy + wz), f32.one - f32.two * (xx + zz), f32.two * (yz - wx),
                f32.two * (xz - wy), f32.two * (yz + wx), f32.one - f32.two * (xx + yy)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3S Inverse()
        {
            f32 determinant =
                m00 * (m11 * m22 - m12 * m21) -
                m01 * (m10 * m22 - m12 * m20) +
                m02 * (m10 * m21 - m11 * m20);

            if (MathS.Abs(determinant) < f32.epsilon)
            {
                throw new InvalidOperationException("Matrix is not invertible.");
            }

            f32 invDet = f32.one / determinant;

            var invM00 = invDet * (m11 * m22 - m12 * m21);
            var invM01 = invDet * (m02 * m21 - m01 * m22);
            var invM02 = invDet * (m01 * m12 - m02 * m11);
            var invM10 = invDet * (m12 * m20 - m10 * m22);
            var invM11 = invDet * (m00 * m22 - m02 * m20);
            var invM12 = invDet * (m02 * m10 - m00 * m12);
            var invM20 = invDet * (m10 * m21 - m11 * m20);
            var invM21 = invDet * (m01 * m20 - m00 * m21);
            var invM22 = invDet * (m00 * m11 - m01 * m10);

            return new Matrix3S(invM00, invM01, invM02, invM10, invM11, invM12, invM20, invM21, invM22);
        }
    }
}

using System;

namespace stupid.Maths
{
    public struct Matrix3S
    {
        public f32 M11, M12, M13;
        public f32 M21, M22, M23;
        public f32 M31, M32, M33;

        public Matrix3S(f32 m11, f32 m12, f32 m13, f32 m21, f32 m22, f32 m23, f32 m31, f32 m32, f32 m33)
        {
            M11 = m11;
            M12 = m12;
            M13 = m13;
            M21 = m21;
            M22 = m22;
            M23 = m23;
            M31 = m31;
            M32 = m32;
            M33 = m33;
        }

        public Matrix3S(Vector3S row1, Vector3S row2, Vector3S row3)
        {
            M11 = row1.x;
            M12 = row1.y;
            M13 = row1.z;
            M21 = row2.x;
            M22 = row2.y;
            M23 = row2.z;
            M31 = row3.x;
            M32 = row3.y;
            M33 = row3.z;
        }

        public static Vector3S operator *(Matrix3S m, Vector3S v)
        {
            return new Vector3S(
                m.M11 * v.x + m.M12 * v.y + m.M13 * v.z,
                m.M21 * v.x + m.M22 * v.y + m.M23 * v.z,
                m.M31 * v.x + m.M32 * v.y + m.M33 * v.z
            );
        }

        public static Matrix3S operator *(Matrix3S a, Matrix3S b)
        {
            return new Matrix3S(
                a.M11 * b.M11 + a.M12 * b.M21 + a.M13 * b.M31, a.M11 * b.M12 + a.M12 * b.M22 + a.M13 * b.M32, a.M11 * b.M13 + a.M12 * b.M23 + a.M13 * b.M33,
                a.M21 * b.M11 + a.M22 * b.M21 + a.M23 * b.M31, a.M21 * b.M12 + a.M22 * b.M22 + a.M23 * b.M32, a.M21 * b.M13 + a.M22 * b.M23 + a.M23 * b.M33,
                a.M31 * b.M11 + a.M32 * b.M21 + a.M33 * b.M31, a.M31 * b.M12 + a.M32 * b.M22 + a.M33 * b.M32, a.M31 * b.M13 + a.M32 * b.M23 + a.M33 * b.M33
            );
        }

        public Matrix3S Transpose()
        {
            return new Matrix3S(
                M11, M21, M31,
                M12, M22, M32,
                M13, M23, M33
            );
        }

        public static Matrix3S Scale(Vector3S v)
        {
            return new Matrix3S(
                v.x, f32.zero, f32.zero,
                f32.zero, v.y, f32.zero,
                f32.zero, f32.zero, v.z
            );
        }

        public static Matrix3S Rotate(SQuaternion q)
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

        public Matrix3S Inverse()
        {
            f32 determinant =
                M11 * (M22 * M33 - M23 * M32) -
                M12 * (M21 * M33 - M23 * M31) +
                M13 * (M21 * M32 - M22 * M31);

            f32 invDet = f32.one / determinant;

            var invM11 = invDet * (M22 * M33 - M23 * M32);
            var invM12 = invDet * (M13 * M32 - M12 * M33);
            var invM13 = invDet * (M12 * M23 - M13 * M22);
            var invM21 = invDet * (M23 * M31 - M21 * M33);
            var invM22 = invDet * (M11 * M33 - M13 * M31);
            var invM23 = invDet * (M13 * M21 - M11 * M23);
            var invM31 = invDet * (M21 * M32 - M22 * M31);
            var invM32 = invDet * (M12 * M31 - M11 * M32);
            var invM33 = invDet * (M11 * M22 - M12 * M21);

            return new Matrix3S(invM11, invM12, invM13, invM21, invM22, invM23, invM31, invM32, invM33);
        }

        private static void Swap(ref f32 a, ref f32 b)
        {
            f32 temp = a;
            a = b;
            b = temp;
        }
    }
}

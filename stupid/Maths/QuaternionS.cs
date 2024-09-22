using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public struct QuaternionS : IEquatable<QuaternionS>
    {
        public f32 x, y, z, w;

        public static readonly QuaternionS identity = new QuaternionS(f32.zero, f32.zero, f32.zero, f32.one);
        public static readonly QuaternionS zero = new QuaternionS(f32.zero, f32.zero, f32.zero, f32.zero);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuaternionS(f32 x, f32 y, f32 z, f32 w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuaternionS(float x, float y, float z, float w)
        {
            this.x = f32.FromFloat(x);
            this.y = f32.FromFloat(y);
            this.z = f32.FromFloat(z);
            this.w = f32.FromFloat(w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static QuaternionS FromAxisAngle(in Vector3S axis, f32 angle)
        {
            f32 halfAngle = angle * f32.half;
            f32 sinHalfAngle = MathS.Sin(halfAngle);
            return new QuaternionS(
                axis.x * sinHalfAngle,
                axis.y * sinHalfAngle,
                axis.z * sinHalfAngle,
                MathS.Cos(halfAngle)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static QuaternionS FromEulerAngles(in Vector3S eulerAngles)
        {
            f32 c1 = MathS.Cos(eulerAngles.y * f32.half);
            f32 c2 = MathS.Cos(eulerAngles.z * f32.half);
            f32 c3 = MathS.Cos(eulerAngles.x * f32.half);

            f32 s1 = MathS.Sin(eulerAngles.y * f32.half);
            f32 s2 = MathS.Sin(eulerAngles.z * f32.half);
            f32 s3 = MathS.Sin(eulerAngles.x * f32.half);

            return new QuaternionS(
                s1 * c2 * c3 + c1 * s2 * s3,
                c1 * s2 * c3 - s1 * c2 * s3,
                c1 * c2 * s3 + s1 * s2 * c3,
                c1 * c2 * c3 - s1 * s2 * s3
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static QuaternionS operator *(in QuaternionS a, in QuaternionS b)
        {
            QuaternionS result;

            result.x.rawValue = (a.w.rawValue * b.x.rawValue + a.x.rawValue * b.w.rawValue + a.y.rawValue * b.z.rawValue - a.z.rawValue * b.y.rawValue) >> f32.FractionalBits;
            result.y.rawValue = (a.w.rawValue * b.y.rawValue - a.x.rawValue * b.z.rawValue + a.y.rawValue * b.w.rawValue + a.z.rawValue * b.x.rawValue) >> f32.FractionalBits;
            result.z.rawValue = (a.w.rawValue * b.z.rawValue + a.x.rawValue * b.y.rawValue - a.y.rawValue * b.x.rawValue + a.z.rawValue * b.w.rawValue) >> f32.FractionalBits;
            result.w.rawValue = (a.w.rawValue * b.w.rawValue - a.x.rawValue * b.x.rawValue - a.y.rawValue * b.y.rawValue - a.z.rawValue * b.z.rawValue) >> f32.FractionalBits;

            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator *(in QuaternionS q, in Vector3S v)
        {
            Vector3S result;

            // Precompute quaternion terms
            long qx2 = q.x.rawValue + q.x.rawValue;
            long qy2 = q.y.rawValue + q.y.rawValue;
            long qz2 = q.z.rawValue + q.z.rawValue;

            long qwqx2 = q.w.rawValue * qx2 >> f32.FractionalBits;
            long qwqy2 = q.w.rawValue * qy2 >> f32.FractionalBits;
            long qwqz2 = q.w.rawValue * qz2 >> f32.FractionalBits;

            long qxqx2 = q.x.rawValue * qx2 >> f32.FractionalBits;
            long qxqy2 = q.x.rawValue * qy2 >> f32.FractionalBits;
            long qxqz2 = q.x.rawValue * qz2 >> f32.FractionalBits;

            long qyqy2 = q.y.rawValue * qy2 >> f32.FractionalBits;
            long qyqz2 = q.y.rawValue * qz2 >> f32.FractionalBits;

            long qzqz2 = q.z.rawValue * qz2 >> f32.FractionalBits;

            // Rotate vector using quaternion
            result.x.rawValue = ((f32.One - qyqy2 - qzqz2) * v.x.rawValue + (qxqy2 - qwqz2) * v.y.rawValue + (qxqz2 + qwqy2) * v.z.rawValue) >> f32.FractionalBits;
            result.y.rawValue = ((qxqy2 + qwqz2) * v.x.rawValue + (f32.One - qxqx2 - qzqz2) * v.y.rawValue + (qyqz2 - qwqx2) * v.z.rawValue) >> f32.FractionalBits;
            result.z.rawValue = ((qxqz2 - qwqy2) * v.x.rawValue + (qyqz2 + qwqx2) * v.y.rawValue + (f32.One - qxqx2 - qyqy2) * v.z.rawValue) >> f32.FractionalBits;

            return result;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static QuaternionS operator /(in QuaternionS q, f32 scalar)
        {
            long invScalar = 1L / scalar.rawValue;

            return new QuaternionS(
                new f32((q.x.rawValue * invScalar) >> f32.FractionalBits),
                new f32((q.y.rawValue * invScalar) >> f32.FractionalBits),
                new f32((q.z.rawValue * invScalar) >> f32.FractionalBits),
                new f32((q.w.rawValue * invScalar) >> f32.FractionalBits)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public f32 Magnitude()
        {
            f32 ms = sqrMagnitude;
            return ms > f32.zero ? MathS.Sqrt(ms) : f32.zero;
        }

        public f32 sqrMagnitude
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                f32 result;
                long xx = (x.rawValue * x.rawValue) >> f32.FractionalBits;
                long yy = (y.rawValue * y.rawValue) >> f32.FractionalBits;
                long zz = (z.rawValue * z.rawValue) >> f32.FractionalBits;
                long ww = (w.rawValue * w.rawValue) >> f32.FractionalBits;

                result.rawValue = (xx + yy + zz + ww);
                return result;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static QuaternionS Inverse(in QuaternionS q)
        {
            f32 magSq = q.sqrMagnitude;
            if (magSq > f32.epsilon)
            {
                long invMagSq = (f32.one.rawValue << f32.FractionalBits) / magSq.rawValue;
                return new QuaternionS(
                    new f32((-q.x.rawValue * invMagSq) >> f32.FractionalBits),
                    new f32((-q.y.rawValue * invMagSq) >> f32.FractionalBits),
                    new f32((-q.z.rawValue * invMagSq) >> f32.FractionalBits),
                    new f32((q.w.rawValue * invMagSq) >> f32.FractionalBits)
                );
            }

            return identity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuaternionS Normalize()
        {
            f32 magnitude = Magnitude();

            if (magnitude > f32.zero)
            {
                long invMagnitude = (f32.one.rawValue << f32.FractionalBits) / magnitude.rawValue;

                return new QuaternionS(
                    new f32((x.rawValue * invMagnitude) >> f32.FractionalBits),
                    new f32((y.rawValue * invMagnitude) >> f32.FractionalBits),
                    new f32((z.rawValue * invMagnitude) >> f32.FractionalBits),
                    new f32((w.rawValue * invMagnitude) >> f32.FractionalBits)
                );
            }

            return identity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuaternionS Conjugate()
        {
            return new QuaternionS(-x, -y, -z, w);
        }

        // The LookRotation method remains as-is, since it does not involve much raw value optimization.

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() => $"Quaternion({x}, {y}, {z}, {w})";

        public override bool Equals(object obj)
        {
            return obj is QuaternionS quaternion &&
                   x.Equals(quaternion.x) &&
                   y.Equals(quaternion.y) &&
                   z.Equals(quaternion.z) &&
                   w.Equals(quaternion.w);
        }

        public override int GetHashCode() => HashCode.Combine(x, y, z, w);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(QuaternionS other)
        {
            return x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z) && w.Equals(other.w);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(in QuaternionS left, in QuaternionS right) => left.Equals(right);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(in QuaternionS left, in QuaternionS right) => !(left == right);
    }
}

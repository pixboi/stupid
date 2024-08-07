using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public struct Vector3S : IEquatable<Vector3S>
    {
        public static readonly Vector3S zero = new Vector3S(0f, 0f, 0f);
        public static readonly Vector3S one = new Vector3S(1f, 1f, 1f);
        public static readonly Vector3S up = new Vector3S(0f, 1f, 0f);
        public static readonly Vector3S down = new Vector3S(0f, -1f, 0f);
        public static readonly Vector3S left = new Vector3S(-1f, 0f, 0f);
        public static readonly Vector3S right = new Vector3S(1f, 0f, 0f);
        public static readonly Vector3S forward = new Vector3S(0f, 0f, 1f);
        public static readonly Vector3S back = new Vector3S(0f, 0f, -1f);

        public f32 x, y, z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S(f32 x, f32 y, f32 z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S(float x, float y, float z)
        {
            this.x = f32.FromFloat(x);
            this.y = f32.FromFloat(y);
            this.z = f32.FromFloat(z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator +(in Vector3S a, in Vector3S b)
        {
            return new Vector3S(
                new f32(a.x.rawValue + b.x.rawValue),
                new f32(a.y.rawValue + b.y.rawValue),
                new f32(a.z.rawValue + b.z.rawValue)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddInPlace(in Vector3S b)
        {
            this.x.AddInPlace(b.x);
            this.y.AddInPlace(b.y);
            this.z.AddInPlace(b.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator -(in Vector3S a, in Vector3S b)
        {
            return new Vector3S(
                new f32(a.x.rawValue - b.x.rawValue),
                new f32(a.y.rawValue - b.y.rawValue),
                new f32(a.z.rawValue - b.z.rawValue)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SubtractInPlace(in Vector3S b)
        {
            this.x.SubtractInPlace(b.x);
            this.y.SubtractInPlace(b.y);
            this.z.SubtractInPlace(b.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator -(in Vector3S a)
        {
            return new Vector3S(
                new f32(-a.x.rawValue),
                new f32(-a.y.rawValue),
                new f32(-a.z.rawValue)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator *(in Vector3S a, f32 d)
        {
            return new Vector3S(
                new f32((a.x.rawValue * d.rawValue) >> f32.FractionalBits),
                new f32((a.y.rawValue * d.rawValue) >> f32.FractionalBits),
                new f32((a.z.rawValue * d.rawValue) >> f32.FractionalBits)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void MultiplyInPlace(in Vector3S b)
        {
            this.x.MultiplyInPlace(b.x);
            this.y.MultiplyInPlace(b.y);
            this.z.MultiplyInPlace(b.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator /(in Vector3S a, f32 d)
        {
            return new Vector3S(
                new f32((a.x.rawValue << f32.FractionalBits) / d.rawValue),
                new f32((a.y.rawValue << f32.FractionalBits) / d.rawValue),
                new f32((a.z.rawValue << f32.FractionalBits) / d.rawValue)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void DivideInPlace(in Vector3S b)
        {
            this.x.DivideInPlace(b.x);
            this.y.DivideInPlace(b.y);
            this.z.DivideInPlace(b.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator *(f32 d, in Vector3S a)
        {
            return new Vector3S(
                new f32((d.rawValue * a.x.rawValue) >> f32.FractionalBits),
                new f32((d.rawValue * a.y.rawValue) >> f32.FractionalBits),
                new f32((d.rawValue * a.z.rawValue) >> f32.FractionalBits)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void MultiplyInPlace(in f32 b)
        {
            this.x.MultiplyInPlace(b);
            this.y.MultiplyInPlace(b);
            this.z.MultiplyInPlace(b);
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() => $"({x}, {y}, {z})";

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Dot(in Vector3S a, in Vector3S b)
        {
            long dotX = (a.x.rawValue * b.x.rawValue) >> f32.FractionalBits;
            long dotY = (a.y.rawValue * b.y.rawValue) >> f32.FractionalBits;
            long dotZ = (a.z.rawValue * b.z.rawValue) >> f32.FractionalBits;
            return new f32(dotX + dotY + dotZ);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 AbsDot(in Vector3S a, in Vector3S b)
        {
            long dotX = (a.x.rawValue * b.x.rawValue) >> f32.FractionalBits;
            long dotY = (a.y.rawValue * b.y.rawValue) >> f32.FractionalBits;
            long dotZ = (a.z.rawValue * b.z.rawValue) >> f32.FractionalBits;
            long dotProduct = dotX + dotY + dotZ;

            long mask = dotProduct >> 63; // Create a mask of all 1s if the value is negative, all 0s otherwise
            long absDotProduct = (dotProduct + mask) ^ mask; // If negative, flip the sign bits

            return new f32(absDotProduct);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S ProjectPointOnPlane(in Vector3S point, in Vector3S planeNormal, in Vector3S planePoint)
        {
            Vector3S toPoint = point - planePoint;
            f32 distance = Dot(toPoint, planeNormal);
            return point - planeNormal * distance;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Cross(in Vector3S a, in Vector3S b)
        {
            long crossX = (a.y.rawValue * b.z.rawValue - a.z.rawValue * b.y.rawValue) >> f32.FractionalBits;
            long crossY = (a.z.rawValue * b.x.rawValue - a.x.rawValue * b.z.rawValue) >> f32.FractionalBits;
            long crossZ = (a.x.rawValue * b.y.rawValue - a.y.rawValue * b.x.rawValue) >> f32.FractionalBits;

            return new Vector3S
            (
                new f32(crossX),
                new f32(crossY),
                new f32(crossZ)
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
                long xx = (x.rawValue * x.rawValue) >> f32.FractionalBits;
                long yy = (y.rawValue * y.rawValue) >> f32.FractionalBits;
                long zz = (z.rawValue * z.rawValue) >> f32.FractionalBits;
                return new f32(xx + yy + zz);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Distance(in Vector3S a, in Vector3S b) => (a - b).Magnitude();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 DistanceSquared(in Vector3S a, in Vector3S b) => (a - b).sqrMagnitude;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S Normalize()
        {
            f32 mag = Magnitude();
            return mag > f32.zero ? this / mag : zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S NormalizeWithMagnitude(out f32 mag)
        {
            mag = Magnitude();
            return mag > f32.zero ? this / mag : zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Min(in Vector3S a, in Vector3S b) => new Vector3S(
            MathS.Min(a.x, b.x),
            MathS.Min(a.y, b.y),
            MathS.Min(a.z, b.z)
        );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Max(in Vector3S a, in Vector3S b) => new Vector3S(
            MathS.Max(a.x, b.x),
            MathS.Max(a.y, b.y),
            MathS.Max(a.z, b.z)
        );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Abs(in Vector3S a) => new Vector3S(
            MathS.Abs(a.x),
            MathS.Abs(a.y),
            MathS.Abs(a.z)
        );


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S Clamp(f32 min, f32 max) => Clamp(this, min, max);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S Clamp(in Vector3S v, f32 min, f32 max) => new Vector3S(
            MathS.Clamp(v.x, min, max),
            MathS.Clamp(v.y, min, max),
            MathS.Clamp(v.z, min, max)
        );

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S ClampMagnitude(f32 min, f32 max) => ClampMagnitude(this, min, max);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S ClampMagnitude(in Vector3S v, f32 min, f32 max)
        {
            f32 sqrMagnitude = v.sqrMagnitude;
            if (sqrMagnitude > max * max)
            {
                f32 scale = max / MathS.Sqrt(sqrMagnitude);
                return v * scale;
            }
            else if (sqrMagnitude < min * min)
            {
                f32 scale = min / MathS.Sqrt(sqrMagnitude);
                return v * scale;
            }
            return v;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S Rotate(in QuaternionS rotation)
        {
            var qVector = new QuaternionS(x, y, z, f32.zero);
            var qConjugate = rotation.Conjugate();
            var qResult = rotation * qVector * qConjugate;
            return new Vector3S(qResult.x, qResult.y, qResult.z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Vector3S other) => x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z);

        public override bool Equals(object obj) => obj is Vector3S other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(x, y, z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(in Vector3S left, in Vector3S right) => left.Equals(right);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(in Vector3S left, in Vector3S right) => !(left == right);
    }
}

using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public readonly struct Vector3S : IEquatable<Vector3S>
    {
        public readonly f32 x, y, z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S(f32 x, f32 y, f32 z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S(f32 x)
        {
            this.x = x;
            this.y = x;
            this.z = x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S(float x, float y, float z)
        {
            this.x = f32.FromFloat(x);
            this.y = f32.FromFloat(y);
            this.z = f32.FromFloat(z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S(float x)
        {
            this.x = f32.FromFloat(x);
            this.y = this.x;
            this.z = this.x;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator +(in Vector3S a, in Vector3S b)
        {
            return new Vector3S(
                f32.FromRaw(a.x._value + b.x._value),
                f32.FromRaw(a.y._value + b.y._value),
                f32.FromRaw(a.z._value + b.z._value)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator -(in Vector3S a, in Vector3S b)
        {
            return new Vector3S(
                f32.FromRaw(a.x._value - b.x._value),
                f32.FromRaw(a.y._value - b.y._value),
                f32.FromRaw(a.z._value - b.z._value)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator -(in Vector3S a)
        {
            return new Vector3S(
                f32.FromRaw(-a.x._value),
                f32.FromRaw(-a.y._value),
                f32.FromRaw(-a.z._value)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator *(in Vector3S a, f32 d)
        {
            return new Vector3S(
                f32.FromRaw((a.x._value * d._value) >> f32.FractionalBits),
                f32.FromRaw((a.y._value * d._value) >> f32.FractionalBits),
                f32.FromRaw((a.z._value * d._value) >> f32.FractionalBits)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator /(in Vector3S a, f32 d)
        {
            return new Vector3S(
                f32.FromRaw((a.x._value << f32.FractionalBits) / d._value),
                f32.FromRaw((a.y._value << f32.FractionalBits) / d._value),
                f32.FromRaw((a.z._value << f32.FractionalBits) / d._value)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator *(f32 d, in Vector3S a)
        {
            return new Vector3S(
                f32.FromRaw((d._value * a.x._value) >> f32.FractionalBits),
                f32.FromRaw((d._value * a.y._value) >> f32.FractionalBits),
                f32.FromRaw((d._value * a.z._value) >> f32.FractionalBits)
            );
        }

        public static readonly Vector3S zero = new Vector3S(0f);
        public static readonly Vector3S one = new Vector3S(1f);
        public static readonly Vector3S up = new Vector3S(0f, 1f, 0f);
        public static readonly Vector3S down = new Vector3S(0f, -1f, 0f);
        public static readonly Vector3S left = new Vector3S(-1f, 0f, 0f);
        public static readonly Vector3S right = new Vector3S(1f, 0f, 0f);
        public static readonly Vector3S forward = new Vector3S(0f, 0f, 1f);
        public static readonly Vector3S back = new Vector3S(0f, 0f, -1f);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() => $"({x}, {y}, {z})";

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Dot(in Vector3S a, in Vector3S b)
        {
            long dotX = (a.x._value * b.x._value) >> f32.FractionalBits;
            long dotY = (a.y._value * b.y._value) >> f32.FractionalBits;
            long dotZ = (a.z._value * b.z._value) >> f32.FractionalBits;
            return f32.FromRaw(dotX + dotY + dotZ);
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
            long crossX = (a.y._value * b.z._value - a.z._value * b.y._value) >> f32.FractionalBits;
            long crossY = (a.z._value * b.x._value - a.x._value * b.z._value) >> f32.FractionalBits;
            long crossZ = (a.x._value * b.y._value - a.y._value * b.x._value) >> f32.FractionalBits;
            return new Vector3S(
                f32.FromRaw(crossX),
                f32.FromRaw(crossY),
                f32.FromRaw(crossZ)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public f32 Magnitude()
        {
            f32 magnitudeSquared = sqrMagnitude;
            return magnitudeSquared > f32.zero ? MathS.Sqrt(magnitudeSquared) : f32.zero;
        }

        public f32 sqrMagnitude
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                long xx = (x._value * x._value) >> f32.FractionalBits;
                long yy = (y._value * y._value) >> f32.FractionalBits;
                long zz = (z._value * z._value) >> f32.FractionalBits;
                return f32.FromRaw(xx + yy + zz);
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

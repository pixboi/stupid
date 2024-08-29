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
            this.x.rawValue += b.x.rawValue;
            this.y.rawValue += b.y.rawValue;
            this.z.rawValue += b.z.rawValue;
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
            this.x.rawValue -= b.x.rawValue;
            this.y.rawValue -= b.y.rawValue;
            this.z.rawValue -= b.z.rawValue;
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
            this.x.rawValue = (this.x.rawValue * b.x.rawValue) >> f32.FractionalBits;
            this.y.rawValue = (this.y.rawValue * b.y.rawValue) >> f32.FractionalBits;
            this.z.rawValue = (this.z.rawValue * b.z.rawValue) >> f32.FractionalBits;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator /(in Vector3S a, f32 d)
        {
            if (d.rawValue == 0)
            {
                throw new DivideByZeroException("Cannot divide by zero.");
            }

            return new Vector3S(
                new f32((a.x.rawValue << f32.FractionalBits) / d.rawValue),
                new f32((a.y.rawValue << f32.FractionalBits) / d.rawValue),
                new f32((a.z.rawValue << f32.FractionalBits) / d.rawValue)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void DivideInPlace(in f32 d)
        {
            if (d.rawValue == 0)
            {
                throw new DivideByZeroException("Cannot divide by zero.");
            }

            this.x.rawValue = (this.x.rawValue << f32.FractionalBits) / d.rawValue;
            this.y.rawValue = (this.y.rawValue << f32.FractionalBits) / d.rawValue;
            this.z.rawValue = (this.z.rawValue << f32.FractionalBits) / d.rawValue;
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
            this.x.rawValue = (this.x.rawValue * b.rawValue) >> f32.FractionalBits;
            this.y.rawValue = (this.y.rawValue * b.rawValue) >> f32.FractionalBits;
            this.z.rawValue = (this.z.rawValue * b.rawValue) >> f32.FractionalBits;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void MultiplyInPlace(in Matrix3S m)
        {
            this.x.rawValue = ((m.m00.rawValue * x.rawValue) + (m.m01.rawValue * y.rawValue) + (m.m02.rawValue * z.rawValue)) >> f32.FractionalBits;
            this.y.rawValue = ((m.m10.rawValue * x.rawValue) + (m.m11.rawValue * y.rawValue) + (m.m12.rawValue * z.rawValue)) >> f32.FractionalBits;
            this.z.rawValue = ((m.m20.rawValue * x.rawValue) + (m.m21.rawValue * y.rawValue) + (m.m22.rawValue * z.rawValue)) >> f32.FractionalBits;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() => $"({x}, {y}, {z})";

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 Dot(in Vector3S a, in Vector3S b)
        {
            long dotX = (a.x.rawValue * b.x.rawValue);
            long dotY = (a.y.rawValue * b.y.rawValue);
            long dotZ = (a.z.rawValue * b.z.rawValue);
            return new f32((dotX + dotY + dotZ) >> f32.FractionalBits);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 AbsDot(in Vector3S a, in Vector3S b)
        {
            var f = RawAbsDot(a, b);
            return new f32(f);    
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long RawAbsDot(in Vector3S a, in Vector3S b)
        {
            long dotProduct = ((a.x.rawValue * b.x.rawValue) +
                               (a.y.rawValue * b.y.rawValue) +
                               (a.z.rawValue * b.z.rawValue)) >> f32.FractionalBits;

            // Use bitwise operation to get the absolute value
            long mask = dotProduct >> 63; // Create a mask based on the sign of dotProduct
            return (dotProduct + mask) ^ mask;
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
            return new Vector3S
            (
                new f32(((a.y.rawValue * b.z.rawValue - a.z.rawValue * b.y.rawValue)) >> f32.FractionalBits),
                new f32(((a.z.rawValue * b.x.rawValue - a.x.rawValue * b.z.rawValue)) >> f32.FractionalBits),
                new f32(((a.x.rawValue * b.y.rawValue - a.y.rawValue * b.x.rawValue)) >> f32.FractionalBits)
            );
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CrossInPlace(in Vector3S b)
        {
            this.x.rawValue = (this.y.rawValue * b.z.rawValue - this.z.rawValue * b.y.rawValue) >> f32.FractionalBits;
            this.y.rawValue = (this.z.rawValue * b.x.rawValue - this.x.rawValue * b.z.rawValue) >> f32.FractionalBits;
            this.z.rawValue = (this.x.rawValue * b.y.rawValue - this.y.rawValue * b.x.rawValue) >> f32.FractionalBits;
        }

        public void Reset()
        {
            this.x.rawValue = 0;
            this.y.rawValue = 0;
            this.z.rawValue = 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public f32 Magnitude()
        {
            f32 ms = sqrMagnitude;
            return ms > f32.zero ? MathS.Sqrt(ms) : f32.zero;
        }

        //Useful for like, to see if more than zero
        public long rawSqrMagnitude
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                long xx = (x.rawValue * x.rawValue);
                long yy = (y.rawValue * y.rawValue);
                long zz = (z.rawValue * z.rawValue);
                return xx + yy + zz;
            }
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
        public Vector3S NormalizeInPlace()
        {
            f32 mag = Magnitude();

            if (mag > f32.zero)
            {
                this.DivideInPlace(mag);
            }

            return this;
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
                // Clamp to max magnitude
                f32 magnitude = MathS.Sqrt(sqrMagnitude);
                f32 scale = max / magnitude;
                return v * scale;
            }
            else if (sqrMagnitude < min * min && sqrMagnitude > f32.zero)
            {
                // Clamp to min magnitude, ensuring we don't divide by zero
                f32 magnitude = MathS.Sqrt(sqrMagnitude);
                f32 scale = min / magnitude;
                return v * scale;
            }

            return v; // Already within the desired magnitude range
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

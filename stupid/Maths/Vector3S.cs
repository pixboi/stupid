using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public partial struct Vector3S : IEquatable<Vector3S>
    {
        // Static readonly vectors
        public static readonly Vector3S zero = new Vector3S(0f, 0f, 0f);
        public static readonly Vector3S one = new Vector3S(1f, 1f, 1f);
        public static readonly Vector3S up = new Vector3S(0f, 1f, 0f);
        public static readonly Vector3S down = new Vector3S(0f, -1f, 0f);
        public static readonly Vector3S left = new Vector3S(-1f, 0f, 0f);
        public static readonly Vector3S right = new Vector3S(1f, 0f, 0f);
        public static readonly Vector3S forward = new Vector3S(0f, 0f, 1f);
        public static readonly Vector3S back = new Vector3S(0f, 0f, -1f);

        // Fields
        public f32 x, y, z;

        // Constructors
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

        #region OPERATORS

        // Addition
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
        public void Add(in Vector3S b)
        {
            this.x.rawValue += b.x.rawValue;
            this.y.rawValue += b.y.rawValue;
            this.z.rawValue += b.z.rawValue;
        }

        // Subtraction
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
        public void Subtract(in Vector3S b)
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

        public void Negate()
        {
            this.x.Negate();
            this.y.Negate();
            this.z.Negate();
        }

        public void Reset()
        {
            this.x.rawValue = 0;
            this.y.rawValue = 0;
            this.z.rawValue = 0;
        }

        // Multiplication
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
        public static Vector3S operator *(f32 d, in Vector3S a)
        {
            return new Vector3S(
                new f32((d.rawValue * a.x.rawValue) >> f32.FractionalBits),
                new f32((d.rawValue * a.y.rawValue) >> f32.FractionalBits),
                new f32((d.rawValue * a.z.rawValue) >> f32.FractionalBits)
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Multiply(in f32 b)
        {
            this.x.rawValue = (this.x.rawValue * b.rawValue) >> f32.FractionalBits;
            this.y.rawValue = (this.y.rawValue * b.rawValue) >> f32.FractionalBits;
            this.z.rawValue = (this.z.rawValue * b.rawValue) >> f32.FractionalBits;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Multiply(in Vector3S b)
        {
            this.x.rawValue = (this.x.rawValue * b.x.rawValue) >> f32.FractionalBits;
            this.y.rawValue = (this.y.rawValue * b.y.rawValue) >> f32.FractionalBits;
            this.z.rawValue = (this.z.rawValue * b.z.rawValue) >> f32.FractionalBits;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Multiply(in Matrix3S m)
        {
            this.x.rawValue = ((m.m00.rawValue * x.rawValue) + (m.m01.rawValue * y.rawValue) + (m.m02.rawValue * z.rawValue)) >> f32.FractionalBits;
            this.y.rawValue = ((m.m10.rawValue * x.rawValue) + (m.m11.rawValue * y.rawValue) + (m.m12.rawValue * z.rawValue)) >> f32.FractionalBits;
            this.z.rawValue = ((m.m20.rawValue * x.rawValue) + (m.m21.rawValue * y.rawValue) + (m.m22.rawValue * z.rawValue)) >> f32.FractionalBits;
        }

        // Division
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
        public void Divide(in f32 d)
        {
            if (d.rawValue == 0)
            {
                throw new DivideByZeroException("Cannot divide by zero.");
            }

            this.x.rawValue = (this.x.rawValue << f32.FractionalBits) / d.rawValue;
            this.y.rawValue = (this.y.rawValue << f32.FractionalBits) / d.rawValue;
            this.z.rawValue = (this.z.rawValue << f32.FractionalBits) / d.rawValue;
        }

        #endregion

        // Magnitude
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public f32 Magnitude()
        {
            f32 ms = sqrMagnitude;
            return ms > f32.zero ? MathS.Sqrt(ms) : f32.zero;
        }

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

        // Equality
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Vector3S other) => x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z);

        public override bool Equals(object obj) => obj is Vector3S other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(x, y, z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(in Vector3S left, in Vector3S right) => left.Equals(right);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(in Vector3S left, in Vector3S right) => !(left == right);

        // ToString
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() => $"({x}, {y}, {z})";
    }
}

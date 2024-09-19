using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public partial struct Vector3S : IEquatable<Vector3S>
    {
        // Fields
        public f32 x, y, z;
        public f32 this[int i]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                unsafe
                {
                    fixed (f32* ptr = &x)
                    {
                        if (i < 0 || i > 2)
                            throw new ArgumentOutOfRangeException();
                        return *(ptr + i);
                    }
                }
            }
        }

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
            Vector3S result;

            result.x.rawValue = a.x.rawValue + b.x.rawValue;
            result.y.rawValue = a.y.rawValue + b.y.rawValue;
            result.z.rawValue = a.z.rawValue + b.z.rawValue;

            return result;
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
            Vector3S result;

            result.x.rawValue = a.x.rawValue - b.x.rawValue;
            result.y.rawValue = a.y.rawValue - b.y.rawValue;
            result.z.rawValue = a.z.rawValue - b.z.rawValue;

            return result;
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
            Vector3S result;

            result.x.rawValue = -a.x.rawValue;
            result.y.rawValue = -a.y.rawValue;
            result.z.rawValue = -a.z.rawValue;

            return result;
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
            Vector3S result;

            result.x.rawValue = (a.x.rawValue * d.rawValue) >> f32.FractionalBits;
            result.y.rawValue = (a.y.rawValue * d.rawValue) >> f32.FractionalBits;
            result.z.rawValue = (a.z.rawValue * d.rawValue) >> f32.FractionalBits;

            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator *(f32 d, in Vector3S a)
        {
            Vector3S result;

            result.x.rawValue = (d.rawValue * a.x.rawValue) >> f32.FractionalBits;
            result.y.rawValue = (d.rawValue * a.y.rawValue) >> f32.FractionalBits;
            result.z.rawValue = (d.rawValue * a.z.rawValue) >> f32.FractionalBits;

            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Multiply(in f32 b)
        {
            this.x.rawValue = (this.x.rawValue * b.rawValue) >> f32.FractionalBits;
            this.y.rawValue = (this.y.rawValue * b.rawValue) >> f32.FractionalBits;
            this.z.rawValue = (this.z.rawValue * b.rawValue) >> f32.FractionalBits;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Multiply(in Matrix3S m)
        {
            // Temporary variables to store intermediate results
            var xResult = (m.m00.rawValue * x.rawValue) + (m.m01.rawValue * y.rawValue) + (m.m02.rawValue * z.rawValue);
            var yResult = (m.m10.rawValue * x.rawValue) + (m.m11.rawValue * y.rawValue) + (m.m12.rawValue * z.rawValue);
            var zResult = (m.m20.rawValue * x.rawValue) + (m.m21.rawValue * y.rawValue) + (m.m22.rawValue * z.rawValue);

            // Apply right shift to account for fixed-point fractional bits
            this.x.rawValue = xResult >> f32.FractionalBits;
            this.y.rawValue = yResult >> f32.FractionalBits;
            this.z.rawValue = zResult >> f32.FractionalBits;
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

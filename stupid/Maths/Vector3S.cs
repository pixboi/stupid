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
        public static Vector3S operator -(in Vector3S a)
        {
            Vector3S result;

            result.x.rawValue = -a.x.rawValue;
            result.y.rawValue = -a.y.rawValue;
            result.z.rawValue = -a.z.rawValue;

            return result;
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
        public static Vector3S MultiplyAndAdd(in Vector3S a, in f32 b, in Vector3S parent)
        {
            Vector3S result;
            result.x.rawValue = parent.x.rawValue + ((a.x.rawValue * b.rawValue) >> f32.FractionalBits);
            result.y.rawValue = parent.y.rawValue + ((a.y.rawValue * b.rawValue) >> f32.FractionalBits);
            result.z.rawValue = parent.y.rawValue + ((a.z.rawValue * b.rawValue) >> f32.FractionalBits);
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S MultiplyAndSubtract(in Vector3S a, in f32 b, in Vector3S parent)
        {
            Vector3S result;
            result.x.rawValue = parent.x.rawValue - ((a.x.rawValue * b.rawValue) >> f32.FractionalBits);
            result.y.rawValue = parent.y.rawValue - ((a.y.rawValue * b.rawValue) >> f32.FractionalBits);
            result.z.rawValue = parent.y.rawValue - ((a.z.rawValue * b.rawValue) >> f32.FractionalBits);
            return result;
        }

        // Division
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator /(in Vector3S a, f32 d)
        {
            if (d.rawValue == 0)
            {
                throw new DivideByZeroException("Cannot divide by zero.");
            }

            Vector3S result;

            result.x.rawValue = (a.x.rawValue << f32.FractionalBits) / d.rawValue;
            result.y.rawValue = (a.y.rawValue << f32.FractionalBits) / d.rawValue;
            result.z.rawValue = (a.z.rawValue << f32.FractionalBits) / d.rawValue;

            return result;
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

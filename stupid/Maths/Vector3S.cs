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
        public Vector3S(in f32 x, in f32 y, in f32 z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        // Constructors
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3S(in f32 value)
        {
            this.x = value;
            this.y = value;
            this.z = value;
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
        public f32 Sum()
        {
            f32 result;
            result.rawValue = this.x.rawValue + this.y.rawValue + this.z.rawValue;
            return result;
        }

        // Addition
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator +(in Vector3S a, in f32 b)
        {
            Vector3S result;

            result.x.rawValue = a.x.rawValue + b.rawValue;
            result.y.rawValue = a.y.rawValue + b.rawValue;
            result.z.rawValue = a.z.rawValue + b.rawValue;

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

        // Multiplication
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator *(in Vector3S a, in Vector3S b)
        {
            Vector3S result;

            result.x.rawValue = (a.x.rawValue * b.x.rawValue) >> f32.FractionalBits;
            result.y.rawValue = (a.y.rawValue * b.y.rawValue) >> f32.FractionalBits;
            result.z.rawValue = (a.z.rawValue * b.z.rawValue) >> f32.FractionalBits;

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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S MultiplyAndAddBatch(in Vector3S a, in f32 am, in Vector3S b, in f32 bm)
        {
            Vector3S result;

            // Calculate inte rmediate multiplication results
            var ax_am = a.x.rawValue * am.rawValue;
            var bx_bm = b.x.rawValue * bm.rawValue;
            var ay_am = a.y.rawValue * am.rawValue;
            var by_bm = b.y.rawValue * bm.rawValue;
            var az_am = a.z.rawValue * am.rawValue;
            var bz_bm = b.z.rawValue * bm.rawValue;

            // Perform bit shifting and addition for each component
            result.x.rawValue = (ax_am + bx_bm) >> f32.FractionalBits;
            result.y.rawValue = (ay_am + by_bm) >> f32.FractionalBits;
            result.z.rawValue = (az_am + bz_bm) >> f32.FractionalBits;

            return result;
        }



        // Division
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator /(in Vector3S a, in f32 d)
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

        // Division
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3S operator /(in Vector3S a, in Vector3S b)
        {
            if (b.x.rawValue == 0 || b.y.rawValue == 0 || b.z.rawValue == 0)
            {
                throw new DivideByZeroException("Cannot divide by zero.");
            }

            Vector3S result;

            result.x.rawValue = (a.x.rawValue << f32.FractionalBits) / b.x.rawValue;
            result.y.rawValue = (a.y.rawValue << f32.FractionalBits) / b.y.rawValue;
            result.z.rawValue = (a.z.rawValue << f32.FractionalBits) / b.z.rawValue;

            return result;
        }

        #endregion

        // Equality
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Vector3S other) => x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj) => obj is Vector3S other && Equals(other);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode() => HashCode.Combine(x, y, z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(in Vector3S left, in Vector3S right) => left.Equals(right);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(in Vector3S left, in Vector3S right) => !(left == right);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <=(in Vector3S left, in Vector3S right) => left.x.rawValue <= right.x.rawValue && left.y.rawValue <= right.y.rawValue && left.z.rawValue <= right.z.rawValue;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >=(in Vector3S left, in Vector3S right) => left.x.rawValue >= right.x.rawValue && left.y.rawValue >= right.y.rawValue && left.z.rawValue >= right.z.rawValue;

        // ToString
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() => $"({x}, {y}, {z})";
    }
}

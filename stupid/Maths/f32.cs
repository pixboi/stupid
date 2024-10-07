using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    [Serializable]
    public struct f32 : IEquatable<f32>, IComparable<f32>
    {
        public long rawValue;
        public const int FractionalBits = 16;
        public const int TotalBitsMinusOne = 63;

        public const long One = 1L << FractionalBits;
        public const long Two = 2L << FractionalBits;

        public static readonly f32 epsilon = new f32(1);
        public static readonly f32 small = FromFloat(0.01f); //These are not guaranteed to be same on diff machines, Quantum uses string parsing, which is deterministic :O
        public static readonly f32 quarter = FromFloat(0.25f);
        public static readonly f32 half = new f32(One >> 1);
        public static readonly f32 zero = new f32(0);
        public static readonly f32 one = new f32(One);
        public static readonly f32 two = new f32(Two);
        public static readonly f32 negativeOne = new f32(-One);
        public static readonly f32 negativeTwo = new f32(-Two);
        public static readonly f32 pi = FromFloat(3.14159265358979323846f);
        public static readonly f32 twoPi = FromFloat(6.28318530717958647692f);
        public static readonly f32 halfPi = FromFloat(1.57079632679489661923f);
        public static readonly f32 maxValue = new f32(long.MaxValue);
        public static readonly f32 minValue = new f32(long.MinValue);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public f32(long value)
        {
            this.rawValue = value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 FromFloat(float value) => new f32((long)(value * One));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float ToFloat() => (float)rawValue / One;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator +(in f32 a, in f32 b)
        {
            f32 result;
            result.rawValue = a.rawValue + b.rawValue;
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator -(in f32 a, in f32 b)
        {
            f32 result;
            result.rawValue = a.rawValue - b.rawValue;
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator *(in f32 a, in f32 b)
        {
            f32 result;
            result.rawValue = (a.rawValue * b.rawValue) >> FractionalBits;
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator /(in f32 a, in f32 b)
        {
            if (b.rawValue == 0) throw new DivideByZeroException("Cannot divide by zero.");

            f32 result;
            var dividend = (a.rawValue << FractionalBits);

            result.rawValue = dividend / b.rawValue;
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator -(in f32 value)
        {
            f32 result;
            result.rawValue = -value.rawValue;
            return result;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator float(in f32 fp) => fp.ToFloat();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator f32(float value) => FromFloat(value);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() => ToFloat().ToString();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(f32 other) => rawValue == other.rawValue;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int CompareTo(f32 other) => rawValue.CompareTo(other.rawValue);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj) => obj is f32 other && Equals(other);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode() => rawValue.GetHashCode();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(in f32 left, in f32 right) => left.rawValue == right.rawValue;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(in f32 left, in f32 right) => left.rawValue != right.rawValue;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <(in f32 left, in f32 right) => left.rawValue < right.rawValue;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >(in f32 left, in f32 right) => left.rawValue > right.rawValue;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <=(in f32 left, in f32 right) => left.rawValue <= right.rawValue;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >=(in f32 left, in f32 right) => left.rawValue >= right.rawValue;
    }
}

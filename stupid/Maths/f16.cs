using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public struct f16 : IEquatable<f16>, IComparable<f16>
    {
        public int rawValue;
        public const int FractionalBits = 16;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public f16(int value)
        {
            rawValue = value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f16 operator +(in f16 a, in f16 b)
        {
            f16 result;
            result.rawValue = a.rawValue + b.rawValue;
            return result;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f16 operator -(in f16 a, in f16 b)
        {
            f16 result;
            result.rawValue = a.rawValue - b.rawValue;
            return result;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f16 operator *(in f16 a, in f16 b)
        {
            f16 result;
            result.rawValue = (a.rawValue * b.rawValue) >> FractionalBits;
            return result;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f16 operator /(in f16 a, in f16 b)
        {
            if (b.rawValue == 0) throw new DivideByZeroException("Cannot divide by zero.");

            f16 result;
            int dividend = (a.rawValue << FractionalBits);

            result.rawValue = dividend / b.rawValue;
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f16 operator -(in f16 value)
        {
            f16 result;
            result.rawValue = -value.rawValue;
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(f16 other) => rawValue == other.rawValue;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int CompareTo(f16 other) => rawValue.CompareTo(other.rawValue);

        public override bool Equals(object obj) => obj is f16 other && Equals(other);

        public override int GetHashCode() => rawValue.GetHashCode();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(in f16 left, in f16 right) => left.rawValue == right.rawValue;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(in f16 left, in f16 right) => left.rawValue != right.rawValue;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <(in f16 left, in f16 right) => left.rawValue < right.rawValue;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >(in f16 left, in f16 right) => left.rawValue > right.rawValue;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <=(in f16 left, in f16 right) => left.rawValue <= right.rawValue;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >=(in f16 left, in f16 right) => left.rawValue >= right.rawValue;
    }
}

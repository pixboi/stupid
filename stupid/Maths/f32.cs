using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public struct f32 : IEquatable<f32>, IComparable<f32>
    {
        public long rawValue;
        public const int FractionalBits = 24;
        public const long One = 1L << FractionalBits;
        public const long Two = 2L << FractionalBits;

        public static readonly f32 epsilon = new f32(1L);
        public static readonly f32 small = FromFloat(0.01f);
        public static readonly f32 zero = new f32(0L);
        public static readonly f32 one = new f32(One);
        public static readonly f32 two = new f32(Two);
        public static readonly f32 quarter = FromFloat(0.25f);
        public static readonly f32 half = new f32(One >> 1);
        public static readonly f32 negativeOne = new f32(-One);
        public static readonly f32 negativeTwo = new f32(-(2L << FractionalBits));
        public static readonly f32 pi = FromFloat(3.14159265358979323846f);
        public static readonly f32 twoPi = FromFloat(6.28318530717958647692f);
        public static readonly f32 halfPi = FromFloat(1.57079632679489661923f);

        public static readonly f32 maxValue = new f32(long.MaxValue);
        public static readonly f32 minValue = new f32(long.MinValue);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public f32(long value)
        {
            rawValue = value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public long ToRaw() => rawValue;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 FromFloat(float value) => new f32((long)(value * One));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float ToFloat() => (float)rawValue / One;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator +(in f32 a, in f32 b) => new f32(a.rawValue + b.rawValue);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(in f32 b) { rawValue += b.rawValue; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator -(in f32 a, in f32 b) => new f32(a.rawValue - b.rawValue);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Subtract(in f32 b) { rawValue -= b.rawValue; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator *(in f32 a, in f32 b) => new f32((a.rawValue * b.rawValue) >> FractionalBits);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Multiply(in f32 b) { rawValue = (rawValue * b.rawValue) >> FractionalBits; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator /(in f32 a, in f32 b)
        {
            if (b.rawValue == 0) throw new DivideByZeroException("Cannot divide by zero.");
            long dividend = (a.rawValue << FractionalBits);
            long result = dividend / b.rawValue;
            return new f32(result);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Divide(in f32 b)
        {
            if (b.rawValue == 0) throw new DivideByZeroException("Cannot divide by zero.");
            long dividend = (this.rawValue << FractionalBits);
            this.rawValue = dividend / b.rawValue;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Abs()
        {
            long mask = rawValue >> 63;  // This creates a mask that is 0xFFFFFFFFFFFFFFFF if rawValue is negative, or 0x0000000000000000 if positive.
            rawValue = (rawValue + mask) ^ mask;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Reset() { rawValue = 0; }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator -(in f32 value) => new f32(-value.rawValue);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Negate()
        {
            this.rawValue = -this.rawValue;
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

        public override bool Equals(object obj) => obj is f32 other && Equals(other);

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

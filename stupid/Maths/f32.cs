using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public readonly struct f32 : IEquatable<f32>, IComparable<f32>
    {
        public readonly long _value;
        public const int FractionalBits = 16;
        private const long One = 1L << FractionalBits;

        public static readonly f32 zero = new f32(0);
        public static readonly f32 one = new f32(One);
        public static readonly f32 two = new f32(2L << FractionalBits);
        public static readonly f32 quarter = FromFloat(0.25f);
        public static readonly f32 half = new f32(One >> 1);
        public static readonly f32 negativeOne = new f32(-One);
        public static readonly f32 negativeTwo = new f32(-(2L << FractionalBits));
        public static readonly f32 pi = FromFloat(3.14159265358979323846f);
        public static readonly f32 twoPi = FromFloat(6.28318530717958647692f);
        public static readonly f32 halfPi = FromFloat(1.57079632679489661923f);
        public static readonly f32 epsilon = new f32(1); // Smallest possible value greater than zero
        public static readonly f32 maxValue = new f32(long.MaxValue);
        public static readonly f32 minValue = new f32(long.MinValue);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public f32(long value)
        {
            _value = value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public long ToRaw() => _value;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 FromFloat(float value) => new f32((long)(value * One));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float ToFloat() => (float)_value / One;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator +(in f32 a, in f32 b) => new f32(a._value + b._value);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator -(in f32 a, in f32 b) => new f32(a._value - b._value);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator *(in f32 a, in f32 b) => new f32((a._value * b._value) >> FractionalBits);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator /(in f32 a, in f32 b)
        {
            if (b._value == 0)
            {
                throw new DivideByZeroException("Cannot divide by zero.");
            }
            long dividend = (a._value << FractionalBits);
            long result = dividend / b._value;
            return new f32(result);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator -(in f32 value) => new f32(-value._value);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator float(in f32 fp) => fp.ToFloat();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator f32(float value) => FromFloat(value);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() => ToFloat().ToString();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(f32 other) => _value == other._value;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int CompareTo(f32 other) => _value.CompareTo(other._value);

        public override bool Equals(object obj) => obj is f32 other && Equals(other);

        public override int GetHashCode() => _value.GetHashCode();

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(in f32 left, in f32 right) => left._value == right._value;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(in f32 left, in f32 right) => left._value != right._value;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <(in f32 left, in f32 right) => left._value < right._value;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >(in f32 left, in f32 right) => left._value > right._value;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <=(in f32 left, in f32 right) => left._value <= right._value;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >=(in f32 left, in f32 right) => left._value >= right._value;
    }
}

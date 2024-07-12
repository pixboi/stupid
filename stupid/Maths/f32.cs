using System;
using System.Runtime.CompilerServices;

namespace stupid.Maths
{
    public static class f32helpers
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static long ToRaw(this f32 value)
        {
            return value._value;
        }
    }

    public readonly struct f32 : IEquatable<f32>, IComparable<f32>
    {
        public readonly long _value;
        public const int FractionalBits = 16;
        private const long One = 1L << FractionalBits;
        public static readonly f32 zero = FromRaw(0);
        public static readonly f32 one = FromRaw(One);
        public static readonly f32 two = FromRaw(2L << FractionalBits);
        public static readonly f32 half = FromRaw(One >> 1);
        public static readonly f32 negativeOne = FromRaw(-One);
        public static readonly f32 negativeTwo = FromRaw(-(2L << FractionalBits));
        public static readonly f32 pi = FromFloat(3.14159265358979323846f);
        public static readonly f32 twoPi = FromFloat(6.28318530717958647692f);
        public static readonly f32 halfPi = FromFloat(1.57079632679489661923f);
        public static readonly f32 epsilon = FromRaw(1); // Smallest possible value greater than zero
        public static readonly f32 maxValue = FromRaw(long.MaxValue);
        public static readonly f32 minValue = FromRaw(long.MinValue);

        private f32(long value)
        {
            _value = value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 FromRaw(long rawValue)
        {
            return new f32(rawValue);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 FromFloat(float value)
        {
            return new f32((long)(value * One));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float ToFloat()
        {
            return (float)_value / One;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator +(f32 a, f32 b)
        {
            return new f32(a._value + b._value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator -(f32 a, f32 b)
        {
            return new f32(a._value - b._value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator *(f32 a, f32 b)
        {
            long result = (a._value * b._value) >> FractionalBits;
            return new f32(result);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static f32 operator /(f32 a, f32 b)
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
        public static f32 operator -(f32 value)
        {
            return new f32(-value._value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator float(f32 fp)
        {
            return fp.ToFloat();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static explicit operator f32(float value)
        {
            return FromFloat(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return ToFloat().ToString();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(f32 other)
        {
            return _value == other._value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int CompareTo(f32 other)
        {
            return _value.CompareTo(other._value);
        }

        public override bool Equals(object obj)
        {
            return obj is f32 other && Equals(other);
        }

        public override int GetHashCode()
        {
            return _value.GetHashCode();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(f32 left, f32 right)
        {
            return left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(f32 left, f32 right)
        {
            return !left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <(f32 left, f32 right)
        {
            return left.CompareTo(right) < 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >(f32 left, f32 right)
        {
            return left.CompareTo(right) > 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <=(f32 left, f32 right)
        {
            return left.CompareTo(right) <= 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >=(f32 left, f32 right)
        {
            return left.CompareTo(right) >= 0;
        }
    }
}

﻿using System;

namespace stupid.Maths
{
    public static class f32helpers
    {
        public static int ToRaw(this f32 value)
        {
            return value._value;
        }
    }

    public struct f32 : IEquatable<f32>, IComparable<f32>
    {
        public readonly int _value;
        private const int FractionalBits = 16;
        private const int One = 1 << FractionalBits;
        public static f32 zero => FromRaw(0);
        public static f32 one => FromRaw(One);
        public static f32 two => FromRaw(2 << FractionalBits);
        public static f32 half => FromRaw(One >> 1);
        public static f32 negativeOne => FromRaw(-One);
        public static f32 negativeTwo => FromRaw(-(2 << FractionalBits));
        public static f32 pi => FromFloat(3.14159265358979323846f);
        public static f32 twoPi => FromFloat(6.28318530717958647692f);
        public static f32 halfPi => FromFloat(1.57079632679489661923f);
        public static f32 epsilon => FromRaw(1); // Smallest possible value greater than zero
        public static f32 maxValue => FromRaw(int.MaxValue);
        public static f32 minValue => FromRaw(int.MinValue);

        private f32(int value)
        {
            _value = value;
        }

        public static f32 FromRaw(int rawValue)
        {
            return new f32(rawValue);
        }

        public static f32 FromFloat(float value)
        {
            return new f32((int)(value * One));
        }

        public float ToFloat()
        {
            return (float)_value / One;
        }

        public static f32 operator +(f32 a, f32 b)
        {
            return new f32(a._value + b._value);
        }

        public static f32 operator -(f32 a, f32 b)
        {
            return new f32(a._value - b._value);
        }

        public static f32 operator *(f32 a, f32 b)
        {
            long result = (long)a._value * b._value;
            return new f32((int)(result >> FractionalBits));
        }

        public static f32 operator /(f32 a, f32 b)
        {
            long dividend = ((long)a._value << FractionalBits);
            return new f32((int)(dividend / b._value));
        }

        public static f32 operator -(f32 value)
        {
            return new f32(-value._value);
        }

        public static explicit operator float(f32 fp)
        {
            return fp.ToFloat();
        }

        public static explicit operator f32(float value)
        {
            return FromFloat(value);
        }

        public override string ToString()
        {
            return ToFloat().ToString();
        }

        public bool Equals(f32 other)
        {
            return _value == other._value;
        }

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

        public static bool operator ==(f32 left, f32 right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(f32 left, f32 right)
        {
            return !left.Equals(right);
        }

        public static bool operator <(f32 left, f32 right)
        {
            return left.CompareTo(right) < 0;
        }

        public static bool operator >(f32 left, f32 right)
        {
            return left.CompareTo(right) > 0;
        }

        public static bool operator <=(f32 left, f32 right)
        {
            return left.CompareTo(right) <= 0;
        }

        public static bool operator >=(f32 left, f32 right)
        {
            return left.CompareTo(right) >= 0;
        }
    }
}
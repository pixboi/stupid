using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;
using stupid.Maths;
using System;
using System.Runtime.InteropServices;

public class MyBenchmarks
{
    // Define two structs: one with 256 bytes and another broken into two 128-byte structs
    [StructLayout(LayoutKind.Sequential)]
    public struct LargeStruct
    {
        public long A1, A2, A3, A4;
        public long B1, B2, B3, B4;
        public long C1, C2, C3, C4;
        public long D1, D2, D3, D4;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct HalfStruct
    {
        public long A1, A2, A3, A4;
        public long B1, B2, B3, B4;
    }

    public Vector3S[] vectors;
    public int iterations = 10000;

    [GlobalSetup]
    public void Setup()
    {
        vectors = new Vector3S[iterations];

        for (int i = 0; i < iterations; i++)
        {
            vectors[i] = new Vector3S(i, i, i);
        }
    }

    // Benchmark for processing half structs in two parts
    [Benchmark]
    public void ProcessHalfStructs()
    {

    }
}

// Main program to run the benchmarks
public class Program
{
    public static void Main(string[] args)
    {
        var summary = BenchmarkRunner.Run<MyBenchmarks>();
    }
}

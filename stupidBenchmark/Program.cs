using System;
using System.Numerics;
using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;
using stupid;
using stupid.Maths;

public class VectorBenchmark
{
    private Vector3S[] a1;
    private Vector3S[] a2;

    private f32[] b1;
    private f32[] b2;

    private long[] c1;
    private long[] c2;

    [GlobalSetup]
    public void Setup()
    {
        a1 = new Vector3S[10000];
        a2 = new Vector3S[10000];

        b1 = new f32[10000];
        b2 = new f32[10000];

        c1 = new long[10000];
        c2 = new long[10000];

        for (int i = 0; i < a1.Length; i++)
        {
            var v = i % 100;

            a1[i] = new Vector3S(v, v, v);
            a2[i] = new Vector3S(v, v, v);

            b1[i] = new f32(v);
            b2[i] = new f32(v);

            c1[i] = v;
            c2[i] = v;
        }

    }

    [Benchmark]
    public void VectorLoopOp()
    {
        for (int i = 0; i < a1.Length; i++)
        {
            ref var a = ref a1[i];
            ref var b = ref a2[i];

            a = b.x * a + b;
        }
    }

    [Benchmark]
    public void VectorLoopBatched()
    {
        for (int i = 0; i < a1.Length; i++)
        {
            ref var a = ref a1[i];
            ref var b = ref a2[i];

            a = Vector3S.MultiplyAndAdd(a, b.x, b);
        }
    }

    /*
    [Benchmark]
    public void f32Loop()
    {
        for (int i = 0; i < b1.Length; i++)
        {
            b1[i] = b1[i] + b2[i];
        }
    }

    [Benchmark]
    public void LongLoop()
    {
        for (int i = 0; i < b1.Length; i++)
        {
            c1[i] = c1[i] + c2[i];
        }
    }
    */
}

public class Program
{
    public static void Main(string[] args)
    {
        var summary = BenchmarkRunner.Run<VectorBenchmark>();
    }
}
using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;
using stupid;
using stupid.Maths; // Reference your DLL namespace

public class MyBenchmarks
{

    f32[] list;
    public int iterations = 10000;

    [GlobalSetup]
    public void Setup()
    {
        // Initialize your objects here
        list = new f32[iterations];

        for (int i = 0; i < iterations; i++)
        {
            list[i] = new f32(i + 1);
        }
    }


    [Benchmark]
    public void AddOperator()
    {
        var temp = f32.zero;

        foreach (var item in list)
        {
            temp += item;
        }
    }


    [Benchmark]
    public void AddInPlace()
    {
        var temp = f32.zero;

        foreach (var item in list)
        {
     
        }
    }

    /*
    [Benchmark]
    public void AddOperatorSpan()
    {
        var temp = f32.zero;
        var span = list.AsSpan();

        foreach (var item in span)
        {
            temp += item;
        }
    }


    [Benchmark]
    public void AddInPlaceSpan()
    {
        var temp = f32.zero;
        var span = list.AsSpan();

        foreach (var item in span)
        {
            temp.Add(item);
        }
    }
    */
}

public class Program
{
    public static void Main(string[] args)
    {
        var summary = BenchmarkRunner.Run<MyBenchmarks>();
    }
}

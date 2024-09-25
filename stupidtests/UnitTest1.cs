using stupid.Maths;
using System.Diagnostics;
using System.Numerics;

namespace stupidtests
{
    [TestClass]
    public class UnitTest1
    {
        public int iterations = 100000000;

        [TestMethod]
        public void f32Test()
        {
            var list = new f32[iterations];

            for (int i = 0; i < iterations; i++)
            {
                list[i] = new f32(i + 1);
            }

            var span = list.AsSpan();

            var sw = new System.Diagnostics.Stopwatch();
            sw.Start();

            var temp = new f32();

            foreach (var item in span)
            {
                temp += item;
                temp -= item;
                // temp *= item;
                // temp /= item;
            }

            sw.Stop();
            Console.WriteLine(sw.Elapsed.ToString());
        }

        [TestMethod]
        public void Vectorf32Test()
        {
            var list = new Vector3S[iterations];

            for (int i = 0; i < iterations; i++)
            {
                list[i] = new Vector3S(i, i, i);
            }

            var span = list.AsSpan();

            var sw = new System.Diagnostics.Stopwatch();
            sw.Start();

            var temp = Vector3S.zero;

            foreach (var item in span)
            {
                temp += item;
                temp -= item;
                // temp *= item;
                //temp /= item;
            }

            sw.Stop();
            Console.WriteLine(sw.Elapsed.ToString());
        }
        [TestMethod]

        public void LongTest()
        {
            var list = new long[iterations];

            for (int i = 0; i < iterations; i++)
            {
                list[i] = i + 1;
            }

            var sw = new System.Diagnostics.Stopwatch();
            sw.Start();

            var temp = 0L;

            foreach (var item in list)
            {
                temp += item;
                temp -= item;
                temp *= item;
                temp /= item;
            }

            sw.Stop();
            Console.WriteLine(sw.Elapsed.ToString());
        }

        [TestMethod]

        public void IntTest()
        {
            var list = new int[iterations];

            for (int i = 0; i < iterations; i++)
            {
                list[i] = i + 1;
            }

            var sw = new System.Diagnostics.Stopwatch();
            sw.Start();

            var temp = 0;

            foreach (var item in list)
            {
                temp += item;
                temp -= item;
                temp *= item;
                temp /= item;
            }

            sw.Stop();
            Console.WriteLine(sw.Elapsed.ToString());
        }

        [TestMethod]

        public void Vector3STest()
        {
            var list = new Vector3S[iterations];

            for (int i = 0; i < iterations; i++)
            {
                list[i] = new Vector3S(i, i, i);
            }

            var sw = new System.Diagnostics.Stopwatch();
            sw.Start();

            var temp = Vector3S.zero;

            foreach (var item in list)
            {
                temp += item;
                temp -= item;
                temp += item * Vector3S.Dot(item, temp);

            }

            sw.Stop();
            Console.WriteLine(sw.Elapsed.ToString());
        }

        [TestMethod]
        public void Vector3Test()
        {
            var list = new Vector3[iterations];

            for (int i = 0; i < iterations; i++)
            {
                list[i] = new Vector3(i, i, i);
            }

            var sw = new System.Diagnostics.Stopwatch();
            sw.Start();

            var temp = Vector3.Zero;

            foreach (var item in list)
            {
                temp += item;
                temp -= item;
                temp += item * Vector3.Dot(item, temp);

            }

            sw.Stop();
            Console.WriteLine(sw.Elapsed.ToString());
        }

        [TestMethod]
        public void DotTest()
        {
            var list = new Vector3S[iterations];

            for (int i = 0; i < iterations; i++)
            {
                list[i] = new Vector3S(i, i, i);
            }

            var sw = new System.Diagnostics.Stopwatch();
            sw.Start();

            var temp = f32.zero;

            foreach (var item in list)
            {
                temp += Vector3S.Dot(item, Vector3S.one);
            }

            sw.Stop();
            Console.WriteLine(sw.Elapsed.ToString());
        }

        [TestMethod]
        public void DotRawTest()
        {
            var list = new Vector3S[iterations];

            for (int i = 0; i < iterations; i++)
            {
                list[i] = new Vector3S(i, i, i);
            }

            var sw = new System.Diagnostics.Stopwatch();
            sw.Start();

            var temp = 0L;

            foreach (var item in list)
            {
                temp += Vector3S.RawDot(item, Vector3S.one);
            }

            sw.Stop();
            Console.WriteLine(sw.Elapsed.ToString());
        }
    }
}

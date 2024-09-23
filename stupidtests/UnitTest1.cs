using stupid.Maths;
using System.Diagnostics;

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

            var sw = new System.Diagnostics.Stopwatch();
            sw.Start();

            var temp = new f32();

            for (int i = 0; i < iterations; i++)
            {
                temp += list[i];
                temp -= list[i];
                temp *= list[i];
                temp /= list[i];

                list[i] = temp;
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

            for (int i = 0; i < iterations; i++)
            {
                temp += list[i];
                temp -= list[i];
                temp *= list[i];
                temp /= list[i];

                list[i] = temp;
            }

            sw.Stop();
            Console.WriteLine(sw.Elapsed.ToString());
        }
    }
}

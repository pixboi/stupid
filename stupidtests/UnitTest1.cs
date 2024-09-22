using stupid.Maths;
using System.Diagnostics;

namespace stupidtests
{
    [TestClass]
    public class UnitTest1
    {
        public int iterations = 10000000;

        [TestMethod]
        public void f32Test()
        {
            var list = new f32[iterations];

            for (int i = 0; i < iterations; i++)
            {
                list[i] = new f32(i);
            }

            var temp = new f32();

            for (int i = 0; i < iterations; i++)
            {
                temp += list[i];

                list[i] = temp;
            }
        }

        [TestMethod]
        public void LongTest()
        {
            var list = new long[iterations];

            for (int i = 0; i < iterations; i++)
            {
                list[i] = i;
            }

            var temp = 0L;

            for (int i = 0; i < iterations; i++)
            {
                temp += list[i];

                list[i] = temp;
            }
        }

        [TestMethod]
        public void IntTest()
        {
            var list = new int[iterations];

            for (int i = 0; i < iterations; i++)
            {
                list[i] = i;
            }

            var temp = 0;

            for (int i = 0; i < iterations; i++)
            {
                temp += list[i];

                list[i] = temp;
            }
        }
    }
}

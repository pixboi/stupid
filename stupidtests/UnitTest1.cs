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
                list[i] = new f32(i);
            }

            var temp = new f32();
           
            for (int i = 0; i < iterations; i++)
            {
                temp += list[i];
                temp *= 
                list[i] = temp;
            }
        }
    }
}

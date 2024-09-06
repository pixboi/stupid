using stupid;
using stupid.Colliders;
using stupid.Maths;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace stupidtests
{
    [TestClass]
    public class UnitTest1
    {

        public void TestMethod1()
        {
            int iterations = 100;

            var random = new Random(1);
            var settings = WorldSettings.Default();
            var dt = (f32)0.02;
            var world = new World(settings, iterations);

            var ground = new BoxColliderS(new Vector3S(256, 2, 256));
            var groundTransform = new TransformS(new Vector3S(0, -2, 0), QuaternionS.identity, new Vector3S(256, 1, 256));
            var g = new Collidable(-1, ground, groundTransform, false);
            world.AddCollidable(g);

            for (int i = 0; i < iterations; i++)
            {
                var randomVector = new Vector3S((f32)random.NextSingle(), (f32)random.NextSingle(), (f32)random.NextSingle());
                var transform = new TransformS(randomVector, QuaternionS.identity, Vector3S.one);
                transform.position = randomVector;
                var box = new BoxColliderS(Vector3S.one);
                var body = new RigidbodyS(-1, box, true, transform, Vector3S.zero, Vector3S.zero, f32.one, true, false);
                world.AddCollidable(body);
            }

            var sw = new Stopwatch();
            var times = new List<long>();

            for (int i = 0; i < iterations; i++)
            {
                sw.Restart();
                world.Simulate(dt);
                sw.Stop();
                times.Add(sw.ElapsedMilliseconds);
            }

            var avgTime = times.Average();
            var stdDevTime = Math.Sqrt(times.Average(v => Math.Pow(v - avgTime, 2)));

            Console.WriteLine($"Average simulation time per frame: {avgTime} ms");
            Console.WriteLine($"Standard deviation of simulation time: {stdDevTime} ms");
        }

        public int it = 10000000;
        [TestMethod]
        public void Vector3SAdd()
        {
            var v = Vector3S.zero;

            for (int i = 0; i < it; i++)
            {
                v += Vector3S.one;
                v -= Vector3S.left;
                v /= f32.two;
            }

            Console.WriteLine(v.ToString());

        }

        [TestMethod]
        public void Vector3SAddFast()
        {
            var v = Vector3S.zero;

            for (int i = 0; i < it; i++)
            {
                v.AddInPlace(Vector3S.one);
                v.SubtractInPlace(Vector3S.left);
                v.DivideInPlace(f32.two);
            }

            Console.WriteLine(v.ToString());
        }

        [TestMethod]
        public void Vector3SDot()
        {
            var v = Vector3S.zero;

            for (int i = 0; i < it; i++)
            {
            }

            Console.WriteLine(v.ToString());
        }

    }
}

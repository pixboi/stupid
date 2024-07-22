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
        [TestMethod]
        public void TestMethod1()
        {
            int iterations = 100;

            var random = new Random(1);
            var settings = new WorldSettings();
            var dt = (f32)0.02;
            var world = new World(settings, iterations);

            var ground = new BoxColliderS(new Vector3S(256, 2, 256));
            var groundTransform = new TransformS(Vector3S.zero, QuaternionS.identity);
            var g = new Collidable(-1, ground, false, groundTransform);
            world.AddCollidable(g);

            for (int i = 0; i < iterations; i++)
            {
                var transform = new TransformS();
                transform.position = Vector3S.one * (f32)random.NextSingle() * (f32)4;
                var box = new BoxColliderS(Vector3S.one);
                var body = new RigidbodyS(-1, box, true, transform);
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

            Assert.IsTrue(avgTime > 0, "Average time should be greater than zero.");
        }
    }
}

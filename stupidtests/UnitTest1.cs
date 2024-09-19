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


        public struct HugeStruct
        {
            public f32 a, b, c, d, f, g, h, k, l, m;
            public f32 sum;
        }

        public class HugeClass
        {
            public f32 a, b, c, d, f, g, h, k, l, m;
            public f32 sum;
        }

        int cap = 1000000;

        [TestMethod]
        public void IterationTest()
        {
            var manifolds = new List<ContactManifoldS>(cap);
            var contacts = new List<ContactS>(cap);

            var sw = new Stopwatch();

            // Normal iteration with structs
            sw.Start();
            foreach (var m in manifolds)
            {

            }
            sw.Stop();
            Console.WriteLine($"Struct iteration time: {sw.ElapsedMilliseconds} ms");

            // Reset stopwatch
            sw.Reset();

            // Normal iteration with classes
            sw.Start();
            foreach (var c in contacts)
            {

            }
            sw.Stop();
            Console.WriteLine($"Class iteration time: {sw.ElapsedMilliseconds} ms");
        }


    }
}

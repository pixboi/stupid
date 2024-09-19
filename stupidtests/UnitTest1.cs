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
            var structs = new List<HugeStruct>(cap);
            var classes = new List<HugeClass>(cap);

            // Initialize lists with dummy data
            for (int i = 0; i < cap; i++)
            {
                f32 index = f32.FromFloat(1.0f);
                structs.Add(new HugeStruct { a = index, b = index, c = index, d = index, f = index, g = index, h = index, k = index, l = index, m = index });
                classes.Add(new HugeClass { a = index, b = index, c = index, d = index, f = index, g = index, h = index, k = index, l = index, m = index });
            }

            var sw = new Stopwatch();

            // Normal iteration with structs
            sw.Start();
            for (int i = 0; i < structs.Count; i++)
            {
                var s = structs[i];
                s.sum = s.a + s.b + s.c + s.d + s.f + s.g + s.h + s.k + s.l + s.m;
                structs[i] = s; // Retain the updated struct in the list
            }
            sw.Stop();
            Console.WriteLine($"Struct iteration time: {sw.ElapsedMilliseconds} ms");

            // Reset stopwatch
            sw.Reset();

            // Normal iteration with classes
            sw.Start();
            foreach (var c in classes)
            {
                c.sum = c.a + c.b + c.c + c.d + c.f + c.g + c.h + c.k + c.l + c.m;
            }
            sw.Stop();
            Console.WriteLine($"Class iteration time: {sw.ElapsedMilliseconds} ms");

            // Reset stopwatch
            sw.Reset();

            // Super iteration with structs using unsafe pointers
            sw.Start();
            unsafe
            {
                fixed (HugeStruct* ptr = structs.ToArray())
                {
                    HugeStruct* current = ptr;
                    HugeStruct* end = ptr + structs.Count;
                    while (current < end)
                    {
                        current->sum = current->a + current->b + current->c + current->d + current->f + current->g + current->h + current->k + current->l + current->m;
                        current++;
                    }
                }
            }
            sw.Stop();
            Console.WriteLine($"Unsafe struct iteration time: {sw.ElapsedMilliseconds} ms");
        }


    }
}

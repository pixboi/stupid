using stupid;
using stupid.Colliders;
using stupid.Maths;
using System;
using System.Collections.Generic;
using System.Collections.Immutable;
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

        public struct SmallStruct
        {
            public float x, y, z, w; // 16 bytes in total
        }

        public struct LargeStruct
        {
            // 128 floats, 512 bytes in total
            public float a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16,
                         a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32,
                         a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48,
                         a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64,
                         a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80,
                         a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96,
                         a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112,
                         a113, a114, a115, a116, a117, a118, a119, a120, a121, a122, a123, a124, a125, a126, a127, a128;
        }

        [TestMethod]
        public void SmallStructVsLargeStructIteration()
        {
            int count = 1000000; // Number of elements in the collection
            var smallStructs = new List<SmallStruct>(count);
            var largeStructs = new List<LargeStruct>(count);

            // Initialize lists with dummy data
            for (int i = 0; i < count; i++)
            {
                smallStructs.Add(new SmallStruct { x = 1f, y = 2f, z = 3f, w = 4f });
                largeStructs.Add(new LargeStruct { a1 = 1f, a2 = 2f, a3 = 3f, a4 = 4f });
            }

            var sw = new Stopwatch();

            // Iteration over SmallStruct
            sw.Start();
            float sumSmall = 0;
            foreach (var s in smallStructs)
            {
                sumSmall += s.x + s.y + s.z + s.w;
            }
            sw.Stop();
            Console.WriteLine($"SmallStruct iteration time: {sw.ElapsedMilliseconds} ms");

            // Reset stopwatch
            sw.Reset();

            // Iteration over LargeStruct
            sw.Start();
            float sumLarge = 0;
            foreach (var l in largeStructs)
            {
                sumLarge += l.a1 + l.a2 + l.a3 + l.a4; // Only summing a few fields to simulate some access
            }
            sw.Stop();
            Console.WriteLine($"LargeStruct iteration time: {sw.ElapsedMilliseconds} ms");
        }


    }
}

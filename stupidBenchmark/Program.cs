using System;
using BenchmarkDotNet.Attributes; // Import BenchmarkDotNet for benchmarking
using BenchmarkDotNet.Running;
using Perfolizer.Mathematics.Common;
using stupid; // Assuming these are your namespaces for physics, math, and other utilities
using stupid.Colliders;
using stupid.Maths;

public class VectorBenchmark
{
    public World world;
    f32 delta;

    [GlobalSetup]
    public void Setup()
    {
        var settings = WorldSettings.Default(); // Initialize world settings with default values
        this.delta = (f32)0.01f; // Set a fixed delta time for the simulation using f32 custom constructor

        var list = new List<Collidable>();

        // Define the world boundary as a large static box
        var boxCol = new BoxColliderS(new Vector3S(32, 2, 32));
        var worldBox = new Collidable(
            -1,
            boxCol,
            new TransformS(new Vector3S(16, -1, 16), QuaternionS.identity, Vector3S.one),
            false // Static collidable (not dynamic)
        );

        list.Add(worldBox);
        // Add the world boundary first to ensure it is registered before other collidables

        // Define a sphere collider
        var sphereCol = new SphereColliderS(f32.half);

        // Create a 3D grid of spheres, with each sphere having a spacing of 1 unit apart
        int gridSize = 8;  // Define the number of spheres along each axis
        float spacing = 1f; // Spacing between spheres to avoid overlap

        for (int x = 0; x < gridSize; x++)
        {
            for (int y = 0; y < gridSize; y++)
            {
                for (int z = 0; z < gridSize; z++)
                {
                    // Calculate the position in the 3D grid
                    Vector3S position = new Vector3S(
                     (x * spacing),
                      (y * spacing + 3), // Start above the ground to avoid intersection with the world box
                       (z * spacing)
                    );

                    // Create a dynamic sphere at the calculated position
                    var sphere = new Collidable(
                        -1,
                        sphereCol,
                        new TransformS(position, QuaternionS.identity, Vector3S.one),
                        true, // Make it dynamic
                        Vector3S.zero,
                        Vector3S.zero,
                        f32.one,  // Mass of 1
                        true, // Use gravity
                        false // Not kinematic
                    );

                    // Add the sphere to the world
                    list.Add(sphere);
                }
            }
        }

        this.world = new World(settings, list);
    }

    // Benchmark the simulation loop
    [Benchmark]
    public void SimulationLoop()
    {
        for (int i = 0; i < 1000; i++) // Run the simulation for 1000 steps
        {
            this.world.Simulate(delta); // Simulate with the defined delta time
        }
    }
}

public class Program
{
    public static void Main(string[] args)
    {
        //  var summary = BenchmarkRunner.Run<VectorBenchmark>();
        var settings = WorldSettings.Default(); // Initialize world settings with default values
        var delta = (f32)0.01f; // Set a fixed delta time for the simulation using f32 custom constructor

        var list = new List<Collidable>();

        // Define the world boundary as a large static box
        var boxCol = new BoxColliderS(new Vector3S(32, 2, 32));
        var smallBox = new BoxColliderS(Vector3S.one);

        var worldBox = new Collidable(
            -1,
            boxCol,
            new TransformS(new Vector3S(16, -1, 16), QuaternionS.identity, Vector3S.one),
            false // Static collidable (not dynamic)
        );

        list.Add(worldBox);
        // Add the world boundary first to ensure it is registered before other collidables

        // Define a sphere collider
        var sphereCol = new SphereColliderS(f32.half);

        // Create a 3D grid of spheres, with each sphere having a spacing of 1 unit apart
        int gridSize = 8;  // Define the number of spheres along each axis
        float spacing = 1f; // Spacing between spheres to avoid overlap

        for (int x = 0; x < gridSize; x++)
        {
            for (int y = 0; y < gridSize; y++)
            {
                for (int z = 0; z < gridSize; z++)
                {
                    // Calculate the position in the 3D grid
                    Vector3S position = new Vector3S(
                     (x * spacing),
                      (y * spacing + 3), // Start above the ground to avoid intersection with the world box
                       (z * spacing)
                    );

                    // Create a dynamic sphere at the calculated position
                    var sphere = new Collidable(
                        -1,
                        sphereCol,
                        new TransformS(position, QuaternionS.identity, Vector3S.one),
                        true, // Make it dynamic
                        Vector3S.zero,
                        Vector3S.zero,
                        f32.one,  // Mass of 1
                        true, // Use gravity
                        false // Not kinematic
                    );

                    // Add the sphere to the world
                    list.Add(sphere);
                }
            }
        }

        var world = new World(settings, list);

        for (int i = 0; i < 1000; i++) // Run the simulation for 1000 steps
        {
            world.Simulate(delta); // Simulate with the defined delta time
        }
    }
}

using System.Collections.Generic;
using System.Threading.Tasks;
using SoftFloat;

namespace stupid
{
    public interface IBroadphase
    {
        List<ContactPair> ComputePairs(List<Rigidbody> rigidbodies);
    }

    public class GridBasedBroadphase : IBroadphase
    {
        private int cellSize;
        private Dictionary<(int, int, int), CustomList<Rigidbody>> grid;
        private Dictionary<(int, int), bool> checkedPairs;
        private CustomList<ContactPair> pairs;
        private Stack<CustomList<Rigidbody>> listPool;
        private int maxRigidbodies;

        public GridBasedBroadphase(int cellSize, int initialCapacity = 100)
        {
            this.cellSize = cellSize;
            this.grid = new Dictionary<(int, int, int), CustomList<Rigidbody>>(initialCapacity);
            this.checkedPairs = new Dictionary<(int, int), bool>(initialCapacity);
            this.pairs = new CustomList<ContactPair>(initialCapacity);
            this.listPool = new Stack<CustomList<Rigidbody>>(initialCapacity);
            this.maxRigidbodies = initialCapacity;

            // Preallocate lists in pool
            for (int i = 0; i < initialCapacity; i++)
            {
                listPool.Push(new CustomList<Rigidbody>(initialCapacity));
            }
        }

        public List<ContactPair> ComputePairs(List<Rigidbody> rigidbodies)
        {
            // Adjust capacity if necessary
            if (rigidbodies.Count > maxRigidbodies)
            {
                maxRigidbodies = rigidbodies.Count;
                pairs = new CustomList<ContactPair>(maxRigidbodies);
                listPool.Clear();

                // Preallocate lists in pool
                for (int i = 0; i < maxRigidbodies; i++)
                {
                    listPool.Push(new CustomList<Rigidbody>(maxRigidbodies));
                }
            }

            // Clear previous state
            foreach (var cell in grid.Values)
            {
                cell.Clear();
                listPool.Push(cell);
            }
            grid.Clear();
            checkedPairs.Clear();
            pairs.Clear();

            // Place each rigidbody in the grid
            foreach (var body in rigidbodies)
            {
                var bounds = body.collider.GetBounds();
                var minCell = GetCell(bounds.Min);
                var maxCell = GetCell(bounds.Max);

                for (int x = minCell.x; x <= maxCell.x; x++)
                {
                    for (int y = minCell.y; y <= maxCell.y; y++)
                    {
                        for (int z = minCell.z; z <= maxCell.z; z++)
                        {
                            var cell = (x, y, z);
                            if (!grid.TryGetValue(cell, out var cellBodies))
                            {
                                cellBodies = listPool.Count > 0 ? listPool.Pop() : new CustomList<Rigidbody>(maxRigidbodies);
                                grid[cell] = cellBodies;
                            }
                            cellBodies.Add(body);
                        }
                    }
                }
            }

            // Check for collisions within the same cell
            foreach (var cell in grid.Values)
            {
                for (int i = 0; i < cell.Count; i++)
                {
                    for (int j = i + 1; j < cell.Count; j++)
                    {
                        var a = cell[i];
                        var b = cell[j];

                        int idA = a.index;
                        int idB = b.index;
                        if (idA > idB)
                        {
                            (idA, idB) = (idB, idA);
                        }

                        if (checkedPairs.ContainsKey((idA, idB)))
                            continue;

                        var aBounds = a.collider.GetBounds();
                        var bBounds = b.collider.GetBounds();

                        if (aBounds.Intersects(bBounds))
                        {
                            pairs.Add(new ContactPair { bodyA = a, bodyB = b });
                        }

                        checkedPairs[(idA, idB)] = true;
                    }
                }

                // Reuse the list for the next cell
                ReturnReusableList(cell);
            }

            // Convert pairs to a List<ContactPair> to return
            List<ContactPair> result = new List<ContactPair>(pairs.Count);
            for (int i = 0; i < pairs.Count; i++)
            {
                result.Add(pairs[i]);
            }

            return result;
        }

        private (int x, int y, int z) GetCell(Vector3S position)
        {
            return (
                ((int)position.x / cellSize),
                ((int)position.y / cellSize),
               ((int)position.z / cellSize)
            );
        }

        private void ReturnReusableList(CustomList<Rigidbody> list)
        {
            list.Clear();
            listPool.Push(list);
        }
    }
}

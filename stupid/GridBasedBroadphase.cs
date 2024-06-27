using System.Collections.Generic;
using SoftFloat;

namespace stupid
{
    public interface IBroadphase
    {
        List<ContactPair> ComputePairs(List<Rigidbody> rigidbodies);
    }

    public class GridBasedBroadphase : IBroadphase
    {
        private sfloat cellSize;
        private Dictionary<Vector3S, List<Rigidbody>> grid;
        private Dictionary<(int, int), bool> checkedPairs;
        private List<ContactPair> pairs;
        private Stack<List<Rigidbody>> listPool;
        private Vector3S minCell;
        private Vector3S maxCell;

        public GridBasedBroadphase(sfloat cellSize)
        {
            this.cellSize = cellSize;
            this.grid = new Dictionary<Vector3S, List<Rigidbody>>();
            this.checkedPairs = new Dictionary<(int, int), bool>();
            this.pairs = new List<ContactPair>();
            this.listPool = new Stack<List<Rigidbody>>();
            this.minCell = new Vector3S();
            this.maxCell = new Vector3S();
        }

        public List<ContactPair> ComputePairs(List<Rigidbody> rigidbodies)
        {
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
                var bounds = body.collider.GetBounds(body.position);
                GetCell(bounds.Min, ref minCell);
                GetCell(bounds.Max, ref maxCell);

                for (int x = (int)minCell.x; x <= (int)maxCell.x; x++)
                {
                    for (int y = (int)minCell.y; y <= (int)maxCell.y; y++)
                    {
                        for (int z = (int)minCell.z; z <= (int)maxCell.z; z++)
                        {
                            var cell = new Vector3S((sfloat)x, (sfloat)y, (sfloat)z);
                            if (!grid.TryGetValue(cell, out var cellBodies))
                            {
                                cellBodies = listPool.Count > 0 ? listPool.Pop() : new List<Rigidbody>();
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

                        int idA = a.GetHashCode();
                        int idB = b.GetHashCode();
                        if (idA > idB)
                        {
                            (idA, idB) = (idB, idA);
                        }

                        if (checkedPairs.ContainsKey((idA, idB)))
                            continue;

                        var aBounds = a.collider.GetBounds(a.position);
                        var bBounds = b.collider.GetBounds(b.position);

                        if (aBounds.Intersects(bBounds))
                        {
                            pairs.Add(new ContactPair { bodyA = a, bodyB = b });
                        }

                        checkedPairs[(idA, idB)] = true;
                    }
                }
            }

            return pairs;
        }

        private void GetCell(Vector3S position, ref Vector3S cell)
        {
            cell.x = MathS.Floor(position.x / cellSize);
            cell.y = MathS.Floor(position.y / cellSize);
            cell.z = MathS.Floor(position.z / cellSize);
        }
    }
}

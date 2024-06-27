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
        private HashSet<(Rigidbody, Rigidbody)> checkedPairs;

        public GridBasedBroadphase(sfloat cellSize)
        {
            this.cellSize = cellSize;
            this.grid = new Dictionary<Vector3S, List<Rigidbody>>();
            this.checkedPairs = new HashSet<(Rigidbody, Rigidbody)>();
        }

        public List<ContactPair> ComputePairs(List<Rigidbody> rigidbodies)
        {
            grid.Clear();
            checkedPairs.Clear();
            var pairs = new List<ContactPair>();

            // Place each rigidbody in the grid
            foreach (var body in rigidbodies)
            {
                var bounds = body.collider.GetBounds(body.position);
                var minCell = GetCell(bounds.Min);
                var maxCell = GetCell(bounds.Max);

                for (int x = (int)minCell.x; x <= (int)maxCell.x; x++)
                {
                    for (int y = (int)minCell.y; y <= (int)maxCell.y; y++)
                    {
                        for (int z = (int)minCell.z; z <= (int)maxCell.z; z++)
                        {
                            var cell = new Vector3S((sfloat)x, (sfloat)y, (sfloat)z);
                            if (!grid.ContainsKey(cell))
                            {
                                grid[cell] = new List<Rigidbody>();
                            }
                            grid[cell].Add(body);
                        }
                    }
                }
            }

            // Check for collisions within the same cell
            foreach (var cell in grid)
            {
                var cellBodies = cell.Value;
                for (int i = 0; i < cellBodies.Count; i++)
                {
                    for (int j = i + 1; j < cellBodies.Count; j++)
                    {
                        var a = cellBodies[i];
                        var b = cellBodies[j];

                        if (checkedPairs.Contains((a, b)) || checkedPairs.Contains((b, a)))
                            continue;

                        var aBounds = a.collider.GetBounds(a.position);
                        var bBounds = b.collider.GetBounds(b.position);

                        if (aBounds.Intersects(bBounds))
                        {
                            var cp = new ContactPair { bodyA = a, bodyB = b };
                            pairs.Add(cp);
                        }

                        checkedPairs.Add((a, b));
                    }
                }
            }

            return pairs;
        }

        private Vector3S GetCell(Vector3S position)
        {
            return new Vector3S(
                MathS.Floor(position.x / cellSize),
                MathS.Floor(position.y / cellSize),
                MathS.Floor(position.z / cellSize)
            );
        }
    }
}


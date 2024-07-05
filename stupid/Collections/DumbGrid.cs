using System;
using System.Collections.Generic;
using stupid.Maths;
using stupid.Colliders;
using SoftFloat;

namespace stupid.Collections
{
    public class DumbGrid<T>
    {
        public T[] Contents { get; private set; }

        public int Width { get; private set; }
        public int Height { get; private set; }
        public int Depth { get; private set; }
        public sfloat CellSize { get; private set; }

        public DumbGrid(int width, int height, int depth, sfloat cellSize)
        {
            Width = width;
            Height = height;
            Depth = depth;
            CellSize = cellSize;
            Contents = new T[width * height * depth];
        }

        public void Add(Vector3S position, T value)
        {
            int i = (int)(position.x / CellSize);
            int j = (int)(position.y / CellSize);
            int k = (int)(position.z / CellSize);

            if (IsValidIndex(i, j, k))
            {
                this[i, j, k] = value;
            }
        }

        public void Add(SBounds bounds, T value)
        {
            int minI = (int)libm.floorf(bounds.min.x / CellSize);
            int minJ = (int)libm.floorf(bounds.min.y / CellSize);
            int minK = (int)libm.floorf(bounds.min.z / CellSize);
            int maxI = (int)libm.ceilf(bounds.max.x / CellSize);
            int maxJ = (int)libm.ceilf(bounds.max.y / CellSize);
            int maxK = (int)libm.ceilf(bounds.max.z / CellSize);

            for (int i = minI; i <= maxI; i++)
            {
                for (int j = minJ; j <= maxJ; j++)
                {
                    for (int k = minK; k <= maxK; k++)
                    {
                        if (IsValidIndex(i, j, k))
                        {
                            this[i, j, k] = value;
                        }
                    }
                }
            }
        }

        private bool IsValidIndex(int i, int j, int k)
        {
            return i >= 0 && i < Width && j >= 0 && j < Height && k >= 0 && k < Depth;
        }

        private int GetIndex(int i, int j, int k)
        {
            if (!IsValidIndex(i, j, k))
            {
                return -1;
            }

            return (i * Height * Depth) + (j * Depth) + k;
        }

        public (int i, int j, int k) ReverseIndex(int index)
        {
            int i = index / (Height * Depth);
            int j = (index % (Height * Depth)) / Depth;
            int k = index % Depth;

            // Scale up the indices to account for the cell size
            i = (int)((sfloat)i * CellSize);
            j = (int)((sfloat)j * CellSize);
            k = (int)((sfloat)k * CellSize);

            return (i, j, k);
        }

        public T this[int i, int j, int k]
        {
            get
            {
                int index = GetIndex(i, j, k);
                return index == -1 ? default : Contents[index];
            }
            set
            {
                int index = GetIndex(i, j, k);
                if (index != -1)
                {
                    Contents[index] = value;
                }
            }
        }

        public IEnumerable<T> GetAllBuckets()
        {
            for (int i = 0; i < Width; i++)
            {
                for (int j = 0; j < Height; j++)
                {
                    for (int k = 0; k < Depth; k++)
                    {
                        yield return Contents[GetIndex(i, j, k)];
                    }
                }
            }
        }

        public void Clear()
        {
            Array.Clear(Contents, 0, Contents.Length);
        }
    }
}

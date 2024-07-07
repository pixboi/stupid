using System;
using System.Collections.Generic;
using stupid.Maths;
using stupid.Colliders;
using System.Runtime.CompilerServices;

namespace stupid.Collections
{
    public class DumbGrid<T>
    {
        public readonly T[] Contents;
        public readonly int Width;
        public readonly int Height;
        public readonly int Depth;
        public readonly f32 CellSize;
        public readonly f32 HalfSize;
        public readonly f32 _InvCellSize;
        public DumbGrid(int width, int height, int depth, f32 cellSize)
        {
            Width = width;
            Height = height;
            Depth = depth;
            CellSize = cellSize;
            Contents = new T[width * height * depth];

            HalfSize = cellSize * f32.half;
            _InvCellSize = f32.one / this.CellSize;
        }

        public void Add(Vector3S position, T value)
        {
            int i = (int)MathS.Floor((position.x + HalfSize) / CellSize);
            int j = (int)MathS.Floor((position.y + HalfSize) / CellSize);
            int k = (int)MathS.Floor((position.z + HalfSize) / CellSize);
            if (IsValidIndex(i, j, k))
            {
                this[i, j, k] = value;
            }
        }


        public void Add(SBounds bounds, T value)
        {
            // Precompute bounds adjusted for cell center
            f32 adjustedMinX = bounds.min.x + HalfSize;
            f32 adjustedMinY = bounds.min.y + HalfSize;
            f32 adjustedMinZ = bounds.min.z + HalfSize;
            f32 adjustedMaxX = bounds.max.x + HalfSize;
            f32 adjustedMaxY = bounds.max.y + HalfSize;
            f32 adjustedMaxZ = bounds.max.z + HalfSize;

            // Calculate the minimum and maximum indices for the bounds
            int minI = (int)MathS.Floor(adjustedMinX * _InvCellSize);
            int minJ = (int)MathS.Floor(adjustedMinY * _InvCellSize);
            int minK = (int)MathS.Floor(adjustedMinZ * _InvCellSize);
            int maxI = (int)MathS.Floor(adjustedMaxX * _InvCellSize);
            int maxJ = (int)MathS.Floor(adjustedMaxY * _InvCellSize);
            int maxK = (int)MathS.Floor(adjustedMaxZ * _InvCellSize);

            // Add the value to all cells covered by the bounds
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool IsValidIndex(int i, int j, int k)
        {
            return i >= 0 && i < Width && j >= 0 && j < Height && k >= 0 && k < Depth;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int GetIndex(int i, int j, int k)
        {
            if (!IsValidIndex(i, j, k))
            {
                return -1;
            }

            return (i * Height * Depth) + (j * Depth) + k;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public (int i, int j, int k) ReverseIndex(int index)
        {
            int i = index / (Height * Depth);
            int j = (index % (Height * Depth)) / Depth;
            int k = index % Depth;

            // Scale up the indices to account for the cell size
            i = (int)((f32)i * CellSize);
            j = (int)((f32)j * CellSize);
            k = (int)((f32)k * CellSize);

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

        public void Invalidate(T v)
        {
            for (int i = 0; i < Contents.Length; i++) Contents[i] = v;
        }
    }
}

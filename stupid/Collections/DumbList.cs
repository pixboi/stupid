using System;

namespace stupid.Collections
{
    public class DumbList<T> where T : struct
    {
        private T[] items;
        private int count;
        private const int DefaultCapacity = 16;

        public DumbList(int capacity = DefaultCapacity)
        {
            items = new T[capacity];
            count = 0;
        }

        public void Add(T item)
        {
            if (count < items.Length)
            {
                items[count++] = item;
            }
            else
            {
                throw new InvalidOperationException("Bucket is full");
            }
        }

        public void Clear()
        {
            count = 0;
        }

        public bool Remove(T item)
        {
            int index = Array.IndexOf(items, item, 0, count);
            if (index < 0)
            {
                return false;
            }

            for (int i = index; i < count - 1; i++)
            {
                items[i] = items[i + 1];
            }
            count--;
            items[count] = default(T); // Clear the last item
            return true;
        }

        public int Count => count;

        public T this[int index]
        {
            get
            {
                if (index < 0 || index >= count)
                {
                    throw new IndexOutOfRangeException();
                }
                return items[index];
            }
        }
    }
}

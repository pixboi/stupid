using System;

namespace stupid.Collections
{
    public class CustomList<T>
    {
        private T[] items;
        private int count;
        private const int DefaultCapacity = 4;

        public CustomList(int capacity = DefaultCapacity)
        {
            items = new T[capacity];
            count = 0;
        }

        public void Add(T item)
        {
            if (count >= items.Length)
            {
                Resize();
            }
            items[count++] = item;
        }

        public void Clear()
        {
            count = 0;
        }

        public int Count => count;
        public T this[int index] => items[index];

        private void Resize()
        {
            int newCapacity = items.Length > 0 ? items.Length * 3 / 2 : DefaultCapacity;
            T[] newItems = new T[newCapacity];
            items.AsSpan(0, count).CopyTo(newItems);
            items = newItems;
        }
    }
}

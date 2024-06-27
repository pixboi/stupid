using System;
using System.Collections.Generic;
using System.Text;

namespace stupid
{
    public class CustomList<T>
    {
        private T[] items;
        private int count;

        public CustomList(int capacity)
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
            T[] newItems = new T[items.Length * 2];
            System.Array.Copy(items, newItems, items.Length);
            items = newItems;
        }
    }

}

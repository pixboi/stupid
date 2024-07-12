using System;
using System.Collections;
using System.Collections.Generic;

public class DumbList<T> : IEnumerable<T>
{
    private T[] _items;
    private int _size;

    public DumbList(int capacity = 4)
    {
        _items = new T[capacity];
        _size = 0;
    }

    public int Count => _size;

    public T this[int index]
    {
        get
        {
            if (index < 0 || index >= _size) throw new ArgumentOutOfRangeException();
            return _items[index];
        }
        set
        {
            if (index < 0 || index >= _size) throw new ArgumentOutOfRangeException();
            _items[index] = value;
        }
    }

    public void Add(T item)
    {
        if (_size == _items.Length)
        {
            Resize(_items.Length * 2);
        }
        _items[_size++] = item;
    }

    public void Clear()
    {
        Array.Clear(_items, 0, _size);
        _size = 0;
    }

    public void RemoveAt(int index)
    {
        if (index < 0 || index >= _size) throw new ArgumentOutOfRangeException();
        _size--;
        if (index < _size)
        {
            Array.Copy(_items, index + 1, _items, index, _size - index);
        }
        _items[_size] = default;
    }

    public void Remove(T item)
    {
        int index = Array.IndexOf(_items, item, 0, _size);
        if (index >= 0)
        {
            RemoveAt(index);
        }
    }

    private void Resize(int newSize)
    {
        var newItems = new T[newSize];
        Array.Copy(_items, newItems, _size);
        _items = newItems;
    }

    public T[] ToArray()
    {
        var result = new T[_size];
        Array.Copy(_items, result, _size);
        return result;
    }

    public struct Enumerator : IEnumerator<T>
    {
        private readonly DumbList<T> _list;
        private int _index;
        private T _current;

        public Enumerator(DumbList<T> list)
        {
            _list = list;
            _index = 0;
            _current = default;
        }

        public T Current => _current;

        object IEnumerator.Current => Current;

        public bool MoveNext()
        {
            if (_index < _list._size)
            {
                _current = _list._items[_index];
                _index++;
                return true;
            }
            return false;
        }

        public void Reset()
        {
            _index = 0;
            _current = default;
        }

        public void Dispose()
        {
            // No resources to release
        }
    }

    public Enumerator GetEnumerator()
    {
        return new Enumerator(this);
    }

    IEnumerator<T> IEnumerable<T>.GetEnumerator()
    {
        return GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }
}

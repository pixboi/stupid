using stupid.Colliders;
using stupid.Constraints;
using System;

namespace stupid
{
    public class Collidable : IEquatable<Collidable?>
    {
        public int index;
        public void Register(int index) => this.index = index;
        public virtual bool isDynamic => false;
        public Shape collider { get; private set; }

        public TransformS transform;

        public PhysicsMaterialS material = PhysicsMaterialS.DEFAULT_MATERIAL;

        public Collidable(int index, Shape collider, TransformS transform)
        {
            this.index = index;
            this.collider = collider;
            this.transform = transform;

            collider?.Attach(this);
            material = PhysicsMaterialS.DEFAULT_MATERIAL;
        }

        public BoundsS _bounds;
        public BoundsS CalculateBounds()
        {
            _bounds = collider.GetBounds(transform);
            return _bounds;
        }

        public override bool Equals(object? obj)
        {
            return Equals(obj as Collidable);
        }

        public bool Equals(Collidable? other)
        {
            return !(other is null) &&
                   index == other.index;
        }

        public override int GetHashCode()
        {
            return index;
        }
    }
}

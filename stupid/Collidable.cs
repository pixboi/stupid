using stupid.Colliders;
using stupid.Maths;
using System;

namespace stupid
{
    public class Collidable : IEquatable<Collidable?>
    {
        public int index { get; private set; }
        public void Register(int index) => this.index = index;

        public bool isDynamic = false;
        public IShape collider { get; private set; }

        public TransformS transform = new TransformS(Vector3S.zero, QuaternionS.zero, Vector3S.one);
        public PhysicsMaterialS material = PhysicsMaterialS.DEFAULT_MATERIAL;

        public Collidable(int index, IShape collider, bool isDynamic = false, TransformS transform = default, PhysicsMaterialS material = default)
        {
            this.index = index;
            this.isDynamic = isDynamic;
            this.collider = collider;
            this.transform = transform;

            collider?.Attach(this);

            this.material = PhysicsMaterialS.DEFAULT_MATERIAL;
        }

        public BoundsS GetBounds() => this.collider.GetAABB();
        public BoundsS CalculateBounds() => this.collider.CalculateAABB(transform.position, transform.rotation);

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

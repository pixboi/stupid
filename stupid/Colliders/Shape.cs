using stupid.Constraints;
using stupid.Maths;

namespace stupid.Colliders
{
    public interface IShape
    {
        public Collidable GetCollidable { get; }
        public void Attach(in Collidable collidable);
        public BoundsS GetBounds(in TransformS t);
        public abstract int Intersects(Collidable other, ref ContactData[] contacts);
        public abstract Matrix3S CalculateInertiaTensor(in f32 mass);
        public abstract bool NeedsRotationUpdate { get; }
        public abstract void OnRotationUpdate();
    }
}

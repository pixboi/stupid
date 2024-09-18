using stupid.Maths;

namespace stupid.Colliders
{
    public abstract class Shape
    {
        public Collidable collidable;
        public void Attach(Collidable collidable)
        {
            this.collidable = collidable;
        }

        public abstract BoundsS GetBounds(TransformS t);
        public abstract int Intersects(Collidable other, ref ContactS[] contacts);

        public abstract Matrix3S CalculateInertiaTensor(in f32 mass);

        public abstract bool NeedsRotationUpdate { get; }

        public abstract void OnRotationUpdate();
    }
}

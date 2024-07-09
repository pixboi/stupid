using stupid.Maths;

namespace stupid.Colliders
{
    public interface IShape
    {
        void Attach(Collidable body);
        Collidable GetCollidable();
        BoundsS CalculateBounds(Vector3S position);
        BoundsS GetBounds();
        bool Intersects(Collidable other, out ContactS contact); // Add this method
        int Intersects(Collidable other, ref ContactS[] contactCache);
        Matrix3S CalculateInertiaTensor(f32 mass);
    }

    public abstract class BaseShape : IShape
    {
        protected Collidable attachedCollidable;
        public virtual Collidable GetCollidable() => attachedCollidable;
        public void Attach(Collidable c)
        {
            attachedCollidable = c;
            CalculateBounds(c.transform.position);
        }

        public abstract BoundsS CalculateBounds(Vector3S position);

        protected BoundsS _bounds;
        public virtual BoundsS GetBounds() => _bounds;
        public abstract bool Intersects(Collidable other, out ContactS contact);
        public abstract int Intersects(Collidable other, ref ContactS[] contactCache);

        public abstract Matrix3S CalculateInertiaTensor(f32 mass);

    }
}

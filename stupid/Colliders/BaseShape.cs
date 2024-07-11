using stupid.Maths;

namespace stupid.Colliders
{
    public interface IShape
    {
        void Attach(Collidable body);
        Collidable GetCollidable();
        BoundsS CalculateAABB(Vector3S position);
        BoundsS GetAABB();
        bool Intersects(Collidable other, out ContactS contact); // Add this method
        Matrix3S CalculateInertiaTensor(f32 mass);
    }

    public abstract class BaseShape : IShape
    {
        protected Collidable attachedCollidable;
        public virtual Collidable GetCollidable() => attachedCollidable;
        public void Attach(Collidable c)
        {
            attachedCollidable = c;
            CalculateAABB(c.transform.position);
        }

        public abstract BoundsS CalculateAABB(Vector3S position);

        protected BoundsS _bounds;
        public virtual BoundsS GetAABB() => _bounds;
        public abstract bool Intersects(Collidable other, out ContactS contact);

        public abstract Matrix3S CalculateInertiaTensor(f32 mass);

    }
}

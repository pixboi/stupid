using stupid.Maths;

namespace stupid.Colliders
{
    public interface IShape
    {
        void Attach(Collidable body);
        Collidable GetCollidable();
        BoundsS CalculateAABB(Vector3S position, QuaternionS rotation);
        BoundsS GetAABB();
        int Intersects(Collidable other, ref ContactS contact); // Add this method
        Matrix3S CalculateInertiaTensor(f32 mass);
        bool NeedsRotationUpdate { get; }
        void OnRotationUpdate();
    }

    public abstract class BaseShape : IShape
    {
        public Collidable collidable { get; private set; }
        public virtual bool NeedsRotationUpdate => false;
        public virtual void OnRotationUpdate() { return; }

        public virtual Collidable GetCollidable() => collidable;
        public void Attach(Collidable c)
        {
            collidable = c;
            CalculateAABB(c.transform.position, c.transform.rotation);

            OnRotationUpdate();
        }

        public abstract BoundsS CalculateAABB(Vector3S position, QuaternionS rotation);

        protected BoundsS _bounds;
        public virtual BoundsS GetAABB() => _bounds;
        public abstract int Intersects(Collidable other, ref ContactS contact);
        public abstract Matrix3S CalculateInertiaTensor(f32 mass);

    }
}

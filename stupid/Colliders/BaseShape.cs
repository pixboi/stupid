using stupid.Maths;

namespace stupid.Colliders
{
    public interface IShape
    {
        void Attach(Collidable body);
        Collidable GetCollidable();
        BoundsS CalculateAABB(Vector3S position, QuaternionS rotation);
        BoundsS GetAABB();
        bool Intersects(Collidable other, out ContactS contact); // Add this method
        Matrix3S CalculateInertiaTensor(f32 mass);
        bool NeedsRotationUpdate { get; }
        void OnRotationUpdate();
    }

    public abstract class BaseShape : IShape
    {
        public Collidable attachedCollidable { get; private set; }
        public virtual bool NeedsRotationUpdate => false;
        public virtual void OnRotationUpdate() { return; }

        public virtual Collidable GetCollidable() => attachedCollidable;
        public void Attach(Collidable c)
        {
            attachedCollidable = c;
            CalculateAABB(c.transform.position, c.transform.rotation);

            OnRotationUpdate();
        }

        public abstract BoundsS CalculateAABB(Vector3S position, QuaternionS rotation);

        protected BoundsS _bounds;
        public virtual BoundsS GetAABB() => _bounds;
        public abstract bool Intersects(Collidable other, out ContactS contact);
        public abstract Matrix3S CalculateInertiaTensor(f32 mass);

    }
}

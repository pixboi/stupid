using stupid.Colliders;

namespace stupid
{

    public class Collidable
    {
        public int index { get; private set; }
        public void Register(int index) => this.index = index;
        public override int GetHashCode() => index;

        public bool isDynamic = false;

        public IShape collider;

        public TransformS transform;

        public PhysicsMaterialS material;



        public Collidable(int index, IShape collider, bool isDynamic = false, TransformS transform = default, PhysicsMaterialS material = default)
        {
            this.index = index;
            this.isDynamic = isDynamic;
            this.collider = collider;
            this.transform = transform;

            if (collider != null)
            {
                collider.Attach(this);
            }

            this.material = PhysicsMaterialS.DEFAULT_MATERIAL;
        }

        public BoundsS GetBounds() => this.collider.GetBounds();
        public BoundsS CalculateBounds() => this.collider.CalculateBounds(transform.position);
    }
}

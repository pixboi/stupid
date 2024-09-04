using stupid.Maths;
using stupid;

public struct ContactS
{
    public readonly Vector3S point, normal, tangent, localAnchorA, localAnchorB;
    public readonly f32 normalMass, tangentMass;
    public readonly f32 penetrationDepth;
    public readonly int featureId;

    public f32 accumulatedImpulse, accumulatedFriction;

    public ContactS(Vector3S point, Vector3S normal, f32 penetrationDepth, Collidable a, Collidable b, int featureId = 0)
    {
        // Contact point on A, normal points towards B
        this.point = point;
        this.normal = normal;
        this.penetrationDepth = penetrationDepth;
        this.featureId = featureId;

        this.localAnchorA = this.point - a.transform.position;
        this.localAnchorB = this.point - b.transform.position;

        this.tangent = Vector3S.zero;
        this.normalMass = f32.zero;
        this.tangentMass = f32.zero;
        this.accumulatedFriction = f32.zero;
        this.accumulatedImpulse = f32.zero;

        if (a.isDynamic || b.isDynamic)
        {
            var ab = (RigidbodyS)a;
            var bb = b.isDynamic ? (RigidbodyS)b : null;

            this.normalMass = CalculateMassNormal(ab, bb);
            CalculateTangentAndMass(ab, bb, out var tangent, out var tmass);

            this.tangent = tangent;
            this.tangentMass = tmass;
        }
    }

    private f32 CalculateMassNormal(RigidbodyS ab, RigidbodyS bb)
    {
        f32 invMassA = ab.inverseMass;
        f32 invMassB = bb != null ? bb.inverseMass : f32.zero;

        f32 effectiveMass = invMassA + invMassB;

        Vector3S raCrossNormal = Vector3S.Cross(this.localAnchorA, this.normal);
        f32 angularMassA = Vector3S.Dot(raCrossNormal, ab.tensor.inertiaWorld * raCrossNormal);
        effectiveMass += angularMassA;

        if (bb != null)
        {
            Vector3S rbCrossNormal = Vector3S.Cross(this.localAnchorB, this.normal);
            f32 angularMassB = Vector3S.Dot(rbCrossNormal, bb.tensor.inertiaWorld * rbCrossNormal);
            effectiveMass += angularMassB;
        }

        return effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;
    }

    public void CalculateTangentAndMass(in RigidbodyS a, in RigidbodyS b, out Vector3S tangent, out f32 mass)
    {
        var contactVelocity = CalculateContactVelocity(a, b);
        Vector3S normalVelocity = this.normal * Vector3S.Dot(contactVelocity, this.normal);
        Vector3S tangentialVelocity = contactVelocity - normalVelocity;
        tangent = tangentialVelocity.Normalize();

        mass = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(this.localAnchorA, tangent), this.localAnchorA), tangent);

        if (b != null)
        {
            mass += b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * Vector3S.Cross(this.localAnchorB, tangent), this.localAnchorB), tangent);
        }

        mass = f32.one / mass;

    }


    public void WarmStart(in RigidbodyS a, in Collidable b)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        // Reapply the accumulated impulses
        var normalImpulse = (this.normal * this.accumulatedImpulse);
        var tangentImpulse = (this.tangent * this.accumulatedFriction);

        Vector3S warmImpulse = normalImpulse + tangentImpulse;

        ApplyImpulse(a, bb, warmImpulse);
    }

    public void SolveImpulse(in RigidbodyS a, in Collidable b, in f32 inverseDt, in WorldSettings settings, bool bias = true)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        var baum = f32.zero;

        if (this.penetrationDepth > f32.zero)
        {
            baum = this.penetrationDepth * inverseDt;
        }
        else if (bias)
        {
            var separation = CalculateSeparation(a.transform, b.transform, settings.DefaultContactOffset);
            baum = MathS.Max(settings.Baumgartner * separation * inverseDt, -settings.DefaultMaxDepenetrationVelocity);
        }

        var contactVelocity = CalculateContactVelocity(a, bb);
        var vn = Vector3S.Dot(contactVelocity, this.normal);
        //if (vn > f32.zero) return;

        var impulse = -this.normalMass * (vn + baum);
        var newImpulse = MathS.Max(impulse + this.accumulatedImpulse, f32.zero);
        impulse = newImpulse - this.accumulatedImpulse;
        this.accumulatedImpulse = newImpulse;

        var normalImpulse = this.normal * impulse;
        ApplyImpulse(a, bb, normalImpulse);
    }

    public void SolveFriction(in RigidbodyS a, in Collidable b, in f32 friction)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        var contactVelocity = CalculateContactVelocity(a, bb);

        // Calculate tangential velocity and the corresponding impulse
        //CalculateTangentialImpulse(a, bb, out var tangent, out var lambda);

        var vt = Vector3S.Dot(contactVelocity, this.tangent);
        var incrementalFriction = -this.tangentMass * vt;

        var maxFric = this.accumulatedImpulse * friction;
        var newImpulse = MathS.Clamp(this.accumulatedFriction + incrementalFriction, -maxFric, maxFric);
        incrementalFriction = newImpulse - this.accumulatedFriction;
        this.accumulatedFriction = newImpulse;

        ApplyImpulse(a, bb, this.tangent * incrementalFriction);
    }

    public Vector3S CalculateContactVelocity(in RigidbodyS a, in RigidbodyS bb)
    {
        var av = a.velocity + Vector3S.Cross(a.angularVelocity, this.localAnchorA);
        var bv = bb != null ? bb.velocity + Vector3S.Cross(bb.angularVelocity, this.localAnchorB) : Vector3S.zero;
        return bv - av;
    }

    public void ApplyImpulse(in RigidbodyS a, in RigidbodyS bb, in Vector3S impulse)
    {
        a.velocity -= impulse * a.inverseMass; // A moves away
        a.angularVelocity -= a.tensor.inertiaWorld * Vector3S.Cross(this.localAnchorA, impulse);

        if (bb != null)
        {
            bb.velocity += impulse * bb.inverseMass; // B moves along normal
            bb.angularVelocity += bb.tensor.inertiaWorld * Vector3S.Cross(this.localAnchorB, impulse);
        }
    }

    private f32 CalculateSeparation(in TransformS a, in TransformS b, in f32 slop)
    {
        Vector3S worldPointA = (a.position + a.deltaPosition) + this.localAnchorA;
        Vector3S worldPointB = (b.position + b.deltaPosition) + this.localAnchorB;
        f32 separation = Vector3S.Dot(worldPointB - worldPointA, this.normal) + this.penetrationDepth;
        return MathS.Min(f32.zero, separation + slop);
    }
}

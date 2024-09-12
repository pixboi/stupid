using stupid.Maths;
using stupid;
using System.Runtime.CompilerServices;
using System;

public struct ContactS
{
    public readonly Vector3S point, normal, tangent, localAnchorA, localAnchorB;
    public readonly f32 normalMass, tangentMass, twistMass, penetrationDepth;
    public readonly byte featureId;

    public Vector3S ra, rb;
    public f32 accumulatedImpulse, accumulatedFriction, accumulatedTwist;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ContactS(Vector3S point, Vector3S normal, f32 penetrationDepth, in Collidable a, in Collidable b, byte featureId = byte.MaxValue)
    {
        // Contact point on A, normal points towards B
        this.point = point;
        this.normal = normal;
        this.penetrationDepth = penetrationDepth;
        this.featureId = featureId;

        //Then we transform rb and ra every iteratation into world directions!
        //We dont need to transform the initial
        this.ra = this.point - a.transform.position;
        this.rb = this.point - b.transform.position;

        //This needs to be rechecked, we must compute this dir as a local dir
        this.localAnchorA = a.transform.ToLocalPoint(this.point);
        this.localAnchorB = b.transform.ToLocalPoint(this.point);

        this.normalMass = f32.zero;
        this.tangent = Vector3S.zero;
        this.tangentMass = f32.zero;
        this.twistMass = f32.zero;
        this.accumulatedFriction = f32.zero;
        this.accumulatedImpulse = f32.zero;
        this.accumulatedTwist = f32.zero;

        if (a.isDynamic || b.isDynamic)
        {
            var ab = (RigidbodyS)a;
            var bb = b.isDynamic ? (RigidbodyS)b : null;

            this.normalMass = CalculateMassNormal(ab, bb);

            CalculateTangentAndMass(ab, bb, out this.tangent, out this.tangentMass);
            this.twistMass = CalculateTwistMass(ab, bb);
        }
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void WarmStart(in RigidbodyS a, in Collidable b)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        /*
        // Reapply the accumulated impulses
        var normalImpulse = (this.normal * this.accumulatedImpulse);
        var tangentImpulse = (this.tangent * this.accumulatedFriction);

        Vector3S warmImpulse = normalImpulse + tangentImpulse;
        ApplyImpulse(a, bb, warmImpulse);

        ApplyTwistImpulse(a, bb, this.accumulatedTwist);
        */

        var ni = this.normal;
        ni.Multiply(this.accumulatedImpulse);

        var ti = this.tangent;
        ti.Multiply(this.accumulatedFriction);

        ni.Add(ti);

        ApplyImpulse(a, bb, ni);
        ApplyTwistImpulse(a, bb, this.accumulatedTwist);

    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    f32 CalculateMassNormal(RigidbodyS ab, RigidbodyS bb)
    {
        f32 invMassA = ab.inverseMass;
        f32 invMassB = bb != null ? bb.inverseMass : f32.zero;

        f32 effectiveMass = invMassA + invMassB;

        Vector3S raCrossNormal = Vector3S.Cross(this.ra, this.normal);
        f32 angularMassA = Vector3S.Dot(raCrossNormal, ab.tensor.inertiaWorld * raCrossNormal);
        effectiveMass += angularMassA;

        if (bb != null)
        {
            Vector3S rbCrossNormal = Vector3S.Cross(this.rb, this.normal);
            f32 angularMassB = Vector3S.Dot(rbCrossNormal, bb.tensor.inertiaWorld * rbCrossNormal);
            effectiveMass += angularMassB;
        }

        return effectiveMass > f32.zero ? f32.one / effectiveMass : f32.zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void CalculateTangentAndMass(in RigidbodyS a, in RigidbodyS b, out Vector3S t1, out f32 m1)
    {
        // Calculate relative velocity at the contact point
        var contactVelocity = CalculateContactVelocity(a, b);

        // Project the contact velocity onto the plane perpendicular to the normal (get the tangential velocity)
        Vector3S normalVelocity = this.normal * Vector3S.Dot(contactVelocity, this.normal);
        Vector3S tangentialVelocity = contactVelocity - normalVelocity;
        t1 = tangentialVelocity.Normalize();

        // Calculate effective mass along the first tangent (t1)
        m1 = a.inverseMass + Vector3S.Dot(Vector3S.Cross(a.tensor.inertiaWorld * Vector3S.Cross(this.ra, t1), this.ra), t1);
        if (b != null) m1 += b.inverseMass + Vector3S.Dot(Vector3S.Cross(b.tensor.inertiaWorld * Vector3S.Cross(this.rb, t1), this.rb), t1);

        m1 = m1 > f32.zero ? f32.one / m1 : f32.zero;  // Invert the mass to get the effective mass
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SolveImpulse(in RigidbodyS a, in Collidable b, in f32 inverseDt, in WorldSettings settings, bool useBias = true)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;
        var bias = f32.zero;

        var separation = CalculateSeparation(a.transform, b.transform, settings.DefaultContactOffset);

        if (separation > f32.zero)
        {
            bias = this.penetrationDepth * inverseDt;
        }
        else if (useBias)
        {
            //var separation = CalculateSeparation(a.transform, b.transform, settings.DefaultContactOffset);
            bias = MathS.Max(settings.Baumgartner * separation * inverseDt, -settings.DefaultMaxDepenetrationVelocity);
        }

        var contactVelocity = CalculateContactVelocity(a, bb);
        var vn = Vector3S.Dot(contactVelocity, this.normal);

        var impulse = -this.normalMass * (vn + bias);
        var newImpulse = MathS.Max(impulse + this.accumulatedImpulse, f32.zero);
        impulse = newImpulse - this.accumulatedImpulse;
        this.accumulatedImpulse = newImpulse;

        var normalImpulse = this.normal;
        normalImpulse.Multiply(impulse);

        ApplyImpulse(a, bb, normalImpulse);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SolveFriction(in RigidbodyS a, in Collidable b, in f32 friction)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        var contactVelocity = CalculateContactVelocity(a, bb);

        var vt = Vector3S.Dot(contactVelocity, this.tangent);
        var incrementalFriction = -this.tangentMass * vt;

        var couloumbMax = this.accumulatedImpulse * friction;
        var newImpulse = MathS.Clamp(this.accumulatedFriction + incrementalFriction, -couloumbMax, couloumbMax);
        incrementalFriction = newImpulse - this.accumulatedFriction;
        this.accumulatedFriction = newImpulse;

        var tangentImpulse = this.tangent;
        tangentImpulse.Multiply(incrementalFriction);

        ApplyImpulse(a, bb, tangentImpulse);
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    Vector3S CalculateContactVelocity(in RigidbodyS a, in RigidbodyS bb)
    {
        /*
    var av = a.velocity + Vector3S.Cross(a.angularVelocity, this.ra);
    var bv = bb != null ? bb.velocity + Vector3S.Cross(bb.angularVelocity, this.rb) : Vector3S.zero;
    return bv - av;
*/

        var ai = a.velocity;
        var ac = a.angularVelocity;
        ac.CrossInPlace(this.ra);// Vector3S.Cross(a.angularVelocity, this.ra);
        ai.Add(ac);

        var bi = Vector3S.zero;
        if (bb != null)
        {
            bi = bb.velocity;
            var bc = bb.angularVelocity; //Vector3S.Cross(bb.angularVelocity, this.rb);
            bc.CrossInPlace(this.rb);
            bi.Add(bc);
        }

        bi.Subtract(ai);
        return bi;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void ApplyImpulse(in RigidbodyS a, in RigidbodyS bb, in Vector3S impulse)
    {
        /*
        a.velocity -= impulse * a.inverseMass; // A moves away

        var ab = Vector3S.Cross(this.ra, impulse);
        ab.Multiply(a.tensor.inertiaWorld);
        a.angularVelocity -= ab;

        if (bb != null)
        {
            bb.velocity += impulse * bb.inverseMass; // B moves along normal
            var cb = Vector3S.Cross(this.rb, impulse);
            cb.Multiply(bb.tensor.inertiaWorld);
            bb.angularVelocity += cb;
        }
        */

        // Update linear velocity for 'a'
        Vector3S ai = impulse;
        ai.Multiply(a.inverseMass); // ai = impulse * a.inverseMass
        a.velocity.Subtract(ai); // a.velocity += ai

        // Update angular velocity for 'a'
        Vector3S raCrossImpulse = this.ra;
        raCrossImpulse.CrossInPlace(impulse);// Vector3S.Cross(this.ra, impulse); // raCrossImpulse = Cross(ra, impulse)
        raCrossImpulse.Multiply(a.tensor.inertiaWorld); // raCrossImpulse = a.tensor.inertiaWorld * raCrossImpulse
        a.angularVelocity.Subtract(raCrossImpulse); // a.angularVelocity -= raCrossImpulse

        // If 'bb' is not null, update linear and angular velocities for 'bb'
        if (bb != null)
        {
            // Update linear velocity for 'bb'
            Vector3S bi = impulse;
            bi.Multiply(bb.inverseMass); // bi = impulse * bb.inverseMass
            bb.velocity.Add(bi); // bb.velocity += bi

            // Update angular velocity for 'bb'
            Vector3S rbCrossImpulse = this.rb;
            rbCrossImpulse.CrossInPlace(impulse);//Vector3S.Cross(this.rb, impulse); // rbCrossImpulse = Cross(rb, impulse)
            rbCrossImpulse.Multiply(bb.tensor.inertiaWorld); // rbCrossImpulse = bb.tensor.inertiaWorld * rbCrossImpulse
            bb.angularVelocity.Add(rbCrossImpulse); // bb.angularVelocity += rbCrossImpulse
        }

    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    f32 CalculateSeparation(in TransformS a, in TransformS b, in f32 slop)
    {
        //return MathS.Min(f32.zero, this.penetrationDepth + slop);

        
        Vector3S worldPointA = a.position + this.ra;
        Vector3S worldPointB = b.position + this.rb;
        f32 separation = Vector3S.Dot(worldPointB - worldPointA, this.normal) + this.penetrationDepth;
       // if (separation != this.penetrationDepth) throw new System.ArgumentException("LOL");

        return MathS.Min(f32.zero, separation + slop);
        
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    f32 CalculateTwistMass(in RigidbodyS a, in RigidbodyS bb)
    {
        // Effective mass for twist friction is based on the moment of inertia along the normal
        var twistMassA = Vector3S.Dot(this.normal, a.tensor.inertiaWorld * this.normal);
        var twistMassB = bb != null ? Vector3S.Dot(this.normal, bb.tensor.inertiaWorld * this.normal) : f32.zero;

        var totalTwistMass = twistMassA + twistMassB;
        return totalTwistMass > f32.zero ? f32.one / totalTwistMass : f32.zero;
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SolveTwistFriction(in RigidbodyS a, in Collidable b, in f32 friction)
    {
        var bb = b.isDynamic ? (RigidbodyS)b : null;

        // Calculate relative angular velocity around the normal
        var angularVelocityA = Vector3S.Dot(a.angularVelocity, this.normal);
        var angularVelocityB = bb != null ? Vector3S.Dot(bb.angularVelocity, this.normal) : f32.zero;

        var relativeTwistVelocity = angularVelocityB - angularVelocityA;

        // Calculate twist friction impulse
        var twistImpulse = -this.twistMass * relativeTwistVelocity;

        // Limit the twist friction impulse by Coulomb's law
        var maxTwist = this.accumulatedImpulse * friction;
        var newTwist = MathS.Clamp(this.accumulatedTwist + twistImpulse, -maxTwist, maxTwist);
        twistImpulse = newTwist - this.accumulatedTwist;
        this.accumulatedTwist = newTwist;


        // Apply the angular impulse for twist friction
        ApplyTwistImpulse(a, bb, twistImpulse);
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void ApplyTwistImpulse(in RigidbodyS a, in RigidbodyS bb, in f32 twistImpulse)
    {
        // Apply angular impulse around the normal for twist friction
        //var angularImpulse = this.normal * twistImpulse;

        var impulse = this.normal;
        impulse.Multiply(twistImpulse);

        var ai = impulse;
        ai.Multiply(a.tensor.inertiaWorld);

        a.angularVelocity.Subtract(ai);

        if (bb != null)
        {
            var bi = impulse;
            bi.Multiply(bb.tensor.inertiaWorld);
            bb.angularVelocity.Add(bi);
        }

    }
}

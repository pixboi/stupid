using stupid.Maths;

namespace stupid.Colliders
{
    public struct ContactManifoldS
    {
        public readonly Collidable a, b;
        public readonly RigidbodyS ab, bb;
        public readonly f32 friction, restitution;

        public ContactS c1, c2, c3, c4;
        public readonly int contactCount;
        public void CopyToArray(ref ContactS[] array)
        {

            if (contactCount == 1)
            {
                array[0] = c1;
            }
            else if (contactCount == 2)
            {
                array[0] = c1;
                array[1] = c2;
            }
            else if (contactCount == 3)
            {
                array[0] = c1;
                array[1] = c2;
                array[2] = c3;
            }
            else if (contactCount == 4)
            {
                array[0] = c1;
                array[1] = c2;
                array[2] = c3;
                array[3] = c4;
            }
        }

        void TransferOldImpulse(ref ContactS c, in ContactS old)
        {
            if (c.featureId == old.featureId)
            {
                c.accFric1 = old.accFric1;
                c.accumulatedImpulse = old.accumulatedImpulse;
                c.accumulatedTwist = old.accumulatedTwist;
            }
        }

        public void PrepareWarmup(in ContactManifoldS old)
        {
            if (old.contactCount == this.contactCount)
            {
                TransferOldImpulse(ref c1, old.c1);
                TransferOldImpulse(ref c2, old.c2);
                TransferOldImpulse(ref c3, old.c3);
                TransferOldImpulse(ref c4, old.c4);
            }
        }

        public void Warmup()
        {
            //Impulses
            if (contactCount >= 1) c1.WarmStart(ab, b);
            if (contactCount >= 2) c2.WarmStart(ab, b);
            if (contactCount >= 3) c3.WarmStart(ab, b);
            if (contactCount >= 4) c4.WarmStart(ab, b);
        }

        public ContactManifoldS(Collidable a, Collidable b, int contactCount, in ContactS[] contacts)
        {
            this.a = a;
            this.b = b;
            this.ab = a.isDynamic ? (RigidbodyS)a : null;
            this.bb = b.isDynamic ? (RigidbodyS)b : null;

            this.friction = (a.material.staticFriction + b.material.staticFriction) * f32.half;
            this.restitution = (a.material.restitution + b.material.restitution) * f32.half;

            this.contactCount = contactCount;

            this.c1 = contacts[0];
            this.c2 = contacts[1];
            this.c3 = contacts[2];
            this.c4 = contacts[3];
        }

        public void Resolve(in f32 inverseDt, in WorldSettings settings, in bool bias)
        {
            if (contactCount == 0)
            {
                return;
            }

            //Impulses
            if (contactCount >= 1)
            {
                c1.SolveImpulse(ab, b, inverseDt, settings, bias);
            }

            if (contactCount >= 2)
            {
                c2.SolveImpulse(ab, b, inverseDt, settings, bias);
            }

            if (contactCount >= 3)
            {
                c3.SolveImpulse(ab, b, inverseDt, settings, bias);
            }

            if (contactCount >= 4)
            {
                c4.SolveImpulse(ab, b, inverseDt, settings, bias);
            }

            //Frictions
            if (contactCount >= 1)
            {
                c1.SolveFriction(ab, b, friction);
            }

            if (contactCount >= 2)
            {
                c2.SolveFriction(ab, b, friction);
            }

            if (contactCount >= 3)
            {
                c3.SolveFriction(ab, b, friction);
            }

            if (contactCount >= 4)
            {
                c4.SolveFriction(ab, b, friction);
            }

            //Frictions
            if (contactCount >= 1)
            {
                //    c1.SolveTwistFriction(ab, b, friction);
            }

            if (contactCount >= 2)
            {
                //     c2.SolveTwistFriction(ab, b, friction);
            }

            if (contactCount >= 3)
            {
                //     c3.SolveTwistFriction(ab, b, friction);
            }

            if (contactCount >= 4)
            {
                //      c4.SolveTwistFriction(ab, b, friction);
            }
        }
    }
}

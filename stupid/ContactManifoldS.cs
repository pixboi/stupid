using stupid.Maths;

namespace stupid.Colliders
{
    public struct ContactManifoldS
    {
        public readonly Collidable a, b;
        public readonly RigidbodyS ab, bb;
        public readonly f32 friction, restitution;

        public ContactS one, two, three, four;
        public readonly byte contactCount;

        public IntPair ToPair() => new IntPair(a.index, b.index);
        public ContactS GetContact(int index)
        {
            switch (index)
            {
                case 0: return one;
                case 1: return two;
                case 2: return three;
                case 3: return four;
            }

            return default;
        }

        public void SetContact(int index, ContactS c)
        {
            switch (index)
            {
                case 0:
                    one = c;
                    break;
                case 1:
                    two = c;
                    break;
                case 2:
                    three = c;
                    break;
                case 3:
                    four = c;
                    break;

            }

        }

        public void CopyToArray(ref ContactS[] array)
        {
            if (contactCount == 1)
            {
                array[0] = one;
            }
            else if (contactCount == 2)
            {
                array[0] = one;
                array[1] = two;
            }
            else if (contactCount == 3)
            {
                array[0] = one;
                array[1] = two;
                array[2] = three;
            }
            else if (contactCount == 4)
            {
                array[0] = one;
                array[1] = two;
                array[2] = three;
                array[3] = four;
            }
        }

        ///Its good to account for some change, we can handle a bit of noise for more accurate things
        public void RetainData(in ContactManifoldS old)
        {
            //if (old.contactCount != this.contactCount) return;

            for (int i = 0; i < contactCount; i++)
            {
                var c = GetContact(i);
                for (int j = 0; j < old.contactCount; j++)
                {
                    var o = old.GetContact(j);
                    var ok = Transfer(ref c, o);

                    if (ok)
                    {
                        SetContact(i, c);
                    }
                }
            }

            return;


            Transfer(ref one, old.one);
            Transfer(ref two, old.two);
            Transfer(ref three, old.three);
            Transfer(ref four, old.four);

        }

        bool Transfer(ref ContactS c, in ContactS old)
        {
            if (c.featureId == old.featureId)
            {
                c.accumulatedImpulse = old.accumulatedImpulse;
                c.accumulatedFriction = old.accumulatedFriction;
                c.prevTangent = old.tangent;
                c.prevTangentMass = old.tangentMass;
                return true;
            }

            return false;
        }

        public void CalculatePrestep()
        {
            if (contactCount >= 1) one.CalculatePrestep(ab, b);
            if (contactCount >= 2) two.CalculatePrestep(ab, b);
            if (contactCount >= 3) three.CalculatePrestep(ab, b);
            if (contactCount >= 4) four.CalculatePrestep(ab, b);
        }

        public void SubtickUpdate()
        {
            if (contactCount >= 1) one.SubtickUpdate(a, b);
            if (contactCount >= 2) two.SubtickUpdate(a, b);
            if (contactCount >= 3) three.SubtickUpdate(a, b);
            if (contactCount >= 4) four.SubtickUpdate(a, b);
        }


        public void Warmup()
        {
            if (contactCount >= 1) one.WarmStart(ab, b);
            if (contactCount >= 2) two.WarmStart(ab, b);
            if (contactCount >= 3) three.WarmStart(ab, b);
            if (contactCount >= 4) four.WarmStart(ab, b);
        }

        public ContactManifoldS(Collidable a, Collidable b, byte contactCount, in ContactS[] contacts)
        {
            this.a = a;
            this.b = b;
            this.ab = a.isDynamic ? (RigidbodyS)a : null;
            this.bb = b.isDynamic ? (RigidbodyS)b : null;

            this.friction = (a.material.staticFriction + b.material.staticFriction) * f32.half;
            this.restitution = (a.material.restitution + b.material.restitution) * f32.half;

            this.contactCount = contactCount;
            this.one = contacts[0];
            this.two = contacts[1];
            this.three = contacts[2];
            this.four = contacts[3];
        }

        public void Resolve(in f32 inverseDt, in WorldSettings settings, in bool bias)
        {
            if (contactCount == 0)
            {
                throw new System.ArgumentException("No contacts in manifold?");
            }

            //Impulses
            if (contactCount >= 1)
            {
                one.SolveImpulse(ab, b, inverseDt, settings, bias);

            }

            if (contactCount >= 2)
            {
                two.SolveImpulse(ab, b, inverseDt, settings, bias);

            }

            if (contactCount >= 3)
            {
                three.SolveImpulse(ab, b, inverseDt, settings, bias);

            }

            if (contactCount >= 4)
            {
                four.SolveImpulse(ab, b, inverseDt, settings, bias);


            }

            //Impulses
            if (contactCount >= 1)
            {
                one.SolveFriction(ab, b, friction);
            }

            if (contactCount >= 2)
            {
                two.SolveFriction(ab, b, friction);
            }

            if (contactCount >= 3)
            {
                three.SolveFriction(ab, b, friction);
            }

            if (contactCount >= 4)
            {
                four.SolveFriction(ab, b, friction);
            }

        }
    }
}

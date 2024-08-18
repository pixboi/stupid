using stupid.Maths;

namespace stupid.Colliders
{
    public struct ContactManifoldS
    {
        public Collidable a, b;
        public f32 friction, restitution;
        public int count; // Number of valid contacts
        public ContactS c1, c2, c3, c4;

        public void PreStep()
        {
            if (count > 0) c1.PreStep();
            if (count > 1) c2.PreStep();
            if (count > 2) c3.PreStep();
            if (count > 3) c4.PreStep();
        }

        public void ResolveContacts(f32 deltaTime, in WorldSettings settings)
        {
            if (count > 0) c1.ResolveContact(deltaTime, settings);
            if (count > 1) c2.ResolveContact(deltaTime, settings);
            if (count > 2) c3.ResolveContact(deltaTime, settings);
            if (count > 3) c4.ResolveContact(deltaTime, settings);
        }

        public bool AddContact(ContactS newContact)
        {
            if (count >= 4)
            {
                // Manifold is full, cannot add more contacts
                return false;
            }

            // Add the new contact to the next available slot
            switch (count)
            {
                case 0:
                    c1 = newContact;
                    break;
                case 1:
                    c2 = newContact;
                    break;
                case 2:
                    c3 = newContact;
                    break;
                case 3:
                    c4 = newContact;
                    break;
            }

            // Increment the count of valid contacts
            count++;
            return true;
        }
    }
}

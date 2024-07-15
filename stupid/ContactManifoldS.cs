using System;
using System.Collections.Generic;

namespace stupid
{
    public readonly struct ContactManifoldS : IEquatable<ContactManifoldS>
    {
        public readonly Collidable a;
        public readonly Collidable b;
        public readonly ContactS[] contacts;
        public readonly int count;

        public ContactManifoldS(Collidable a, Collidable b, ContactS[] contacts, int count)
        {
            this.a = a;
            this.b = b;
            this.contacts = contacts;
            this.count = count;
        }

        public ContactManifoldS(ContactManifoldS fresh, ContactManifoldS old)
        {
            this.a = fresh.a;
            this.b = fresh.b;
            this.contacts = fresh.contacts;
            this.count = fresh.count;

            for (int i = 0; i < count; i++)
            {
                if (i < old.count)
                {
                    this.contacts[i] = new ContactS(contacts[i], old.contacts[i]);
                }
                else
                {
                    this.contacts[i] = contacts[i];
                }
            }
        }

        public ContactManifoldS(Collidable a, Collidable b, ContactS[] contacts, int count, ContactManifoldS old)
        {
            this.a = a;
            this.b = b;
            this.contacts = contacts;
            this.count = count;

            for (int i = 0; i < count; i++)
            {
                if (i < old.count)
                {
                    this.contacts[i] = new ContactS(contacts[i], old.contacts[i]);
                }
                else
                {
                    this.contacts[i] = contacts[i];
                }
            }
        }

        public override bool Equals(object? obj)
        {
            return obj is ContactManifoldS s && Equals(s);
        }

        public bool Equals(ContactManifoldS other)
        {
            return EqualityComparer<Collidable>.Default.Equals(a, other.a) &&
                   EqualityComparer<Collidable>.Default.Equals(b, other.b);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(a, b);
        }
    }
}

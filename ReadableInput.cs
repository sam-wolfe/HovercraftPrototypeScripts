using UnityEngine;

namespace DefaultNamespace {

    public interface ReadableInput {

        public Vector2 ReadMove();
        public float ReadAltitude();
        public float ReadSails();
        public bool ReadAim();
        public bool ReadBrake();
        public float ReadLatBrake();
        public bool ReadFire();
        public bool ReadBoost();
        public Vector2 ReadGunAim();
    }

}
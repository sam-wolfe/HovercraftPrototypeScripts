using System.Collections;
using System.Collections.Generic;
using DefaultNamespace;
using UnityEngine;

public class EnemyAI : MonoBehaviour, ReadableInput {
    
    public float altitude { get; private set; }
    public Vector2 move { get; private set; }
    public float sails { get; private set; }
    public bool aim { get; private set; }
    public float lateralBrake { get; private set; }

    void Start() {
        move = new Vector2(0, 0.1f);
    }

    void Update() {
        
    }
    
    public Vector2 ReadMove() {
        return move;
    }

    public float ReadAltitude() {
        return altitude;
    }

    public float ReadSails() {
        return sails;
    }

    public bool ReadAim() {
        return aim;
    }

    bool ReadableInput.ReadBrake() {
        throw new System.NotImplementedException();
    }

    public float ReadLatBrake() {
        throw new System.NotImplementedException();
    }

    public bool ReadFire() {
        throw new System.NotImplementedException();
    }

    public bool ReadBoost() {
        throw new System.NotImplementedException();
    }

    public Vector2 ReadGunAim() {
        throw new System.NotImplementedException();
    }

    public float ReadBrake() {
        return lateralBrake;
    }
}

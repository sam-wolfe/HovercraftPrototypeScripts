using System;
using System.Collections;
using System.Collections.Generic;
using DefaultNamespace;
using UnityEngine;
using UnityEngine.InputSystem;

public class EnemyAI : MonoBehaviour, ReadableInput {
    
    public float altitude { get; private set; }
    public Vector2 move { get; private set; }
    public float sails { get; private set; }
    public bool aim { get; private set; }
    public float lateralBrake { get; private set; }
    
    public event Action<InputAction.CallbackContext> OnMoveEvent;
    public event Action<InputAction.CallbackContext> OnSailEvent;

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
    
    public float ReadBrake() {
        return lateralBrake;
    }
}

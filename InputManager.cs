using System.Collections;
using System.Collections.Generic;
using DefaultNamespace;
using UnityEngine;
using UnityEngine.InputSystem;

public class InputManager : MonoBehaviour, ReadableInput {

    public float altitude { get; private set; }
    public Vector2 move { get; private set; }
    public float sails { get; private set; }
    public bool aim { get; private set; }
    public float lateralBrake { get; private set; }
    public bool fire { get; private set; }
    public bool boost { get; private set; }
    public bool brake { get; private set; }
    
    public void OnAltitude(InputAction.CallbackContext context) {
        float newAltitude = context.ReadValue<float>();

        altitude = newAltitude;
        // Debug.Log(altitude);
    }
    
    public void OnMove(InputAction.CallbackContext context) {
        Vector2 newMovement = context.ReadValue<Vector2>();

        move = newMovement;
        // Debug.Log(move);
    }
    
    public void OnSail(InputAction.CallbackContext context) {
        sails = context.ReadValue<float>();;
        // Debug.Log(sails);
    }
    
    public void OnLateralBrake(InputAction.CallbackContext context) {
        lateralBrake = context.ReadValue<float>();
    }
    
    public void OnAim(InputAction.CallbackContext context) {
        switch (context.phase) {
            case InputActionPhase.Started:
                aim = true;
                break;
            case InputActionPhase.Performed:
                return;
            case InputActionPhase.Canceled:
                aim = false;
                break;
        }
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
    
    public float ReadLatBrake() {
        return lateralBrake;
    }
    
    public bool ReadFire() {
        return fire;
    }
    
    public bool ReadBoost() {
        return boost;
    }
    
    public bool ReadBrake() {
        return brake;
    }
    
    public void OnFire(InputAction.CallbackContext context) {
        switch (context.phase) {
            case InputActionPhase.Started:
                Debug.Log("Fire");
                fire = true;
                break;
            case InputActionPhase.Performed:
                return;
            case InputActionPhase.Canceled:
                fire = false;
                Debug.Log("Fire");
                break;
        }
    }
    
    public void OnBoost(InputAction.CallbackContext context) {
        switch (context.phase) {
            case InputActionPhase.Started:
                Debug.Log("Boost");
                boost = true;
                break;
            case InputActionPhase.Performed:
                return;
            case InputActionPhase.Canceled:
                Debug.Log("Boost");
                boost = false;
                break;
        }
    }
    
    public void OnBrake(InputAction.CallbackContext context) {
        switch (context.phase) {
            case InputActionPhase.Started:
                Debug.Log("Brake");
                brake = true;
                break;
            case InputActionPhase.Performed:
                return;
            case InputActionPhase.Canceled:
                Debug.Log("Brake");
                brake = false;
                break;
        }
    }

}

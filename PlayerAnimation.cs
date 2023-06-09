using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerAnimation : MonoBehaviour {
    
    // TODO make interface that inputs share
    [SerializeField] private InputManager _input;
    [SerializeField] private Animator _animator;
    private static readonly int Aiming = Animator.StringToHash("Aiming");

    void Start() {
        
    }

    void Update() {
        if (_input.aim) {
            _animator.SetBool(Aiming, true);
        }
        else {
            _animator.SetBool(Aiming, false);
        }
    }
}

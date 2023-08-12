using System;
using UnityEngine;

[CreateAssetMenu(fileName = "NewBooster", menuName = "ScriptableObjects/CarParts/Booster", order = 2)]
public class Booster : ScriptableObject {

    public float boostForce;
    
    public float maxBoostForce;
    
    public float boostRefillRate;

    public float boostDepleteRate;

    private void OnEnable() {
        boostForce = maxBoostForce;
    }

}

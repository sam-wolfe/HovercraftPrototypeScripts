using UnityEngine;

[CreateAssetMenu(fileName = "NewBooster", menuName = "ScriptableObjects/CarParts/Booster", order = 2)]
public class Booster : ScriptableObject {

    private float boostForce;
    
    public float maxBoostForce;
    
    public float boostRefillRate;

    public float boostDepleteRate;

}

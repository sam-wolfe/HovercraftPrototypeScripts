using UnityEngine;

[CreateAssetMenu(fileName = "NewPressureVent", menuName = "ScriptableObjects/CarParts/PressureVent", order = 4)]
public class PressureVent : ScriptableObject {
    
    [Range(100, 1000)]
    public float brakeForce = 500f;
    
    [Range(100, 1000)]
    public float maxBrakeForce = 500f;
    
    [Tooltip("Rate the hovercars break force will regenerate, lower=faster.")] 
    [Range(1, 10)]
    public float breakRefilRate = 4f;
    
    [Tooltip("Rate the hovercars break force will regenerate, lower=faster.")] 
    [Range(1, 2000)]
    public float breakDepleteRate = 800f;

}

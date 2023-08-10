using UnityEngine;

[CreateAssetMenu(fileName = "NewGyro", menuName = "ScriptableObjects/CarParts/Gyro", order = 1)]
public class Gyro : ScriptableObject {
    
    private float turnRate;
    public float turnDrag;
    public float hovercarRestHeight;

}

using UnityEngine;

[CreateAssetMenu(fileName = "NewHydrolic", menuName = "ScriptableObjects/CarParts/Hydrolic", order = 5)]
public class Hydrolic : ScriptableObject {
    
    [SerializeField] [Tooltip("Rate the hovercar will stop moving when no input is given")] 
    [Range(0.1f, 100f)]
    private float carDrag = 20f;
    

}

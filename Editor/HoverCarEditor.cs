using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(HovercarController))]
public class HoverCarEditor : Editor
{
    public override void OnInspectorGUI()
    {
        // Draw the default inspector for ObjectA
        DrawDefaultInspector();

        // Get the ObjectA instance
        HovercarController hovercar = (HovercarController)target;

        // If the objectBSettings field is not null, draw its properties in the inspector
        if (hovercar._gyro != null)
        {
            GUILayout.Space(10);
            EditorGUI.indentLevel++;
            EditorGUILayout.LabelField("Gyro Settings", EditorStyles.boldLabel);
            Editor editor = CreateEditor(hovercar._gyro);
            editor.OnInspectorGUI();
            EditorGUI.indentLevel--;
        }
                
        if (hovercar._booster != null)
        {
            GUILayout.Space(10);
            EditorGUI.indentLevel++;
            EditorGUILayout.LabelField("Booster Settings", EditorStyles.boldLabel);
            Editor editor = CreateEditor(hovercar._booster);
            editor.OnInspectorGUI();
            EditorGUI.indentLevel--;
        }

        if (hovercar._fan != null)
        {
            GUILayout.Space(10);
            EditorGUI.indentLevel++;
            EditorGUILayout.LabelField("Fan Settings", EditorStyles.boldLabel);
            Editor editor = CreateEditor(hovercar._fan);
            editor.OnInspectorGUI();
            EditorGUI.indentLevel--;
        }
        
        if (hovercar._pressureVent != null)
        {
            GUILayout.Space(10);
            EditorGUI.indentLevel++;
            EditorGUILayout.LabelField("PressureVent Settings", EditorStyles.boldLabel);
            Editor editor = CreateEditor(hovercar._pressureVent);
            editor.OnInspectorGUI();
            EditorGUI.indentLevel--;
        }

        if (hovercar._hydrolic != null)
        {
            GUILayout.Space(10);
            EditorGUI.indentLevel++;
            EditorGUILayout.LabelField("Hydrolic Settings", EditorStyles.boldLabel);
            Editor editor = CreateEditor(hovercar._hydrolic);
            editor.OnInspectorGUI();
            EditorGUI.indentLevel--;
        }
    }
}
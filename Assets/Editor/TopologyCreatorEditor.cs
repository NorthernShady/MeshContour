using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(TopologyCreator))]
public class TopologyCreatorEditor : Editor
{
    public override void OnInspectorGUI()
    {
        TopologyCreator targetCreator = (TopologyCreator)target;

        DrawDefaultInspector();

        if (GUILayout.Button("Create new topology"))
        {
            targetCreator.CreateRandomTopology();
        }

        if (GUILayout.Button("Create grass"))
        {
            targetCreator.CreateGrass();
        }

        if (GUILayout.Button("Update grass"))
        {
            targetCreator.UpdateGrass();
        }
    }
}

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameNode : MonoBehaviour
{
    public int id = -1;
    public Vector2 position;

    public GameGroundType groundType = GameGroundType.NONE;

    public bool isChecked = false; // for testing purposes

    public List<int> chainElements = new List<int>();

    public bool isVisited = false;

    public bool hasGround
    {
        get
        {
            return groundType != GameGroundType.NONE;
            // activeGround != null;
        }
    }

    void Awake()
    {
        if (hasGround)
        {
            isChecked = true;
        }
    }

    void OnDrawGizmos()
    {
        if (hasGround)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(zShift(transform.position, -0.5f, true), 0.1f);
        }
    }

    public static Vector3 zShift(Vector3 position, float z, bool additive = false)
    {
        return new Vector3(position.x, position.y, additive ? position.z + z : z);
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class TopologyCreator : MonoBehaviour
{
    [SerializeField] private int numberOfRows = 10;
    [SerializeField] private int numberOfColumns = 10;
    [SerializeField] private float offset = 0.6f;

    [SerializeField] private GameNode nodePrefab = null;

    public void CreateRandomTopology()
    {
        DestroyCurrentNodes();

        List<GameNode> nodes = new List<GameNode>();
        var rowStart = -numberOfRows * offset * 0.5f;
        var columnStart = -numberOfColumns * offset * 0.5f;

        var nodeIndex = 0;

        var cells = new GameNode[numberOfRows, numberOfColumns];


        for (int i = 0; i < numberOfRows; ++i)
        {
            for (int j = 0; j < numberOfColumns; ++j)
            {
                var node = GameObject.Instantiate<GameNode>(nodePrefab, transform);
                var x = rowStart + j * offset;
                var y = columnStart + i * offset;
                node.transform.position = new Vector3(x, y, 0.0f);
                node.position = new Vector2(x, y);
                node.id = nodeIndex++;

                cells[i, j] = node;
            }
        }

        for (int i = 0; i < numberOfRows; ++i)
        {
            for (int j = 0; j < numberOfColumns; ++j)
            {
                var node = cells[i, j];
                var adjacent = new List<int>();

                if (i > 0)
                {
                    adjacent.Add(cells[i - 1, j].id);
                }

                if (i < numberOfRows - 1)
                {
                    adjacent.Add(cells[i + 1, j].id);
                }

                if (j > 0)
                {
                    adjacent.Add(cells[i, j - 1].id);
                }

                if (j < numberOfColumns - 1)
                {
                    adjacent.Add(cells[i, j + 1].id);
                }

                node.chainElements = adjacent;
            }
        }
    }

    public void CreateGrass()
    {
        var currentNodes = FindObjectsOfType<GameNode>(true);
        var groundMeshCreator = FindObjectOfType<GroundMeshCreator>();
        groundMeshCreator.createGround(currentNodes.ToList());
    }

    public void UpdateGrass()
    {
        var currentNodes = FindObjectsOfType<GameNode>(true);
        var groundMeshCreator = FindObjectOfType<GroundMeshCreator>();
        groundMeshCreator.updateGround(currentNodes.ToList().FindAll(x => x.hasGround && !x.isChecked));
    }

    private void DestroyCurrentNodes()
    {
        var currentNodes = FindObjectsOfType<GameNode>(true);
        foreach (var node in currentNodes)
        {
            DestroyImmediate(node.gameObject);
        }
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Grid : MonoBehaviour {
	public bool onlyDisplayPathGizmos;

	public LayerMask unwalkableMask; // this defines i guess what is unwalkable
	public Vector2 gridWorldSize;//area or coordinates that the grid is going to cover
	public float nodeRadius; // to define how much space each individual node covers
	Node[,] grid; //2 dimensial array of nodes

	float nodeDiameter; // to know how many nodes to fit in our grids
	int gridSizeX, gridSizeY;// gridsize that we are using

	void Awake() {
		nodeDiameter = nodeRadius*2;
		gridSizeX = Mathf.RoundToInt(gridWorldSize.x/nodeDiameter); // we are making sure that it is an int 
		gridSizeY = Mathf.RoundToInt(gridWorldSize.y/nodeDiameter);// l9isma us just to get des carreaux to place
		CreateGrid();
	}
public int MaxSize {
		get {
			return gridSizeX * gridSizeY;
		}
	}
	void CreateGrid() { 
		grid = new Node[gridSizeX,gridSizeY]; //we are assigning that our grids is a 2d arary of those dimensions
		Vector3 worldBottomLeft = transform.position - Vector3.right * gridWorldSize.x/2 - Vector3.forward * gridWorldSize.y/2;
    //this gets bottom left position using transform li hia center of the work right* left edge of world lakhra gives us bottom left corner
		for (int x = 0; x < gridSizeX; x ++) { //llop through all positions that the nodes will be in to see if tehy are walkable or not using a collision check
			for (int y = 0; y < gridSizeY; y ++) {
				Vector3 worldPoint = worldBottomLeft + Vector3.right * (x * nodeDiameter + nodeRadius) + Vector3.forward * (y * nodeDiameter + nodeRadius);
				bool walkable = !(Physics.CheckSphere(worldPoint,nodeRadius,unwalkableMask)); //checks true if we dont collide with stuff in unwalkable mask sinon it gives false basically if checksphere true so kayna collision walkable is false
				grid[x,y] = new Node(walkable,worldPoint,x,y);//populate with the walkable or not walkable points
			}
		}
	}

	public List<Node> GetNeighbours(Node node) { //we dont know how many nodes there are around a node so we use a listo to return the list og nodes
		List<Node> neighbours = new List<Node>(); //list of nodes of neighbors

		for (int x = -1; x <= 1; x++) { //searchs by a 3 by 3 block -1 0 1 glnaha 9bila this is by 3 by 3 block
			for (int y = -1; y <= 1; y++) {
				if (x == 0 && y == 0) //bc this position means que ra hna f teh actual node , we are in the center of that iteration
					continue;

				int checkX = node.gridX + x; //assign the neighbor of the nodes coordinates
				int checkY = node.gridY + y;

				if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY) { //check if that neigbor is inside of the grid
					neighbours.Add(grid[checkX,checkY]); // we add this node to the list o neigbors
				}
			}
		}

		return neighbours;
	}
	

	public Node NodeFromWorldPoint(Vector3 worldPosition) { //this is to know where our character is standing, we convert a world coordinate into a grid position
		float percentX = (worldPosition.x + gridWorldSize.x/2) / gridWorldSize.x; //we need to know where it is middle far left far right uisng % 0 0.5 1
		float percentY = (worldPosition.z + gridWorldSize.y/2) / gridWorldSize.y;//same thg for y
		percentX = Mathf.Clamp01(percentX); // to keep the value btw 0 and 1 so if it is outside of the world it doent give us weird errors and stuff
		percentY = Mathf.Clamp01(percentY);

		int x = Mathf.RoundToInt((gridSizeX-1) * percentX); // -1 is just math stuff to not be outside of the array
		int y = Mathf.RoundToInt((gridSizeY-1) * percentY);
		return grid[x,y]; // we return the node from our grid
	}

	public List<Node> path; // to trace back the path
	public List<Node> path2;
	public List<Node> path3;
	public List<Node> pathUCS;
	public List<Node> pathBFS;
	public List<Node> pathDFS;




	void OnDrawGizmos() {
		Gizmos.DrawWireCube(transform.position,new Vector3(gridWorldSize.x,1,gridWorldSize.y));// this is bc we want our grid to b seen bc y access is representing z in 3d space

		if (grid != null) { //draw the grid using cubes
				Gizmos.DrawWireCube(transform.position,new Vector3(gridWorldSize.x,1,gridWorldSize.y));

				//Gizmos.color = (n.walkable)?Color.white:Color.yellow; //? bhal f racket i guess if true do white sion red

				if (path != null) // drwa the path for the A* in black
						foreach (Node n in path) {
							Gizmos.color = Color.black;
							Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter-.1f));
						}
				if (path2 != null) // drwa the path for the A* in black
					foreach (Node n in path2) {
							Gizmos.color = Color.grey;
							Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter-.1f));

						}
				if (path3 != null) 
					foreach (Node n in path3) {
							Gizmos.color = Color.blue;
							Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter-.1f));

						}
				if (pathUCS != null) 
					foreach (Node n in pathUCS) {
							Gizmos.color = Color.red;
							Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter-.1f));

						}
				if (pathBFS != null) 
					foreach (Node n in pathBFS) {
							Gizmos.color = Color.yellow;
							Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter-.1f));
						}
				if (pathDFS != null) //TRY DIK L3BA BHAL F LWL BUT WITOUT KOL LOOP MFR9A
					foreach (Node n in pathDFS) {
							Gizmos.color = Color.white;
							Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter-.1f));
						}/**/
				}
				
			}
		}
	


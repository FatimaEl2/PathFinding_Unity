
using UnityEngine;
using System.Collections;

public class Node {
	
	public bool walkable; //keep track of if a node is walkable or not
	public Vector3 worldPosition; /* what point in the world does this node represent*/
	public int gridX;//we use this to allow the node to keep track of its own position in the array
	public int gridY;

	public int gCost; // adding attributes of cost to each node
	public int hCost;
	public Node parent;//parent so we can retrace


	public Node(bool _walkable, Vector3 _worldPos,int _gridX,int _gridY) { //constructor
		walkable = _walkable;
		worldPosition = _worldPos;
		gridX=_gridX;
		gridY=_gridY;
	}
	public int fCost{ //we dont need to set it bc its set idrectly mn gcost w hcost
		get{
			return gCost+hCost;
		}
	}

}




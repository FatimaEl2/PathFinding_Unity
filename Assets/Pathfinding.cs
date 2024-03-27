using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;


public class Pathfinding : MonoBehaviour {

	public Transform seeker, target; //this allows us to have our seeker and our target nodes w transform i gues is teh type of game positions
	Grid grid; // we nee to call thsi class to be able to convert positions

	void Awake() {
		grid = GetComponent<Grid> (); //get the grid i think, grid and pathfinding shld be on the ame game object for them to work
	}

	void Update() {
		var watchAstarVideo = new System.Diagnostics.Stopwatch();
		var watchAstarManhattan = new System.Diagnostics.Stopwatch();
        var watchAstarEuclidian = new System.Diagnostics.Stopwatch();
		var watchUCS = new System.Diagnostics.Stopwatch();
        var watchBFS = new System.Diagnostics.Stopwatch();
        var watchDFS = new System.Diagnostics.Stopwatch();

		watchAstarVideo.Start();
		FindPathAstarvideo (seeker.position, target.position);  // we sens the vector3 positions as arguments
		watchAstarVideo.Stop();
		Debug.Log("Execution Time A*: "+watchAstarVideo.ElapsedMilliseconds+"ms");

		watchAstarEuclidian.Start();
		FindPathAstareuclidean(seeker.position, target.position);
        watchAstarEuclidian.Stop();
        Debug.Log("Execution Time A* Euclidean: "+watchAstarEuclidian.ElapsedMilliseconds+"ms");

		watchAstarManhattan.Start();
		FindPathAstarManhattan(seeker.position, target.position);
		watchAstarManhattan.Stop();
        Debug.Log("Execution Time A* Manhattan: "+watchAstarManhattan.ElapsedMilliseconds+"ms");

        watchUCS.Start();
		FindPathUCS(seeker.position, target.position);
		watchUCS.Stop();
        Debug.Log("Execution Time UCS: "+watchUCS.ElapsedMilliseconds+"ms");

        watchBFS.Start();
		FindPathBFS(seeker.position, target.position);
        watchBFS.Stop();
        Debug.Log("Execution Time BFS: "+watchBFS.ElapsedMilliseconds+"ms");
        
		watchDFS.Start();
		FindPathDFS(seeker.position, target.position);
        watchDFS.Stop();
        Debug.Log("Execution Time DFS: "+watchDFS.ElapsedMilliseconds+"ms");


		StartCoroutine(walkminpath());
	}

	void FindPathAstarvideo(Vector3 startPos, Vector3 targetPos) { //to ifnd path we need to convert world positions into nodes
		Node startNode = grid.NodeFromWorldPoint(startPos); //converts world pos from nodes
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		int max_fringe=0;
		int expanded_nodes=0;
		List<Node> openSet = new List<Node>(); //hna we dont need to search so we use a list i think hada hoa lfringe maybe f bfs w dfs
		HashSet<Node> closedSet = new HashSet<Node>();//hna we dont need to search just update etc so we use a hasset
		openSet.Add(startNode); // we add the starting node to teh open set

		while (openSet.Count > 0) { //while openset is not empty
			Node node = openSet[0]; // assign the first el of teh open set to start comparaisons
			for (int i = 1; i < openSet.Count; i ++) { //find node in the openset with the lowest cost by looping through all the nodes in the open set
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost && openSet[i].hCost < node.hCost) { //if fcost is lower or equal and hcost lowe
						node = openSet[i];// assign the new lowest to node
				}
			}
			max_fringe= Mathf.Max(max_fringe,openSet.Count);

			openSet.Remove(node); //remove it from the open set and add it to the closed set 
			closedSet.Add(node);

			if (node == targetNode) { //if we found goal
				RetracePath(startNode,targetNode); //we retrace our path from the start to the ned
				Debug.Log("the maximum size of the fringe for A* is "+max_fringe);
				Debug.Log("the number of expanded nodes for A*  "+expanded_nodes);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) { // for each neigboring node
				if (!neighbour.walkable || closedSet.Contains(neighbour)) { // if the neighbor is not walkable/obstacle or if it is already visited/in the closed set
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistanceAstarvideo(node, neighbour); // gives us cost of the new neighbor
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) { //if its is less than the current nodes gcost or if the neighbor is not in the open set
					neighbour.gCost = newCostToNeighbour; //we need to set the fcost we can do this by setting gcost (mn start node) and hcost
					neighbour.hCost = GetDistanceAstarvideo(neighbour, targetNode); //cost from the end node
					neighbour.parent = node; // we add a parent to the node that we choose to be able to retrace the path

					if (!openSet.Contains(neighbour)) // if neighbor is not in open set, we add it to the open set
						openSet.Add(neighbour);
						expanded_nodes++;

				}
			}
		}
	}
	void FindPathAstareuclidean(Vector3 startPos, Vector3 targetPos) { //to ifnd path we need to convert world positions into nodes
		Node startNode = grid.NodeFromWorldPoint(startPos); //converts world pos from nodes
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
		
		int max_fringe=0;
		int expanded_nodes=0;
		List<Node> openSet = new List<Node>(); //hna we dont need to search so we use a list i think hada hoa lfringe maybe f bfs w dfs
		HashSet<Node> closedSet = new HashSet<Node>();//hna we dont need to search just update etc so we use a hasset
		openSet.Add(startNode); // we add the starting node to teh open set

		while (openSet.Count > 0) { //while openset is not empty
			Node node = openSet[0]; // assign the first el of teh open set to start comparaisons
			for (int i = 1; i < openSet.Count; i ++) { //find node in the openset with the lowest cost by looping through all the nodes in the open set
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost && openSet[i].hCost < node.hCost) { //if fcost is lower or equal and hcost lowe
						node = openSet[i];// assign the new lowest to node
				}
			}
			max_fringe= Mathf.Max(max_fringe,openSet.Count);

			openSet.Remove(node); //remove it from the open set and add it to the closed set 
			closedSet.Add(node);

			if (node == targetNode) { //if we found goal
				RetracePathAstareuclidean(startNode,targetNode); //we retrace our path from the start to the ned
				Debug.Log("the maximum size of the fringe for A* euclidean is "+max_fringe);
				Debug.Log("the number of expanded nodes for A* euclidean "+expanded_nodes);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) { // for each neigboring node
				if (!neighbour.walkable || closedSet.Contains(neighbour)) { // if the neighbor is not walkable/obstacle or if it is already visited/in the closed set
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistanceAstareuclidean(node, neighbour); // gives us cost of the new neighbor
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) { //if its is less than the current nodes gcost or if the neighbor is not in the open set
					neighbour.gCost = newCostToNeighbour; //we need to set the fcost we can do this by setting gcost (mn start node) and hcost
					neighbour.hCost = GetDistanceAstareuclidean(neighbour, targetNode); //cost from the end node
					neighbour.parent = node; // we add a parent to the node that we choose to be able to retrace the path

					if (!openSet.Contains(neighbour)) // if neighbor is not in open set, we add it to the open set
						openSet.Add(neighbour);
						expanded_nodes++;

				}
			}
		}
	}
	void FindPathAstarManhattan(Vector3 startPos, Vector3 targetPos) { //to ifnd path we need to convert world positions into nodes
		Node startNode = grid.NodeFromWorldPoint(startPos); //converts world pos from nodes
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		int max_fringe=0;
		int expanded_nodes=0;
		List<Node> openSet = new List<Node>(); //hna we dont need to search so we use a list i think hada hoa lfringe maybe f bfs w dfs
		HashSet<Node> closedSet = new HashSet<Node>();//hna we dont need to search just update etc so we use a hasset
		openSet.Add(startNode); // we add the starting node to teh open set

		while (openSet.Count > 0) { //while openset is not empty
			Node node = openSet[0]; // assign the first el of teh open set to start comparaisons
			for (int i = 1; i < openSet.Count; i ++) { //find node in the openset with the lowest cost by looping through all the nodes in the open set
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost && openSet[i].hCost < node.hCost) { //if fcost is lower or equal and hcost lowe
						node = openSet[i];// assign the new lowest to node
				}
			}
			max_fringe= Mathf.Max(max_fringe,openSet.Count);

			openSet.Remove(node); //remove it from the open set and add it to the closed set 
			closedSet.Add(node);

			if (node == targetNode) { //if we found goal
				RetracePathAstarmanhattan(startNode,targetNode); //we retrace our path from the start to the ned
				Debug.Log("the maximum size of the fringe for A* manhatatn is "+max_fringe);
				Debug.Log("the number of expanded nodes for A* manhattan "+expanded_nodes);  
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) { // for each neigboring node
				if (!neighbour.walkable || closedSet.Contains(neighbour)) { // if the neighbor is not walkable/obstacle or if it is already visited/in the closed set
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistanceAstarmanhattan(node, neighbour); // gives us cost of the new neighbor
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) { //if its is less than the current nodes gcost or if the neighbor is not in the open set
					neighbour.gCost = newCostToNeighbour; //we need to set the fcost we can do this by setting gcost (mn start node) and hcost
					neighbour.hCost = GetDistanceAstarmanhattan(neighbour, targetNode); //cost from the end node
					neighbour.parent = node; // we add a parent to the node that we choose to be able to retrace the path

					if (!openSet.Contains(neighbour)) // if neighbor is not in open set, we add it to the open set
						openSet.Add(neighbour);
						expanded_nodes++;

				}
			}
		}
	}
	
	void FindPathUCS(Vector3 startPos, Vector3 targetPos) { //to ifnd path we need to convert world positions into nodes
		Node startNode = grid.NodeFromWorldPoint(startPos); //converts world pos from nodes
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>(); 
		HashSet<Node> closedSet = new HashSet<Node>();
		int max_fringe=0;
		int expanded_nodes=0;
		openSet.Add(startNode); 

		while (openSet.Count > 0) { 
			max_fringe= Mathf.Max(max_fringe,openSet.Count);

			Node node = openSet[0]; 
			for (int i = 1; i < openSet.Count; i ++) { 
				if (openSet[i].gCost < node.gCost ) { //CHECK THIS IM NOT SURE
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) { 
				RetracePathUCS(startNode,targetNode);
				Debug.Log("the maximum size of the fringe for UCS is "+max_fringe);
				Debug.Log("the number of expanded nodes for UCS"+expanded_nodes);  
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) { 
				if (!neighbour.walkable || closedSet.Contains(neighbour)) { 
					continue;
				}

				int newCostToNeighbour = node.gCost;
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) { //if its is less than the current nodes gcost or if the neighbor is not in the open set
					neighbour.gCost = newCostToNeighbour; //we need to set the fcost we can do this by setting gcost (mn start node) and hcost
					neighbour.hCost=0;
					neighbour.parent = node; // we add a parent to the node that we choose to be able to retrace the path
					
					if (!openSet.Contains(neighbour)) // if neighbor is not in open set, we add it to the open set
						openSet.Add(neighbour);
						expanded_nodes++;

				}
			}
		}
	}

void FindPathBFS(Vector3 startPos, Vector3 targetPos) { //to ifnd path we need to convert world positions into nodes
		Node startNode = grid.NodeFromWorldPoint(startPos); //converts world pos from nodes
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		Queue<Node> openSet = new Queue<Node>(); 
		HashSet<Node> closedSet = new HashSet<Node>();
		int max_fringe=0;
		int expanded_nodes=0;
		openSet.Enqueue(startNode); 

		while (openSet.Count > 0) { 
			
			max_fringe= Mathf.Max(max_fringe,openSet.Count);
			Node node = openSet.Dequeue(); 
			
			closedSet.Add(node);
			
			if (node == targetNode) { 
				RetracePathBFS(startNode,targetNode);
				Debug.Log("the maximum size of the fringe for BFS is "+max_fringe);
				Debug.Log("the number of expanded nodes for BFS"+expanded_nodes); 
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) { 
				if (!neighbour.walkable || closedSet.Contains(neighbour)) { 
					continue;
				}

				if (neighbour.walkable || !openSet.Contains(neighbour)) { //if it is nor in the fringe but it is walkable we shld add it to the fringe
					neighbour.parent = node;
					 //open set is the fringe i think
					closedSet.Add(neighbour); //hadi darori bach yb9a lian ghi 1 khet not 2
			 // if its already in the fringe shld we enqueue it joj mrat wla la
					openSet.Enqueue(neighbour);
					expanded_nodes++;

				}
			}
		}
	}
void FindPathDFS(Vector3 startPos, Vector3 targetPos) { //to ifnd path we need to convert world positions into nodes
		Node startNode = grid.NodeFromWorldPoint(startPos); //converts world pos from nodes
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		Stack<Node> openSet = new Stack<Node>(); 
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Push(startNode); 
		int max_fringe=0;
		int expanded_nodes=0;
		while (openSet.Count > 0) { 
			
			max_fringe= Mathf.Max(max_fringe,openSet.Count);// aafter we push to keep track of the mmax fringe hit
			Node node = openSet.Pop(); 
			
			closedSet.Add(node);

			if (node == targetNode) { 
				RetracePathDFS(startNode,targetNode); 
				Debug.Log("the maximum size of the fringe for DFS is "+max_fringe);
				Debug.Log("the number of expanded nodes for DFS"+expanded_nodes);

				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) { 
				if (!neighbour.walkable || closedSet.Contains(neighbour)) { 
					continue;
				}

				if (neighbour.walkable || !openSet.Contains(neighbour)) { //if it is nor in the fringe but it is walkable we shld add it to the fringe
					neighbour.parent = node;
					 //open set is the fringe i think
					closedSet.Add(neighbour); //hadi darori bach yb9a lian ghi 1 khet not 2
					// if its already in the fringe shld we enqueue it joj mrat wla la
					openSet.Push(neighbour);
					expanded_nodes++;
	
				}
			}
		}
	}
	void RetracePath(Node startNode, Node endNode) {
		List<Node> path = new List<Node>(); // list of nodes
		Node currentNode = endNode; //current node

		while (currentNode != startNode) { // go back 7tal start node
			path.Add(currentNode);
			currentNode = currentNode.parent; // u do this by going back to the parent
		}
		path.Add(startNode);
		path.Reverse(); // we reverse the path so we can get from start to end instead of from end to start li kant tat3tina

		grid.path = path;

	}
	void RetracePathAstareuclidean(Node startNode, Node endNode) {
		List<Node> path = new List<Node>(); // list of nodes
		Node currentNode = endNode; //current node

		while (currentNode != startNode) { // go back 7tal start node
			path.Add(currentNode);
			currentNode = currentNode.parent; // u do this by going back to the parent
		}
		path.Add(startNode);
		path.Reverse(); // we reverse the path so we can get from start to end instead of from end to start li kant tat3tina

		grid.path2 = path;

	}
	void RetracePathAstarmanhattan(Node startNode, Node endNode) {
		List<Node> path = new List<Node>(); // list of nodes
		Node currentNode = endNode; //current node

		while (currentNode != startNode) { // go back 7tal start node
			path.Add(currentNode);
			currentNode = currentNode.parent; // u do this by going back to the parent
		}
		path.Add(startNode);
		path.Reverse(); // we reverse the path so we can get from start to end instead of from end to start li kant tat3tina

		grid.path3 = path;

	}
	int GetDistanceAstarvideo(Node nodeA, Node nodeB) { //to get the distance between 2 given nodes, i think thsi si teh currently used heuristic w f hadi its at 15.22 mn video
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY); 

		if (dstX > dstY)
			return 14*dstY + 10* (dstX-dstY); // this is the distance calculated
		return 14*dstX + 10 * (dstY-dstX);
	}
	int GetDistanceAstareuclidean(Node nodeA, Node nodeB) { //to get the distance between 2 given nodes, i think thsi si teh currently used heuristic w f hadi its at 15.22 mn video
		int dstX = nodeA.gridX - nodeB.gridX;
		int dstY = nodeA.gridY - nodeB.gridY;

		return (int) Math.Sqrt(dstX * dstX + dstY * dstY);
	}
	int GetDistanceAstarmanhattan(Node nodeA, Node nodeB) { //to get the distance between 2 given nodes, i think thsi si teh currently used heuristic w f hadi its at 15.22 mn video
		int dstX = nodeA.gridX - nodeB.gridX;
	    int dstY = nodeA.gridY - nodeB.gridY;

        return Math.Abs(dstX) + Math.Abs(dstY);
	}

	void RetracePathUCS(Node startNode, Node endNode) {
		List<Node> path = new List<Node>(); 
		Node currentNode = endNode; 

		while (currentNode != startNode) { 
			path.Add(currentNode);
			currentNode = currentNode.parent; 
		path.Add(startNode);
		path.Reverse(); 

		grid.pathUCS = path;

	}
	}
	void RetracePathBFS(Node startNode, Node endNode) {
		List<Node> path = new List<Node>(); 
		Node currentNode = endNode; 

		while (currentNode != startNode) { 
			path.Add(currentNode);
			currentNode = currentNode.parent; 
		path.Add(startNode);
		path.Reverse(); 

		grid.pathBFS = path;

	}
	}
	void RetracePathDFS(Node startNode, Node endNode) {
		List<Node> path = new List<Node>(); 
		Node currentNode = endNode; 

		while (currentNode != startNode) { 
			path.Add(currentNode);
			currentNode = currentNode.parent; 
		path.Add(startNode);
		path.Reverse(); 

		grid.pathDFS = path;

	}
	}

	IEnumerator walkminpath(){

		List<int> pathmin= new List<int> (){grid.pathDFS.Count,grid.pathBFS.Count,grid.pathUCS.Count,grid.path3.Count,grid.path2.Count,grid.path.Count};
		int min = pathmin.Min();
		int index= pathmin.FindIndex(x => x==min);
		List<Node> path=new List<Node>();
		if(index==0){
			path=grid.pathDFS;
		}
		else if(index==1){
			path=grid.pathBFS;
		}
		else if(index==2){
			path=grid.pathUCS;

		}
		else if(index==3){
			path=grid.path3;

		}
		else if(index==4){
			path=grid.path2;

		}
		else{
			path=grid.path;

		}
	foreach (Node node in path)
        {
            seeker.position = node.worldPosition;
            yield return new WaitForSeconds(0f); // wait for a short time before moving to the next node
        }
	}
}
/**
 *  Steiner Tree Problem with/without restrictions
 *  	
 * 	Interface graphique de M. Binh-Minh Bui Xuan.
 * 	
 *  Inspired by :
 *  	 L. Kou, G. Markowsky, and L. Berman,
 *		 A fast Algorithm for Steiner Trees, IBM Thomas J. Watson Research Center,
 *  	 http://aturing.umcs.maine.edu/~markov/SteinerTrees.pdf
 *  
 * 	@author Amine Benslimane    https://github.com/bnslmn
 * 	Master 1 STL,
 * 	Sorbonne University,
 * 	Mai 2021.
 * 
 * GPLv3 Licence, feel free to use this code.
 * */
package algorithms;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;

public class DefaultTeam {

	// definition de la contrainte budget
	public final static double BUDGET = 1664;

	/***
	 * 	Algorithme calcul de l'arbre de Steiner sans restriction de budget
	 * 
	 * 	@param points : Liste d'adjacence (points du graphe)
	 *  @param edgeThreshold : Seuil de tolerance (distance max d'une arete)
	 *  @param hitPoints : Sous ensemble des points du graphe que doit contenir l'arbre de Steiner.
	 * 
	 * */
	public Tree2D calculSteiner(ArrayList<Point> points, int edgeThreshold, ArrayList<Point> hitPoints) {

		// Algorithme de Floyd-Warshall

		// Distancy matrix
		double[][] dist = matDistance(points, edgeThreshold);
		// Paths matrix
		int[][] chemins = matChemin(points, edgeThreshold);

		// Construct the complete undirected distance graph G1 = (S, E1, d1)
		List<Edge> g1 = this.hitPointsEdges(hitPoints, points, dist);

		// Finding the minimal spanning tree T1 of G1. 
		//(If there are several minimal spanning trees, pick an arbitrary one.)
		List<Edge> t1 = computeKruskal(g1, hitPoints);

		// Construct the subgraph Gs of G by replacing each edges in T1 by ts corresponding shortest path in G.
		// If there are several shortest paths, pick an arbitrary one.)
		List<Point> gs = bfsAndOptimize(edgesToTree(t1), chemins, points);

		// Finding the minimal spanning tree, Ts, of Gs.
		// (If there are several minimal spanning trees, pick an arbitrary one.)
		List<Edge> ts = computeKruskal(pointsToEdges(gs), gs);

		return  edgesToTree(ts);
	}

	/***
	 * 	Algorithme calcul de l'arbre de Steiner avec restriction de budget
	 * 
	 * 	@param points : Liste d'adjacence (points du graphe)
	 *  @param edgeThreshold : Seuil de tolerance (distance max d'une arete)
	 *  @param hitPoints : Sous ensemble des points du graphe que doit contenir l'arbre de Steiner.
	 * 
	 * */
	public Tree2D calculSteinerBudget(ArrayList<Point> points, int edgeThreshold, ArrayList<Point> hitPoints) {
		// Application floyd-warshall sur tous les points et retourner la matrice des
		// dist et paths
		double[][] distance = matDistance(points, edgeThreshold);
		int[][] chemin = matChemin(points, edgeThreshold);

		// Construct the complete undirected distance graph G1 = (S, E1, d1)
		List<Edge> g1 = this.hitPointsEdges(hitPoints, points, distance);

		// Finding the minimal spanning tree T1 of G1. (If there are several minimal spanning trees, pick an arbitrary one)
		List<Edge> t1 = computeKruskal(g1, hitPoints);

		// Prim Algorithm : p1 = (Spanning Tree t1, HitPoints Vertex, BUDGET)
		List<Edge> p1 = computePrim(t1, hitPoints.get(0), BUDGET);

		// Construct the subgraph Gs of G by replacing each edges in P1 by ts corresponding shortest path in G.
		// If there are several shortest paths, pick an arbitrary one.)
		List<Edge> gs = computeShortestPathGraph(p1, chemin, points);

		return edgesToTree(gs);
	}

	
	/**
	 * matChemin : Calcul de la matrice des chemins
	 * @param points : Liste d'adjacence du graphe
	 * @param edgeThreshold : seuil de tolerance (distance max)
	 * @return matChemin : la matrice de distance (N x N) ou matDist[i][j] represente le chemin entre les points i et j
	 * ou +oo si elle depasse le seuil de tolerance
	 * */
	public int[][] matChemin(List<Point> points, int edgeThreshold) {

		int[][] matChemin = new int[points.size()][points.size()];
		for (int i = 0; i < matChemin.length; i++)
			for (int j = 0; j < matChemin.length; j++)
				matChemin[i][j] = i;

		double[][] dist = new double[points.size()][points.size()];

		for (int i = 0; i < matChemin.length; i++) {
			for (int j = 0; j < matChemin.length; j++) {
				//si dist(i,j) ne depasse pas le seuil ==> ajouter a la matrice
				if (points.get(i).distance(points.get(j)) <= edgeThreshold)
					dist[i][j] = points.get(i).distance(points.get(j));
				//si meme point ==> dist = 0 et echappement
				else if (i == j) {
					dist[i][i] = 0;
					continue;
				} else 
					// il n'y a pas d'aretes entre i et j
					dist[i][j] = Double.POSITIVE_INFINITY;
				
				matChemin[i][j] = j;
			}
		}

		//reparcourir pour trouver un chemin de distance moindre que l'actuel
		for (int k = 0; k < matChemin.length; k++) 
			for (int i = 0; i < matChemin.length; i++) 
				for (int j = 0; j < matChemin.length; j++) 
					if (dist[i][j] > dist[i][k] + dist[k][j]) { //la distance min doit passer par k !
						dist[i][j] = dist[i][k] + dist[k][j];
						matChemin[i][j] = matChemin[i][k];
					}
		return matChemin;
	}
	
	/**
	 * matDistance : Calcul de la matrice de distance
	 * @param points : Liste d'adjacence du graphe
	 * @param edgeThreshold : seuil de tolerance (distance max)
	 * @return matDist : la matrice de distance (N x N) ou matDist[i][j] represente la distance entre les points i et j
	 * ou +oo si elle depasse le seuil de tolerance
	 * */
	public double[][] matDistance(ArrayList<Point> points, int edgeThreshold) {

		// (Matrice N x N) , tq N nbr de points
		double[][] matDist = new double[points.size()][points.size()];

		for (int i = 0; i < matDist.length; i++)
			for (int j = 0; j < matDist.length; j++) {

				double dist = points.get(i).distance(points.get(j));

				// Test sur la distance seuil
				if (dist <= edgeThreshold) 
					matDist[i][j] = dist; 	// si non atteint --> affecter la valeur
				else 
					matDist[i][j] = Double.POSITIVE_INFINITY; // sinon (seuil atteint) --> affecter +oo
			}
		

		//un chemin de i vers j passant par k est minimal --> choisir ce chemin.
		for (int k = 0; k < matDist.length; k++)
			for (int i = 0; i < matDist.length; i++) 
				for (int j = 0; j < matDist.length; j++)
					if (matDist[i][j] > matDist[i][k] + matDist[k][j])  matDist[i][j] = matDist[i][k] + matDist[k][j];
		
		return matDist;
	}
	
	/**
	 * ppChemin :	    plus court chemin d'un point x vers un point y
	 * @param points 	l'ensemble des points du graphe
	 * @param chemins   la matrice des chemins
	 * @param x         point x
	 * @param y         point y
	 * @return pts		Nouvel ensemble des points du graphe
	 */
	private List<Point> ppChemin(List<Point> points, int[][] chemins, Point x, Point y) {
		List<Point> pts = new ArrayList<Point>();

		int indX = points.indexOf(x); //indx de x
		int indY = points.indexOf(y); //indx de y
		int aux = chemins[indX][indY];//premier point du chemin (x -> y)
		// condition d'arret : point y atteint
		while (aux != indY) {
			// ajouter l'element aux au plus court chemin de x vers y
			pts.add(points.get(aux));
			//continuer parcours du chemin
			aux = chemins[aux][indY];
		}
		return pts;
	}

	/**
	 * dfsAndOptimize :  parcourir en largeur et affecter a l'arete (u,v) le plus court chemin possible de u vers v.
	 * @param tree      Arbre
	 * @param chemins  Matrice des chemins de l'arbre
	 * @param points  Vecteur des points de l'arbre
	 * @return listPoints	Nouvel ensemble des points de G
	 */
	private List<Point> bfsAndOptimize(Tree2D tree, int[][] chemins, ArrayList<Point> points) {
		List<Point> listPoints = new ArrayList<Point>();
		listPoints.add(tree.getRoot());
		Queue<Tree2D> queue = new LinkedList<>();
		queue.add(tree);
		Tree2D aux;
		while (queue.size() != 0) {
			aux = queue.poll(); 			// enfilement de l'arbre en tete de la queue
			for (Tree2D subTrees : aux.getSubTrees()) {
				listPoints.addAll(ppChemin(points, chemins, aux.getRoot(), subTrees.getRoot())); //recuperer le plus court chemin entre aux et subTrees
				listPoints.add(subTrees.getRoot());				// ajouter les sous arbres de la racine a la liste
				queue.add(subTrees);				// enfiler les sous arbres (en attente de parcours)
			}
		}
		return listPoints;
	}



	/**
	 * computeKruskal :  
	 * Algorithme de Kruskal retournant un arbre couvrant de poids minimum.
	 * 
	 * @param graph  		Liste d'adjacence du graphe.
	 * @param noeuds 		Liste des noeuds du graphe.
	 * @return kruskalGraph L'arbre de Kruskal associe.
	 */
	private List<Edge> computeKruskal(List<Edge> graph, List<Point> noeuds) {
		// les aretes sont triees par ordre croissant
		Collections.sort(graph);
		// Map (Point , tag) 
		Map<Point, Integer> tags = this.tag(noeuds);
		//initialisation du graphe Kruskal
		List<Edge> kruskalGraph = new ArrayList<Edge>();
		for (Edge e : graph) {
			// si l'ajout de l'arete cree un cycle, ignorer l'arete
			if (tags.get(e.getU()) == tags.get(e.getV()))
				continue;
			
			// sinon, ajouter l'arete au graphe
			kruskalGraph.add(e);
			// extremite 1 de l'arete ajoutee
			Integer tagX = tags.get(e.getU());
			// extremite 2 de l'arete ajoutee
			Integer tagY = tags.get(e.getV());
			// point avec tag x => remplacer avec tag y
			for (Edge edge : kruskalGraph) {
				//extr 1 avec tag x ==> tag y
				if (tags.get(edge.getU()) == tagX)
					tags.replace(edge.getU(), tagX, tagY);
				//extr 2 avec tag x ==> tag y
				if (tags.get(edge.getV()) == tagX)
					tags.replace(edge.getV(), tagX, tagY);
			}
		}
		return kruskalGraph;
	}

	/**
	 *	Construction de l'arbre 2D avec une liste d'adjacence.
	 * 
	 * @param edges Le graphe sous forme d'une liste d'aretes
	 * @return tree L'arbre correspondant
	 */
	private Tree2D edgesToTree(List<Edge> edges) {
		Edge e = edges.get(0);
		Tree2D tree;
		Tree2D aux;
		Queue<Tree2D> queue = new LinkedList<Tree2D>();
		
		// racine de l'arbre
		tree = new Tree2D(e.getU(), new ArrayList<Tree2D>());
		queue.add(tree);
		
		while (!queue.isEmpty()) {
			//tete de la file
			aux = queue.poll();
			//aretes deja visitees
			List<Edge> visitedEdges = new ArrayList<>();
			
			for (int i = 0; i < edges.size(); i++) {
				Edge edge = edges.get(i);
				Point u = edge.getU();
				Point v = edge.getV();
				// si extr1 = (racine de l'actuelle aux) alors extr2 est son fils
				if (u.equals(aux.getRoot())) {
					//creation du fils 
					Tree2D fils = new Tree2D(v, new ArrayList<Tree2D>());
					//enfilement du fils
					queue.add(fils);
					aux.getSubTrees().add(fils);
					// arete visitee
					visitedEdges.add(edge);
				}
				// le cas analogue pour extr2
				if (v.equals(aux.getRoot())) {
					Tree2D fils = new Tree2D(u, new ArrayList<Tree2D>());
					queue.add(fils);
					aux.getSubTrees().add(fils);
					visitedEdges.add(edge);
				}
			}
			//enlever tout les noeuds visites
			for (Edge i : visitedEdges)
				edges.remove(i);		
		}
		//System.out.println("File vide, arbre completement traite !");
		
		return tree;
	}

	/**
	 * pointsToEdges a partir d'une liste de point, construire une liste d'adjacence.
	 * 
	 * @param pts Liste des points du graphe
	 * @return edges Liste des aretes
	 */
	private List<Edge> pointsToEdges(List<Point> pts) {
		int size = pts.size();
		List<Edge> edges = new ArrayList<Edge>();
		for (int i = 0; i < size - 1; i++)
			for (int j = i + 1; j < size; j++) 
				edges.add(new Edge(pts.get(i), pts.get(j)));
		return edges;
	}

	/**
	 * Calcul du graphe ne contenant que les HitPoints comme sommets
	 *  d'aretes de distance minimales
	 * 
	 * @param hitpoints Les sommets HitPoints
	 * @param pts    Points du graphe
	 * @param matDst      Matrice de distance minimale
	 * @return edges Liste d'adjacence du graphe ne contenant que les hitPoints comme sommets d'aretes de distance min
	 */
	private List<Edge> hitPointsEdges (List<Point> hitpoints, List<Point> pts, double[][] matDst) {
		List<Edge> edges = new ArrayList<Edge>();
		
		for (int i = 0; i < hitpoints.size() - 1; i++)
			for (int j = i + 1; j < hitpoints.size(); j++) {
				Point a = hitpoints.get(i);
				Point b = hitpoints.get(j);
				edges.add(new Edge(a, b,
						matDst[pts.indexOf(a)][pts.indexOf(b)]));
			}
		return edges;
	}

	/**
	 * tag :  Attribuer pour chaque point un tag de type entier.
	 * @param pts Liste de points du graphe
	 * @return	tagMap = (point, tag).	Pour chaque point ==> un tag
	 */
	private Map<Point, Integer> tag(List<Point> pts) {
		Map<Point, Integer> tagMap = new HashMap<>();
		int tag = 0;
		for (Point pt : pts)
			tagMap.put(pt, tag++);
		return tagMap;
	}

	/**
	 * pointToHeapMinEdges :	Construction d'un tas min pour tout point du graphe.
	 * Chaque tas contient les aretes dont le point est une extremite
	 * @param graph Liste d'adjacence du graphe
	 * @return heapMins = (point, tas_min)
	 */
	private Map<Point, PriorityQueue<Edge>> pointToHeapMinEdges(List<Edge> graph) {
		Map<Point, PriorityQueue<Edge>> heapMins = new
												HashMap<Point, PriorityQueue<Edge>>();
		for (Edge edge : graph) {
			Point u = edge.getU();
			Point v = edge.getV();
			if (!heapMins.containsKey(u))
				heapMins.put(u, new PriorityQueue<Edge>());
			if (!heapMins.containsKey(v))
				heapMins.put(v, new PriorityQueue<Edge>());
			heapMins.get(u).add(edge);
			heapMins.get(v).add(edge);
		}
		return heapMins;
	}

	/**
	 * minEdge : Retourne l'arete de cout minimal parmis une liste de point fournie par l'algorithme Prim
	 * 
	 * @param pointsToEdge mapping points --> (tas min de edges)
	 * @param primPoints   Liste de points fournie par l'algorithme Prim
	 * @return minEdge : l'arete de cout min a ajouter
	 */
	private Edge minEdge(Map<Point, PriorityQueue<Edge>> pointsToEdge, List<Point> primPoints) {		
		//un tas min d'arete min
		Queue<Edge> heapEdges = new PriorityQueue<Edge>();
		for (Point p : primPoints) {
			if (pointsToEdge.get(p).isEmpty())
				continue;
			// propriete d'un tas min : arete de cout min == premier element
			heapEdges.add(pointsToEdge.get(p).peek());
		}
		//l'arete de cout minimal est le premier element du tas min d'aretes minimales
		return heapEdges.peek();
	}
	
	/**
	 * computeShortestPathGraph : Toutes les aretes(u,v) du graphes recoivent le plus court chemin de u vers v
	 * 
	 * @param listEdge  Liste d'adjacence du graphe
	 * @param chemins  la matrice des chemins du graphe
	 * @param pts La liste des points du graphe
	 * @return Liste d'adjacence ou les aretes forment un plus court chemin 
	 */
	private List<Edge> computeShortestPathGraph(List<Edge> listEdge, int[][] chemins, List<Point> pts) {
		List<Edge> shortestPathEdges = new ArrayList<Edge>();
		for (Edge e : listEdge) {
			int u = pts.indexOf(e.getU());
			int v = pts.indexOf(e.getV());
			// condition d'arret : on arrive vers v
			while (u != v) {
				// le cas : "(u,v) est deja un ppch" est inclus
				shortestPathEdges.add(
						new Edge(pts.get(u), pts.get(chemins[u][v])));
				u = chemins[u][v];
			}
		}
		return shortestPathEdges;
	}

	/**
	 * 	Algorithme Prim 
	 * 
	 * @param listofedge     Liste d'adjacence du graphe
	 * @param hitpointsVertex Premier sommet des HitPoints (vertex)
	 * @param budget       le budget a ne pas depasser
	 * @return Liste d'adjacence output Prim
	 */
	private List<Edge> computePrim(List<Edge> listofedge, Point hitpointsVertex, double budget) {
		List<Edge> prim = new ArrayList<Edge>();
		// mapping point --> tas min d'aretes
		Map<Point, PriorityQueue<Edge>> pointEdgesMap = pointToHeapMinEdges(listofedge);
		// listes de points a ajouter a l'arbre de Steiner
		List<Point> addPoints = new ArrayList<Point>();
		
		//ajouter la maison mere aux points ajoutes
		addPoints.add(hitpointsVertex);
		//initialisation du cout
		double cout = 0;
		
		boolean budgetAtteint=false;
		while (!budgetAtteint) {
			// arete de cout min
			Edge minedge = minEdge(pointEdgesMap, addPoints);
			Point u = minedge.getU();
			Point v = minedge.getV();

			// Ajouter les deux extremites (points) de cette arete aux points ajoutes
			if (!addPoints.contains(u))
				addPoints.add(u);
			if (!addPoints.contains(v))
				addPoints.add(v);

			// la supprimer de la map
			pointEdgesMap.get(u).remove(minedge);
			pointEdgesMap.get(v).remove(minedge);
			
			
			// si on atteint le seuil, echappement
			if (cout + minedge.getDist() >= budget) {
				budgetAtteint = true;
				break;}
			
			// sinon (budget restant suffisant), accumuler le cout
			cout += minedge.getDist();
			// finalement, ajout de l'arete a la liste des output de l'algorithme
			prim.add(minedge);
		}

		return prim;
	}

	

}
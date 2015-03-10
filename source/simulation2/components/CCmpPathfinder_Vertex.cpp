/* Copyright (C) 2014 Wildfire Games.
 * This file is part of 0 A.D.
 *
 * 0 A.D. is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * 0 A.D. is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with 0 A.D.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file
 * Vertex-based algorithm for CCmpPathfinder.
 * Computes paths around the corners of rectangular obstructions.
 *
 * Useful search term for this algorithm: "points of visibility".
 *
 * Since we sometimes want to use this for avoiding moving units, there is no
 * pre-computation - the whole visibility graph is effectively regenerated for
 * each path, and it does A* over that graph.
 *
 * This scales very poorly in the number of obstructions, so it should be used
 * with a limited range and not exceedingly frequently.
 */

#include "precompiled.h"

#include "CCmpPathfinder_Common.h"

#include "lib/timer.h"
#include "ps/Profile.h"
#include "simulation2/components/ICmpObstructionManager.h"
#include "simulation2/helpers/PriorityQueue.h"
#include "simulation2/helpers/Render.h"

/* Quadrant optimisation:
 * (loosely based on GPG2 "Optimizing Points-of-Visibility Pathfinding")
 *
 * Consider the vertex ("@") at a corner of an axis-aligned rectangle ("#"):
 *
 * TL  :  TR
 *     :
 * ####@ - - -
 * #####
 * #####
 * BL ##  BR
 *
 * The area around the vertex is split into TopLeft, BottomRight etc quadrants.
 *
 * If the shortest path reaches this vertex, it cannot continue to a vertex in
 * the BL quadrant (it would be blocked by the shape).
 * Since the shortest path is wrapped tightly around the edges of obstacles,
 * if the path approached this vertex from the TL quadrant,
 * it cannot continue to the TL or TR quadrants (the path could be shorter if it
 * skipped this vertex).
 * Therefore it must continue to a vertex in the BR quadrant (so this vertex is in
 * *that* vertex's TL quadrant).
 *
 * That lets us significantly reduce the search space by quickly discarding vertexes
 * from the wrong quadrants.
 *
 * (This causes badness if the path starts from inside the shape, so we add some hacks
 * for that case.)
 *
 * (For non-axis-aligned rectangles it's harder to do this computation, so we'll
 * not bother doing any discarding for those.)
 */
static const u8 QUADRANT_NONE = 0;
static const u8 QUADRANT_BL = 1;
static const u8 QUADRANT_TR = 2;
static const u8 QUADRANT_TL = 4;
static const u8 QUADRANT_BR = 8;
static const u8 QUADRANT_BLTR = QUADRANT_BL|QUADRANT_TR;
static const u8 QUADRANT_TLBR = QUADRANT_TL|QUADRANT_BR;
static const u8 QUADRANT_ALL = QUADRANT_BLTR|QUADRANT_TLBR;

// A vertex around the corners of an obstruction
// (paths will be sequences of these vertexes)
struct Vertex
{
	enum
	{
		UNEXPLORED,
		OPEN,
		CLOSED,
	};

	CFixedVector2D p;
	fixed g, h;
	u16 pred;
	u8 status;
	u8 quadInward : 4; // the quadrant which is inside the shape (or NONE)
	u8 quadOutward : 4; // the quadrants of the next point on the path which this vertex must be in, given 'pred'
};

// Obstruction edges (paths will not cross any of these).
// Defines the two points of the edge.
struct Edge
{
	CFixedVector2D p0, p1;
};

// Axis-aligned obstruction squares (paths will not cross any of these).
// Defines the opposing corners of an axis-aligned square
// (from which four individual edges can be trivially computed), requiring p0 <= p1
struct Square
{
	CFixedVector2D p0, p1;
};

// Axis-aligned obstruction edges.
// p0 defines one end; c1 is either the X or Y coordinate of the other end,
// depending on the context in which this is used.
struct EdgeAA
{
	CFixedVector2D p0;
	fixed c1;
};

// When computing vertexes to insert into the search graph,
// add a small delta so that the vertexes of an edge don't get interpreted
// as crossing the edge (given minor numerical inaccuracies)
static const entity_pos_t EDGE_EXPAND_DELTA = entity_pos_t::FromInt(1)/4;

/**
 * Check whether a ray from 'a' to 'b' crosses any of the edges.
 * (Edges are one-sided so it's only considered a cross if going from front to back.)
 */
inline static bool CheckVisibility(CFixedVector2D a, CFixedVector2D b, const std::vector<Edge>& edges)
{
	CFixedVector2D abn = (b - a).Perpendicular();

	// Edges of general non-axis-aligned shapes
	for (size_t i = 0; i < edges.size(); ++i)
	{
		CFixedVector2D p0 = edges[i].p0;
		CFixedVector2D p1 = edges[i].p1;

		CFixedVector2D d = (p1 - p0).Perpendicular();

		// If 'a' is behind the edge, we can't cross
		fixed q = (a - p0).Dot(d);
		if (q < fixed::Zero())
			continue;

		// If 'b' is in front of the edge, we can't cross
		fixed r = (b - p0).Dot(d);
		if (r > fixed::Zero())
			continue;

		// The ray is crossing the infinitely-extended edge from in front to behind.
		// Check the finite edge is crossing the infinitely-extended ray too.
		// (Given the previous tests, it can only be crossing in one direction.)
		fixed s = (p0 - a).Dot(abn);
		if (s > fixed::Zero())
			continue;

		fixed t = (p1 - a).Dot(abn);
		if (t < fixed::Zero())
			continue;

		return false;
	}

	return true;
}

// Handle the axis-aligned shape edges separately (for performance):
// (These are specialised versions of the general unaligned edge code.
// They assume the caller has already excluded edges for which 'a' is
// on the wrong side.)

inline static bool CheckVisibilityLeft(CFixedVector2D a, CFixedVector2D b, const std::vector<EdgeAA>& edges)
{
	if (a.X >= b.X)
		return true;

	CFixedVector2D abn = (b - a).Perpendicular();

	for (size_t i = 0; i < edges.size(); ++i)
	{
		if (b.X < edges[i].p0.X)
			continue;

		CFixedVector2D p0 (edges[i].p0.X, edges[i].c1);
		fixed s = (p0 - a).Dot(abn);
		if (s > fixed::Zero())
			continue;

		CFixedVector2D p1 (edges[i].p0.X, edges[i].p0.Y);
		fixed t = (p1 - a).Dot(abn);
		if (t < fixed::Zero())
			continue;

		return false;
	}

	return true;
}

inline static bool CheckVisibilityRight(CFixedVector2D a, CFixedVector2D b, const std::vector<EdgeAA>& edges)
{
	if (a.X <= b.X)
		return true;

	CFixedVector2D abn = (b - a).Perpendicular();

	for (size_t i = 0; i < edges.size(); ++i)
	{
		if (b.X > edges[i].p0.X)
			continue;

		CFixedVector2D p0 (edges[i].p0.X, edges[i].c1);
		fixed s = (p0 - a).Dot(abn);
		if (s > fixed::Zero())
			continue;

		CFixedVector2D p1 (edges[i].p0.X, edges[i].p0.Y);
		fixed t = (p1 - a).Dot(abn);
		if (t < fixed::Zero())
			continue;

		return false;
	}

	return true;
}

inline static bool CheckVisibilityBottom(CFixedVector2D a, CFixedVector2D b, const std::vector<EdgeAA>& edges)
{
	if (a.Y >= b.Y)
		return true;

	CFixedVector2D abn = (b - a).Perpendicular();

	for (size_t i = 0; i < edges.size(); ++i)
	{
		if (b.Y < edges[i].p0.Y)
			continue;

		CFixedVector2D p0 (edges[i].p0.X, edges[i].p0.Y);
		fixed s = (p0 - a).Dot(abn);
		if (s > fixed::Zero())
			continue;

		CFixedVector2D p1 (edges[i].c1, edges[i].p0.Y);
		fixed t = (p1 - a).Dot(abn);
		if (t < fixed::Zero())
			continue;

		return false;
	}

	return true;
}

inline static bool CheckVisibilityTop(CFixedVector2D a, CFixedVector2D b, const std::vector<EdgeAA>& edges)
{
	if (a.Y <= b.Y)
		return true;

	CFixedVector2D abn = (b - a).Perpendicular();

	for (size_t i = 0; i < edges.size(); ++i)
	{
		if (b.Y > edges[i].p0.Y)
			continue;

		CFixedVector2D p0 (edges[i].p0.X, edges[i].p0.Y);
		fixed s = (p0 - a).Dot(abn);
		if (s > fixed::Zero())
			continue;

		CFixedVector2D p1 (edges[i].c1, edges[i].p0.Y);
		fixed t = (p1 - a).Dot(abn);
		if (t < fixed::Zero())
			continue;

		return false;
	}

	return true;
}


static CFixedVector2D NearestPointOnGoal(CFixedVector2D pos, const PathGoal& goal)
{
	CFixedVector2D g(goal.x, goal.z);

	switch (goal.type)
	{
	case PathGoal::POINT:
	{
		return g;
	}

	case PathGoal::CIRCLE:
	case PathGoal::INVERTED_CIRCLE:
	{
		CFixedVector2D d = pos - g;
		if (d.IsZero())
			d = CFixedVector2D(fixed::FromInt(1), fixed::Zero()); // some arbitrary direction
		d.Normalize(goal.hw);
		return g + d;
	}

	case PathGoal::SQUARE:
	case PathGoal::INVERTED_SQUARE:
	{
		CFixedVector2D halfSize(goal.hw, goal.hh);
		CFixedVector2D d = pos - g;
		return g + Geometry::NearestPointOnSquare(d, goal.u, goal.v, halfSize);
	}

	default:
		debug_warn("invalid type");
		return CFixedVector2D();
	}
}

CFixedVector2D CCmpPathfinder::GetNearestPointOnGoal(CFixedVector2D pos, const PathGoal& goal)
{
	return NearestPointOnGoal(pos, goal);
	// (It's intentional that we don't put the implementation inside this
	// function, to avoid the (admittedly unmeasured and probably trivial)
	// cost of a virtual call inside ComputeShortPath)
}

typedef PriorityQueueHeap<u16, fixed> PriorityQueue;

/**
 * Add edges and vertexes to represent the boundaries between passable and impassable
 * navcells (for impassable terrain and for static obstruction shapes).
 * Navcells i0 <= i <= i1, j0 <= j <= j1 will be considered.
 */
static void AddTerrainEdges(std::vector<Edge>& edges, std::vector<Vertex>& vertexes,
	int i0, int j0, int i1, int j1,
	pass_class_t passClass, const Grid<NavcellData>& grid)
{
	PROFILE("AddTerrainEdges");

	// Clamp the coordinates so we won't attempt to sample outside of the grid.
	// (This assumes the outermost ring of navcells (which are always impassable)
	// won't have a boundary with any passable navcells. TODO: is that definitely
	// safe enough?)

	i0 = clamp(i0, 1, grid.m_W-2);
	j0 = clamp(j0, 1, grid.m_H-2);
	i1 = clamp(i1, 1, grid.m_W-2);
	j1 = clamp(j1, 1, grid.m_H-2);

	for (int j = j0; j <= j1; ++j)
	{
		for (int i = i0; i <= i1; ++i)
		{
			if (IS_PASSABLE(grid.get(i, j), passClass))
				continue;

			if (IS_PASSABLE(grid.get(i+1, j), passClass) && IS_PASSABLE(grid.get(i, j+1), passClass) && IS_PASSABLE(grid.get(i+1, j+1), passClass))
			{
				Vertex vert;
				vert.status = Vertex::UNEXPLORED;
				vert.quadOutward = QUADRANT_ALL;
				vert.quadInward = QUADRANT_BL;
				vert.p = CFixedVector2D(fixed::FromInt(i+1)+EDGE_EXPAND_DELTA, fixed::FromInt(j+1)+EDGE_EXPAND_DELTA).Multiply(Pathfinding::NAVCELL_SIZE);
				vertexes.push_back(vert);
			}

			if (IS_PASSABLE(grid.get(i-1, j), passClass) && IS_PASSABLE(grid.get(i, j+1), passClass) && IS_PASSABLE(grid.get(i-1, j+1), passClass))
			{
				Vertex vert;
				vert.status = Vertex::UNEXPLORED;
				vert.quadOutward = QUADRANT_ALL;
				vert.quadInward = QUADRANT_BR;
				vert.p = CFixedVector2D(fixed::FromInt(i)-EDGE_EXPAND_DELTA, fixed::FromInt(j+1)+EDGE_EXPAND_DELTA).Multiply(Pathfinding::NAVCELL_SIZE);
				vertexes.push_back(vert);
			}

			if (IS_PASSABLE(grid.get(i+1, j), passClass) && IS_PASSABLE(grid.get(i, j-1), passClass) && IS_PASSABLE(grid.get(i+1, j-1), passClass))
			{
				Vertex vert;
				vert.status = Vertex::UNEXPLORED;
				vert.quadOutward = QUADRANT_ALL;
				vert.quadInward = QUADRANT_TL;
				vert.p = CFixedVector2D(fixed::FromInt(i+1)+EDGE_EXPAND_DELTA, fixed::FromInt(j)-EDGE_EXPAND_DELTA).Multiply(Pathfinding::NAVCELL_SIZE);
				vertexes.push_back(vert);
			}

			if (IS_PASSABLE(grid.get(i-1, j), passClass) && IS_PASSABLE(grid.get(i, j-1), passClass) && IS_PASSABLE(grid.get(i-1, j-1), passClass))
			{
				Vertex vert;
				vert.status = Vertex::UNEXPLORED;
				vert.quadOutward = QUADRANT_ALL;
				vert.quadInward = QUADRANT_TR;
				vert.p = CFixedVector2D(fixed::FromInt(i)-EDGE_EXPAND_DELTA, fixed::FromInt(j)-EDGE_EXPAND_DELTA).Multiply(Pathfinding::NAVCELL_SIZE);
				vertexes.push_back(vert);
			}
		}
	}

	// XXX rewrite this stuff

	for (int j = j0; j < j1; ++j)
	{
		std::vector<u16> segmentsR;
		std::vector<u16> segmentsL;

		for (int i = i0; i <= i1; ++i)
		{
			bool a = IS_PASSABLE(grid.get(i, j+1), passClass);
			bool b = IS_PASSABLE(grid.get(i, j), passClass);
			if (a && !b)
				segmentsL.push_back(i);
			if (b && !a)
				segmentsR.push_back(i);
		}

		if (!segmentsR.empty())
		{
			segmentsR.push_back(0); // sentinel value to simplify the loop
			u16 ia = segmentsR[0];
			u16 ib = ia + 1;
			for (size_t n = 1; n < segmentsR.size(); ++n)
			{
				if (segmentsR[n] == ib)
					++ib;
				else
				{
					CFixedVector2D v0 = CFixedVector2D(fixed::FromInt(ia), fixed::FromInt(j+1)).Multiply(Pathfinding::NAVCELL_SIZE);
					CFixedVector2D v1 = CFixedVector2D(fixed::FromInt(ib), fixed::FromInt(j+1)).Multiply(Pathfinding::NAVCELL_SIZE);
					Edge e = { v0, v1 };
					edges.push_back(e);

					ia = segmentsR[n];
					ib = ia + 1;
				}
			}
		}

		if (!segmentsL.empty())
		{
			segmentsL.push_back(0); // sentinel value to simplify the loop
			u16 ia = segmentsL[0];
			u16 ib = ia + 1;
			for (size_t n = 1; n < segmentsL.size(); ++n)
			{
				if (segmentsL[n] == ib)
					++ib;
				else
				{
					CFixedVector2D v0 = CFixedVector2D(fixed::FromInt(ib), fixed::FromInt(j+1)).Multiply(Pathfinding::NAVCELL_SIZE);
					CFixedVector2D v1 = CFixedVector2D(fixed::FromInt(ia), fixed::FromInt(j+1)).Multiply(Pathfinding::NAVCELL_SIZE);
					Edge e = { v0, v1 };
					edges.push_back(e);

					ia = segmentsL[n];
					ib = ia + 1;
				}
			}
		}
	}

	for (int i = i0; i < i1; ++i)
	{
		std::vector<u16> segmentsU;
		std::vector<u16> segmentsD;

		for (int j = j0; j <= j1; ++j)
		{
			bool a = IS_PASSABLE(grid.get(i+1, j), passClass);
			bool b = IS_PASSABLE(grid.get(i, j), passClass);
			if (a && !b)
				segmentsU.push_back(j);
			if (b && !a)
				segmentsD.push_back(j);
		}

		if (!segmentsU.empty())
		{
			segmentsU.push_back(0); // sentinel value to simplify the loop
			u16 ja = segmentsU[0];
			u16 jb = ja + 1;
			for (size_t n = 1; n < segmentsU.size(); ++n)
			{
				if (segmentsU[n] == jb)
					++jb;
				else
				{
					CFixedVector2D v0 = CFixedVector2D(fixed::FromInt(i+1), fixed::FromInt(ja)).Multiply(Pathfinding::NAVCELL_SIZE);
					CFixedVector2D v1 = CFixedVector2D(fixed::FromInt(i+1), fixed::FromInt(jb)).Multiply(Pathfinding::NAVCELL_SIZE);
					Edge e = { v0, v1 };
					edges.push_back(e);

					ja = segmentsU[n];
					jb = ja + 1;
				}
			}
		}

		if (!segmentsD.empty())
		{
			segmentsD.push_back(0); // sentinel value to simplify the loop
			u16 ja = segmentsD[0];
			u16 jb = ja + 1;
			for (size_t n = 1; n < segmentsD.size(); ++n)
			{
				if (segmentsD[n] == jb)
					++jb;
				else
				{
					CFixedVector2D v0 = CFixedVector2D(fixed::FromInt(i+1), fixed::FromInt(jb)).Multiply(Pathfinding::NAVCELL_SIZE);
					CFixedVector2D v1 = CFixedVector2D(fixed::FromInt(i+1), fixed::FromInt(ja)).Multiply(Pathfinding::NAVCELL_SIZE);
					Edge e = { v0, v1 };
					edges.push_back(e);

					ja = segmentsD[n];
					jb = ja + 1;
				}
			}
		}
	}
}

static void SplitAAEdges(CFixedVector2D a,
		const std::vector<Edge>& edges,
		const std::vector<Square>& squares,
		std::vector<Edge>& edgesUnaligned,
		std::vector<EdgeAA>& edgesLeft, std::vector<EdgeAA>& edgesRight,
		std::vector<EdgeAA>& edgesBottom, std::vector<EdgeAA>& edgesTop)
{
	edgesLeft.reserve(squares.size());
	edgesRight.reserve(squares.size());
	edgesBottom.reserve(squares.size());
	edgesTop.reserve(squares.size());

	for (size_t i = 0; i < squares.size(); ++i)
	{
		if (a.X <= squares[i].p0.X)
		{
			EdgeAA e = { squares[i].p0, squares[i].p1.Y };
			edgesLeft.push_back(e);
		}
		if (a.X >= squares[i].p1.X)
		{
			EdgeAA e = { squares[i].p1, squares[i].p0.Y };
			edgesRight.push_back(e);
		}
		if (a.Y <= squares[i].p0.Y)
		{
			EdgeAA e = { squares[i].p0, squares[i].p1.X };
			edgesBottom.push_back(e);
		}
		if (a.Y >= squares[i].p1.Y)
		{
			EdgeAA e = { squares[i].p1, squares[i].p0.X };
			edgesTop.push_back(e);
		}
	}

	for (size_t i = 0; i < edges.size(); ++i)
	{
		if (edges[i].p0.X == edges[i].p1.X)
		{
			if (edges[i].p1.Y < edges[i].p0.Y)
			{
				if (!(a.X <= edges[i].p0.X))
					continue;
				EdgeAA e = { edges[i].p1, edges[i].p0.Y };
				edgesLeft.push_back(e);
			}
			else
			{
				if (!(a.X >= edges[i].p0.X))
					continue;
				EdgeAA e = { edges[i].p1, edges[i].p0.Y };
				edgesRight.push_back(e);
			}
		}
		else if (edges[i].p0.Y == edges[i].p1.Y)
		{
			if (edges[i].p0.X < edges[i].p1.X)
			{
				if (!(a.Y <= edges[i].p0.Y))
					continue;
				EdgeAA e = { edges[i].p0, edges[i].p1.X };
				edgesBottom.push_back(e);
			}
			else
			{
				if (!(a.Y >= edges[i].p0.Y))
					continue;
				EdgeAA e = { edges[i].p0, edges[i].p1.X };
				edgesTop.push_back(e);
			}
		}
		else
		{
			edgesUnaligned.push_back(edges[i]);
		}
	}
}

/**
 * Functor for sorting edge-squares by approximate proximity to a fixed point.
 */
struct SquareSort
{
	CFixedVector2D src;
	SquareSort(CFixedVector2D src) : src(src) { }
	bool operator()(const Square& a, const Square& b)
	{
		if ((a.p0 - src).CompareLength(b.p0 - src) < 0)
			return true;
		return false;
	}
};

void CCmpPathfinder::ComputeShortPath(const IObstructionTestFilter& filter,
	entity_pos_t x0, entity_pos_t z0, entity_pos_t r,
	entity_pos_t range, const PathGoal& goal, pass_class_t passClass, WaypointPath& path)
{

	PROFILE3("ComputeShortPath");
	TIMER(L"ComputeShortPath");

	m_DebugOverlayShortPathLines.clear();

	if (m_DebugOverlay)
	{
		// Render the goal shape
		m_DebugOverlayShortPathLines.push_back(SOverlayLine());
		m_DebugOverlayShortPathLines.back().m_Color = CColor(1, 0, 0, 1);
		switch (goal.type)
		{
		case PathGoal::POINT:
		{
			SimRender::ConstructCircleOnGround(GetSimContext(), goal.x.ToFloat(), goal.z.ToFloat(), 0.2f, m_DebugOverlayShortPathLines.back(), true);
			break;
		}
		case PathGoal::CIRCLE:
		case PathGoal::INVERTED_CIRCLE:
		{
			SimRender::ConstructCircleOnGround(GetSimContext(), goal.x.ToFloat(), goal.z.ToFloat(), goal.hw.ToFloat(), m_DebugOverlayShortPathLines.back(), true);
			break;
		}
		case PathGoal::SQUARE:
		case PathGoal::INVERTED_SQUARE:
		{
			float a = atan2f(goal.v.X.ToFloat(), goal.v.Y.ToFloat());
			SimRender::ConstructSquareOnGround(GetSimContext(), goal.x.ToFloat(), goal.z.ToFloat(), goal.hw.ToFloat()*2, goal.hh.ToFloat()*2, a, m_DebugOverlayShortPathLines.back(), true);
			break;
		}
		}
	}

	// List of collision edges - paths must never cross these.
	// (Edges are one-sided so intersections are fine in one direction, but not the other direction.)
	std::vector<Edge> edges;
	std::vector<Square> edgeSquares; // axis-aligned squares; equivalent to 4 edges

	// Create impassable edges at the max-range boundary, so we can't escape the region
	// where we're meant to be searching
	fixed rangeXMin = x0 - range;
	fixed rangeXMax = x0 + range;
	fixed rangeZMin = z0 - range;
	fixed rangeZMax = z0 + range;
	{
		// (The edges are the opposite direction to usual, so it's an inside-out square)
		Edge e0 = { CFixedVector2D(rangeXMin, rangeZMin), CFixedVector2D(rangeXMin, rangeZMax) };
		Edge e1 = { CFixedVector2D(rangeXMin, rangeZMax), CFixedVector2D(rangeXMax, rangeZMax) };
		Edge e2 = { CFixedVector2D(rangeXMax, rangeZMax), CFixedVector2D(rangeXMax, rangeZMin) };
		Edge e3 = { CFixedVector2D(rangeXMax, rangeZMin), CFixedVector2D(rangeXMin, rangeZMin) };
		edges.push_back(e0);
		edges.push_back(e1);
		edges.push_back(e2);
		edges.push_back(e3);
	}

	// List of obstruction vertexes (plus start/end points); we'll try to find paths through
	// the graph defined by these vertexes
	std::vector<Vertex> vertexes;

	// Add the start point to the graph
	CFixedVector2D posStart(x0, z0);
	fixed hStart = (posStart - NearestPointOnGoal(posStart, goal)).Length();
	Vertex start = { posStart, fixed::Zero(), hStart, 0, Vertex::OPEN, QUADRANT_NONE, QUADRANT_ALL };
	vertexes.push_back(start);
	const size_t START_VERTEX_ID = 0;

	// Add the goal vertex to the graph.
	// Since the goal isn't always a point, this a special magic virtual vertex which moves around - whenever
	// we look at it from another vertex, it is moved to be the closest point on the goal shape to that vertex.
	Vertex end = { CFixedVector2D(goal.x, goal.z), fixed::Zero(), fixed::Zero(), 0, Vertex::UNEXPLORED, QUADRANT_NONE, QUADRANT_ALL };
	vertexes.push_back(end);
	const size_t GOAL_VERTEX_ID = 1;

	// Add terrain obstructions
	{
		u16 i0, j0, i1, j1;
		Pathfinding::NearestNavcell(rangeXMin, rangeZMin, i0, j0, m_MapSize, m_MapSize);
		Pathfinding::NearestNavcell(rangeXMax, rangeZMax, i1, j1, m_MapSize, m_MapSize);
		AddTerrainEdges(edges, vertexes, i0, j0, i1, j1, passClass, *m_Grid);
	}

	// Find all the obstruction squares that might affect us
	CmpPtr<ICmpObstructionManager> cmpObstructionManager(GetSystemEntity());
	std::vector<ICmpObstructionManager::ObstructionSquare> squares;
	cmpObstructionManager->GetObstructionsInRange(filter, rangeXMin - r, rangeZMin - r, rangeXMax + r, rangeZMax + r, squares);

	// Resize arrays to reduce reallocations
	vertexes.reserve(vertexes.size() + squares.size()*4);
	edgeSquares.reserve(edgeSquares.size() + squares.size()); // (assume most squares are AA)

	// Convert each obstruction square into collision edges and search graph vertexes
	for (size_t i = 0; i < squares.size(); ++i)
	{
		CFixedVector2D center(squares[i].x, squares[i].z);
		CFixedVector2D u = squares[i].u;
		CFixedVector2D v = squares[i].v;

		// Expand the vertexes by the moving unit's collision radius, to find the
		// closest we can get to it

		CFixedVector2D hd0(squares[i].hw + r + EDGE_EXPAND_DELTA, squares[i].hh + r + EDGE_EXPAND_DELTA);
		CFixedVector2D hd1(squares[i].hw + r + EDGE_EXPAND_DELTA, -(squares[i].hh + r + EDGE_EXPAND_DELTA));

		// Check whether this is an axis-aligned square
		bool aa = (u.X == fixed::FromInt(1) && u.Y == fixed::Zero() && v.X == fixed::Zero() && v.Y == fixed::FromInt(1));

		Vertex vert;
		vert.status = Vertex::UNEXPLORED;
		vert.quadInward = QUADRANT_NONE;
		vert.quadOutward = QUADRANT_ALL;
		vert.p.X = center.X - hd0.Dot(u); vert.p.Y = center.Y + hd0.Dot(v); if (aa) vert.quadInward = QUADRANT_BR; vertexes.push_back(vert);
		vert.p.X = center.X - hd1.Dot(u); vert.p.Y = center.Y + hd1.Dot(v); if (aa) vert.quadInward = QUADRANT_TR; vertexes.push_back(vert);
		vert.p.X = center.X + hd0.Dot(u); vert.p.Y = center.Y - hd0.Dot(v); if (aa) vert.quadInward = QUADRANT_TL; vertexes.push_back(vert);
		vert.p.X = center.X + hd1.Dot(u); vert.p.Y = center.Y - hd1.Dot(v); if (aa) vert.quadInward = QUADRANT_BL; vertexes.push_back(vert);

		// Add the edges:

		CFixedVector2D h0(squares[i].hw + r,   squares[i].hh + r);
		CFixedVector2D h1(squares[i].hw + r, -(squares[i].hh + r));

		CFixedVector2D ev0(center.X - h0.Dot(u), center.Y + h0.Dot(v));
		CFixedVector2D ev1(center.X - h1.Dot(u), center.Y + h1.Dot(v));
		CFixedVector2D ev2(center.X + h0.Dot(u), center.Y - h0.Dot(v));
		CFixedVector2D ev3(center.X + h1.Dot(u), center.Y - h1.Dot(v));
		if (aa)
		{
			Square e = { ev1, ev3 };
			edgeSquares.push_back(e);
		}
		else
		{
			Edge e0 = { ev0, ev1 };
			Edge e1 = { ev1, ev2 };
			Edge e2 = { ev2, ev3 };
			Edge e3 = { ev3, ev0 };
			edges.push_back(e0);
			edges.push_back(e1);
			edges.push_back(e2);
			edges.push_back(e3);
		}

		// TODO: should clip out vertexes and edges that are outside the range,
		// to reduce the search space
	}

	ENSURE(vertexes.size() < 65536); // we store array indexes as u16

	// Render the debug overlay
	if (m_DebugOverlay)
	{
#define PUSH_POINT(p) xz.push_back(p.X.ToFloat()); xz.push_back(p.Y.ToFloat());
		// Render the vertexes as little Pac-Man shapes to indicate quadrant direction
		for (size_t i = 0; i < vertexes.size(); ++i)
		{
			m_DebugOverlayShortPathLines.push_back(SOverlayLine());
			m_DebugOverlayShortPathLines.back().m_Color = CColor(1, 1, 0, 1);

			float x = vertexes[i].p.X.ToFloat();
			float z = vertexes[i].p.Y.ToFloat();

			float a0 = 0, a1 = 0;
			// Get arc start/end angles depending on quadrant (if any)
			if      (vertexes[i].quadInward == QUADRANT_BL) { a0 = -0.25f; a1 = 0.50f; }
			else if (vertexes[i].quadInward == QUADRANT_TR) { a0 =  0.25f; a1 = 1.00f; }
			else if (vertexes[i].quadInward == QUADRANT_TL) { a0 = -0.50f; a1 = 0.25f; }
			else if (vertexes[i].quadInward == QUADRANT_BR) { a0 =  0.00f; a1 = 0.75f; }

			if (a0 == a1)
				SimRender::ConstructCircleOnGround(GetSimContext(), x, z, 0.5f,
					m_DebugOverlayShortPathLines.back(), true);
			else
				SimRender::ConstructClosedArcOnGround(GetSimContext(), x, z, 0.5f,
					a0 * ((float)M_PI*2.0f), a1 * ((float)M_PI*2.0f),
					m_DebugOverlayShortPathLines.back(), true);
		}

		// Render the edges
		for (size_t i = 0; i < edges.size(); ++i)
		{
			m_DebugOverlayShortPathLines.push_back(SOverlayLine());
			m_DebugOverlayShortPathLines.back().m_Color = CColor(0, 1, 1, 1);
			std::vector<float> xz;
			PUSH_POINT(edges[i].p0);
			PUSH_POINT(edges[i].p1);

			// Add an arrowhead to indicate the direction
			CFixedVector2D d = edges[i].p1 - edges[i].p0;
			d.Normalize(fixed::FromInt(1)/8);
			CFixedVector2D p2 = edges[i].p1 - d*2;
			CFixedVector2D p3 = p2 + d.Perpendicular();
			CFixedVector2D p4 = p2 - d.Perpendicular();
			PUSH_POINT(p3);
			PUSH_POINT(p4);
			PUSH_POINT(edges[i].p1);

			SimRender::ConstructLineOnGround(GetSimContext(), xz, m_DebugOverlayShortPathLines.back(), true);
		}
#undef PUSH_POINT

		// Render the axis-aligned squares
		for (size_t i = 0; i < edgeSquares.size(); ++i)
		{
			m_DebugOverlayShortPathLines.push_back(SOverlayLine());
			m_DebugOverlayShortPathLines.back().m_Color = CColor(0, 1, 1, 1);
			std::vector<float> xz;
			Square s = edgeSquares[i];
			xz.push_back(s.p0.X.ToFloat());
			xz.push_back(s.p0.Y.ToFloat());
			xz.push_back(s.p0.X.ToFloat());
			xz.push_back(s.p1.Y.ToFloat());
			xz.push_back(s.p1.X.ToFloat());
			xz.push_back(s.p1.Y.ToFloat());
			xz.push_back(s.p1.X.ToFloat());
			xz.push_back(s.p0.Y.ToFloat());
			xz.push_back(s.p0.X.ToFloat());
			xz.push_back(s.p0.Y.ToFloat());
			SimRender::ConstructLineOnGround(GetSimContext(), xz, m_DebugOverlayShortPathLines.back(), true);
		}
	}

	// Do an A* search over the vertex/visibility graph:

	// Since we are just measuring Euclidean distance the heuristic is admissible,
	// so we never have to re-examine a node once it's been moved to the closed set.

	// To save time in common cases, we don't precompute a graph of valid edges between vertexes;
	// we do it lazily instead. When the search algorithm reaches a vertex, we examine every other
	// vertex and see if we can reach it without hitting any collision edges, and ignore the ones
	// we can't reach. Since the algorithm can only reach a vertex once (and then it'll be marked
	// as closed), we won't be doing any redundant visibility computations.

	PROFILE_START("A*");

	PriorityQueue open;
	PriorityQueue::Item qiStart = { START_VERTEX_ID, start.h };
	open.push(qiStart);

	u16 idBest = START_VERTEX_ID;
	fixed hBest = start.h;

	while (!open.empty())
	{
		// Move best tile from open to closed
		PriorityQueue::Item curr = open.pop();
		vertexes[curr.id].status = Vertex::CLOSED;

		// If we've reached the destination, stop
		if (curr.id == GOAL_VERTEX_ID)
		{
			idBest = curr.id;
			break;
		}

		// Sort the edges so ones nearer this vertex are checked first by CheckVisibility,
		// since they're more likely to block the rays
		std::sort(edgeSquares.begin(), edgeSquares.end(), SquareSort(vertexes[curr.id].p));

		std::vector<Edge> edgesUnaligned;
		std::vector<EdgeAA> edgesLeft;
		std::vector<EdgeAA> edgesRight;
		std::vector<EdgeAA> edgesBottom;
		std::vector<EdgeAA> edgesTop;
		SplitAAEdges(vertexes[curr.id].p, edges, edgeSquares, edgesUnaligned, edgesLeft, edgesRight, edgesBottom, edgesTop);
		//debug_printf("edges: e=%d aa=%d; u=%d l=%d r=%d b=%d t=%d\n", edges.size(), edgeSquares.size(), edgesUnaligned.size(), edgesLeft.size(), edgesRight.size(), edgesBottom.size(), edgesTop.size());

		// Check the lines to every other vertex
		for (size_t n = 0; n < vertexes.size(); ++n)
		{
			if (vertexes[n].status == Vertex::CLOSED)
				continue;

			// If this is the magical goal vertex, move it to near the current vertex
			CFixedVector2D npos;
			if (n == GOAL_VERTEX_ID)
			{
				npos = NearestPointOnGoal(vertexes[curr.id].p, goal);

				// To prevent integer overflows later on, we need to ensure all vertexes are
				// 'close' to the source. The goal might be far away (not a good idea but
				// sometimes it happens), so clamp it to the current search range
				npos.X = clamp(npos.X, rangeXMin, rangeXMax);
				npos.Y = clamp(npos.Y, rangeZMin, rangeZMax);
			}
			else
			{
				npos = vertexes[n].p;
			}

			// Work out which quadrant(s) we're approaching the new vertex from
			u8 quad = 0;
			if (vertexes[curr.id].p.X <= npos.X && vertexes[curr.id].p.Y <= npos.Y) quad |= QUADRANT_BL;
			if (vertexes[curr.id].p.X >= npos.X && vertexes[curr.id].p.Y >= npos.Y) quad |= QUADRANT_TR;
			if (vertexes[curr.id].p.X <= npos.X && vertexes[curr.id].p.Y >= npos.Y) quad |= QUADRANT_TL;
			if (vertexes[curr.id].p.X >= npos.X && vertexes[curr.id].p.Y <= npos.Y) quad |= QUADRANT_BR;

			// Check that the new vertex is in the right quadrant for the old vertex
			if (!(vertexes[curr.id].quadOutward & quad))
			{
				// Hack: Always head towards the goal if possible, to avoid missing it if it's
				// inside another unit
				if (n != GOAL_VERTEX_ID)
				{
					continue;
				}
			}

			bool visible =
				CheckVisibilityLeft(vertexes[curr.id].p, npos, edgesLeft) &&
				CheckVisibilityRight(vertexes[curr.id].p, npos, edgesRight) &&
				CheckVisibilityBottom(vertexes[curr.id].p, npos, edgesBottom) &&
				CheckVisibilityTop(vertexes[curr.id].p, npos, edgesTop) &&
				CheckVisibility(vertexes[curr.id].p, npos, edgesUnaligned);

			/*
			// Render the edges that we examine
			m_DebugOverlayShortPathLines.push_back(SOverlayLine());
			m_DebugOverlayShortPathLines.back().m_Color = visible ? CColor(0, 1, 0, 0.5) : CColor(1, 0, 0, 0.5);
			std::vector<float> xz;
			xz.push_back(vertexes[curr.id].p.X.ToFloat());
			xz.push_back(vertexes[curr.id].p.Y.ToFloat());
			xz.push_back(npos.X.ToFloat());
			xz.push_back(npos.Y.ToFloat());
			SimRender::ConstructLineOnGround(GetSimContext(), xz, m_DebugOverlayShortPathLines.back(), false);
			//*/

			if (visible)
			{
				fixed g = vertexes[curr.id].g + (vertexes[curr.id].p - npos).Length();

				// If this is a new tile, compute the heuristic distance
				if (vertexes[n].status == Vertex::UNEXPLORED)
				{
					// Add it to the open list:
					vertexes[n].status = Vertex::OPEN;
					vertexes[n].g = g;
					vertexes[n].h = goal.DistanceToPoint(npos);
					vertexes[n].pred = curr.id;

					// If this is an axis-aligned shape, the path must continue in the same quadrant
					// direction (but not go into the inside of the shape).
					// Hack: If we started *inside* a shape then perhaps headed to its corner (e.g. the unit
					// was very near another unit), don't restrict further pathing.
					if (vertexes[n].quadInward && !(curr.id == START_VERTEX_ID && g < fixed::FromInt(8)))
						vertexes[n].quadOutward = ((~vertexes[n].quadInward) & quad) & 0xF;

					if (n == GOAL_VERTEX_ID)
						vertexes[n].p = npos; // remember the new best goal position

					PriorityQueue::Item t = { (u16)n, g + vertexes[n].h };
					open.push(t);

					// Remember the heuristically best vertex we've seen so far, in case we never actually reach the target
					if (vertexes[n].h < hBest)
					{
						idBest = (u16)n;
						hBest = vertexes[n].h;
					}
				}
				else // must be OPEN
				{
					// If we've already seen this tile, and the new path to this tile does not have a
					// better cost, then stop now
					if (g >= vertexes[n].g)
						continue;

					// Otherwise, we have a better path, so replace the old one with the new cost/parent
					fixed gprev = vertexes[n].g;
					vertexes[n].g = g;
					vertexes[n].pred = curr.id;

					// If this is an axis-aligned shape, the path must continue in the same quadrant
					// direction (but not go into the inside of the shape).
					if (vertexes[n].quadInward)
						vertexes[n].quadOutward = ((~vertexes[n].quadInward) & quad) & 0xF;

					if (n == GOAL_VERTEX_ID)
						vertexes[n].p = npos; // remember the new best goal position

					open.promote((u16)n, gprev + vertexes[n].h, g + vertexes[n].h);
				}
			}
		}
	}

	// Reconstruct the path (in reverse)
	for (u16 id = idBest; id != START_VERTEX_ID; id = vertexes[id].pred)
	{
		Waypoint w = { vertexes[id].p.X, vertexes[id].p.Y };
		path.m_Waypoints.push_back(w);
	}

	PROFILE_END("A*");
}

bool CCmpPathfinder::CheckMovement(const IObstructionTestFilter& filter,
	entity_pos_t x0, entity_pos_t z0, entity_pos_t x1, entity_pos_t z1, entity_pos_t r,
	pass_class_t passClass)
{
	// Test against dynamic obstructions first

	CmpPtr<ICmpObstructionManager> cmpObstructionManager(GetSystemEntity());
	if (!cmpObstructionManager)
		return false;

	if (cmpObstructionManager->TestLine(filter, x0, z0, x1, z1, r))
		return false;

	// Test against the passability grid.
	// This should ignore r, and just check that the line (x0,z0)-(x1,z1)
	// does not intersect any impassable navcells.
	// We shouldn't allow lines between diagonally-adjacent navcells.
	// It doesn't matter whether we allow lines precisely along the edge
	// of an impassable navcell.

	// To rasterise the line:
	// If the line is (e.g.) aiming up-right, then we start at the navcell
	// containing the start point and the line must either end in that navcell
	// or else exit along the top edge or the right edge (or through the top-right corner,
	// which we'll arbitrary treat as the horizontal edge).
	// So we jump into the adjacent navcell across that edge, and continue.

	// To handle the special case of units that are stuck on impassable cells,
	// we allow them to move from an impassable to a passable cell (but not
	// vice versa).

	u16 i0, j0, i1, j1;
	Pathfinding::NearestNavcell(x0, z0, i0, j0, m_MapSize, m_MapSize);
	Pathfinding::NearestNavcell(x1, z1, i1, j1, m_MapSize, m_MapSize);

	// Find which direction the line heads in
	int di = (i0 < i1 ? +1 : i1 < i0 ? -1 : 0);
	int dj = (j0 < j1 ? +1 : j1 < j0 ? -1 : 0);

	u16 i = i0;
	u16 j = j0;

// 	debug_printf("(%f,%f)..(%f,%f) [%d,%d]..[%d,%d]\n", x0.ToFloat(), z0.ToFloat(), x1.ToFloat(), z1.ToFloat(), i0, j0, i1, j1);

	bool currentlyOnImpassable = !IS_PASSABLE(m_Grid->get(i0, j0), passClass);

	while (true)
	{
		// Fail if we're moving onto an impassable navcell
		bool passable = IS_PASSABLE(m_Grid->get(i, j), passClass);
		if (passable)
			currentlyOnImpassable = false;
		else if (!currentlyOnImpassable)
			return false;

		// Succeed if we're at the target
		if (i == i1 && j == j1)
			return true;

		// If we can only move horizontally/vertically, then just move in that direction
		if (di == 0)
		{
			j += dj;
			continue;
		}
		else if (dj == 0)
		{
			i += di;
			continue;
		}

		// Otherwise we need to check which cell to move into:

		// Check whether the line intersects the horizontal (top/bottom) edge of
		// the current navcell.
		// Horizontal edge is (i, j + (dj>0?1:0)) .. (i + 1, j + (dj>0?1:0))
		// Since we already know the line is moving from this navcell into a different
		// navcell, we simply need to test that the edge's endpoints are not both on the
		// same side of the line.

		entity_pos_t xia = entity_pos_t::FromInt(i).Multiply(Pathfinding::NAVCELL_SIZE);
		entity_pos_t xib = entity_pos_t::FromInt(i+1).Multiply(Pathfinding::NAVCELL_SIZE);
		entity_pos_t zj = entity_pos_t::FromInt(j + (dj+1)/2).Multiply(Pathfinding::NAVCELL_SIZE);

		CFixedVector2D perp = CFixedVector2D(x1 - x0, z1 - z0).Perpendicular();
		entity_pos_t dota = (CFixedVector2D(xia, zj) - CFixedVector2D(x0, z0)).Dot(perp);
		entity_pos_t dotb = (CFixedVector2D(xib, zj) - CFixedVector2D(x0, z0)).Dot(perp);

// 		debug_printf("(%f,%f)-(%f,%f) :: %f %f\n", xia.ToFloat(), zj.ToFloat(), xib.ToFloat(), zj.ToFloat(), dota.ToFloat(), dotb.ToFloat());

		if ((dota < entity_pos_t::Zero() && dotb < entity_pos_t::Zero()) ||
		    (dota > entity_pos_t::Zero() && dotb > entity_pos_t::Zero()))
		{
			// Horizontal edge is fully on one side of the line, so the line doesn't
			// intersect it, so we should move across the vertical edge instead
			i += di;
		}
		else
		{
			j += dj;
		}
	}
}

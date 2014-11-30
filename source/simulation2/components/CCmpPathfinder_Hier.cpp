/* Copyright (C) 2012 Wildfire Games.
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

#include "precompiled.h"

#include "CCmpPathfinder_Common.h"

#include "renderer/Scene.h"
#include "renderer/TerrainOverlay.h"
#include "simulation2/helpers/Render.h"

#define PATHFINDER_HIER_PROFILE 1
#if PATHFINDER_HIER_PROFILE
	#include "lib/timer.h"
	TIMER_ADD_CLIENT(tc_MakeGoalReachable);
	TIMER_ADD_CLIENT(tc_InitRegions);
	TIMER_ADD_CLIENT(tc_FindNearestNavcellInRegions);
	TIMER_ADD_CLIENT(tc_FindReachableRegions);
#else
	#undef	TIMER_ACCRUE
	#define	TIMER_ACCRUE(a) ;
#endif

class PathfinderHierOverlay;

//	typedef std::map<int, std::pair<RegionID,RegionID>> Edges;

const std::set<std::pair<int, int>> Empty;

// represents Region connection
class Edges
{
public:
	enum direction {right, up, left, down};
	struct Address
	{
		int Value(int ci, int cj, direction dir) const
		{
			ASSERT(ci >= 0 && cj >= 0 && ci < width);
			switch (dir)
			{
			case right:
			case up:
				return (cj * 2 + dir) * width + ci;
			case left:
				return cj * 2 * width + ci - 1;
			case down:
				return ((cj - 1) * 2 + 1) * width + ci;
			default:
				abort();
			}
		}
		int width;
	} Address;

	void Init(int width, int UNUSED(height))
	{
		Address.width = width;
	}

	void Put(int ci, int cj, direction dir, int leftdown, int rightup)
	{
		if (dir == left || dir == down)
			std::swap(leftdown, rightup);
		m_edges[Address.Value(ci, cj, dir)].insert(std::pair<int, int>(leftdown, rightup));
	}

	// ID order is always left to right or down to up, be careful  
	const std::set<std::pair<int, int>>& Get(int ci, int cj, direction dir) const
	{
		if (m_edges.find(Address.Value(ci, cj, dir)) != m_edges.end())
			return m_edges.find(Address.Value(ci, cj, dir))->second;
		else
			return Empty;
	}
private:
	std::map<int, std::set<std::pair<int, int>>> m_edges;
};

/**
 * Hierarchical pathfinder.
 * 
 * Currently this doesn't actually find shortest paths, it just deals with
 * connectivity.
 *
 * The navcell-grid representation of the map is split into fixed-size chunks.
 * Within a chunks, each maximal set of adjacently-connected passable navcells
 * is defined as a region.
 * Each region is a vertex in the hierarchical pathfinder's graph.
 * When two regions in adjacent chunks are connected by passable navcells,
 * the graph contains an edge between the corresponding two vertexes.
 * (There will never be an edge between two regions in the same chunk.)
 *
 * Since regions are typically fairly large, it is possible to determine
 * connectivity between any two navcells by mapping them onto their appropriate
 * region and then doing a relatively small graph search.
 */
class CCmpPathfinder_Hier
{
	NONCOPYABLE(CCmpPathfinder_Hier);

	typedef ICmpPathfinder::pass_class_t pass_class_t;

public:
	struct RegionID
	{
		u8 ci, cj; // chunk ID
		u16 r; // unique-per-chunk local region ID

		RegionID(u8 ci, u8 cj, u16 r) : ci(ci), cj(cj), r(r) { }

		bool operator<(RegionID b) const
		{
			// Sort by chunk ID, then by per-chunk region ID
			if (ci < b.ci)
				return true;
			if (b.ci < ci)
				return false;
			if (cj < b.cj)
				return true;
			if (b.cj < cj)
				return false;
			return r < b.r;
		}

		bool operator==(RegionID b) const
		{
			return ci == b.ci && cj == b.cj && r == b.r;
		}

	};

	CCmpPathfinder_Hier(CCmpPathfinder& pathfinder);
	~CCmpPathfinder_Hier();

	void Init(const std::vector<PathfinderPassability>& passClasses, Grid<NavcellData>* grid);

	RegionID Get(u16 i, u16 j, pass_class_t passClass);

	/**
	 * Updates @p goal so that it's guaranteed to be reachable from the navcell
	 * @p i0, @p j0 (which is assumed to be on a passable navcell).
	 * If any part of the goal area is already reachable then
	 * nothing is changed; otherwise the goal is replaced with a point goal
	 * at the nearest reachable navcell to the original goal's center.
	 * Returns true if the goal was replaced.
	 */
	bool MakeGoalReachable(u16 i0, u16 j0, PathGoal& goal, pass_class_t passClass);

	/**
	 * Updates @p i0, @p j0 (which is assumed to be an impassable navcell)
	 * to the nearest passable navcell.
	 */
	void FindNearestPassableNavcell(u16& i, u16& j, pass_class_t passClass);

	void SetDebugOverlay(bool enabled);

private:
	static const u8 CHUNK_SIZE = 96; // number of navcells per side
									 // TODO PATHFINDER: figure out best number. Probably 64 < n < 128

	struct Chunk
	{
		u8 m_ChunkI, m_ChunkJ; // chunk ID
		u16 m_NumRegions; // number of local region IDs (starting from 1)
		u16 m_Regions[CHUNK_SIZE][CHUNK_SIZE]; // local region ID per navcell

		cassert(CHUNK_SIZE*CHUNK_SIZE/2 < 65536); // otherwise we could overflow m_NumRegions with a checkerboard pattern

		void InitRegions(int ci, int cj, Grid<NavcellData>* grid, pass_class_t passClass);

		RegionID Get(int i, int j);

		void RegionCenter(u16 r, int& i, int& j) const;

		bool RegionContainsGoal(u16 r, const PathGoal& goal) const;

		void RegionNavcellNearest(u16 r, int iGoal, int jGoal, int& iBest, int& jBest, u32& dist2Best) const;

		void FloodFill(int i0, int j0, u16 r);
	};

	void FindEdges(u8 ci, u8 cj, pass_class_t passClass, Edges& edges);

	void Draw(int i0, int j0, int i1, int j1);

	void AddDebugEdges(pass_class_t passClass);

	bool FindReachableRegions(RegionID from, std::set<std::pair<u32, RegionID>>& reachable, pass_class_t passClass, u32 bestdist, PathGoal const& goal, u16 iGoal, u16 jGoal);

	void FindPassableRegions(std::set<RegionID>& regions, pass_class_t passClass);

	/**
	 * Updates @p iGoal and @p jGoal to the navcell that is the nearest to the
	 * initial goal coordinates, in one of the given @p regions.
	 * (Assumes @p regions is non-empty.)
	 */
	void FindNearestNavcellInRegions(const std::set<std::pair<u32, RegionID>>& regions, u16& iGoal, u16& jGoal, pass_class_t passClass);

	Chunk& GetChunk(u8 ci, u8 cj, pass_class_t passClass)
	{
		return m_Chunks[passClass].at(cj * m_ChunksW + ci);
	}

	PathfinderHierOverlay* m_DebugOverlay;

	u16 m_ChunksW, m_ChunksH;
	std::map<pass_class_t, std::vector<Chunk> > m_Chunks;
	
	std::map<pass_class_t, Edges> m_Edges;

public:
	CCmpPathfinder& m_Pathfinder;
	std::vector<SOverlayLine> m_DebugOverlayLines;
};

class PathfinderHierOverlay : public TerrainTextureOverlay
{
public:
	CCmpPathfinder_Hier& m_PathfinderHier;

	PathfinderHierOverlay(CCmpPathfinder_Hier& pathfinderHier) :
		TerrainTextureOverlay(ICmpObstructionManager::NAVCELLS_PER_TILE), m_PathfinderHier(pathfinderHier)
	{
	}

	virtual void BuildTextureRGBA(u8* data, size_t w, size_t h)
	{
		ICmpPathfinder::pass_class_t passClass = m_PathfinderHier.m_Pathfinder.GetPassabilityClass("default");

		for (size_t j = 0; j < h; ++j)
		{
			for (size_t i = 0; i < w; ++i)
			{
				SColor4ub color;

				CCmpPathfinder_Hier::RegionID rid = m_PathfinderHier.Get(i, j, passClass);
				if (rid.r == 0)
					color = SColor4ub(0, 0, 0, 0);
				else if (rid.r == 0xFFFF)
					color = SColor4ub(255, 0, 255, 255);
				else
					color = GetColor(rid.r + rid.ci*5 + rid.cj*7, 127);

				*data++ = color.R;
				*data++ = color.G;
				*data++ = color.B;
				*data++ = color.A;
			}
		}
	}
};

u16 rootID(u16 x, std::vector<u16> v)
{
	while (v[x] < x)
		x = v[x];

	return x;
}

void CCmpPathfinder_Hier::Chunk::InitRegions(int ci, int cj, Grid<NavcellData>* grid, pass_class_t passClass)
{
	TIMER_ACCRUE(tc_InitRegions);
	ENSURE(ci < 256 && cj < 256); // avoid overflows
	m_ChunkI = ci;
	m_ChunkJ = cj;

	memset(m_Regions, 0, sizeof(m_Regions));

	int i0 = ci * CHUNK_SIZE;
	int j0 = cj * CHUNK_SIZE;
	int i1 = std::min(i0 + CHUNK_SIZE, (int)grid->m_W);
	int j1 = std::min(j0 + CHUNK_SIZE, (int)grid->m_H);

	//// for compare

	//// Start by filling the grid with 0 for blocked,
	//// and a 0xFFFF placeholder for unblocked
	//for (int j = j0; j < j1; ++j)
	//{
	//	for (int i = i0; i < i1; ++i)
	//	{
	//		if (IS_PASSABLE(grid->get(i, j), passClass))
	//			m_Regions[j-j0][i-i0] = 0xFFFF;
	//		else
	//			m_Regions[j-j0][i-i0] = 0;
	//	}
	//}

	//u16 old_NumRegions;
	//{
	//	// Scan for tiles with the 0xFFFF placeholder, and then floodfill
	//	// the new region this tile belongs to
	//	int r = 0;
	//	for (int j = 0; j < CHUNK_SIZE; ++j)
	//	{
	//		for (int i = 0; i < CHUNK_SIZE; ++i)
	//		{
	//			if (m_Regions[j][i] == 0xFFFF)
	//				FloodFill(i, j, ++r);
	//		}
	//	}
	//	old_NumRegions = r;
	//}

	//u16 old_Regions[CHUNK_SIZE][CHUNK_SIZE];
	//memcpy(old_Regions, m_Regions, sizeof(old_Regions));
	//memset(m_Regions, 0, sizeof(m_Regions));
	////end of old call

	int regionID = 0;
	std::vector<u16> v;

	u16* pCurID = NULL;
	u16 LeftID = 0;
	u16 DownID = 0;

	v.reserve(32);
	v.push_back(0); //v[0] is unused, but for ease
	// Start by filling the grid with 0 for blocked,
	// and regionID for unblocked
	for (int j = j0; j < j1; ++j)
	{
		for (int i = i0; i < i1; ++i)
		{
			pCurID = &m_Regions[j-j0][i-i0];
			if (!IS_PASSABLE(grid->get(i, j), passClass))
				*pCurID = 0;
			else
			{
				if (j > j0)
					DownID = m_Regions[j-1-j0][i-i0];

				if (i == i0)
					LeftID = 0;
				else
					LeftID = m_Regions[j-j0][i-1-i0];

				if (LeftID > 0)
				{
					*pCurID = LeftID;
					if (*pCurID != DownID && DownID > 0)
					{
						u16 id0 = rootID(DownID, v);
						u16 id1 = rootID(LeftID, v);
						
						//register region connection
						if (id0 < id1)
							v[id1] = id0;
						else if (id0 > id1)
							v[id0] = id1;
					}
				}
				else if (DownID > 0)
					*pCurID = DownID;
				else
				{
					//new ID
					*pCurID = ++regionID;
					v.push_back(regionID);
				}

			}
		}
	}

	//convert to point root ID directry
	m_NumRegions = 0;
	for (u16 i = regionID; i > 0; --i)
	{
		if (v[i] == i)
			++m_NumRegions;
		else
			v[i] = rootID(i,v);
	}

	// Scan for tiles with the non-zero ID, and then integrate ID
	const u16 asize = CHUNK_SIZE * CHUNK_SIZE;
	u16* p = &m_Regions[0][0];
	const u16* pend = p + asize;

	for (; p < pend; ++p)
		*p = v[*p];

}

/**
 * Flood-fill a connected area of navcells that are currently set to
 * region 0xFFFF, starting at chunk-local coords (i0,j0),
 * and assign them to region r.
 */
void CCmpPathfinder_Hier::Chunk::FloodFill(int i0, int j0, u16 r)
{
	std::vector<std::pair<u8, u8> > stack;
	stack.push_back(std::make_pair(i0, j0));

	while (!stack.empty())
	{
		int i = stack.back().first;
		int j = stack.back().second;
		stack.pop_back();
		m_Regions[j][i] = r;

		if (i > 0 && m_Regions[j][i-1] == 0xFFFF)
			stack.push_back(std::make_pair(i-1, j));
		if (j > 0 && m_Regions[j-1][i] == 0xFFFF)
			stack.push_back(std::make_pair(i, j-1));
		if (i < CHUNK_SIZE-1 && m_Regions[j][i+1] == 0xFFFF)
			stack.push_back(std::make_pair(i+1, j));
		if (j < CHUNK_SIZE-1 && m_Regions[j+1][i] == 0xFFFF)
			stack.push_back(std::make_pair(i, j+1));
	}
}


/**
 * Returns a RegionID for the given global navcell coords
 * (which must be inside this chunk);
 */
CCmpPathfinder_Hier::RegionID CCmpPathfinder_Hier::Chunk::Get(int i, int j)
{
	ENSURE(i < CHUNK_SIZE && j < CHUNK_SIZE);
	return RegionID(m_ChunkI, m_ChunkJ, m_Regions[j][i]);
}

/**
 * Return the global navcell coords that correspond roughly to the
 * center of the given region in this chunk.
 * (This is not guaranteed to be actually inside the region.)
 */
void CCmpPathfinder_Hier::Chunk::RegionCenter(u16 r, int& i_out, int& j_out) const
{
	// Find the mean of i,j coords of navcells in this region:

	u32 si = 0, sj = 0; // sum of i,j coords
	u32 n = 0; // number of navcells in region

	cassert(CHUNK_SIZE < 256); // conservative limit to ensure si and sj don't overflow

	for (int j = 0; j < CHUNK_SIZE; ++j)
	{
		for (int i = 0; i < CHUNK_SIZE; ++i)
		{
			if (m_Regions[j][i] == r)
			{
				si += i;
				sj += j;
				n += 1;
			}
		}
	}

	// Avoid divide-by-zero
	if (n == 0)
		n = 1;

	i_out = m_ChunkI * CHUNK_SIZE + si / n;
	j_out = m_ChunkJ * CHUNK_SIZE + sj / n;
}

/**
 * Returns whether any navcell in the given region is inside the goal.
 */
bool CCmpPathfinder_Hier::Chunk::RegionContainsGoal(u16 r, const PathGoal& goal) const
{
	// Inefficiently check every single navcell:
	for (u16 j = 0; j < CHUNK_SIZE; ++j)
	{
		for (u16 i = 0; i < CHUNK_SIZE; ++i)
		{
			if (m_Regions[j][i] == r)
			{
				if (goal.NavcellContainsGoal(m_ChunkI * CHUNK_SIZE + i, m_ChunkJ * CHUNK_SIZE + j))
					return true;
			}
		}
	}

	return false;
}

/**
 * Returns the global navcell coords, and the squared distance to the goal
 * navcell, of whichever navcell inside the given region is closest to
 * that goal.
 */
void CCmpPathfinder_Hier::Chunk::RegionNavcellNearest(u16 r, int iGoal, int jGoal, int& iBest, int& jBest, u32& dist2Best) const
{
	iBest = 0;
	jBest = 0;
	dist2Best = std::numeric_limits<u32>::max();

	for (int j = 0; j < CHUNK_SIZE; ++j)
	{
		for (int i = 0; i < CHUNK_SIZE; ++i)
		{
			if (m_Regions[j][i] == r)
			{
				u32 dist2 = (i + m_ChunkI*CHUNK_SIZE - iGoal)*(i + m_ChunkI*CHUNK_SIZE - iGoal)
				          + (j + m_ChunkJ*CHUNK_SIZE - jGoal)*(j + m_ChunkJ*CHUNK_SIZE - jGoal);

				if (dist2 < dist2Best)
				{
					iBest = i + m_ChunkI*CHUNK_SIZE;
					jBest = j + m_ChunkJ*CHUNK_SIZE;
					dist2Best = dist2;
				}
			}
		}
	}
}

CCmpPathfinder_Hier::CCmpPathfinder_Hier(CCmpPathfinder& pathfinder) :
	m_Pathfinder(pathfinder)
{
	m_DebugOverlay = NULL;
}

CCmpPathfinder_Hier::~CCmpPathfinder_Hier()
{
	SAFE_DELETE(m_DebugOverlay);
}

void CCmpPathfinder_Hier::SetDebugOverlay(bool enabled)
{
	if (enabled && !m_DebugOverlay)
	{
		m_DebugOverlay = new PathfinderHierOverlay(*this);
		m_DebugOverlayLines.clear();
		AddDebugEdges(m_Pathfinder.GetPassabilityClass("default"));
	}
	else if (!enabled && m_DebugOverlay)
	{
		SAFE_DELETE(m_DebugOverlay);
		m_DebugOverlayLines.clear();
	}
}

void CCmpPathfinder_Hier::Init(const std::vector<PathfinderPassability>& passClasses, Grid<NavcellData>* grid)
{
	PROFILE3("hier init");

	// Divide grid into chunks with round-to-positive-infinity
	m_ChunksW = (grid->m_W + CHUNK_SIZE - 1) / CHUNK_SIZE;
	m_ChunksH = (grid->m_H + CHUNK_SIZE - 1) / CHUNK_SIZE;

	ENSURE(m_ChunksW < 256 && m_ChunksH < 256); // else the u8 Chunk::m_ChunkI will overflow

	m_Chunks.clear();

	for (size_t n = 0; n < passClasses.size(); ++n)
	{
		pass_class_t passClass = passClasses[n].m_Mask;

		// Compute the regions within each chunk
		m_Chunks[passClass].resize(m_ChunksW*m_ChunksH);
		for (int cj = 0; cj < m_ChunksH; ++cj)
		{
			for (int ci = 0; ci < m_ChunksW; ++ci)
			{
				m_Chunks[passClass].at(cj*m_ChunksW + ci).InitRegions(ci, cj, grid, passClass);
			}
		}

		// Construct the search graph over the regions

		Edges& edges = m_Edges[passClass];
		edges.Init(m_ChunksW, m_ChunksH);

		for (int cj = 0; cj < m_ChunksH; ++cj)
		{
			for (int ci = 0; ci < m_ChunksW; ++ci)
			{
				FindEdges(ci, cj, passClass, edges);
			}
		}
	}

	if (m_DebugOverlay)
	{
		PROFILE("debug overlay");
		m_DebugOverlayLines.clear();
		AddDebugEdges(m_Pathfinder.GetPassabilityClass("default"));
	}
}

/**
 * Find edges between regions in this chunk and the adjacent below/left chunks.
 */
void CCmpPathfinder_Hier::FindEdges(u8 ci, u8 cj, pass_class_t passClass, Edges& edges)
{
	std::vector<Chunk>& chunks = m_Chunks[passClass];

	Chunk& a = chunks.at(cj*m_ChunksW + ci);

	// For each edge between chunks, we loop over every adjacent pair of
	// navcells in the two chunks. If they are both in valid regions
	// (i.e. are passable navcells) then add a graph edge between those regions.
	// (We don't need to test for duplicates since EdgesMap already uses a
	// std::set which will drop duplicate entries.)

	bool gapped = true;
	if (ci > 0)
	{
		Chunk& b = chunks.at(cj*m_ChunksW + (ci-1));
		for (int j = 0; j < CHUNK_SIZE; ++j)
		{
			RegionID ra = a.Get(0, j);
			RegionID rb = b.Get(CHUNK_SIZE-1, j);
			if (ra.r && rb.r)
			{
				if (gapped == true)
				{
					edges.Put(ci - 1, cj, edges.right, rb.r, ra.r);
					gapped = false;
				}
			}
			else
				gapped = true;
		}
	}

	gapped = true;
	if (cj > 0)
	{
		Chunk& b = chunks.at((cj-1)*m_ChunksW + ci);
		for (int i = 0; i < CHUNK_SIZE; ++i)
		{
			RegionID ra = a.Get(i, 0);
			RegionID rb = b.Get(i, CHUNK_SIZE-1);
			if (ra.r && rb.r)
			{
				if (gapped == true)
				{
					edges.Put(ci, cj - 1, edges.up, rb.r, ra.r);
					gapped = false;
				}
			}
			else
				gapped = true;
		}
	}

}

void CCmpPathfinder_Hier::Draw(int i0, int j0, int i1, int j1)
{
	CFixedVector2D a, b;
	m_Pathfinder.NavcellCenter(i0, j0, a.X, a.Y);
	m_Pathfinder.NavcellCenter(i1, j1, b.X, b.Y);

	// Push the endpoints inwards a little to avoid overlaps
	CFixedVector2D d = b - a;
	d.Normalize(entity_pos_t::FromInt(1));
	a += d;
	b -= d;

	std::vector<float> xz;
	xz.push_back(a.X.ToFloat());
	xz.push_back(a.Y.ToFloat());
	xz.push_back(b.X.ToFloat());
	xz.push_back(b.Y.ToFloat());

	m_DebugOverlayLines.push_back(SOverlayLine());
	m_DebugOverlayLines.back().m_Color = CColor(1.0, 1.0, 1.0, 1.0);
	SimRender::ConstructLineOnGround(m_Pathfinder.GetSimContext(), xz, m_DebugOverlayLines.back(), true);
}

/**
 * Debug visualisation of graph edges between regions.
 */
void CCmpPathfinder_Hier::AddDebugEdges(pass_class_t passClass)
{
	const Edges& edges = m_Edges[passClass];
	const std::vector<Chunk>& chunks = m_Chunks[passClass];

	//for (EdgesMap::const_iterator it = edges.begin(); it != edges.end(); ++it)
	for (int cj = 0; cj < m_ChunksH; ++cj)
	{
		for (int ci = 0; ci < m_ChunksW; ++ci)
		{
			std::set<std::pair<int, int>>::const_iterator it;
			if (ci > 0)
			{
				const std::set<std::pair<int, int>>& IDpairs = edges.Get(ci - 1, cj, edges.right);
				for (it = IDpairs.begin(); it != IDpairs.end(); ++it)
				{
					// Draw a line between the two regions' centers
					int i0, j0, i1, j1;

					chunks[cj * m_ChunksW + ci - 1].RegionCenter(it->first, i0, j0);
					chunks[cj * m_ChunksW + ci].RegionCenter(it->second, i1, j1);

					Draw(i0, j0, i1, j1);
				}
			}

			if (cj > 0)
			{
				const std::set<std::pair<int, int>>& IDpairs = edges.Get(ci, cj - 1, edges.up);
				for (it = IDpairs.begin(); it != IDpairs.end(); ++it)
				{
					// Draw a line between the two regions' centers
					int i0, j0, i1, j1;

					chunks[(cj - 1) * m_ChunksW + ci].RegionCenter(it->first, i0, j0);
					chunks[cj * m_ChunksW + ci].RegionCenter(it->second, i1, j1);

					Draw(i0, j0, i1, j1);
				}
			}
		}
	}
}

CCmpPathfinder_Hier::RegionID CCmpPathfinder_Hier::Get(u16 i, u16 j, pass_class_t passClass)
{
	int ci = i / CHUNK_SIZE;
	int cj = j / CHUNK_SIZE;
	ENSURE(ci < m_ChunksW && cj < m_ChunksH);
	return m_Chunks[passClass][cj*m_ChunksW + ci].Get(i % CHUNK_SIZE, j % CHUNK_SIZE);
}

bool CCmpPathfinder_Hier::MakeGoalReachable(u16 i0, u16 j0, PathGoal& goal, pass_class_t passClass)
{
	TIMER_ACCRUE(tc_MakeGoalReachable);
	RegionID source = Get(i0, j0, passClass);

	u16 iGoal, jGoal;
	m_Pathfinder.NearestNavcell(goal.x, goal.z, iGoal, jGoal);
	//u32 bestdist2 = ((int)i0 - (int)iGoal) * ((int)i0 - (int)iGoal) + ((int)j0 - (int)jGoal) * ((int)j0 - (int)jGoal);

	// Find everywhere that's reachable
	std::set<std::pair<u32, RegionID>> reachableRegions;

	if (!FindReachableRegions(source, reachableRegions, passClass, 0, goal, iGoal, jGoal))
		return false;
// 	debug_printf(L"\nReachable from (%d,%d): ", i0, j0);
// 	for (std::set<RegionID>::iterator it = reachableRegions.begin(); it != reachableRegions.end(); ++it)
// 		debug_printf(L"[%d,%d,%d], ", it->ci, it->cj, it->r);
// 	debug_printf(L"\n");


	// The goal area wasn't reachable,
	// so find the navcell that's nearest to the goal's center


	FindNearestNavcellInRegions(reachableRegions, iGoal, jGoal, passClass);

	// Construct a new point goal at the nearest reachable navcell
	PathGoal newGoal;
	newGoal.type = PathGoal::POINT;
	m_Pathfinder.NavcellCenter(iGoal, jGoal, newGoal.x, newGoal.z);
	goal = newGoal;

	return true;
}

void CCmpPathfinder_Hier::FindNearestPassableNavcell(u16& i, u16& j, pass_class_t passClass)
// Spiral search passable cell from (i, j)
{
	int i0 = i;
	int j0 = j;

	//current i and j
	int ci = i;
	int cj = j;	
	u32 dist2;  //dist^2 from (i, j) to (ci, cj)

	// next corner vector
	int x = 1;
	int y = 0;

	// next corner
	int ni = i0 + x;
	int nj = j0 + y;

	u32 bestdist2 = m_Pathfinder.m_Grid->m_W * m_Pathfinder.m_Grid->m_W + m_Pathfinder.m_Grid->m_H * m_Pathfinder.m_Grid->m_H;
	int besti = -1;
	int bestj = -1;

	while (bestdist2 > (u32)(std::max(abs(ci - i0),abs(cj - j0)) * std::max(abs(ci - i0),abs(cj - j0))))
	{
		do
		{
			if (cj == nj)
			{
				if (cj < 0 || (int)m_Pathfinder.m_Grid->m_H <= cj)
				{
					ci = ni;
					continue;
				}

				ci += ni > ci ? 1 : -1;
				//map out so skip to the corner
				if ((ci < 0 && ni < ci) ||
					(ci >= (int)m_Pathfinder.m_Grid->m_W && ci < ni))
				{
					ci = ni;
					continue;
				}
			}
			else
			{
				if (ci < 0 || (int)m_Pathfinder.m_Grid->m_W <= ci)
				{
					cj = nj;
					continue;
				}
				cj += nj > cj ? 1 : -1;
				//map out so skip to the corner
				if ((cj < 0 && nj < cj) ||
					(cj >= (int)m_Pathfinder.m_Grid->m_H && cj < nj))
				{
					cj = nj;
					continue;
				}
			}

			if (IS_PASSABLE(m_Pathfinder.m_Grid->get(ci, cj), passClass))
			{
				dist2 =(ci - i0)*(ci - i0) + (cj - j0)*(cj - j0);
				if (dist2 < bestdist2)
				{
					bestdist2 = dist2;
					besti= ci;
					bestj= cj;
				}
			}
		}
		while (ci != ni || cj != nj);

		// turn 90 degree
		std::swap(x, y);
		y = -y;
		if (y == 0) //horizontal move should be extended 
			x += x > 0 ? 1 : -1;

		//move next corner
		ni += x;
		nj += y;

		if (abs(x) > (int)m_Pathfinder.m_Grid->m_W || abs(y) > (int)m_Pathfinder.m_Grid->m_H)
			break;
	}
	ENSURE(besti > -1 && bestj > -1);
	ENSURE(besti < (int)m_Pathfinder.m_Grid->m_W && bestj < (int)m_Pathfinder.m_Grid->m_H);
	i = besti;
	j = bestj;
}

void CCmpPathfinder_Hier::FindNearestNavcellInRegions(const std::set<std::pair<u32, RegionID>>& regions, u16& iGoal, u16& jGoal, pass_class_t passClass)
{
	// Find the navcell in the given regions that's nearest to the goal navcell:
	// * For each region, record the (squared) minimal distance to the goal point
	// * Sort regions by that underestimated distance
	// * For each region, find the actual nearest navcell
	// * Stop when the underestimated distances are worse than the best real distance
	TIMER_ACCRUE(tc_FindNearestNavcellInRegions);

	int iBest = iGoal;
	int jBest = jGoal;
	u32 dist2Best = std::numeric_limits<u32>::max();

	for (std::set<std::pair<u32, RegionID>>::const_iterator it = regions.begin(); it != regions.end() ; ++it)
	{
		if ((it->first >= 4 ? ((it->first >> 1) - 1) * ((it->first >> 1) - 1) * CHUNK_SIZE * CHUNK_SIZE * 2 : 0) > dist2Best)
			break;

		int i, j;
		u32 dist2;
		GetChunk(it->second.ci, it->second.cj, passClass).RegionNavcellNearest(it->second.r, iGoal, jGoal, i, j, dist2);

		if (dist2 < dist2Best)
		{
			iBest = i;
			jBest = j;
			dist2Best = dist2;
		}
	}
	ENSURE(iBest > -1 && jBest > -1);
	ENSURE(iBest < (int)m_Pathfinder.m_Grid->m_W && jBest < (int)m_Pathfinder.m_Grid->m_H);

	iGoal = iBest;
	jGoal = jBest;
}

void FindGoalRegionID(PathGoal const& goal, std::set<CCmpPathfinder_Hier::RegionID>& goals, CCmpPathfinder_Hier & hier, const CCmpPathfinder::pass_class_t passClass)
{
	u16 i0, j0, i1, j1;

	switch (goal.type)
	{
	case PathGoal::POINT:
	{
		hier.m_Pathfinder.NearestNavcell(goal.x, goal.z, i0, j0);
		if (goal.NavcellContainsGoal(i0, j0))
		{
			CCmpPathfinder_Hier::RegionID rid = hier.Get(i0, j0, passClass);
			if (rid.r != 0)
				goals.insert(rid);
		}
		return;
	}
	case PathGoal::CIRCLE:
	case PathGoal::INVERTED_CIRCLE:
	{
		hier.m_Pathfinder.NearestNavcell(goal.x - goal.hw, goal.z - goal.hw, i0, j0);
		hier.m_Pathfinder.NearestNavcell(goal.x + goal.hw, goal.z + goal.hw, i1, j1);
		break;
	}
	case PathGoal::SQUARE:
	case PathGoal::INVERTED_SQUARE:
	{
		CFixedVector2D hbb(Geometry::GetHalfBoundingBox(goal.u, goal.v, CFixedVector2D(goal.hw, goal.hh)));
		hier.m_Pathfinder.NearestNavcell(goal.x - hbb.X, goal.z - hbb.Y, i0, j0);
		hier.m_Pathfinder.NearestNavcell(goal.x + hbb.X, goal.z + hbb.Y, i1, j1);
		break;
	}
	default:
		abort();
	}

	for (u16 i = i0; i <= i1; ++i)
	{
		bool hit = false;
		for (u16 j = j0; j <= j1; ++j)
		{
			if (goal.NavcellContainsGoal(i, j))
			{
				hit = true;
				CCmpPathfinder_Hier::RegionID rid = hier.Get(i, j, passClass);
				if (rid.r != 0)
					goals.insert(rid);
			}
			else if (hit == true && (goal.type == goal.SQUARE || goal.type == goal.CIRCLE))
				break;
		}
	}
}

bool CCmpPathfinder_Hier::FindReachableRegions(RegionID from, std::set<std::pair<u32, RegionID>>& reachable, pass_class_t passClass, u32 bestdist, PathGoal const& goal, u16 iGoal, u16 jGoal)
{
	// depth first search the region graph, starting at 'from',
	// collecting the regions that are reachable via edges until reaching the goal.
	// return false if goal is reachable.


	TIMER_ACCRUE(tc_FindReachableRegions);

	ENSURE(from.r != 0);

	std::set<CCmpPathfinder_Hier::RegionID> goals;
	FindGoalRegionID(goal, goals, *this, passClass);

	if (goals.find(from) != goals.end()) //goal is reachable
		return false;

	int gci = iGoal / CHUNK_SIZE;
	int gcj = jGoal / CHUNK_SIZE;

	bestdist = abs(from.ci - gci) + abs(from.cj - gcj);
	
	std::vector<std::pair<u32, RegionID>> regionDistEsts; // pair of (manhattan distance, region)
	regionDistEsts.push_back(std::pair<u32, RegionID>(bestdist, from));
	reachable.insert(std::pair<u32, RegionID>(bestdist, from));

	RegionID back = from;
	for (RegionID curr = from; !regionDistEsts.empty(); curr = regionDistEsts.begin()->second)
	{
		std::pair<u32, RegionID> region = regionDistEsts.back();
		regionDistEsts.pop_back();

		bestdist = region.first;
		int ci = region.second.ci;
		int cj = region.second.cj;
		int rootr = region.second.r;

		const Edges::direction seq_right[] = {Edges::right, Edges::up, Edges::down, Edges::left};
		const Edges::direction seq_up[] = {Edges::up, Edges::left, Edges::right, Edges::down};
		const Edges::direction seq_left[] = {Edges::left, Edges::down, Edges::up, Edges::right};
		const Edges::direction seq_down[] = {Edges::down, Edges::right, Edges::left, Edges::up};
		const Edges::direction* p_seq;

		//determine next chunk
		if (gci > ci && gcj >= cj)
			p_seq = seq_right;
		else if (gci < ci && gcj <= cj)
			p_seq = seq_left;
		else if (gcj > cj)
			p_seq = seq_up;
		else //if (gcj < cj)
			p_seq = seq_down;

		for (int cnt = 0; cnt < 4; ++cnt && ++p_seq)
		{
			//in case of map-out
			if (ci == 0 && *p_seq == Edges::left)
				continue;
			if (ci >= m_ChunksW - 1 && *p_seq == Edges::right)
				continue;
			if (cj == 0 && *p_seq == Edges::down)
				continue;
			if (cj >= m_ChunksH - 1 && *p_seq == Edges::up)
				continue;

			const std::set<std::pair<int, int>>& neighbours = m_Edges[passClass].Get(ci, cj, *p_seq);

			for (std::set<std::pair<int, int>>::const_iterator it = neighbours.begin(); it != neighbours.end(); ++it)
			{
				int r, noder;
				if (*p_seq == Edges::left || *p_seq == Edges::down)
				{
					r = it->second;
					noder = it->first;
				}
				else
				{
					r = it->first;
					noder = it->second;
				}

				if (r != rootr)
					continue;

				int ni = ci + (*p_seq == Edges::right ? 1 : *p_seq == Edges::left ? -1 : 0);
				int nj = cj + (*p_seq == Edges::up ? 1 : *p_seq == Edges::down ? -1 : 0);

				if (RegionID(ni, nj, noder) == back) //in case of backtrack
					continue;

				u32 dist = abs(ni - gci)+abs(nj - gcj);
				std::pair<u32, RegionID> node(dist, RegionID(ni, nj, noder)); 

				if (dist < bestdist)
					bestdist = dist;

				bool bl = reachable.insert(node).second;

				// Add to the reachable set; if this is the first time we added
				// it then also add it to the open list
				if (bl)
				{
					regionDistEsts.push_back(node);
					if (goals.find(node.second) != goals.end()) //goal is reachable
						return false;
				}
			}

		}
		back = region.second;
	}
	return true;
}

void CCmpPathfinder_Hier::FindPassableRegions(std::set<RegionID>& regions, pass_class_t passClass)
{
	// Construct a set of all regions of all chunks for this pass class

	const std::vector<Chunk>& chunks = m_Chunks[passClass];
	for (size_t c = 0; c < chunks.size(); ++c)
	{
		// region 0 is impassable tiles
		for (int r = 1; r <= chunks[c].m_NumRegions; ++r)
			regions.insert(RegionID(chunks[c].m_ChunkI, chunks[c].m_ChunkJ, r));
	}
}

void CCmpPathfinder::PathfinderHierInit()
{
	m_PathfinderHier = new CCmpPathfinder_Hier(*this);
}

void CCmpPathfinder::PathfinderHierDeinit()
{
	SAFE_DELETE(m_PathfinderHier);
}

void CCmpPathfinder::PathfinderHierReload()
{
	m_PathfinderHier->Init(m_PassClasses, m_Grid);
}

void CCmpPathfinder::SetHierDebugOverlay(bool enabled)
{
	m_PathfinderHier->SetDebugOverlay(enabled);
}

void CCmpPathfinder::PathfinderHierRenderSubmit(SceneCollector& collector)
{
	for (size_t i = 0; i < m_PathfinderHier->m_DebugOverlayLines.size(); ++i)
		collector.Submit(&m_PathfinderHier->m_DebugOverlayLines[i]);
}

bool CCmpPathfinder::PathfinderHierMakeGoalReachable(u16 i0, u16 j0, PathGoal& goal, pass_class_t passClass)
{
	return m_PathfinderHier->MakeGoalReachable(i0, j0, goal, passClass);
}

void CCmpPathfinder::PathfinderHierFindNearestPassableNavcell(u16& i, u16& j, pass_class_t passClass)
{
	m_PathfinderHier->FindNearestPassableNavcell(i, j, passClass);
}
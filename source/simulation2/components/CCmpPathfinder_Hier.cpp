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

#include "precompiled.h"

#include "CCmpPathfinder_Common.h"

#include "renderer/Scene.h"
#include "renderer/TerrainOverlay.h"
#include "simulation2/helpers/Render.h"
#include <bitset>

#define PATHFINDER_HIER_PROFILE 1
#if PATHFINDER_HIER_PROFILE
	#include "lib/timer.h"
	TIMER_ADD_CLIENT(tc_MakeGoalReachable);
	TIMER_ADD_CLIENT(tc_InitRegions);
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
			return 0;
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
 * Each region has global unique ID and can have a link to other region which has
 * smaller ID. Thus the connection of regions is represented as tree structure so
 * any of two regions which have common root region are connected.
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

	void FindNearestReachableNavcell(u16& i, u16& j, u16 r, pass_class_t passClass);

	/**
	 * Spiral search nearest desired cell from (i, j)
	 */
	void CCmpPathfinder_Hier::FindNearestDesiredNavcell(u16& i, u16& j, u16 r, pass_class_t passClass, bool (*pCond)(u16, u16, pass_class_t, CCmpPathfinder_Hier*, u16));
	
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

		void InitRegions(int ci, int cj, Grid<NavcellData>* grid, pass_class_t passClass, unsigned* pID, std::vector<u16>& connect, CCmpPathfinder_Hier& hier);

		RegionID Get(int i, int j);

		void RegionCenter(u16 r, int& i, int& j) const;
	};

	void Draw(int i0, int j0, int i1, int j1);

	void AddDebugEdges(pass_class_t passClass);

	bool FindReachableRegions(RegionID from, pass_class_t passClass, PathGoal const& goal, u16 iGoal, u16 jGoal);

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
	std::vector<std::vector<u16> > m_connects;
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

// Find the root ID of a region, used by InitRegions
inline u16 RootID(u16 x, std::vector<u16>& v)
{
	// Just add a basic check for infinite loops
	int checkLoop = 0;
	while (v[x] < x)
	{
		++checkLoop;
		ENSURE(checkLoop < 1000 && "Long loop (probably infinite), unable to find an invariant point");
		x = v[x];
	}

	return x;
}

void CCmpPathfinder_Hier::Chunk::InitRegions(int ci, int cj, Grid<NavcellData>* grid, pass_class_t passClass, unsigned* pID, std::vector<u16>& connect, CCmpPathfinder_Hier& hier)
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

	// Efficiently flood-fill the m_Regions grid

	unsigned regionID = *pID;
	//std::vector<u16> connect;

	u16* pCurrentID = NULL;
	u16 LeftID = 0;
	u16 DownID = 0;

	
	// Start by filling the grid with 0 for blocked,
	// and regionID for unblocked
	for (int j = j0; j < j1; ++j)
	{
		for (int i = i0; i < i1; ++i)
		{
			pCurrentID = &m_Regions[j-j0][i-i0];
			if (!IS_PASSABLE(grid->get(i, j), passClass))
			{
				*pCurrentID = 0;
				continue;
			}
			
			if (j > j0)
				DownID = m_Regions[j-1-j0][i-i0];
			else if (cj > 0)
				DownID = hier.Get(i, j - 1, passClass).r;

			if (i > i0)
				LeftID = m_Regions[j-j0][i-1-i0];
			else if (ci > 0)
				LeftID = hier.Get(i - 1, j, passClass).r;
			else
				LeftID = 0;

			if (LeftID > 0)
			{
				*pCurrentID = LeftID;
				if (*pCurrentID != DownID && DownID > 0)
				{
					u16 id0 = RootID(DownID, connect);
					u16 id1 = RootID(LeftID, connect);

					//if (id0 == 0) id0 = DownID;
					//if (id1 == 0) id1 = DownID;
						
					if (id0 < id1)
						connect[id1] = id0;
					else if (id0 > id1)
						connect[id0] = id1;
				}
			}
			else if (DownID > 0)
				*pCurrentID = DownID;
			else
			{
				// New ID
				*pCurrentID = ++regionID;
				if (connect.size() > regionID)
					connect[regionID] = regionID;
				else
					connect.push_back(regionID);
			}
		}
	}

	// Directly point the root ID
	if (regionID + 1 < connect.size())
	{
		std::vector<u16>::iterator it = connect.begin() + regionID + 1;
		connect.erase(it, connect.end());
	}
	m_NumRegions = 0;
	u16 imax = *pID;
	for (u16 i = regionID; i > *pID; --i)
	{
		if (connect[i] == i)
		{
			++m_NumRegions;
			imax = std::max(imax, i);
		}
		else
			connect[i] = RootID(i,connect);
	}
	*pID = imax; //ID must be unique globally

	// Replace IDs by the root ID
	for (int i = 0; i < CHUNK_SIZE; ++i)
		for (int j = 0; j < CHUNK_SIZE; ++j)
			m_Regions[i][j] = connect[m_Regions[i][j]];
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

	m_connects.resize(passClasses.size());

	for (size_t n = 0; n < passClasses.size(); ++n)
	{
		if (!passClasses[n].m_HasClearance) //no clearance class is never used
			continue;

		pass_class_t passClass = passClasses[n].m_Mask;

		// Compute the regions within each chunk
		m_Chunks[passClass].resize(m_ChunksW*m_ChunksH);

		std::vector<u16>& connect = m_connects[n];
		connect.reserve(32); // TODO: What's a sensible number?
		connect.push_back(0); // connect[0] = 0
		unsigned RegionID = 0;

		for (int cj = 0; cj < m_ChunksH; ++cj)
		{
			for (int ci = 0; ci < m_ChunksW; ++ci)
			{
				m_Chunks[passClass].at(cj*m_ChunksW + ci).InitRegions(ci, cj, grid, passClass, &RegionID, connect, *this);
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

	if (!FindReachableRegions(source, passClass, goal, iGoal, jGoal))
		return false;
// 	debug_printf(L"\nReachable from (%d,%d): ", i0, j0);
// 	for (std::set<RegionID>::iterator it = reachableRegions.begin(); it != reachableRegions.end(); ++it)
// 		debug_printf(L"[%d,%d,%d], ", it->ci, it->cj, it->r);
// 	debug_printf(L"\n");


	// The goal area wasn't reachable,
	// so find the navcell that's nearest to the goal's center

	u16 r = Get(i0, j0, passClass).r;

	FindNearestReachableNavcell(iGoal, jGoal, r, passClass);

	// Construct a new point goal at the nearest reachable navcell
	PathGoal newGoal;
	newGoal.type = PathGoal::POINT;
	m_Pathfinder.NavcellCenter(iGoal, jGoal, newGoal.x, newGoal.z);
	goal = newGoal;

	return true;
}

static bool IsPassable(u16 i, u16 j, ICmpPathfinder::pass_class_t passClass, CCmpPathfinder_Hier* pHier, u16 UNUSED(r))
{
	return IS_PASSABLE(pHier->m_Pathfinder.m_Grid->get(i, j), passClass);
}

static size_t MaskToIndex(ICmpPathfinder::pass_class_t passClass)
{
	std::bitset<16> bitSeries((passClass & (-passClass)) - 1);
	return bitSeries.count() - 2;
}

static bool IsReachable(u16 i, u16 j, ICmpPathfinder::pass_class_t passClass, CCmpPathfinder_Hier* pHier, u16 r)
{
	std::vector<u16>& connect = pHier->m_connects[MaskToIndex(passClass)];
	
	return connect[r] == connect[pHier->Get(i, j, passClass).r];
}

void CCmpPathfinder_Hier::FindNearestPassableNavcell(u16& i, u16& j, pass_class_t passClass)
{
	bool (*pCond)(u16, u16, ICmpPathfinder::pass_class_t, CCmpPathfinder_Hier*, u16) = IsPassable;

	FindNearestDesiredNavcell(i, j, 0, passClass, pCond);
}

void CCmpPathfinder_Hier::FindNearestReachableNavcell(u16& i, u16& j, u16 r, pass_class_t passClass)
{
	bool (*pCond)(u16, u16, ICmpPathfinder::pass_class_t, CCmpPathfinder_Hier*, u16) = IsReachable;

	FindNearestDesiredNavcell(i, j, r, passClass, pCond);
}

void CCmpPathfinder_Hier::FindNearestDesiredNavcell(u16& i, u16& j, u16 r, pass_class_t passClass, bool (*pCond)(u16, u16, ICmpPathfinder::pass_class_t, CCmpPathfinder_Hier*, u16))
// Spiral search passable cell from (i, j)
{
	const u16 Width = m_Pathfinder.m_Grid->m_W;
	const u16 Height = m_Pathfinder.m_Grid->m_H;

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

	u32 bestdist2 = Width * Width + Height * Height;
	int besti = -1;
	int bestj = -1;

	while (bestdist2 > (u32)(std::max(abs(ci - i0),abs(cj - j0)) * std::max(abs(ci - i0),abs(cj - j0))))
	{
		do
		{
			if (cj == nj)
			{
				if (cj < 0 || (int)Height <= cj)
				{
					ci = ni;
					continue;
				}

				ci += ni > ci ? 1 : -1;
				//map out so skip to the corner
				if ((ci < 0 && ni < ci) ||
					(ci >= (int)Width && ci < ni))
				{
					ci = ni;
					continue;
				}
			}
			else
			{
				if (ci < 0 || (int)Width <= ci)
				{
					cj = nj;
					continue;
				}
				cj += nj > cj ? 1 : -1;
				//map out so skip to the corner
				if ((cj < 0 && nj < cj) ||
					(cj >= (int)Height && cj < nj))
				{
					cj = nj;
					continue;
				}
			}

			if ((*pCond)(ci, cj, passClass, this, r))
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

		if (abs(x) > (int)Width || abs(y) > (int)Height)
			break;
	}
	ENSURE(besti > -1 && bestj > -1);
	ENSURE(besti < (int)Width && bestj < (int)Height);
	i = besti;
	j = bestj;
}

void FindGoalRegionID(PathGoal const& goal, std::set<u16>& goals, CCmpPathfinder_Hier & hier, const CCmpPathfinder::pass_class_t passClass, const std::vector<u16>& connect)
{
	u16 i0(0), j0(0), i1(0), j1(0);

	switch (goal.type)
	{
	case PathGoal::POINT:
	{
		hier.m_Pathfinder.NearestNavcell(goal.x, goal.z, i0, j0);
		if (goal.NavcellContainsGoal(i0, j0))
		{
			CCmpPathfinder_Hier::RegionID rid = hier.Get(i0, j0, passClass);
			if (rid.r != 0)
				goals.insert(connect[rid.r]);
		}
		return;
	}
	case PathGoal::CIRCLE:
	{
		hier.m_Pathfinder.NearestNavcell(goal.x + goal.hw, goal.z, i0, j0);
		int i = i0;
		int j = j0;

		fixed dx = fixed::FromInt(i) - goal.x;
		fixed dz = fixed::FromInt(j) - goal.z;

		fixed IsOut = dx.Multiply(dx) + dz.Multiply(dz) - goal.hw.Multiply(goal.hw);

		//int count = 0;
		bool passable = false;
		int ci, cj;
		do
		{
			int di = dz > fixed::Zero() ? 1 : -1;
			int dj = dx < fixed::Zero() ? 1 : -1;
			if (IsOut * (di * dj) < fixed::Zero())
			{
				IsOut += (dz * (2 * dj) + fixed::FromInt(1));
				j += dj;
				dz += fixed::FromInt(dj);
			}
			else
			{
				IsOut += (dx * (2 * di) + fixed::FromInt(1));
				i += di;
				dx += fixed::FromInt(di);

			}

			ci = i - (di + 1) / 2;
			cj = j - (dj + 1) / 2;

			u16 r = hier.Get(ci, cj, passClass).r;
			if (r != 0 && passable == false)
			{
				goals.insert(connect[r]);
				passable = true;
			}
			else
				passable = false;
		} while (i != i0 || j != j0);

		//debug_printf(L"count :%d\n", count);
		//debug_printf(L"set size:%d, radius:%d, count:%d\n", goals.size(), goal.hw.ToInt_RoundToZero(), count);
		return;

	}
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
		bool passable = false;
		for (u16 j = j0; j <= j1; ++j)
		{
			if (goal.NavcellContainsGoal(i, j))
			{
				hit = true;
				u16 r = hier.Get(i, j, passClass).r;
				if (r != 0 && passable == false) //continuous passable cells need not to be stored
				{
					goals.insert(connect[r]);
					passable = true;
				}
				else
					passable = false;

			}
			else if (hit == true && (goal.type == goal.SQUARE || goal.type == goal.CIRCLE))
				break;
		}
	}
}

bool CCmpPathfinder_Hier::FindReachableRegions(RegionID from, pass_class_t passClass, PathGoal const& goal, u16 iGoal, u16 jGoal)
{
	// depth first search the region graph, starting at 'from',
	// collecting the regions that are reachable via edges until reaching the goal.
	// return false if goal is reachable.

	TIMER_ACCRUE(tc_FindReachableRegions);
	ENSURE(from.r != 0);

	const std::vector<u16>& connect = m_connects[MaskToIndex(passClass)];

	u16 fr = connect[from.r];

	if (goal.type == goal.POINT || goal.type == goal.SQUARE || goal.type == goal.CIRCLE)
	{
		u16 gr = connect[Get(iGoal, jGoal, passClass).r];
		if (fr == gr)
			return false;
	}

	std::set<u16> goals;
	FindGoalRegionID(goal, goals, *this, passClass, connect);

	if (goals.find(fr) != goals.end()) //goal is reachable
		return false;

	//goal is unreachable
	return true;
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
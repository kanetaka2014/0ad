/* Copyright (C) 2013 Wildfire Games.
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

#include "simulation2/system/ComponentTest.h"

#include "simulation2/components/ICmpObstructionManager.h"
#include "simulation2/components/ICmpPathfinder.h"

#include "graphics/MapReader.h"
#include "graphics/Terrain.h"
#include "graphics/TerrainTextureManager.h"
#include "lib/timer.h"
#include "lib/tex/tex.h"
#include "ps/Loader.h"
#include "ps/Pyrogenesis.h"
#include "simulation2/Simulation2.h"

class TestCmpPathfinder : public CxxTest::TestSuite
{
public:
	void setUp()
	{
		CXeromyces::Startup();

		g_VFS = CreateVfs(20 * MiB);
		TS_ASSERT_OK(g_VFS->Mount(L"", DataDir()/"mods"/"public", VFS_MOUNT_MUST_EXIST));
		TS_ASSERT_OK(g_VFS->Mount(L"cache/", DataDir()/"cache"));

		// Need some stuff for terrain movement costs:
		// (TODO: this ought to be independent of any graphics code)
		new CTerrainTextureManager;
		g_TexMan.LoadTerrainTextures();
	}

	void tearDown()
	{
		delete &g_TexMan;

		g_VFS.reset();

		CXeromyces::Terminate();
	}

	// disabled by default; run tests with the "-test TestCmpPathfinder" flag to enable
	void test_performance_DISABLED()
	{
		CTerrain terrain;

		CSimulation2 sim2(NULL, g_ScriptRuntime, &terrain);
		sim2.LoadDefaultScripts();
		sim2.ResetState();

		CMapReader* mapReader = new CMapReader(); // it'll call "delete this" itself

		LDR_BeginRegistering();
		mapReader->LoadMap(L"maps/skirmishes/Median Oasis (2).pmp", CScriptValRooted(), &terrain, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
			&sim2, &sim2.GetSimContext(), -1, false);
		LDR_EndRegistering();
		TS_ASSERT_OK(LDR_NonprogressiveLoad());

		sim2.Update(0);

		CmpPtr<ICmpPathfinder> cmp(sim2, SYSTEM_ENTITY);

#if 0
		entity_pos_t x0 = entity_pos_t::FromInt(10);
		entity_pos_t z0 = entity_pos_t::FromInt(495);
		entity_pos_t x1 = entity_pos_t::FromInt(500);
		entity_pos_t z1 = entity_pos_t::FromInt(495);
		ICmpPathfinder::Goal goal = { ICmpPathfinder::Goal::POINT, x1, z1 };

		ICmpPathfinder::Path path;
		cmp->ComputePath(x0, z0, goal, cmp->GetPassabilityClass("default"), path);
		for (size_t i = 0; i < path.m_Waypoints.size(); ++i)
			printf("%d: %f %f\n", (int)i, path.m_Waypoints[i].x.ToDouble(), path.m_Waypoints[i].z.ToDouble());
#endif

		double t = timer_Time();

		srand(1234);
		for (size_t j = 0; j < 1024*2; ++j)
		{
			entity_pos_t x0 = entity_pos_t::FromInt(rand() % 512);
			entity_pos_t z0 = entity_pos_t::FromInt(rand() % 512);
			entity_pos_t x1 = x0 + entity_pos_t::FromInt(rand() % 64);
			entity_pos_t z1 = z0 + entity_pos_t::FromInt(rand() % 64);
			PathGoal goal = { PathGoal::POINT, x1, z1 };

			ICmpPathfinder::Path path;
			cmp->ComputePath(x0, z0, goal, cmp->GetPassabilityClass("default"), path);
		}

		t = timer_Time() - t;
		printf("[%f]", t);
	}

	void test_performance_short_DISABLED()
	{
		CTerrain terrain;
		terrain.Initialize(5, NULL);

		CSimulation2 sim2(NULL, g_ScriptRuntime, &terrain);
		sim2.LoadDefaultScripts();
		sim2.ResetState();

		const entity_pos_t range = entity_pos_t::FromInt(TERRAIN_TILE_SIZE*12);

		CmpPtr<ICmpObstructionManager> cmpObstructionMan(sim2, SYSTEM_ENTITY);
		CmpPtr<ICmpPathfinder> cmpPathfinder(sim2, SYSTEM_ENTITY);

		srand(0);
		for (size_t i = 0; i < 200; ++i)
		{
			fixed x = fixed::FromFloat(1.5f*range.ToFloat() * rand()/(float)RAND_MAX);
			fixed z = fixed::FromFloat(1.5f*range.ToFloat() * rand()/(float)RAND_MAX);
//			printf("# %f %f\n", x.ToFloat(), z.ToFloat());
			cmpObstructionMan->AddUnitShape(INVALID_ENTITY, x, z, fixed::FromInt(2), 0, INVALID_ENTITY);
		}

		NullObstructionFilter filter;
		PathGoal goal = { PathGoal::POINT, range, range };
		ICmpPathfinder::Path path;
		cmpPathfinder->ComputeShortPath(filter, range/3, range/3, fixed::FromInt(2), range, goal, 0, path);
		for (size_t i = 0; i < path.m_Waypoints.size(); ++i)
			printf("# %d: %f %f\n", (int)i, path.m_Waypoints[i].x.ToFloat(), path.m_Waypoints[i].z.ToFloat());
	}

	template<typename T>
	void DumpGrid(std::ostream& stream, const Grid<T>& grid, int mask)
	{
		for (u16 j = 0; j < grid.m_H; ++j)
		{
			for (u16 i = 0; i < grid.m_W; )
			{
				if (!(grid.get(i, j) & mask))
				{
					i++;
					continue;
				}

				u16 i0 = i;
				for (i = i0+1; ; ++i)
				{
					if (i >= grid.m_W || !(grid.get(i, j) & mask))
					{
						stream << "  <rect x='" << i0 << "' y='" << j << "' width='" << (i-i0) << "' height='1'/>\n";
						break;
					}
				}
			}
		}
	}

#define USE_JPS 1

	void test_perf2()
	{
		CTerrain terrain;

		CSimulation2 sim2(NULL, &terrain);
		sim2.LoadDefaultScripts();
		sim2.ResetState();

		CMapReader* mapReader = new CMapReader(); // it'll call "delete this" itself

		LDR_BeginRegistering();
		mapReader->LoadMap(L"maps/scenarios/Peloponnese.pmp", CScriptValRooted(), &terrain, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
			&sim2, &sim2.GetSimContext(), -1, false);
		LDR_EndRegistering();
		TS_ASSERT_OK(LDR_NonprogressiveLoad());

		sim2.Update(0);

		std::ofstream stream(OsString("perf2.html").c_str(), std::ofstream::out | std::ofstream::trunc);

		CmpPtr<ICmpObstructionManager> cmpObstructionManager(sim2, SYSTEM_ENTITY);
		CmpPtr<ICmpPathfinder> cmpPathfinder(sim2, SYSTEM_ENTITY);

		ICmpPathfinder::pass_class_t obstructionsMask = cmpPathfinder->GetPassabilityClass("default") | ICmpObstructionManager::TILE_OBSTRUCTED_PATHFINDING;
		const Grid<u16>& obstructions = cmpPathfinder->GetPassabilityGrid();

		int scale = 1;
		stream << "<!DOCTYPE html>\n";
		stream << "<style>\n";
		stream << ".passability rect { fill: black; }\n";
		stream << ".astar-open rect { fill: rgba(255,255,0,0.75); }\n";
		stream << ".astar-closed rect { fill: rgba(255,0,0,0.75); }\n";
//		stream << ".astar-closed rect { fill: rgba(0,255,0,0.75); }\n";
		stream << ".path { stroke: rgba(0,0,255,0.75); stroke-width: 1; fill: none; }\n";
		stream << "</style>\n";
		stream << "<svg width='" << obstructions.m_W*scale << "' height='" << obstructions.m_H*scale << "'>\n";
		stream << "<g transform='translate(0 " << obstructions.m_H*scale << ") scale(" << scale << " -" << scale << ")'>\n";

		stream << " <g class='passability'>\n";
		DumpGrid(stream, obstructions, obstructionsMask);
		stream << " </g>\n";

		DumpPath(stream, 128*4, 256*4, 128*4, 384*4, cmpPathfinder);
// 	  	RepeatPath(500, 128*4, 256*4, 128*4, 384*4, cmpPathfinder);
// //		RepeatPath(100, 128*4, 256*4, 128*4, 384*4, cmpPathfinder);
// 
// 		DumpPath(stream, 128*4, 204*4, 192*4, 204*4, cmpPathfinder);
// 
// 		DumpPath(stream, 128*4, 230*4, 32*4, 230*4, cmpPathfinder);

		stream << "</g>\n";
		stream << "</svg>\n";
	}
	
	void test_perf3()
	{
		CTerrain terrain;

		CSimulation2 sim2(NULL, &terrain);
		sim2.LoadDefaultScripts();
		sim2.ResetState();

		CMapReader* mapReader = new CMapReader(); // it'll call "delete this" itself

		LDR_BeginRegistering();
		mapReader->LoadMap(L"maps/scenarios/Peloponnese.pmp", CScriptValRooted(), &terrain, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
			&sim2, &sim2.GetSimContext(), -1, false);
		LDR_EndRegistering();
		TS_ASSERT_OK(LDR_NonprogressiveLoad());

		sim2.Update(0);

		std::ofstream stream(OsString("perf3.html").c_str(), std::ofstream::out | std::ofstream::trunc);

		CmpPtr<ICmpObstructionManager> cmpObstructionManager(sim2, SYSTEM_ENTITY);
		CmpPtr<ICmpPathfinder> cmpPathfinder(sim2, SYSTEM_ENTITY);

		ICmpPathfinder::pass_class_t obstructionsMask = cmpPathfinder->GetPassabilityClass("default") | ICmpObstructionManager::TILE_OBSTRUCTED_PATHFINDING;
		const Grid<u16>& obstructions = cmpPathfinder->GetPassabilityGrid();

		int scale = 31;
		stream << "<!DOCTYPE html>\n";
		stream << "<style>\n";
		stream << ".passability rect { fill: black; }\n";
		stream << ".astar-open rect { fill: rgba(255,255,0,0.75); }\n";
		stream << ".astar-closed rect { fill: rgba(255,0,0,0.75); }\n";
		stream << ".path { stroke: rgba(0,0,255,0.75); stroke-width: " << (1.0f / scale) << "; fill: none; }\n";
		stream << "</style>\n";
		stream << "<svg width='" << obstructions.m_W*scale << "' height='" << obstructions.m_H*scale << "'>\n";
		stream << "<defs><pattern id='grid' width='1' height='1' patternUnits='userSpaceOnUse'>\n";
		stream << "<rect fill='none' stroke='#888' stroke-width='" << (1.0f / scale) << "' width='" << scale << "' height='" << scale << "'/>\n";
		stream << "</pattern></defs>\n";
		stream << "<g transform='translate(0 " << obstructions.m_H*scale << ") scale(" << scale << " -" << scale << ")'>\n";

		stream << " <g class='passability'>\n";
		DumpGrid(stream, obstructions, obstructionsMask);
		stream << " </g>\n";

		for (int j = 160; j < 190; ++j)
			for (int i = 220; i < 290; ++i)
				DumpPath(stream, 230, 178, i, j, cmpPathfinder);

		stream << "<rect fill='url(#grid)' width='100%' height='100%'/>\n";
		stream << "</g>\n";
		stream << "</svg>\n";
	}

	void DumpPath(std::ostream& stream, int i0, int j0, int i1, int j1, CmpPtr<ICmpPathfinder>& cmpPathfinder)
	{
		entity_pos_t x0 = entity_pos_t::FromInt(i0);
		entity_pos_t z0 = entity_pos_t::FromInt(j0);
		entity_pos_t x1 = entity_pos_t::FromInt(i1);
		entity_pos_t z1 = entity_pos_t::FromInt(j1);

		PathGoal goal = { PathGoal::POINT, x1, z1 };

		ICmpPathfinder::Path path;
#if USE_JPS
		cmpPathfinder->ComputePathJPS(x0, z0, goal, cmpPathfinder->GetPassabilityClass("default"), path);
#else
		cmpPathfinder->ComputePath(x0, z0, goal, cmpPathfinder->GetPassabilityClass("default"), path);
#endif

		u32 debugSteps;
		double debugTime;
		Grid<u8> debugGrid;
#if USE_JPS
		cmpPathfinder->GetDebugDataJPS(debugSteps, debugTime, debugGrid);
#else
		cmpPathfinder->GetDebugData(debugSteps, debugTime, debugGrid);
#endif
// 		stream << " <g style='visibility:hidden'>\n";

		stream << " <g>\n";
// 		stream << " <g class='astar-open'>\n";
// 		DumpGrid(stream, debugGrid, 1);
// 		stream << " </g>\n";
// 		stream << " <g class='astar-closed'>\n";
// 		DumpGrid(stream, debugGrid, 2);
// 		stream << " </g>\n";
// 		stream << " <g class='astar-closed'>\n";
// 		DumpGrid(stream, debugGrid, 3);
// 		stream << " </g>\n";
		stream << " </g>\n";

		stream << " <polyline";
		stream << " onmouseover='this.previousElementSibling.style.visibility=\"visible\"' onmouseout='this.previousElementSibling.style.visibility=\"hidden\"'";
		double length = 0;
		for (ssize_t i = 0; i < (ssize_t)path.m_Waypoints.size()-1; ++i)
		{
			double dx = (path.m_Waypoints[i+1].x - path.m_Waypoints[i].x).ToDouble();
			double dz = (path.m_Waypoints[i+1].z - path.m_Waypoints[i].z).ToDouble();
			ENSURE(dx == 0 || dz == 0 || abs(dx) == abs(dz));
			length += sqrt(dx*dx + dz*dz);
		}
		stream << " title='length: " << length << "; tiles explored: " << debugSteps << "; time: " << debugTime*1000 << " msec'";
		stream << " class='path' points='";
		for (size_t i = 0; i < path.m_Waypoints.size(); ++i)
			stream << path.m_Waypoints[i].x.ToDouble()*ICmpObstructionManager::NAVCELLS_PER_TILE/TERRAIN_TILE_SIZE << "," << path.m_Waypoints[i].z.ToDouble()*ICmpObstructionManager::NAVCELLS_PER_TILE/TERRAIN_TILE_SIZE << " ";
		stream << "'/>\n";
	}

	void RepeatPath(int n, int i0, int j0, int i1, int j1, CmpPtr<ICmpPathfinder>& cmpPathfinder)
	{
		entity_pos_t x0 = entity_pos_t::FromInt(i0);
		entity_pos_t z0 = entity_pos_t::FromInt(j0);
		entity_pos_t x1 = entity_pos_t::FromInt(i1);
		entity_pos_t z1 = entity_pos_t::FromInt(j1);

		PathGoal goal = { PathGoal::POINT, x1, z1 };

		double t = timer_Time();
		for (int i = 0; i < n; ++i)
		{
			ICmpPathfinder::Path path;
#if USE_JPS
			cmpPathfinder->ComputePathJPS(x0, z0, goal, cmpPathfinder->GetPassabilityClass("default"), path);
#else
			cmpPathfinder->ComputePath(x0, z0, goal, cmpPathfinder->GetPassabilityClass("default"), path);
#endif
		}
		t = timer_Time() - t;
		debug_printf(L"### RepeatPath %fms each (%fs total)\n", 1000*t / n, t);
	}

};

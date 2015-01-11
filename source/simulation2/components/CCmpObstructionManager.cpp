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

#include "simulation2/system/Component.h"
#include "ICmpObstructionManager.h"

#include "simulation2/MessageTypes.h"
#include "simulation2/helpers/Geometry.h"
#include "simulation2/helpers/Rasterize.h"
#include "simulation2/helpers/Render.h"
#include "simulation2/helpers/Spatial.h"
#include "simulation2/serialization/SerializeTemplates.h"

#include "graphics/Overlay.h"
#include "graphics/Terrain.h"
#include "maths/MathUtil.h"
#include "ps/Overlay.h"
#include "ps/Profile.h"
#include "renderer/Scene.h"
#include "ps/CLogger.h"

// Externally, tags are opaque non-zero positive integers.
// Internally, they are tagged (by shape) indexes into shape lists.
// idx must be non-zero.
#define TAG_IS_VALID(tag) ((tag).valid())
#define TAG_IS_UNIT(tag) (((tag).n & 1) == 0)
#define TAG_IS_STATIC(tag) (((tag).n & 1) == 1)
#define UNIT_INDEX_TO_TAG(idx) tag_t(((idx) << 1) | 0)
#define STATIC_INDEX_TO_TAG(idx) tag_t(((idx) << 1) | 1)
#define TAG_TO_INDEX(tag) ((tag).n >> 1)

/**
 * Internal representation of axis-aligned sometimes-square sometimes-circle shapes for moving units
 */
struct UnitShape
{
	entity_id_t entity;
	entity_pos_t x, z;
	entity_pos_t r; // radius of circle, or half width of square
	ICmpObstructionManager::flags_t flags;
	entity_id_t group; // control group (typically the owner entity, or a formation controller entity) (units ignore collisions with others in the same group)
};

/**
 * Internal representation of arbitrary-rotation static square shapes for buildings
 */
struct StaticShape
{
	entity_id_t entity;
	entity_pos_t x, z; // world-space coordinates
	CFixedVector2D u, v; // orthogonal unit vectors - axes of local coordinate space
	entity_pos_t hw, hh; // half width/height in local coordinate space
	ICmpObstructionManager::flags_t flags;
	entity_id_t group;
	entity_id_t group2;
};

/**
 * Serialization helper template for UnitShape
 */
struct SerializeUnitShape
{
	template<typename S>
	void operator()(S& serialize, const char* UNUSED(name), UnitShape& value)
	{
		serialize.NumberU32_Unbounded("entity", value.entity);
		serialize.NumberFixed_Unbounded("x", value.x);
		serialize.NumberFixed_Unbounded("z", value.z);
		serialize.NumberFixed_Unbounded("r", value.r);
		serialize.NumberU8_Unbounded("flags", value.flags);
		serialize.NumberU32_Unbounded("group", value.group);
	}
};

/**
 * Serialization helper template for StaticShape
 */
struct SerializeStaticShape
{
	template<typename S>
	void operator()(S& serialize, const char* UNUSED(name), StaticShape& value)
	{
		serialize.NumberU32_Unbounded("entity", value.entity);
		serialize.NumberFixed_Unbounded("x", value.x);
		serialize.NumberFixed_Unbounded("z", value.z);
		serialize.NumberFixed_Unbounded("u.x", value.u.X);
		serialize.NumberFixed_Unbounded("u.y", value.u.Y);
		serialize.NumberFixed_Unbounded("v.x", value.v.X);
		serialize.NumberFixed_Unbounded("v.y", value.v.Y);
		serialize.NumberFixed_Unbounded("hw", value.hw);
		serialize.NumberFixed_Unbounded("hh", value.hh);
		serialize.NumberU8_Unbounded("flags", value.flags);
		serialize.NumberU32_Unbounded("group", value.group);
		serialize.NumberU32_Unbounded("group2", value.group2);
	}
};

class CCmpObstructionManager : public ICmpObstructionManager
{
public:
	static void ClassInit(CComponentManager& componentManager)
	{
		componentManager.SubscribeToMessageType(MT_RenderSubmit); // for debug overlays
	}

	DEFAULT_COMPONENT_ALLOCATOR(ObstructionManager)

	bool m_DebugOverlayEnabled;
	bool m_DebugOverlayDirty;
	std::vector<SOverlayLine> m_DebugOverlayLines;

	SpatialSubdivision m_UnitSubdivision;
	SpatialSubdivision m_StaticSubdivision;

	// TODO: using std::map is a bit inefficient; is there a better way to store these?
	std::map<u32, UnitShape> m_UnitShapes;
	std::map<u32, StaticShape> m_StaticShapes;
	u32 m_UnitShapeNext; // next allocated id
	u32 m_StaticShapeNext;

	bool m_PassabilityCircular;

	entity_pos_t m_WorldX0;
	entity_pos_t m_WorldZ0;
	entity_pos_t m_WorldX1;
	entity_pos_t m_WorldZ1;

	static std::string GetSchema()
	{
		return "<a:component type='system'/><empty/>";
	}

	virtual void Init(const CParamNode& UNUSED(paramNode))
	{
		m_DebugOverlayEnabled = false;
		m_DebugOverlayDirty = true;

		m_UnitShapeNext = 1;
		m_StaticShapeNext = 1;

		m_DirtyID = 1; // init to 1 so default-initialised grids are considered dirty

		m_PassabilityCircular = false;

		m_WorldX0 = m_WorldZ0 = m_WorldX1 = m_WorldZ1 = entity_pos_t::Zero();

		// Initialise with bogus values (these will get replaced when
		// SetBounds is called)
		ResetSubdivisions(entity_pos_t::FromInt(1024), entity_pos_t::FromInt(1024));
	}

	virtual void Deinit()
	{
	}

	template<typename S>
	void SerializeCommon(S& serialize)
	{
		SerializeSpatialSubdivision()(serialize, "unit subdiv", m_UnitSubdivision);
		SerializeSpatialSubdivision()(serialize, "static subdiv", m_StaticSubdivision);

		SerializeMap<SerializeU32_Unbounded, SerializeUnitShape>()(serialize, "unit shapes", m_UnitShapes);
		SerializeMap<SerializeU32_Unbounded, SerializeStaticShape>()(serialize, "static shapes", m_StaticShapes);
		serialize.NumberU32_Unbounded("unit shape next", m_UnitShapeNext);
		serialize.NumberU32_Unbounded("static shape next", m_StaticShapeNext);

		serialize.Bool("circular", m_PassabilityCircular);

		serialize.NumberFixed_Unbounded("world x0", m_WorldX0);
		serialize.NumberFixed_Unbounded("world z0", m_WorldZ0);
		serialize.NumberFixed_Unbounded("world x1", m_WorldX1);
		serialize.NumberFixed_Unbounded("world z1", m_WorldZ1);
	}

	virtual void Serialize(ISerializer& serialize)
	{
		// TODO: this could perhaps be optimised by not storing all the obstructions,
		// and instead regenerating them from the other entities on Deserialize

		SerializeCommon(serialize);
	}

	virtual void Deserialize(const CParamNode& paramNode, IDeserializer& deserialize)
	{
		Init(paramNode);

		SerializeCommon(deserialize);
	}

	virtual void HandleMessage(const CMessage& msg, bool UNUSED(global))
	{
		switch (msg.GetType())
		{
		case MT_RenderSubmit:
		{
			const CMessageRenderSubmit& msgData = static_cast<const CMessageRenderSubmit&> (msg);
			RenderSubmit(msgData.collector);
			break;
		}
		}
	}

	virtual void SetBounds(entity_pos_t x0, entity_pos_t z0, entity_pos_t x1, entity_pos_t z1)
	{
		m_WorldX0 = x0;
		m_WorldZ0 = z0;
		m_WorldX1 = x1;
		m_WorldZ1 = z1;
		MakeDirtyAll();

		// Subdivision system bounds:
		ENSURE(x0.IsZero() && z0.IsZero()); // don't bother implementing non-zero offsets yet
		ResetSubdivisions(x1, z1);
	}

	void ResetSubdivisions(entity_pos_t x1, entity_pos_t z1)
	{
		// Use 8x8 tile subdivisions
		// (TODO: find the optimal number instead of blindly guessing)
		m_UnitSubdivision.Reset(x1, z1, entity_pos_t::FromInt(8*TERRAIN_TILE_SIZE));
		m_StaticSubdivision.Reset(x1, z1, entity_pos_t::FromInt(8*TERRAIN_TILE_SIZE));

		for (std::map<u32, UnitShape>::iterator it = m_UnitShapes.begin(); it != m_UnitShapes.end(); ++it)
		{
			CFixedVector2D center(it->second.x, it->second.z);
			CFixedVector2D halfSize(it->second.r, it->second.r);
			m_UnitSubdivision.Add(it->first, center - halfSize, center + halfSize);
		}

		for (std::map<u32, StaticShape>::iterator it = m_StaticShapes.begin(); it != m_StaticShapes.end(); ++it)
		{
			CFixedVector2D center(it->second.x, it->second.z);
			CFixedVector2D bbHalfSize = Geometry::GetHalfBoundingBox(it->second.u, it->second.v, CFixedVector2D(it->second.hw, it->second.hh));
			m_StaticSubdivision.Add(it->first, center - bbHalfSize, center + bbHalfSize);
		}
	}

	virtual tag_t AddUnitShape(entity_id_t ent, entity_pos_t x, entity_pos_t z, entity_pos_t r, flags_t flags, entity_id_t group)
	{
		UnitShape shape = { ent, x, z, r, flags, group };
		u32 id = m_UnitShapeNext++;
		m_UnitShapes[id] = shape;
		MakeDirtyUnit(shape);

		m_UnitSubdivision.Add(id, CFixedVector2D(x - r, z - r), CFixedVector2D(x + r, z + r));

		return UNIT_INDEX_TO_TAG(id);
	}

	virtual tag_t AddStaticShape(entity_id_t ent, entity_pos_t x, entity_pos_t z, entity_angle_t a, entity_pos_t w, entity_pos_t h, flags_t flags, entity_id_t group, entity_id_t group2 /* = INVALID_ENTITY */)
	{
		fixed s, c;
		sincos_approx(a, s, c);
		CFixedVector2D u(c, -s);
		CFixedVector2D v(s, c);

		StaticShape shape = { ent, x, z, u, v, w/2, h/2, flags, group, group2 };
		u32 id = m_StaticShapeNext++;
		m_StaticShapes[id] = shape;
		MakeDirtyStatic(shape);

		CFixedVector2D center(x, z);
		CFixedVector2D bbHalfSize = Geometry::GetHalfBoundingBox(u, v, CFixedVector2D(w/2, h/2));
		m_StaticSubdivision.Add(id, center - bbHalfSize, center + bbHalfSize);

		return STATIC_INDEX_TO_TAG(id);
	}

	virtual ObstructionSquare GetUnitShapeObstruction(entity_pos_t x, entity_pos_t z, entity_pos_t r)
	{
		CFixedVector2D u(entity_pos_t::FromInt(1), entity_pos_t::Zero());
		CFixedVector2D v(entity_pos_t::Zero(), entity_pos_t::FromInt(1));
		ObstructionSquare o = { x, z, u, v, r, r };
		return o;
	}

	virtual ObstructionSquare GetStaticShapeObstruction(entity_pos_t x, entity_pos_t z, entity_angle_t a, entity_pos_t w, entity_pos_t h)
	{
		fixed s, c;
		sincos_approx(a, s, c);
		CFixedVector2D u(c, -s);
		CFixedVector2D v(s, c);

		ObstructionSquare o = { x, z, u, v, w/2, h/2 };
		return o;
	}

	virtual void MoveShape(tag_t tag, entity_pos_t x, entity_pos_t z, entity_angle_t a)
	{
		ENSURE(TAG_IS_VALID(tag));

		if (TAG_IS_UNIT(tag))
		{
			UnitShape& shape = m_UnitShapes[TAG_TO_INDEX(tag)];

			m_UnitSubdivision.Move(TAG_TO_INDEX(tag),
				CFixedVector2D(shape.x - shape.r, shape.z - shape.r),
				CFixedVector2D(shape.x + shape.r, shape.z + shape.r),
				CFixedVector2D(x - shape.r, z - shape.r),
				CFixedVector2D(x + shape.r, z + shape.r));

			shape.x = x;
			shape.z = z;

			MakeDirtyUnit(shape);
		}
		else
		{
			fixed s, c;
			sincos_approx(a, s, c);
			CFixedVector2D u(c, -s);
			CFixedVector2D v(s, c);

			StaticShape& shape = m_StaticShapes[TAG_TO_INDEX(tag)];

			CFixedVector2D fromBbHalfSize = Geometry::GetHalfBoundingBox(shape.u, shape.v, CFixedVector2D(shape.hw, shape.hh));
			CFixedVector2D toBbHalfSize = Geometry::GetHalfBoundingBox(u, v, CFixedVector2D(shape.hw, shape.hh));
			m_StaticSubdivision.Move(TAG_TO_INDEX(tag),
				CFixedVector2D(shape.x, shape.z) - fromBbHalfSize,
				CFixedVector2D(shape.x, shape.z) + fromBbHalfSize,
				CFixedVector2D(x, z) - toBbHalfSize,
				CFixedVector2D(x, z) + toBbHalfSize);

			shape.x = x;
			shape.z = z;
			shape.u = u;
			shape.v = v;

			MakeDirtyStatic(shape);
		}
	}

	virtual void SetUnitMovingFlag(tag_t tag, bool moving)
	{
		ENSURE(TAG_IS_VALID(tag) && TAG_IS_UNIT(tag));

		if (TAG_IS_UNIT(tag))
		{
			UnitShape& shape = m_UnitShapes[TAG_TO_INDEX(tag)];
			if (moving)
				shape.flags |= FLAG_MOVING;
			else
				shape.flags &= (flags_t)~FLAG_MOVING;

			MakeDirtyDebug();
		}
	}

	virtual void SetUnitControlGroup(tag_t tag, entity_id_t group)
	{
		ENSURE(TAG_IS_VALID(tag) && TAG_IS_UNIT(tag));

		if (TAG_IS_UNIT(tag))
		{
			UnitShape& shape = m_UnitShapes[TAG_TO_INDEX(tag)];
			shape.group = group;
		}
	}

	virtual void SetStaticControlGroup(tag_t tag, entity_id_t group, entity_id_t group2)
	{
		ENSURE(TAG_IS_VALID(tag) && TAG_IS_STATIC(tag));

		if (TAG_IS_STATIC(tag))
		{
			StaticShape& shape = m_StaticShapes[TAG_TO_INDEX(tag)];
			shape.group = group;
			shape.group2 = group2;
		}
	}

	virtual void RemoveShape(tag_t tag)
	{
		ENSURE(TAG_IS_VALID(tag));

		if (TAG_IS_UNIT(tag))
		{
			UnitShape& shape = m_UnitShapes[TAG_TO_INDEX(tag)];
			m_UnitSubdivision.Remove(TAG_TO_INDEX(tag),
				CFixedVector2D(shape.x - shape.r, shape.z - shape.r),
				CFixedVector2D(shape.x + shape.r, shape.z + shape.r));

			MakeDirtyUnit(shape);
			m_UnitShapes.erase(TAG_TO_INDEX(tag));
		}
		else
		{
			StaticShape& shape = m_StaticShapes[TAG_TO_INDEX(tag)];

			CFixedVector2D center(shape.x, shape.z);
			CFixedVector2D bbHalfSize = Geometry::GetHalfBoundingBox(shape.u, shape.v, CFixedVector2D(shape.hw, shape.hh));
			m_StaticSubdivision.Remove(TAG_TO_INDEX(tag), center - bbHalfSize, center + bbHalfSize);

			MakeDirtyStatic(shape);
			m_StaticShapes.erase(TAG_TO_INDEX(tag));
		}
	}

	virtual ObstructionSquare GetObstruction(tag_t tag)
	{
		ENSURE(TAG_IS_VALID(tag));

		if (TAG_IS_UNIT(tag))
		{
			UnitShape& shape = m_UnitShapes[TAG_TO_INDEX(tag)];
			CFixedVector2D u(entity_pos_t::FromInt(1), entity_pos_t::Zero());
			CFixedVector2D v(entity_pos_t::Zero(), entity_pos_t::FromInt(1));
			ObstructionSquare o = { shape.x, shape.z, u, v, shape.r, shape.r };
			return o;
		}
		else
		{
			StaticShape& shape = m_StaticShapes[TAG_TO_INDEX(tag)];
			ObstructionSquare o = { shape.x, shape.z, shape.u, shape.v, shape.hw, shape.hh };
			return o;
		}
	}

	virtual bool TestLine(const IObstructionTestFilter& filter, entity_pos_t x0, entity_pos_t z0, entity_pos_t x1, entity_pos_t z1, entity_pos_t r);
	virtual bool TestStaticShape(const IObstructionTestFilter& filter, entity_pos_t x, entity_pos_t z, entity_pos_t a, entity_pos_t w, entity_pos_t h, std::vector<entity_id_t>* out);
	virtual bool TestUnitShape(const IObstructionTestFilter& filter, entity_pos_t x, entity_pos_t z, entity_pos_t r, std::vector<entity_id_t>* out);

	virtual void SetMaxClearance(int max)
	{
		m_MaxClearance = max;
	}

	virtual int GetMaxClearance()
	{
		return m_MaxClearance;
	}

	virtual void ObstructionDirtyReset(int i0, int j0, int i1, int j1);
	virtual void GetDirtyRange(Grid<u16>& grid, int& i0, int& j0, int& i1, int& j1);

	virtual void Rasterize(Grid<u16>& grid, entity_pos_t expand, ICmpObstructionManager::flags_t requireMask, u16 setMask);
	virtual void GetObstructionsInRange(const IObstructionTestFilter& filter, entity_pos_t x0, entity_pos_t z0, entity_pos_t x1, entity_pos_t z1, std::vector<ObstructionSquare>& squares);
	void GetShapesInRange(entity_pos_t x0, entity_pos_t z0, entity_pos_t x1, entity_pos_t z1, std::map<u32, UnitShape>& outUnitShapes, std::map<u32, StaticShape>& outStaticShapes); 
	virtual void GetUnitsOnObstruction(const ObstructionSquare& square, std::vector<entity_id_t>& out);

	virtual void SetPassabilityCircular(bool enabled)
	{
		m_PassabilityCircular = enabled;
		MakeDirtyAll();

		CMessageObstructionMapShapeChanged msg;
		GetSimContext().GetComponentManager().BroadcastMessage(msg);
	}

	virtual void SetDebugOverlay(bool enabled)
	{
		m_DebugOverlayEnabled = enabled;
		m_DebugOverlayDirty = true;
		if (!enabled)
			m_DebugOverlayLines.clear();
	}

	void RenderSubmit(SceneCollector& collector);

private:
	// To support lazy updates of grid rasterisations of obstruction data,
	// we maintain a DirtyID here and increment it whenever obstructions change;
	// if a grid has a lower DirtyID then it needs to be updated.

	size_t m_DirtyID;
	entity_pos_t m_DirtyX0;
	entity_pos_t m_DirtyZ0;
	entity_pos_t m_DirtyX1;
	entity_pos_t m_DirtyZ1;
 	// XXX: this probably breaks if used with multiple independently-dirtyable grids 

	int m_MaxClearance;

	/**
	 * Mark all previous Rasterise()d grids as dirty, and the debug display.
	 * Call this when the world bounds have changed.
	 */
	void MakeDirtyAll()
	{
		++m_DirtyID;
		m_DebugOverlayDirty = true;
		m_DirtyX0 = m_WorldX0;
		m_DirtyZ0 = m_WorldZ0; 
 		m_DirtyX1 = m_WorldX1; 
 		m_DirtyZ1 = m_WorldZ1;
	}

	/**
	 * Mark the debug display as dirty.
	 * Call this when nothing has changed except a unit's 'moving' flag.
	 */
	void MakeDirtyDebug()
	{
		m_DebugOverlayDirty = true;
	}

	/**
	 * Mark all previous Rasterise()d grids as dirty, if they depend on this shape.
	 * Call this when a static shape has changed.
	 */
	void MakeDirtyStatic(const StaticShape& shape)
	{
		if (shape.flags & (FLAG_BLOCK_PATHFINDING|FLAG_BLOCK_FOUNDATION))
		{
			++m_DirtyID;
			CFixedVector2D halfSize(shape.hw + fixed::FromInt(m_MaxClearance), shape.hh + fixed::FromInt(m_MaxClearance)); 
 			CFixedVector2D halfBound = Geometry::GetHalfBoundingBox(shape.u, shape.v, halfSize); 
			m_DirtyX0 = std::min(m_DirtyX0, shape.x - halfBound.X); 
 			m_DirtyZ0 = std::min(m_DirtyZ0, shape.z - halfBound.Y); 
 			m_DirtyX1 = std::max(m_DirtyX1, shape.x + halfBound.X); 
 			m_DirtyZ1 = std::max(m_DirtyZ1, shape.z + halfBound.Y); 
 		}

		m_DebugOverlayDirty = true;
	}

	/**
	 * Mark all previous Rasterise()d grids as dirty, if they depend on this shape.
	 * Call this when a unit shape has changed.
	 */
	void MakeDirtyUnit(const UnitShape& shape)
	{
		if (shape.flags & (FLAG_BLOCK_PATHFINDING|FLAG_BLOCK_FOUNDATION))
		{
			++m_DirtyID;
			entity_pos_t r = shape.r + fixed::FromInt(m_MaxClearance); 
			m_DirtyX0 = std::min(m_DirtyX0, shape.x - r); 
			m_DirtyZ0 = std::min(m_DirtyZ0, shape.z - r); 
			m_DirtyX1 = std::max(m_DirtyX1, shape.x + r); 
			m_DirtyZ1 = std::max(m_DirtyZ1, shape.z + r);  
		}

		m_DebugOverlayDirty = true;
	}

	virtual bool NeedUpdate(size_t* dirtyID)
	{
		ENSURE(*dirtyID <= m_DirtyID);
		if (*dirtyID != m_DirtyID)
		{
			*dirtyID = m_DirtyID;
			return true;
		}
		return false;
	}

	/**
	 * Return whether the given point is within the world bounds by at least r
	 */
	bool IsInWorld(entity_pos_t x, entity_pos_t z, entity_pos_t r)
	{
		return (m_WorldX0+r <= x && x <= m_WorldX1-r && m_WorldZ0+r <= z && z <= m_WorldZ1-r);
	}

	/**
	 * Return whether the given point is within the world bounds
	 */
	bool IsInWorld(CFixedVector2D p)
	{
		return (m_WorldX0 <= p.X && p.X <= m_WorldX1 && m_WorldZ0 <= p.Y && p.Y <= m_WorldZ1);
	}
};

REGISTER_COMPONENT_TYPE(ObstructionManager)

bool CCmpObstructionManager::TestLine(const IObstructionTestFilter& filter, entity_pos_t x0, entity_pos_t z0, entity_pos_t x1, entity_pos_t z1, entity_pos_t r)
{
	PROFILE("TestLine");

	// Check that both end points are within the world (which means the whole line must be)
	if (!IsInWorld(x0, z0, r) || !IsInWorld(x1, z1, r))
		return true;

	CFixedVector2D posMin (std::min(x0, x1) - r, std::min(z0, z1) - r);
	CFixedVector2D posMax (std::max(x0, x1) + r, std::max(z0, z1) + r);

	std::vector<entity_id_t> unitShapes;
	m_UnitSubdivision.GetInRange(unitShapes, posMin, posMax);
	for (size_t i = 0; i < unitShapes.size(); ++i)
	{
		std::map<u32, UnitShape>::iterator it = m_UnitShapes.find(unitShapes[i]);
		ENSURE(it != m_UnitShapes.end());

		if (!filter.TestShape(UNIT_INDEX_TO_TAG(it->first), it->second.flags, it->second.group, INVALID_ENTITY))
			continue;

		CFixedVector2D center(it->second.x, it->second.z);
		CFixedVector2D halfSize(it->second.r + r, it->second.r + r);
		if (Geometry::TestRayAASquare(CFixedVector2D(x0, z0) - center, CFixedVector2D(x1, z1) - center, halfSize))
			return true;
	}

	std::vector<entity_id_t> staticShapes;
	m_StaticSubdivision.GetInRange(staticShapes, posMin, posMax);
	for (size_t i = 0; i < staticShapes.size(); ++i)
	{
		std::map<u32, StaticShape>::iterator it = m_StaticShapes.find(staticShapes[i]);
		ENSURE(it != m_StaticShapes.end());

		if (!filter.TestShape(STATIC_INDEX_TO_TAG(it->first), it->second.flags, it->second.group, it->second.group2))
			continue;

		CFixedVector2D center(it->second.x, it->second.z);
		CFixedVector2D halfSize(it->second.hw + r, it->second.hh + r);
		if (Geometry::TestRaySquare(CFixedVector2D(x0, z0) - center, CFixedVector2D(x1, z1) - center, it->second.u, it->second.v, halfSize))
			return true;
	}

	return false;
}

bool CCmpObstructionManager::TestStaticShape(const IObstructionTestFilter& filter,
	entity_pos_t x, entity_pos_t z, entity_pos_t a, entity_pos_t w, entity_pos_t h,
	std::vector<entity_id_t>* out)
{
	PROFILE("TestStaticShape");

	// TODO: should use the subdivision stuff here, if performance is non-negligible

	if (out)
		out->clear();

	fixed s, c;
	sincos_approx(a, s, c);
	CFixedVector2D u(c, -s);
	CFixedVector2D v(s, c);
	CFixedVector2D center(x, z);
	CFixedVector2D halfSize(w/2, h/2);

	// Check that all corners are within the world (which means the whole shape must be)
	if (!IsInWorld(center + u.Multiply(halfSize.X) + v.Multiply(halfSize.Y)) ||
		!IsInWorld(center + u.Multiply(halfSize.X) - v.Multiply(halfSize.Y)) ||
		!IsInWorld(center - u.Multiply(halfSize.X) + v.Multiply(halfSize.Y)) ||
		!IsInWorld(center - u.Multiply(halfSize.X) - v.Multiply(halfSize.Y)))
	{
		if (out)
			out->push_back(INVALID_ENTITY); // no entity ID, so just push an arbitrary marker
		else
			return true;
	}

	for (std::map<u32, UnitShape>::iterator it = m_UnitShapes.begin(); it != m_UnitShapes.end(); ++it)
	{
		if (!filter.TestShape(UNIT_INDEX_TO_TAG(it->first), it->second.flags, it->second.group, INVALID_ENTITY))
			continue;

		CFixedVector2D center1(it->second.x, it->second.z);

		if (Geometry::PointIsInSquare(center1 - center, u, v, CFixedVector2D(halfSize.X + it->second.r, halfSize.Y + it->second.r)))
		{
			if (out)
				out->push_back(it->second.entity);
			else
				return true;
		}
	}

	for (std::map<u32, StaticShape>::iterator it = m_StaticShapes.begin(); it != m_StaticShapes.end(); ++it)
	{
		if (!filter.TestShape(STATIC_INDEX_TO_TAG(it->first), it->second.flags, it->second.group, it->second.group2))
			continue;

		CFixedVector2D center1(it->second.x, it->second.z);
		CFixedVector2D halfSize1(it->second.hw, it->second.hh);
		if (Geometry::TestSquareSquare(center, u, v, halfSize, center1, it->second.u, it->second.v, halfSize1))
		{
			if (out)
				out->push_back(it->second.entity);
			else
				return true;
		}
	}

	if (out)
		return !out->empty(); // collided if the list isn't empty
	else
		return false; // didn't collide, if we got this far
}

bool CCmpObstructionManager::TestUnitShape(const IObstructionTestFilter& filter,
	entity_pos_t x, entity_pos_t z, entity_pos_t r,
	std::vector<entity_id_t>* out)
{
	PROFILE("TestUnitShape");

	// TODO: should use the subdivision stuff here, if performance is non-negligible

	// Check that the shape is within the world
	if (!IsInWorld(x, z, r))
	{
		if (out)
			out->push_back(INVALID_ENTITY); // no entity ID, so just push an arbitrary marker
		else
			return true;
	}

	CFixedVector2D center(x, z);

	for (std::map<u32, UnitShape>::iterator it = m_UnitShapes.begin(); it != m_UnitShapes.end(); ++it)
	{
		if (!filter.TestShape(UNIT_INDEX_TO_TAG(it->first), it->second.flags, it->second.group, INVALID_ENTITY))
			continue;

		entity_pos_t r1 = it->second.r;

		if (!(it->second.x + r1 < x - r || it->second.x - r1 > x + r || it->second.z + r1 < z - r || it->second.z - r1 > z + r))
		{
			if (out)
				out->push_back(it->second.entity);
			else
				return true;
		}
	}

	for (std::map<u32, StaticShape>::iterator it = m_StaticShapes.begin(); it != m_StaticShapes.end(); ++it)
	{
		if (!filter.TestShape(STATIC_INDEX_TO_TAG(it->first), it->second.flags, it->second.group, it->second.group2))
			continue;

		CFixedVector2D center1(it->second.x, it->second.z);
		if (Geometry::PointIsInSquare(center1 - center, it->second.u, it->second.v, CFixedVector2D(it->second.hw + r, it->second.hh + r)))
		{
			if (out)
				out->push_back(it->second.entity);
			else
				return true;

		}
	}

	if (out)
		return !out->empty(); // collided if the list isn't empty
	else
		return false; // didn't collide, if we got this far
}

/**
 * Compute the navcell indexes on the grid nearest to a given point
 */
static void NearestNavcell(entity_pos_t x, entity_pos_t z, u16& i, u16& j, u16 w, u16 h)
{
	i = (u16)clamp((x / ICmpObstructionManager::NAVCELL_SIZE).ToInt_RoundToZero(), 0, w-1);
	j = (u16)clamp((z / ICmpObstructionManager::NAVCELL_SIZE).ToInt_RoundToZero(), 0, h-1);
}

void CCmpObstructionManager::GetDirtyRange(Grid<u16>& grid, int& i0, int& j0, int& i1, int& j1)
{
	u16 ui0 = i0;
	u16 uj0 = j0;
	u16 ui1 = i1;
	u16 uj1 = j1;

	NearestNavcell(m_DirtyX0, m_DirtyZ0, ui0, uj0, grid.m_W, grid.m_H);
	NearestNavcell(m_DirtyX1, m_DirtyZ1, ui1, uj1, grid.m_W, grid.m_H);

	i0 = ui0;
	j0 = uj0;
	i1 = ui1;
	j1 = uj1;
}

void CCmpObstructionManager::ObstructionDirtyReset(int i0, int j0, int i1, int j1)
{
	// Reset dirty region to empty 
	m_DirtyX0 = m_WorldX1;
	m_DirtyZ0 = m_WorldZ1;
	m_DirtyX1 = m_WorldX0;
	m_DirtyZ1 = m_WorldZ0;
}

void CCmpObstructionManager::Rasterize(Grid<u16>& grid, entity_pos_t expand, ICmpObstructionManager::flags_t requireMask, u16 setMask)
{
	PROFILE3("Rasterize");

	// Since m_DirtyID is only updated for pathfinding/foundation blocking shapes,
	// NeedUpdate+Rasterise will only be accurate for that subset of shapes.
	// (If we ever want to support rasterizing more shapes, we need to improve
	// the dirty-detection system too.)
	ENSURE(!(requireMask & ~(FLAG_BLOCK_PATHFINDING|FLAG_BLOCK_FOUNDATION)));

	// Cells are only marked as blocked if the whole cell is strictly inside the shape.
	// (That ensures the shape's geometric border is always reachable.)

	// TODO: it might be nice to rasterize with rounded corners for large 'expand' values.

	// (This could be implemented much more efficiently.)

	//  debug_printf(L"# Rasterising region %f %f %f %f\n", m_DirtyX0.ToFloat(), m_DirtyZ0.ToFloat(), m_DirtyX1.ToFloat(), m_DirtyZ1.ToFloat()); 

	// XXX: if it's a large region (e.g. the whole map changed), we should just 
	// reset the entire grid and redraw every shape, instead of doing GetShapesInRange etc

	// XXX: we should do a fancier dirty-rectangles, so if there were two small changes 
 	// on opposite sides of the map we won't have to re-rasterise all the tiles in between 

	// I don't know these 3 values are appropriate for current implementation.
	entity_pos_t expandPathfinding = entity_pos_t::FromInt(0);
	//entity_pos_t expandFoundation = entity_pos_t::FromInt(TERRAIN_TILE_SIZE *3 / 4);
	//entity_pos_t dirtyExpand = std::max(expandPathfinding, expandFoundation) * 2; // expand by hopefully enough to avoid missing some edges (XXX: this might not be right) 
	entity_pos_t expandFoundation = entity_pos_t::FromInt(0);
	entity_pos_t dirtyExpand = entity_pos_t::FromInt(0);

	//u16 dirtyI0, dirtyJ0, dirtyI1, dirtyJ1;
	//NearestNavcell(m_DirtyX0, m_DirtyZ0, dirtyI0, dirtyJ0, grid.m_W, grid.m_H); 
	//NearestNavcell(m_DirtyX1, m_DirtyZ1, dirtyI1, dirtyJ1, grid.m_W, grid.m_H);
	//grid.reset(dirtyI0, dirtyJ0, dirtyI1, dirtyJ1);

	std::map<u32, StaticShape> staticShapes;
	std::map<u32, UnitShape> unitShapes;

	GetShapesInRange(m_DirtyX0 - dirtyExpand*2, m_DirtyZ0 - dirtyExpand*2, m_DirtyX1 + dirtyExpand*2, m_DirtyZ1 + dirtyExpand*2, unitShapes, staticShapes); 
	// XXX: should make GetShapesInRange only return shapes with the appropriate flags
	// XXX: to minimise memory allocation, should return shape lists as std::vector<u32>
	// instead of creating a std::map and copying the actual shape structs.
	// Or we could just use m_StaticSubdivision.GetInRange directly here.

	for (std::map<u32, StaticShape>::iterator it = m_StaticShapes.begin(); it != m_StaticShapes.end(); ++it)
	{
		const StaticShape& shape = it->second;
		if (shape.flags & requireMask)
		{
			CFixedVector2D uw(shape.u.Multiply(shape.hw + expand));
			CFixedVector2D vh(shape.v.Multiply(shape.hh + expand));

			CFixedVector2D u(shape.u.Multiply(shape.hw));
			CFixedVector2D v(shape.v.Multiply(shape.hh));

			if (uw.Y < fixed::Zero() && uw.X < fixed::Zero())
			{
				uw = -uw;
				vh = -vh;
				u = -u;
				v = -v;
			}
			else if (uw.Y < fixed::Zero())
			{
				std::swap(uw, vh);
				vh = -vh;
				std::swap(u, v);
				v = -v;
			}
			else if (uw.X < fixed::Zero())
			{
				std::swap(uw, vh);
				uw = -uw;
				std::swap(u, v);
				u = -u;
			}

			const CFixedVector2D center(shape.x, shape.z);
			const CFixedVector2D halfsize(shape.hw, shape.hh);

			CFixedVector2D P[4] = {
				center + uw + vh,	//north
				center + uw - vh,	//east
				center - uw - vh,	//south
				center - uw + vh	//west
			};

			CFixedVector2D PnoExpand[4] = {
				center + u + v,
				center + u - v,
				center - u - v,
				center - u + v
			};
			const fixed cellSize = ICmpObstructionManager::NAVCELL_SIZE;

			u16 i, j;
			u16 pi[4], pj[4];

			NearestNavcell(PnoExpand[0].X, PnoExpand[0].Y + expand, pi[0], pj[0], grid.m_W, grid.m_H);
			NearestNavcell(PnoExpand[1].X + expand, PnoExpand[1].Y, pi[1], pj[1], grid.m_W, grid.m_H);
			NearestNavcell(PnoExpand[2].X, PnoExpand[2].Y - expand, pi[2], pj[2], grid.m_W, grid.m_H);
			NearestNavcell(PnoExpand[3].X - expand, PnoExpand[3].Y, pi[3], pj[3], grid.m_W, grid.m_H);
			i = pi[0];
			j = pj[0];
			ASSERT(pj[0] > pj[2]);
			ASSERT(pi[1] > pi[3]);
			
			//axis aligned square
			if (shape.u.X == fixed::Zero() || shape.u.Y == fixed::Zero()  || shape.v.X == fixed::Zero() || shape.v.Y == fixed::Zero())
			{
				for (--j; j > pj[1]; --j)
				{
					for (--i; i > pi[2]; --i)
						grid.set(i, j, grid.get(i, j) | setMask);
				}
				continue;
			}

			//no axis-aligned square
			std::vector<int> ibegin(pj[0] - pj[2] + 1);
			std::vector<int> iend(pj[0] - pj[2] + 1);

			fixed dx;
			fixed dy;

			++i;			

			for (int dir = 0; dir <= 1; ++dir)
			{
				dx = P[dir + 1].X - P[dir].X;
				dy = P[dir + 1].Y - P[dir].Y;

				int di = (dx > fixed::Zero() ? 1 : -1);
				while (j > pj[dir + 1])
				{
					while (dir == 0 ? Geometry::DistanceToSquare(
						CFixedVector2D(cellSize * i, cellSize * j) - center,
						shape.u, shape.v, halfsize, true) <= expand
						: Geometry::DistanceToSquare(
						CFixedVector2D(cellSize * i, cellSize * j) - center,
						shape.u, shape.v, halfsize, true) > expand && i >= pi[2])
					{
						i += di;
					}

					if (0 == iend[j - (1 - dir) - pj[2]])
						iend[j - (1 - dir) - pj[2]] = std::min(i - (1 - dir), grid.m_W + 0);
					else
						iend[j - (1 - dir) - pj[2]] = std::min(i - (1 - dir), iend[j - (1 - dir) - pj[2]]);

					--j;
				}
				i = pi[1];
			}

			for (int dir = 2; dir <= 3; ++dir)
			{
				i = pi[dir];
				int ndir = (dir + 1) % 4;
				dx = P[ndir].X - P[dir].X;
				dy = P[ndir].Y - P[dir].Y;

				int di = (dx > fixed::Zero() ? 1 : -1);
				while (j <= pj[ndir])
				{
					while (dir == 2 ? Geometry::DistanceToSquare(
						CFixedVector2D(cellSize * i, cellSize * j) - center,
						shape.u, shape.v, halfsize, true) <= expand
						: Geometry::DistanceToSquare(
						CFixedVector2D(cellSize * i, cellSize * j) - center,
						shape.u, shape.v, halfsize, true) > expand && i <= pi[0])
					{
						i += di;
					}
					ibegin[j - (dir - 2) - pj[2]] = std::max(i + 1 - (dir - 2), ibegin[j - (dir - 2) - pj[2]]);
					++j;
				}
			}

			for (j = pj[0] - 1; j > pj[2]; --j)
			{
				if (ibegin[j - pj[2]] >= iend[j - pj[2]])
					continue;

				for (i = ibegin[j - pj[2]]; i < iend[j - pj[2]]; ++i)
					grid.set(i, j, grid.get(i, j) | setMask);
			}

#if DEBUG
			//for test, comare result of previous code.
			ObstructionSquare square = { shape.x, shape.z, shape.u, shape.v, shape.hw, shape.hh };
			SimRasterize::Spans spans;
			SimRasterize::RasterizeRectWithClearance(spans, square, expand, ICmpObstructionManager::NAVCELL_SIZE);
			for (size_t k = 0; k < spans.size(); ++k)
			{
				i16 j = spans[k].j;
				if (j >= 0 && j < grid.m_H)
				{
					i16 ii0 = std::max(spans[k].i0, (i16)0);
					i16 ii1 = std::min(spans[k].i1, (i16)grid.m_W);

					if (!(ii0 == ibegin[j - pj[2]] && ii1 == iend[j - pj[2]]))
					{
						debug_printf(L"P1:%d,%d P2:%d,%d P3:%d,%d P4:%d,%d\n", pi[0], pj[0], pi[1], pj[1], pi[2], pj[2], pi[3], pj[3]);

						const fixed cellSize(ICmpObstructionManager::NAVCELL_SIZE);
						fixed clearance(expand);

						debug_printf(L"ibegin[%d  - j3], ii0, ii1, iend[%d - j3]: %d %d %d %d\n", j, j, ibegin[j - pj[2]], ii0, ii1, iend[j - pj[2]]);
					}
				}
			}
#endif
		}
	}

	for (std::map<u32, UnitShape>::iterator it = unitShapes.begin(); it != unitShapes.end(); ++it)
	{
		CFixedVector2D center(it->second.x, it->second.z);

		if (it->second.flags & requireMask)
		{
			entity_pos_t r = it->second.r + expand;

			u16 i0, j0, i1, j1;
			NearestNavcell(center.X - r, center.Y - r, i0, j0, grid.m_W, grid.m_H);
			NearestNavcell(center.X + r, center.Y + r, i1, j1, grid.m_W, grid.m_H);
			for (u16 j = j0+1; j < j1; ++j)
				for (u16 i = i0+1; i < i1; ++i)
					grid.set(i, j, grid.get(i, j) | setMask);
		}
	}
}

void CCmpObstructionManager::GetShapesInRange(entity_pos_t x0, entity_pos_t z0, entity_pos_t x1, entity_pos_t z1, std::map<u32, UnitShape>& outUnitShapes, std::map<u32, StaticShape>& outStaticShapes)
{
	ENSURE(x0 <= x1 && z0 <= z1);

	std::vector<u32> unitShapes;
	m_UnitSubdivision.GetInRange(unitShapes, CFixedVector2D(x0, z0), CFixedVector2D(x1, z1));
	for (size_t i = 0; i < unitShapes.size(); ++i)
	{
		std::map<u32, UnitShape>::iterator it = m_UnitShapes.find(unitShapes[i]);
		ENSURE(it != m_UnitShapes.end());

		entity_pos_t r = it->second.r;

		// Skip this object if it's completely outside the requested range
		if (it->second.x + r < x0 || it->second.x - r > x1 || it->second.z + r < z0 || it->second.z - r > z1)
			continue;

		outUnitShapes.insert(*it);
	}

	std::vector<u32> staticShapes;
	m_StaticSubdivision.GetInRange(staticShapes, CFixedVector2D(x0, z0), CFixedVector2D(x1, z1));
	for (size_t i = 0; i < staticShapes.size(); ++i)
	{
		std::map<u32, StaticShape>::iterator it = m_StaticShapes.find(staticShapes[i]);
		ENSURE(it != m_StaticShapes.end());

		entity_pos_t r = it->second.hw + it->second.hh; // overestimate the max dist of an edge from the center

		// Skip this object if its overestimated bounding box is completely outside the requested range
		if (it->second.x + r < x0 || it->second.x - r > x1 || it->second.z + r < z0 || it->second.z - r > z1)
			continue;

		outStaticShapes.insert(*it);
	}
}

void CCmpObstructionManager::GetObstructionsInRange(const IObstructionTestFilter& filter, entity_pos_t x0, entity_pos_t z0, entity_pos_t x1, entity_pos_t z1, std::vector<ObstructionSquare>& squares)
{
	PROFILE("GetObstructionsInRange");

	ENSURE(x0 <= x1 && z0 <= z1);

	std::vector<entity_id_t> unitShapes;
	m_UnitSubdivision.GetInRange(unitShapes, CFixedVector2D(x0, z0), CFixedVector2D(x1, z1));
	for (size_t i = 0; i < unitShapes.size(); ++i)
	{
		std::map<u32, UnitShape>::iterator it = m_UnitShapes.find(unitShapes[i]);
		ENSURE(it != m_UnitShapes.end());

		if (!filter.TestShape(UNIT_INDEX_TO_TAG(it->first), it->second.flags, it->second.group, INVALID_ENTITY))
			continue;

		entity_pos_t r = it->second.r;

		// Skip this object if it's completely outside the requested range
		if (it->second.x + r < x0 || it->second.x - r > x1 || it->second.z + r < z0 || it->second.z - r > z1)
			continue;

		CFixedVector2D u(entity_pos_t::FromInt(1), entity_pos_t::Zero());
		CFixedVector2D v(entity_pos_t::Zero(), entity_pos_t::FromInt(1));
		ObstructionSquare s = { it->second.x, it->second.z, u, v, r, r };
		squares.push_back(s);
	}

	std::vector<entity_id_t> staticShapes;
	m_StaticSubdivision.GetInRange(staticShapes, CFixedVector2D(x0, z0), CFixedVector2D(x1, z1));
	for (size_t i = 0; i < staticShapes.size(); ++i)
	{
		std::map<u32, StaticShape>::iterator it = m_StaticShapes.find(staticShapes[i]);
		ENSURE(it != m_StaticShapes.end());

		if (!filter.TestShape(STATIC_INDEX_TO_TAG(it->first), it->second.flags, it->second.group, it->second.group2))
			continue;

		entity_pos_t r = it->second.hw + it->second.hh; // overestimate the max dist of an edge from the center

		// Skip this object if its overestimated bounding box is completely outside the requested range
		if (it->second.x + r < x0 || it->second.x - r > x1 || it->second.z + r < z0 || it->second.z - r > z1)
			continue;

		// TODO: maybe we should use Geometry::GetHalfBoundingBox to be more precise?

		ObstructionSquare s = { it->second.x, it->second.z, it->second.u, it->second.v, it->second.hw, it->second.hh };
		squares.push_back(s);
	}
}

void CCmpObstructionManager::GetUnitsOnObstruction(const ObstructionSquare& square, std::vector<entity_id_t>& out)
{
	PROFILE3("GetUnitsOnObstruction");

	// Ideally we'd want to find all units s.t. the RasterizeRectWithClearance
	// of the building's shape with the unit's clearance covers the navcell
	// the unit is on.
	// But ObstructionManager doesn't actually know the clearance values
	// (only the radiuses), so we can't easily do that properly.
	// Instead, cheat and just ignore the clearance entirely;
	// this might allow players to build buildings so a unit ends up on
	// an impassable tile, which is slightly bad and might let players cheat,
	// so TODO: fix this properly.

	SimRasterize::Spans spans;
	SimRasterize::RasterizeRectWithClearance(spans, square, fixed::Zero(), ICmpObstructionManager::NAVCELL_SIZE);
 
	for (std::map<u32, UnitShape>::iterator it = m_UnitShapes.begin(); it != m_UnitShapes.end(); ++it)
 	{
		// Check whether the unit's center is on a navcell that's in
		// any of the spans

		u16 i = (it->second.x / ICmpObstructionManager::NAVCELL_SIZE).ToInt_RoundToNegInfinity();
		u16 j = (it->second.z / ICmpObstructionManager::NAVCELL_SIZE).ToInt_RoundToNegInfinity();

		for (size_t k = 0; k < spans.size(); ++k)
 		{
			if (j == spans[k].j && spans[k].i0 <= i && i < spans[k].i1)
				out.push_back(it->second.entity);
 		}
 	}
 	//TODO Should we expand by r here?
}

void CCmpObstructionManager::RenderSubmit(SceneCollector& collector)
{
	if (!m_DebugOverlayEnabled)
		return;

	CColor defaultColour(0, 0, 1, 1);
	CColor movingColour(1, 0, 1, 1);
	CColor boundsColour(1, 1, 0, 1);

	// If the shapes have changed, then regenerate all the overlays
	if (m_DebugOverlayDirty)
	{
		m_DebugOverlayLines.clear();

		m_DebugOverlayLines.push_back(SOverlayLine());
		m_DebugOverlayLines.back().m_Color = boundsColour;
		SimRender::ConstructSquareOnGround(GetSimContext(),
				(m_WorldX0+m_WorldX1).ToFloat()/2.f, (m_WorldZ0+m_WorldZ1).ToFloat()/2.f,
				(m_WorldX1-m_WorldX0).ToFloat(), (m_WorldZ1-m_WorldZ0).ToFloat(),
				0, m_DebugOverlayLines.back(), true);

		for (std::map<u32, UnitShape>::iterator it = m_UnitShapes.begin(); it != m_UnitShapes.end(); ++it)
		{
			m_DebugOverlayLines.push_back(SOverlayLine());
			m_DebugOverlayLines.back().m_Color = ((it->second.flags & FLAG_MOVING) ? movingColour : defaultColour);
			SimRender::ConstructSquareOnGround(GetSimContext(), it->second.x.ToFloat(), it->second.z.ToFloat(), it->second.r.ToFloat()*2, it->second.r.ToFloat()*2, 0, m_DebugOverlayLines.back(), true);
		}

		for (std::map<u32, StaticShape>::iterator it = m_StaticShapes.begin(); it != m_StaticShapes.end(); ++it)
		{
			m_DebugOverlayLines.push_back(SOverlayLine());
			m_DebugOverlayLines.back().m_Color = defaultColour;
			float a = atan2f(it->second.v.X.ToFloat(), it->second.v.Y.ToFloat());
			SimRender::ConstructSquareOnGround(GetSimContext(), it->second.x.ToFloat(), it->second.z.ToFloat(), it->second.hw.ToFloat()*2, it->second.hh.ToFloat()*2, a, m_DebugOverlayLines.back(), true);
		}

		m_DebugOverlayDirty = false;
	}

	for (size_t i = 0; i < m_DebugOverlayLines.size(); ++i)
		collector.Submit(&m_DebugOverlayLines[i]);
}

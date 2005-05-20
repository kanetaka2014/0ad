#include "precompiled.h"

#include "CLogger.h"

#include "Interact.h"
#include "Renderer.h"
#include "input.h"
#include "CConsole.h"
#include "HFTracer.h"
#include "Hotkey.h"
#include "timer.h"
#include "Game.h"
#include "Network/NetMessage.h"
#include "BoundingObjects.h"
#include "Unit.h"
#include "Model.h"
#include "scripting/GameEvents.h"

extern CConsole* g_Console;
extern int g_mouse_x, g_mouse_y;
extern bool keys[SDLK_LAST];
extern bool g_active;
extern CStr g_CursorName;

static const float SELECT_DBLCLICK_RATE = 0.5f;
const int ORDER_DELAY = 5;

bool customSelectionMode=false;

void CSelectedEntities::addSelection( HEntity entity )
{
	m_group = -1;
	assert( !isSelected( entity ) );
	m_selected.push_back( entity );
	entity->m_selected = true;
	m_selectionChanged = true;
}

void CSelectedEntities::removeSelection( HEntity entity )
{
	m_group = -1;
	assert( isSelected( entity ) );
	entity->m_selected = false;
	std::vector<HEntity>::iterator it;
	for( it = m_selected.begin(); it < m_selected.end(); it++ )
	{
		if( (*it) == entity ) 
		{
			m_selected.erase( it );
			m_selectionChanged = true;
			break;
		}
	}
}

void CSelectedEntities::renderSelectionOutlines()
{
	std::vector<HEntity>::iterator it;
	for( it = m_selected.begin(); it < m_selected.end(); it++ )
		(*it)->renderSelectionOutline();

	if( m_group_highlight != -1 )
	{
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
		glEnable( GL_BLEND );

		std::vector<HEntity>::iterator it;
		for( it = m_groups[m_group_highlight].begin(); it < m_groups[m_group_highlight].end(); it++ )
			(*it)->renderSelectionOutline( 0.5f );

		glDisable( GL_BLEND );
	}
}

void CSelectedEntities::renderOverlays()
{
	CTerrain *pTerrain=g_Game->GetWorld()->GetTerrain();
	CCamera *pCamera=g_Game->GetView()->GetCamera();

	glPushMatrix();
	glEnable( GL_TEXTURE_2D );
	std::vector<HEntity>::iterator it;
	for( it = m_selected.begin(); it < m_selected.end(); it++ )
	{
		if( (*it)->m_grouped != -1 )
		{
			if( !(*it)->m_bounds ) continue;
			
			glLoadIdentity();
			float x, y;
			CVector3D labelpos = (*it)->m_graphics_position - pCamera->m_Orientation.GetLeft() * (*it)->m_bounds->m_radius;
#ifdef SELECTION_TERRAIN_CONFORMANCE
			labelpos.Y = pTerrain->getExactGroundLevel( labelpos.X, labelpos.Z );
#endif
			pCamera->GetScreenCoordinates( labelpos, x, y );
			glColor4f( 1.0f, 1.0f, 1.0f, 1.0f );
			glTranslatef( x, g_Renderer.GetHeight() - y, 0.0f );
			glScalef( 1.0f, -1.0f, 1.0f );
			glwprintf( L"%d", (i32) (*it)->m_grouped );
			
		}
	}
	if( m_group_highlight != -1 )
	{
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
		glEnable( GL_BLEND );

		std::vector<HEntity>::iterator it;
		for( it = m_groups[m_group_highlight].begin(); it < m_groups[m_group_highlight].end(); it++ )
		{
			if( !(*it)->m_bounds ) continue;
			
			glLoadIdentity();
			float x, y;
			CVector3D labelpos = (*it)->m_graphics_position - pCamera->m_Orientation.GetLeft() * (*it)->m_bounds->m_radius;
#ifdef SELECTION_TERRAIN_CONFORMANCE
			labelpos.Y = pTerrain->getExactGroundLevel( labelpos.X, labelpos.Z );
#endif
			pCamera->GetScreenCoordinates( labelpos, x, y );
			glColor4f( 1.0f, 1.0f, 1.0f, 0.5f );
			glTranslatef( x, g_Renderer.GetHeight() - y, 0.0f );
			glScalef( 1.0f, -1.0f, 1.0f );
			glwprintf( L"%d", (i32) (*it)->m_grouped );
		}

		glDisable( GL_BLEND );
	}

	/*
	glLoadIdentity();
	glTranslatef( (float)( g_mouse_x + 16 ), (float)( g_Renderer.GetHeight() - g_mouse_y - 8 ), 0.0f );
	glScalef( 1.0f, -1.0f, 1.0f );
	glColor4f( 1.0f, 1.0f, 1.0f, 0.5f );

	switch( m_contextOrder )
	{
	case CEntityOrder::ORDER_GOTO:
		glwprintf( L"Go to" );
		break;
	case CEntityOrder::ORDER_PATROL:
		glwprintf( L"Patrol to" );
		break;
	case CEntityOrder::ORDER_ATTACK_MELEE:
		glwprintf( L"Attack" );
		break;
	case CEntityOrder::ORDER_GATHER:
		glwprintf( L"Gather" );
		break;
	}
	*/

	glDisable( GL_TEXTURE_2D );
	glPopMatrix();
}

void CSelectedEntities::setSelection( HEntity entity )
{
	m_group = -1;
	clearSelection();
	m_selected.push_back( entity );
}

void CSelectedEntities::clearSelection()
{
	m_group = -1;
	std::vector<HEntity>::iterator it;
	for( it = m_selected.begin(); it < m_selected.end(); it++ )
		(*it)->m_selected = false;
	m_selected.clear();
	m_selectionChanged = true;
}

void CSelectedEntities::removeAll( HEntity entity )
{
	// Remove a reference to an entity from everywhere
	// (for use when said entity is being destroyed)
	std::vector<HEntity>::iterator it;
	for( it = m_selected.begin(); it < m_selected.end(); it++ )
	{
		if( (*it) == entity ) 
		{
			m_selected.erase( it );
			m_selectionChanged = true;
			break;
		}
	}
	for( u8 group = 0; group < MAX_GROUPS; group++ )
	{
		for( it = m_groups[group].begin(); it < m_groups[group].end(); it++ )
		{
			if( (*it) == entity ) 
			{
				m_groups[group].erase( it );
				m_selectionChanged = true;
				break;
			}
		}
	}
}

CVector3D CSelectedEntities::getSelectionPosition()
{
	CVector3D avg;
	std::vector<HEntity>::iterator it;
	for( it = m_selected.begin(); it < m_selected.end(); it++ )
		avg += (*it)->m_graphics_position;
	return( avg * ( 1.0f / m_selected.size() ) );
}

void CSelectedEntities::saveGroup( i8 groupid )
{
	std::vector<HEntity>::iterator it;
	// Clear all entities in the group...
	for( it = m_groups[groupid].begin(); it < m_groups[groupid].end(); it++ )
		(*it)->m_grouped = -1;
	
	m_groups[groupid].clear();
	// Remove selected entities from each group they're in, and flag them as
	// members of the new group
	for( it = m_selected.begin(); it < m_selected.end(); it++ )
	{
		if( (*it)->m_grouped != -1 )
		{
			std::vector<HEntity>& group = m_groups[(*it)->m_grouped];
			std::vector<HEntity>::iterator it2;
			for( it2 = group.begin(); it2 < group.end(); it2++ )
			{
				if( (*it2) == &(**it) )
				{
					group.erase( it2 );
					break;
				}
			}
		}
		(*it)->m_grouped = groupid;
	}
	// Copy the group across
	m_groups[groupid] = m_selected;
	// Set the group selection memory
	m_group = groupid;
}

void CSelectedEntities::addToGroup( i8 groupid, HEntity entity )
{
	std::vector<HEntity>::iterator it;

	// Remove selected entities from each group they're in, and flag them as
	// members of the new group
	if( entity->m_grouped != -1 )
	{
		std::vector<HEntity>& group = m_groups[(*it)->m_grouped];
		std::vector<HEntity>::iterator it2;
		for( it2 = group.begin(); it2 < group.end(); it2++ )
		{
			if( (*it2) == entity )
			{
				group.erase( it2 );
				break;
			}
		}
	}
	entity->m_grouped = groupid;

	m_groups[groupid].push_back( entity );
}

void CSelectedEntities::loadGroup( i8 groupid )
{
	if( m_group == groupid )
		return;

	clearSelection();
	m_selected = m_groups[groupid];

	std::vector<HEntity>::iterator it;
	for( it = m_selected.begin(); it < m_selected.end(); it++ )
		(*it)->m_selected = true;
	m_group = groupid;

	m_selectionChanged = true;
}

void CSelectedEntities::addGroup( i8 groupid )
{
	std::vector<HEntity>::iterator it;
	for( it = m_groups[groupid].begin(); it < m_groups[groupid].end(); it++ )
	{
		if( !isSelected( *it ) )
			addSelection( *it );
	}
	for( it = m_selected.begin(); it < m_selected.end(); it++ )
		(*it)->m_selected = true;
}

void CSelectedEntities::changeGroup( HEntity entity, i8 groupid )
{
	// Remove from current group
	i32 current = entity->m_grouped;
	if( current != -1 )
	{
		std::vector<HEntity>::iterator it;
		for( it = m_groups[current].begin(); it < m_groups[current].end(); it++ )
		{
			if( (*it) == entity ) 
			{
				m_groups[current].erase( it );
				break;
			}
		}
	}
	if( groupid != -1 )
		m_groups[groupid].push_back( entity );
	entity->m_grouped = groupid;
}

bool CSelectedEntities::isSelected( HEntity entity )
{
	std::vector<HEntity>::iterator it;
	for( it = m_selected.begin(); it < m_selected.end(); it++ )
	{
		if( (*it) == entity ) 
			return( true );
	}
	return( false );
}

void CSelectedEntities::highlightGroup( i8 groupid )
{
	if( m_group_highlight != -1 )
		return;
	if( !getGroupCount( groupid ) )
		return;
	m_group_highlight = groupid;
	g_Game->GetView()->PushCameraTarget( getGroupPosition( groupid ) );
}

void CSelectedEntities::highlightNone()
{

	if( m_group_highlight != -1 )
		g_Game->GetView()->PopCameraTarget();
	m_group_highlight = -1;

}

int CSelectedEntities::getGroupCount( i8 groupid )
{
	return( (int)m_groups[groupid].size() );
}

CVector3D CSelectedEntities::getGroupPosition( i8 groupid )
{
	CVector3D avg;
	std::vector<HEntity>::iterator it;
	for( it = m_groups[groupid].begin(); it < m_groups[groupid].end(); it++ )
		avg += (*it)->m_graphics_position;
	return( avg * ( 1.0f / m_groups[groupid].size() ) );
}

void CSelectedEntities::update()
{
	static std::vector<HEntity> lastSelection;
	
	// Drop out immediately if we're in some special interaction mode
	if (customSelectionMode)
		return;

	if( !( m_selected == lastSelection ) )
	{
		g_JSGameEvents.FireSelectionChanged( m_selectionChanged );
		lastSelection = m_selected; 
	}

	if( m_selectionChanged || g_Mouseover.m_targetChanged )
	{
		// Can't order anything off the map
		if( !g_Game->GetWorld()->GetTerrain()->isOnMap( g_Mouseover.m_worldposition ) )
		{
			m_defaultCommand = -1;
			return;
		}
		
		// Quick count to see which is the modal default order.

		const int numCommands=NMT_COMMAND_LAST - NMT_COMMAND_FIRST;
		int defaultPoll[numCommands];
		std::map<CStrW, int, CStrW_hash_compare> defaultCursor[numCommands];

		int t, vote;
		for( t = 0; t < numCommands; t++ )
			defaultPoll[t] = 0;

		std::vector<HEntity>::iterator it;
		for( it = m_selected.begin(); it < m_selected.end(); it++ )
		{
			CEventTargetChanged evt( g_Mouseover.m_target );
			(*it)->DispatchEvent( &evt );
			vote = evt.m_defaultAction - NMT_COMMAND_FIRST;
			
			if( ( vote >= 0 ) && ( vote < numCommands ) )
			{
				defaultPoll[vote]++;
				defaultCursor[vote][evt.m_defaultCursor]++;
			}
		}

		vote = -1;
		for( t = 0; t < numCommands; t++ )
		{
			if( ( vote == -1 ) || ( defaultPoll[t] > defaultPoll[vote] ) )
				vote = t;
		}

		std::map<CStrW, int, CStrW_hash_compare>::iterator itv;
		m_defaultCommand = vote + NMT_COMMAND_FIRST;

		// Now find the most appropriate cursor
		t = 0;
		for( itv = defaultCursor[vote].begin(); itv != defaultCursor[vote].end(); itv++ )
			if( itv->second > t )
			{
				t = itv->second;
				g_CursorName = itv->first;
			}

		m_selectionChanged = false;
		g_Mouseover.m_targetChanged = false;
	}

	if( ( m_group_highlight != -1 ) && getGroupCount( m_group_highlight ) )
		g_Game->GetView()->SetCameraTarget( getGroupPosition( m_group_highlight ) );

}

void CMouseoverEntities::update( float timestep )
{
	CCamera *pCamera=g_Game->GetView()->GetCamera();
	CTerrain *pTerrain=g_Game->GetWorld()->GetTerrain();

	CVector3D origin, dir;
	pCamera->BuildCameraRay( origin, dir );

	CUnit* hit = g_UnitMan.PickUnit( origin, dir );
	
	m_worldposition = pCamera->GetWorldCoordinates();

	if( hit && hit->GetEntity() && hit->GetEntity()->m_extant )
	{
		m_target = hit->GetEntity()->me;
	}
	else
		m_target = HEntity();

	if( m_target != m_lastTarget )
	{
		m_targetChanged = true;
		m_lastTarget = m_target;
	}

	if( m_viewall )
	{
		// 'O' key. Show selection outlines for all player units on screen
		// (as if bandboxing them all).
		// These aren't selectable; clicking when 'O' is pressed won't select
		// all units on screen.

		m_mouseover.clear();

		std::vector<HEntity>* onscreen = g_EntityManager.matches( isOnScreen );
		std::vector<HEntity>::iterator it;
		
		for( it = onscreen->begin(); it < onscreen->end(); it++ )
			if( (*it)->m_extant && ( (*it)->GetPlayer() == g_Game->GetLocalPlayer() ) )
				m_mouseover.push_back( SMouseoverFader( *it, m_fademaximum, false ) );

		delete( onscreen );
	}
	else if( m_bandbox )
	{
		m_x2 = g_mouse_x;
		m_y2 = g_mouse_y;
		// Here's the fun bit:
		// Get the screen-space coordinates of all onscreen entities
		// then find the ones falling within the box.
		//
		// Fade in the ones in the box at (in+out) speed, then fade everything
		// out at (out) speed.
		std::vector<HEntity>* onscreen = g_EntityManager.matches( isOnScreen );
		std::vector<HEntity>::iterator it;

		// Reset active flags on everything...

		std::vector<SMouseoverFader>::iterator it2;
		for( it2 = m_mouseover.begin(); it2 < m_mouseover.end(); it2++ )
			it2->isActive = false;

		for( it = onscreen->begin(); it < onscreen->end(); it++ )
		{
			if( !(*it)->m_extant )
				continue;

			// Can only bandbox units the local player controls.
			if( (*it)->GetPlayer() != g_Game->GetLocalPlayer() )
				continue;

			CVector3D worldspace = (*it)->m_graphics_position;

			float x, y;

			pCamera->GetScreenCoordinates( worldspace, x, y );

			bool inBox;
			if( m_x1 < m_x2 )
			{
				inBox = ( x >= m_x1 ) && ( x < m_x2 );
			}
			else
			{
				inBox = ( x >= m_x2 ) && ( x < m_x1 );
			}
			
			if( m_y1 < m_y2 )
			{
				inBox &= ( y >= m_y1 ) && ( y < m_y2 );
			}
			else
			{
				inBox &= ( y >= m_y2 ) && ( y < m_y1 );
			}
			
			if( inBox )
			{
				bool found = false;
				for( it2 = m_mouseover.begin(); it2 < m_mouseover.end(); it2++ )
					if( it2->entity == &(**it) )
					{
						found = true;
						it2->fade += ( m_fadeinrate + m_fadeoutrate ) * timestep;
						it2->isActive = true;
					}
				if( !found )
					m_mouseover.push_back( SMouseoverFader( *it, ( m_fadeinrate + m_fadeoutrate ) * timestep ) );
			}
		}
		delete( onscreen );
		
		for( it2 = m_mouseover.begin(); it2 < m_mouseover.end(); )
		{
			it2->fade -= m_fadeoutrate * timestep;
			if( it2->fade > m_fademaximum ) it2->fade = m_fademaximum;
			if( it2->fade < 0.0f )
			{
				it2 = m_mouseover.erase( it2 );
			}
			else
				it2++;
		}
	}
	else
	{
		std::vector<SMouseoverFader>::iterator it;
		bool found = false;

		for( it = m_mouseover.begin(); it < m_mouseover.end(); )
		{
			if( it->entity == m_target )
			{
				found = true;
				it->fade += m_fadeinrate * timestep;
				if( it->fade > m_fademaximum ) it->fade = m_fademaximum;
				it->isActive = true;
				it++; continue;
			}
			else
			{
				it->fade -= m_fadeoutrate * timestep;
				if( it->fade <= 0.0f )
				{
					it = m_mouseover.erase( it ); continue;
				}
				it++; continue;
			}
		}
		if( !found && (bool)m_target )
		{
			float initial = m_fadeinrate * timestep;
			if( initial > m_fademaximum ) initial = m_fademaximum;
			m_mouseover.push_back( SMouseoverFader( m_target, initial ) );
		}
	}
}

void CMouseoverEntities::addSelection()
{
	// Rules for shift-click selection:

	// If selecting a non-allied unit, you can only select one. You can't 
	// select a mix of allied and non-allied units. Therefore:
	// Forbid shift-click of enemy units unless the selection is empty
	// Forbid shift-click of allied units if the selection contains one
	//   or more enemy units.

	if( ( m_mouseover.size() != 0 ) &&
		( m_mouseover.front().entity->GetPlayer() != g_Game->GetLocalPlayer() ) && 
		( g_Selection.m_selected.size() != 0 ) )
		return;
	
	if( ( g_Selection.m_selected.size() != 0 ) &&
		( g_Selection.m_selected.front()->GetPlayer() != g_Game->GetLocalPlayer() ) )
		return;

	std::vector<SMouseoverFader>::iterator it;
	for( it = m_mouseover.begin(); it < m_mouseover.end(); it++ )
		if( it->isActive && !g_Selection.isSelected( it->entity ) )
			g_Selection.addSelection( it->entity );
}

void CMouseoverEntities::removeSelection()
{
	std::vector<SMouseoverFader>::iterator it;
	for( it = m_mouseover.begin(); it < m_mouseover.end(); it++ )
		if( it->isActive && g_Selection.isSelected( it->entity ) )
			g_Selection.removeSelection( it->entity );
}

void CMouseoverEntities::setSelection()
{
	g_Selection.clearSelection();
	addSelection();
}

void CMouseoverEntities::expandAcrossScreen()
{
	std::vector<HEntity>* activeset = g_EntityManager.matches( 
		CEntityManager::EntityPredicateLogicalAnd<isMouseoverType,isOnScreen> );
	m_mouseover.clear();
	std::vector<HEntity>::iterator it;
	for( it = activeset->begin(); it < activeset->end(); it++ )
		if( (*it)->m_extant )
			m_mouseover.push_back( SMouseoverFader( *it ) );
	delete( activeset );
}

void CMouseoverEntities::expandAcrossWorld()
{
	std::vector<HEntity>* activeset = g_EntityManager.matches( isMouseoverType );
	m_mouseover.clear();
	std::vector<HEntity>::iterator it;
	for( it = activeset->begin(); it < activeset->end(); it++ )
		if( (*it)->m_extant )
			m_mouseover.push_back( SMouseoverFader( *it ) );
	delete( activeset );
}

void CMouseoverEntities::renderSelectionOutlines()
{
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	glEnable( GL_BLEND );

	std::vector<SMouseoverFader>::iterator it;
	for( it = m_mouseover.begin(); it < m_mouseover.end(); it++ )
		it->entity->renderSelectionOutline( it->fade );

	glDisable( GL_BLEND );
}

void CMouseoverEntities::renderOverlays()
{
	CCamera *pCamera=g_Game->GetView()->GetCamera();
	CTerrain *pTerrain=g_Game->GetWorld()->GetTerrain();

	glLoadIdentity();
	glDisable( GL_TEXTURE_2D );
	if( m_bandbox )
	{
		//glPushMatrix();
		glColor3f( 1.0f, 1.0f, 1.0f );
		glBegin( GL_LINE_LOOP );
		glVertex2i( m_x1, g_Renderer.GetHeight() - m_y1 );
		glVertex2i( m_x2, g_Renderer.GetHeight() - m_y1 );
		glVertex2i( m_x2, g_Renderer.GetHeight() - m_y2 );
		glVertex2i( m_x1, g_Renderer.GetHeight() - m_y2 );
		glEnd();
		//glPopMatrix();
	}
	glEnable( GL_TEXTURE_2D );
	
	std::vector<SMouseoverFader>::iterator it;
	for( it = m_mouseover.begin(); it < m_mouseover.end(); it++ )
	{
		if( it->entity->m_grouped != -1 )
		{
			if( !it->entity->m_bounds ) continue;
			glPushMatrix();
			glEnable( GL_TEXTURE_2D );
			glLoadIdentity();
			float x, y;
			CVector3D labelpos = it->entity->m_graphics_position - pCamera->m_Orientation.GetLeft() * it->entity->m_bounds->m_radius;
#ifdef SELECTION_TERRAIN_CONFORMANCE
			labelpos.Y = pTerrain->getExactGroundLevel( labelpos.X, labelpos.Z );
#endif
			pCamera->GetScreenCoordinates( labelpos, x, y );
			glColor4f( 1.0f, 1.0f, 1.0f, it->fade );
			glTranslatef( x, g_Renderer.GetHeight() - y, 0.0f );
			glScalef( 1.0f, -1.0f, 1.0f );
			glwprintf( L"%d", (i32) it->entity->m_grouped );
			glDisable( GL_TEXTURE_2D );
			glPopMatrix();
		}
	}
}

void CMouseoverEntities::startBandbox( u16 x, u16 y )
{
	m_bandbox = true;
	m_x1 = x; m_y1 = y;
}

void CMouseoverEntities::stopBandbox()
{
	m_bandbox = false;
}

void FireWorldClickEvent(uint button, int clicks)
{
	debug_printf("FireWorldClickEvent: button %d, clicks %d\n", button, clicks);
	g_JSGameEvents.FireWorldClick(
		button,
		clicks,
		g_Selection.m_defaultCommand,
		-1, // FIXME Secondary command, depends entity scripts etc
		g_Mouseover.m_target,
		(uint)g_Mouseover.m_worldposition.x,
		(uint)g_Mouseover.m_worldposition.y);
}

void MouseButtonUpHandler(const SDL_Event *ev, int clicks)
{
	FireWorldClickEvent(ev->button.button, clicks);
	
	switch( ev->button.button )
	{
	case SDL_BUTTON_LEFT:
		if (customSelectionMode)
			break;
	
		if( g_Mouseover.m_viewall )
			break;

		if( clicks == 2 )
		{
			// Double click
			g_Mouseover.expandAcrossScreen();
		}
		else if( clicks == 3 )
		{
			// Triple click
			g_Mouseover.expandAcrossWorld();
		}

		g_Mouseover.stopBandbox();
		if( hotkeys[HOTKEY_SELECTION_ADD] )
		{
			g_Mouseover.addSelection();
		}
		else if( hotkeys[HOTKEY_SELECTION_REMOVE] )
		{
			g_Mouseover.removeSelection();
		}
		else
			g_Mouseover.setSelection();
		break;
	}
}

int interactInputHandler( const SDL_Event* ev )
{
	if (!g_active || !g_Game)
		return EV_PASS;

	CGameView *pView=g_Game->GetView();
	CCamera *pCamera=pView->GetCamera();
	CTerrain *pTerrain=g_Game->GetWorld()->GetTerrain();

	// One entry for each of five mouse buttons (SDL mouse buttons 1-5, mouse
	// buttons over 5 if existant, will be ignored)
	static float lastclicktime[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	static HEntity lastclickobject[5];
	static u8 clicks[5] = {0, 0, 0, 0, 0};

	static u16 button_down_x, button_down_y;
	static double button_down_time;
	static bool button_down = false;
	
	if (customSelectionMode && ev->type != SDL_MOUSEBUTTONUP)
		return EV_PASS;
	
	switch( ev->type )
	{	
	case SDL_HOTKEYDOWN:
		switch( ev->user.code )
		{
		case HOTKEY_HIGHLIGHTALL:
			g_Mouseover.m_viewall = true;
			break;
		case HOTKEY_SELECTION_SNAP:
			if( g_Selection.m_selected.size() )
				pView->SetCameraTarget( g_Selection.getSelectionPosition() );
			break;
		default:
			if( ( ev->user.code >= HOTKEY_SELECTION_GROUP_0 ) && ( ev->user.code <= HOTKEY_SELECTION_GROUP_19 ) )
			{
				// The above test limits it to 20 groups, so don't worry about overflowing

				i8 id = (i8)( ev->user.code - HOTKEY_SELECTION_GROUP_0 );
				
				if( hotkeys[HOTKEY_SELECTION_GROUP_ADD] )
				{
					g_Selection.addGroup( id );
				}
				else if( hotkeys[HOTKEY_SELECTION_GROUP_SAVE] )
				{
					g_Selection.saveGroup( id );
				}
				else if( hotkeys[HOTKEY_SELECTION_GROUP_SNAP] )
				{
					g_Selection.highlightGroup( id );
				}
				else
				{
					if( ( g_Selection.m_group == id ) && g_Selection.getGroupCount( id ) )
					{
						pView->SetCameraTarget( g_Selection.getGroupPosition( id ) );
					}
					else
						g_Selection.loadGroup( id );
				}
				return( EV_HANDLED );
			}
		
			return( EV_PASS );
		}
		return( EV_HANDLED );
	case SDL_HOTKEYUP:
		switch( ev->user.code )
		{
		case HOTKEY_SELECTION_GROUP_SNAP:
			if( g_Selection.m_group_highlight != -1 )
				g_Selection.highlightNone();
			break;
		case HOTKEY_HIGHLIGHTALL:
			g_Mouseover.m_viewall = false;
			break;
		default:
			return( EV_PASS );
		}
		return( EV_HANDLED );
	case SDL_MOUSEBUTTONUP:
	{
		// Assumes SDL button enums are contiguous
		int button = ev->button.button - SDL_BUTTON_LEFT;
		// Only process buttons within the range for which we have button state
		// arrays above.
		if (button >= 0 && button < 5)
		{
			float time;
			time = (float)get_time();
			// Reset clicks counter if too slow or if the cursor's
			// hovering over something else now.
			
			if( time - lastclicktime[button] >= SELECT_DBLCLICK_RATE )
				clicks[button] = 0;
			if( g_Mouseover.m_target != lastclickobject[button] )
				clicks[button] = 0;
			clicks[button]++;

			lastclicktime[button] = time;
			lastclickobject[button] = g_Mouseover.m_target;

			if (ev->button.button == SDL_BUTTON_LEFT)
				button_down = false;

			MouseButtonUpHandler(ev, clicks[button]);
		}
		break;
	}
	case SDL_MOUSEBUTTONDOWN:
		switch( ev->button.button )
		{
		case SDL_BUTTON_LEFT:
			button_down = true;
			button_down_x = ev->button.x;
			button_down_y = ev->button.y;
			button_down_time = get_time();
			break;
		}
		break;
	case SDL_MOUSEMOTION:
		if( !g_Mouseover.isBandbox() && button_down )
		{
			int deltax = ev->motion.x - button_down_x;
			int deltay = ev->motion.y - button_down_y;
			if( ABS( deltax ) > 2 || ABS( deltay ) > 2 )
				g_Mouseover.startBandbox( button_down_x, button_down_y );
		}
		break;
	}
	return( EV_PASS );
}

bool isOnScreen( CEntity* ev, void* userdata )
{
	CCamera *pCamera=g_Game->GetView()->GetCamera();

	CFrustum frustum = pCamera->GetFrustum();

	if( ev->m_actor )
		return( frustum.IsBoxVisible( CVector3D(), ev->m_actor->GetModel()->GetBounds() ) );
	else
		// If there's no actor, just treat the entity as a point
		return( frustum.IsBoxVisible( ev->m_graphics_position, CBound() ) );
}

bool isMouseoverType( CEntity* ev, void* userdata )
{
	std::vector<SMouseoverFader>::iterator it;
	for( it = g_Mouseover.m_mouseover.begin(); it < g_Mouseover.m_mouseover.end(); it++ )
	{
		if( it->isActive && ( (CBaseEntity*)it->entity->m_base == (CBaseEntity*)ev->m_base ) )
			return( true );
	}
	return( false );
}

void StartCustomSelection()
{
	customSelectionMode = true;
}

void ResetInteraction()
{
	customSelectionMode = false;
}

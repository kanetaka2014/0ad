#include "precompiled.h"

#include "MessageHandler.h"
#include "../GameLoop.h"
#include "../View.h"

#include "maths/Vector3D.h"
#include "maths/Quaternion.h"
#include "ps/Game.h"
#include "renderer/Renderer.h"
#include "graphics/GameView.h"

#include <assert.h>

namespace AtlasMessage {


MESSAGEHANDLER(ScrollConstant)
{
	if (g_Game->GetView()->GetCinema()->IsPlaying())
		return;

	if (msg->dir < 0 || msg->dir > 3)
	{
		debug_warn("ScrollConstant: invalid direction");
	}
	else
	{
		g_GameLoop->input.scrollSpeed[msg->dir] = msg->speed;
	}
}

// TODO: change all these g_Game->...GetCamera() bits to use the current View's
// camera instead.

MESSAGEHANDLER(Scroll)
{
	if (g_Game->GetView()->GetCinema()->IsPlaying()) // TODO: do this better (probably a separate View class for cinematics)
		return;

	static CVector3D targetPos;
	static float targetDistance = 0.f;

	CMatrix3D& camera = g_Game->GetView()->GetCamera()->m_Orientation;

	static CVector3D lastCameraPos = camera.GetTranslation();

	// Ensure roughly correct motion when dragging is combined with other
	// movements.
	if (lastCameraPos != camera.GetTranslation())
		targetPos += camera.GetTranslation() - lastCameraPos;

	// General operation:
	//
	// When selecting a target point to drag, remember targetPos (a world-space
	// point on the terrain, underneath the mouse) and targetDistance (from the
	// camera to the target point).
	//
	// When dragging to a different position, the target point should remain
	// under the moved mouse; so calculate the ray through the camera and mouse,
	// multiply by targetDistance and add to targetPos, resulting in the required
	// camera position.

	if (msg->type == eScrollType::FROM)
	{
		targetPos = msg->pos->GetWorldSpace();
		targetDistance = (targetPos - camera.GetTranslation()).GetLength();
	}
	else if (msg->type == eScrollType::TO)
	{
		CVector3D origin, dir;
		float x, y;
		msg->pos->GetScreenSpace(x, y);
		g_Game->GetView()->GetCamera()->BuildCameraRay((int)x, (int)y, origin, dir);
		dir *= targetDistance;
		camera.Translate(targetPos - dir - origin);
		g_Game->GetView()->GetCamera()->UpdateFrustum();
	}
	else
	{
		debug_warn("Scroll: Invalid type");
	}
	lastCameraPos = camera.GetTranslation();
}

MESSAGEHANDLER(SmoothZoom)
{
	if (g_Game->GetView()->GetCinema()->IsPlaying())
		return;

	g_GameLoop->input.zoomDelta += msg->amount;
}

MESSAGEHANDLER(RotateAround)
{
	if (g_Game->GetView()->GetCinema()->IsPlaying())
		return;

	static CVector3D focusPos;
	static float lastX = 0.f, lastY = 0.f;

	CMatrix3D& camera = g_Game->GetView()->GetCamera()->m_Orientation;

	if (msg->type == eRotateAroundType::FROM)
	{
		msg->pos->GetScreenSpace(lastX, lastY); // get mouse position
		focusPos = msg->pos->GetWorldSpace(); // get point on terrain under mouse
	}
	else if (msg->type == eRotateAroundType::TO)
	{
		float x, y;
		msg->pos->GetScreenSpace(x, y); // get mouse position

		// Rotate around X and Y axes by amounts depending on the mouse delta
		float rotX = 6.f * (y-lastY) / g_Renderer.GetHeight();
		float rotY = 6.f * (x-lastX) / g_Renderer.GetWidth();

		CQuaternion q0, q1;
		q0.FromAxisAngle(camera.GetLeft(), -rotX);
		q1.FromAxisAngle(CVector3D(0.f, 1.f, 0.f), rotY);
		CQuaternion q = q0*q1;

		CVector3D origin = camera.GetTranslation();
		CVector3D offset = q.Rotate(origin - focusPos);

		q *= camera.GetRotation();
		q.Normalize(); // to avoid things blowing up when turning upside-down, for some reason I don't understand
		q.ToMatrix(camera);

		// Make sure up is still pointing up, regardless of any rounding errors.
		// (Maybe this distorts the camera in other ways, but at least the errors
		// are far less noticeable to me.)
		camera._21 = 0.f; // (_21 = Y component returned by GetLeft())

		camera.Translate(focusPos + offset);
		g_Game->GetView()->GetCamera()->UpdateFrustum();

		lastX = x;
		lastY = y;
	}
	else
	{
		debug_warn("RotateAround: Invalid type");
	}
}

MESSAGEHANDLER(LookAt)
{
	// TODO: different camera depending on msg->view
	CCamera& camera = View::GetView_Actor()->GetCamera();

	CVector3D tgt = msg->target->GetWorldSpace();
	CVector3D eye = msg->pos->GetWorldSpace();
 	tgt.Y = -tgt.Y; // ??? why is this needed?
 	eye.Y = -eye.Y; // ???

	// Based on http://www.opengl.org/documentation/specs/man_pages/hardcopy/GL/html/glu/lookat.html
	CVector3D f = tgt - eye;
	f.Normalize();
	CVector3D s = f.Cross(CVector3D(0, 1, 0));
	CVector3D u = s.Cross(f);
	s.Normalize(); // (not in that man page, but necessary for correctness, and done by Mesa)
	u.Normalize();
	CMatrix3D M (
		s[0], s[1], s[2], 0,
		u[0], u[1], u[2], 0,
		-f[0], -f[1], -f[2], 0,
		0, 0, 0, 1
	);

	M.GetTranspose(camera.m_Orientation);
	camera.m_Orientation.Translate(-eye);

	camera.UpdateFrustum();
}


}

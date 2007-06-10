#include "precompiled.h"

#ifndef WX_PRECOMP
	#include <wx/wx.h>
#endif

#include "../../common/main.h"
#include "../../common/apiwrap.h"

#include <wx/spinctrl.h>

#include "jsevent.h"
#include "../misc/size.h"
#include "spinevt.h"

using namespace wxjs;
using namespace wxjs::gui;

WXJS_INIT_CLASS(SpinEvent, "wxSpinEvent", 0)

WXJS_BEGIN_PROPERTY_MAP(SpinEvent)
	WXJS_PROPERTY(P_POSITION, "position")
WXJS_END_PROPERTY_MAP()

bool SpinEvent::GetProperty(PrivSpinEvent *p, JSContext *cx, JSObject *obj, int id, jsval *vp)
{
	wxSpinEvent *event = (wxSpinEvent*) p->GetEvent();

	switch ( id )
	{
	case P_POSITION:
		*vp = ToJS(cx, event->GetPosition());
		break;
	}
	return true;
}

bool SpinEvent::SetProperty(PrivSpinEvent *p, JSContext *cx, JSObject *obj, int id, jsval *vp)
{
	wxSpinEvent *event = (wxSpinEvent*) p->GetEvent();

	switch ( id )
	{
	case P_POSITION:
		{
			int pos;
			if ( FromJS(cx, *vp, pos) )
				event->SetPosition(pos);
			break;
		}
	}
	return true;
}

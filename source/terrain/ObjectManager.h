#ifndef _OBJECTMANAGER_H
#define _OBJECTMANAGER_H

#include <vector>
#include "ObjectEntry.h"

class CObjectManager 
{
public:
	struct SObjectType
	{
		// name of this object type (derived from directory name)
		CStr m_Name;
		// index in parent array
		int m_Index;
		// list of objects of this type (found from the objects directory)
		std::vector<CObjectEntry*> m_Objects;
	};

public:
	CObjectManager();

	void LoadObjects();


	void AddObjectType(const char* name);

	CObjectEntry* FindObject(const char* objname);
	void AddObject(CObjectEntry* entry,int type);
	void DeleteObject(CObjectEntry* entry);

	CObjectEntry* GetSelectedObject() const { return m_SelectedObject; }
	void SetSelectedObject(CObjectEntry* obj) { m_SelectedObject=obj; }

	std::vector<SObjectType> m_ObjectTypes;

private:
	void BuildObjectTypes();
	void LoadObjects(int type);

	CObjectEntry* m_SelectedObject;
};

extern CObjectManager g_ObjMan;


#endif
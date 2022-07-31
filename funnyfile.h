#include "core/logic/HandleSys.h"
#include <memory>

class HandleSystemHack : public HandleSystem
{
public:
	HandleType_t CreateTypeAllowChild(const char *name, 
										IHandleTypeDispatch *dispatch, 
										HandleType_t parent, 
										const TypeAccess *typeAccess, 
										const HandleAccess *hndlAccess, 
										IdentityToken_t *ident,
										HandleError *err)
	{
		return CreateType__(name, dispatch, parent, typeAccess, hndlAccess, ident, err);
	}

	HandleType_t CreateType__(const char *name, 
										IHandleTypeDispatch *dispatch, 
										HandleType_t parent, 
										const TypeAccess *typeAccess, 
										const HandleAccess *hndlAccess, 
										IdentityToken_t *ident,
										HandleError *err)
	{
		if (!dispatch)
		{
			if (err)
			{
				*err = HandleError_Parameter;
			}
			return 0;
		}

		if (typeAccess && typeAccess->hsVersion > SMINTERFACE_HANDLESYSTEM_VERSION)
		{
			if (err)
			{
				*err = HandleError_Version;
			}
			return 0;
		}

		if (hndlAccess && hndlAccess->hsVersion > SMINTERFACE_HANDLESYSTEM_VERSION)
		{
			if (err)
			{
				*err = HandleError_Version;
			}
			return 0;
		}

		bool isChild = false;

		if (parent != 0)
		{
			isChild = true;
		#if 0
			if (parent & HANDLESYS_SUBTYPE_MASK)
			{
				if (err)
				{
					*err = HandleError_NoInherit;
				}
				return 0;
			}
		#endif
			if (parent >= HANDLESYS_TYPEARRAY_SIZE
				|| m_Types[parent].dispatch == NULL)
			{
				if (err)
				{
					*err = HandleError_Parameter;
				}
				return 0;
			}
			if (m_Types[parent].typeSec.access[HTypeAccess_Inherit] == false
				&& (m_Types[parent].typeSec.ident != ident))
			{
				if (err)
				{
					*err = HandleError_Access;
				}
				return 0;
			}
		}

		if (name && name[0] != '\0')
		{
			if (m_TypeLookup.contains(name))
			{
				if (err)
					*err = HandleError_Parameter;
				return 0;
			}
		}

		unsigned int index;

		if (isChild)
		{
			QHandleType *pParent = &m_Types[parent];
			if (pParent->children >= HANDLESYS_MAX_SUBTYPES)
			{
				if (err)
				{
					*err = HandleError_Limit;
				}
				return 0;
			}
			index = 0;
			for (unsigned int i=1; i<=HANDLESYS_MAX_SUBTYPES; i++)
			{
				if (m_Types[parent + i].dispatch == NULL)
				{
					index = parent + i;
					break;
				}
			}
			if (!index)
			{
				if (err)
				{
					*err = HandleError_Limit;
				}
				return 0;
			}
			pParent->children++;
		} else {
			if (m_FreeTypes == 0)
			{
				/* Reserve another index */
				if (m_TypeTail >= HANDLESYS_TYPEARRAY_SIZE)
				{
					if (err)
					{
						*err = HandleError_Limit;
					}
					return 0;
				} else {
					m_TypeTail += (HANDLESYS_MAX_SUBTYPES + 1);
					index = m_TypeTail;
				}
			} else {
				/* The "free array" is compacted into the normal array for easiness */
				index = m_Types[m_FreeTypes--].freeID;
			}
		}

		QHandleType *pType = &m_Types[index];
		
		pType->dispatch = dispatch;
		if (name && name[0] != '\0')
		{
		#ifdef __OLDSM
			pType->name = new ke::AString(name);
		#else
			pType->name = std::make_unique<std::string>(name);
		#endif
			m_TypeLookup.insert(name, pType);
		}

		pType->opened = 0;

		if (typeAccess)
		{
			pType->typeSec = *typeAccess;
		} else {
			InitAccessDefaults(&pType->typeSec, NULL);
			pType->typeSec.ident = ident;
		}

		if (hndlAccess)
		{
			pType->hndlSec = *hndlAccess;
		} else {
			InitAccessDefaults(NULL, &pType->hndlSec);
		}

		if (!isChild)
		{
			pType->children = 0;
		}

		return index;
	}
};

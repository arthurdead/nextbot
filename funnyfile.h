#include <memory>

#include <core/logic/CellArray.h>

#include "core/logic/HandleSys.h"

HandleType_t arraylist_handle = 0;
IdentityType_t coreidenttype = 0;

inline HandleType_t TypeParent(HandleType_t type)
{
	return (type & ~HANDLESYS_SUBTYPE_MASK);
}

class HandleSystemHack : public HandleSystem
{
public:
	static void init()
	{
		handlesys->FindHandleType("CellArray", &arraylist_handle);
		coreidenttype = sharesys->FindIdentType("CORE");
	}

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

	HandleError ReadCoreHandle(Handle_t handle, HandleType_t type, const HandleSecurity *pSecurity, void **object)
	{
		return ReadHandle__(handle, type, pSecurity, object);
	}

private:
	HandleError ReadHandle__(Handle_t handle, HandleType_t type, const HandleSecurity *pSecurity, void **object)
	{
		unsigned int index;
		QHandle *pHandle;
		HandleError err;
		IdentityToken_t *ident = pSecurity ? pSecurity->pIdentity : NULL;

		if ((err=GetHandle__(handle, ident, &pHandle, &index)) != HandleError_None)
		{
			return err;
		}

		if (!CheckAccess__(pHandle, HandleAccess_Read, pSecurity))
		{
			return HandleError_Access;
		}

		/* Check the type inheritance */
		if (pHandle->type & HANDLESYS_SUBTYPE_MASK)
		{
			if (pHandle->type != type
				&& (TypeParent(pHandle->type) != TypeParent(type)))
			{
				return HandleError_Type;
			}
		} else if (type) {
			if (pHandle->type != type)
			{
				return HandleError_Type;
			}
		}

		if (object)
		{
			/* if we're a clone, the rules change - object is ONLY in our reference */
			if (pHandle->clone)
			{
				pHandle = &m_Handles[pHandle->clone];
			}
			*object = pHandle->object;
		}

		return HandleError_None;
	}

	HandleError GetHandle__(Handle_t handle,
										IdentityToken_t *ident, 
										QHandle **in_pHandle, 
										unsigned int *in_index,
										bool ignoreFree = false)
	{
		unsigned int serial = (handle >> HANDLESYS_HANDLE_BITS);
		unsigned int index = (handle & HANDLESYS_HANDLE_MASK);

		if (index == 0 || index > m_HandleTail || index > HANDLESYS_MAX_HANDLES)
		{
			return HandleError_Index;
		}

		QHandle *pHandle = &m_Handles[index];

		if (!pHandle->set
			|| (pHandle->set == HandleSet_Freed && !ignoreFree))
		{
			return HandleError_Freed;
		} else if (pHandle->set == HandleSet_Identity
			#if 0
				   && ident != GetIdentRoot()
			#endif
		)
		{
			/* Only IdentityHandle() can read this! */
			return HandleError_Identity;
		}
		if (pHandle->serial != serial)
		{
			return HandleError_Changed;
		}

		*in_pHandle = pHandle;
		*in_index = index;

		return HandleError_None;
	}

	bool CheckAccess__(QHandle *pHandle, HandleAccessRight right, const HandleSecurity *pSecurity)
	{
		QHandleType *pType = &m_Types[pHandle->type];
		unsigned int access;

		if (pHandle->access_special)
		{
			access = pHandle->sec.access[right];
		} else {
			access = pType->hndlSec.access[right];
		}

		/* Check if the type's identity matches */
	#if 0
		if (access & HANDLE_RESTRICT_IDENTITY)
		{
			IdentityToken_t *owner = pType->typeSec.ident;
			if (!owner
				|| (!pSecurity || pSecurity->pIdentity != owner))
			{
				return false;
			}
		}
	#endif

		/* Check if the owner is allowed */
		if (access & HANDLE_RESTRICT_OWNER)
		{
			IdentityToken_t *owner = pHandle->owner;
			if (owner
				&& (!pSecurity || pSecurity->pOwner != owner))
			{
				return false;
			}
		}

		return true;
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

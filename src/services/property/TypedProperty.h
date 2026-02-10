/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *
 *    Template-based typed property accessor that reads raw bytes from
 *    PropertyService's RawDataStorage and reinterprets them as C++ structs.
 *    Uses the existing Inheritor system for archetype/MetaProp chain resolution.
 *
 *    Usage:
 *      #include "TypedProperty.h"
 *      #include "DarkPropertyDefs.h"
 *
 *      PropPosition pos;
 *      if (getTypedProperty<PropPosition>(propSvc, "Position", objID, pos)) {
 *          printf("Object %d at (%.1f, %.1f, %.1f)\n", objID, pos.x, pos.y, pos.z);
 *      }
 *
 *    Property names are the label names (e.g. "Position", "ModelName", "Scale"),
 *    NOT the chunk names (e.g. "P$Position"). PropertyService stores them by label.
 *
 *****************************************************************************/

#ifndef __TYPEDPROPERTY_H
#define __TYPEDPROPERTY_H

#include "property/Property.h"
#include "property/PropertyService.h"
#include <cstring>
#include <vector>

namespace Darkness {

// ============================================================================
// Core typed property access
// ============================================================================

/** Get raw property bytes for an object, resolving inheritance.
 * @param propSvc The property service (must not be null)
 * @param propName Property label name (e.g. "Position", "ModelName")
 * @param objID The object ID to query
 * @param outSize Filled with the byte count of the returned data
 * @return Pointer to the raw bytes, or nullptr if property not found.
 *         Pointer is valid as long as the PropertyService data is not modified. */
inline const uint8_t *getPropertyRawData(PropertyService *propSvc,
                                         const std::string &propName,
                                         int objID, size_t &outSize) {
    Property *prop = propSvc->getProperty(propName);
    if (!prop) {
        outSize = 0;
        return nullptr;
    }

    // Resolve inheritance: walk archetype + MetaProp chain to find the
    // object that actually holds this property's data
    int effectiveID = prop->getEffectiveID(objID);
    if (effectiveID == 0) {
        outSize = 0;
        return nullptr;
    }

    DataStorage *storage = prop->getStorage();
    if (!storage) {
        outSize = 0;
        return nullptr;
    }

    return storage->getRawData(effectiveID, outSize);
}

/** Get a typed property for an object, resolving inheritance.
 * Copies sizeof(T) bytes from RawDataStorage into the output struct.
 * @param propSvc The property service
 * @param propName Property label name
 * @param objID The object ID to query
 * @param out The struct to fill with property data
 * @return true if the property was found and data size >= sizeof(T) */
template <typename T>
inline bool getTypedProperty(PropertyService *propSvc,
                             const std::string &propName, int objID, T &out) {
    size_t size = 0;
    const uint8_t *data = getPropertyRawData(propSvc, propName, objID, size);
    if (!data || size < sizeof(T))
        return false;

    std::memcpy(&out, data, sizeof(T));
    return true;
}

// ============================================================================
// Convenience wrappers
// ============================================================================

/** Check if an object has a property (with inheritance resolution).
 * Equivalent to PropertyService::has() but takes a raw pointer. */
inline bool hasProperty(PropertyService *propSvc, const std::string &propName,
                        int objID) {
    Property *prop = propSvc->getProperty(propName);
    if (!prop)
        return false;
    return prop->has(objID);
}

/** Check if an object directly owns a property (no inheritance).
 * Returns false if the object only inherits the property from an archetype. */
inline bool ownsProperty(PropertyService *propSvc,
                         const std::string &propName, int objID) {
    Property *prop = propSvc->getProperty(propName);
    if (!prop)
        return false;
    return prop->owns(objID);
}

/** Get all object IDs that directly store a given property.
 * Does NOT include objects that only inherit the property.
 * Useful for iterating all objects of a certain type. */
inline std::vector<int> getAllObjectsWithProperty(
    PropertyService *propSvc, const std::string &propName) {
    std::vector<int> result;

    Property *prop = propSvc->getProperty(propName);
    if (!prop)
        return result;

    DataStorage *storage = prop->getStorage();
    if (!storage)
        return result;

    IntIteratorPtr iter = storage->getAllStoredObjects();
    while (!iter->end()) {
        result.push_back(iter->next());
    }
    return result;
}

} // namespace Darkness

#endif // __TYPEDPROPERTY_H

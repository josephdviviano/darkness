/******************************************************************************
 *
 *    This file is part of openDarkEngine project
 *    Copyright (C) 2005-2006 openDarkEngine team
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *****************************************************************************/

#ifndef __RAWDATASTORAGE_H
#define __RAWDATASTORAGE_H

#include "config.h"

#include "DataStorage.h"
#include "File.h"
#include "Iterator.h"

#include <cstdint>
#include <map>
#include <vector>

namespace Darkness {

/** @brief Raw byte-blob DataStorage implementation.
 *
 * Stores raw byte vectors per object ID without typed field access.
 * For properties, reads the size prefix from the chunk (sizeStored=true).
 * For links, uses a fixed size passed at construction.
 */
class RawDataStorage : public DataStorage {
public:
    /** @param fixedSize Fixed data size in bytes for getDataSize().
     *  Use 0 for variable-size data (reads size from file). */
    explicit RawDataStorage(size_t fixedSize = 0)
        : mFixedSize(fixedSize), mFieldDesc() {}

    virtual ~RawDataStorage() {}

    bool create(int objID) override {
        auto res = mDataMap.insert(
            std::make_pair(objID, std::vector<uint8_t>()));
        return res.second;
    }

    bool createWithValue(int objID, const Variant &) override {
        return create(objID);
    }

    bool destroy(int objID) override {
        auto it = mDataMap.find(objID);
        if (it != mDataMap.end()) {
            mDataMap.erase(it);
            return true;
        }
        return false;
    }

    bool has(int objID) override {
        return mDataMap.find(objID) != mDataMap.end();
    }

    bool clone(int srcID, int dstID) override {
        auto it = mDataMap.find(srcID);
        if (it != mDataMap.end()) {
            auto res = mDataMap.insert(std::make_pair(dstID, it->second));
            return res.second;
        }
        return false;
    }

    bool getField(int, const std::string &, Variant &) override {
        return false;
    }

    bool setField(int, const std::string &, const Variant &) override {
        return false;
    }

    bool writeToFile(FilePtr &file, int objID, bool sizeStored) override {
        auto it = mDataMap.find(objID);
        if (it == mDataMap.end())
            return false;

        const auto &data = it->second;
        uint32_t size = static_cast<uint32_t>(data.size());

        if (sizeStored)
            file->writeElem(&size, sizeof(uint32_t));

        if (size > 0)
            file->write(data.data(), size);

        return true;
    }

    bool readFromFile(FilePtr &file, int objID, bool sizeStored) override {
        if (mDataMap.find(objID) != mDataMap.end()) {
            // Already exists - skip
            uint32_t size = static_cast<uint32_t>(mFixedSize);
            if (sizeStored)
                file->readElem(&size, sizeof(uint32_t));
            file->seek(size, File::FSEEK_CUR);
            return false;
        }

        uint32_t size = static_cast<uint32_t>(mFixedSize);
        if (sizeStored)
            file->readElem(&size, sizeof(uint32_t));

        std::vector<uint8_t> data(size);
        if (size > 0)
            file->read(data.data(), size);

        mDataMap.insert(std::make_pair(objID, std::move(data)));
        return true;
    }

    void clear() override { mDataMap.clear(); }

    bool isEmpty() override { return mDataMap.empty(); }

    const uint8_t *getRawData(int objID, size_t &outSize) const override {
        auto it = mDataMap.find(objID);
        if (it == mDataMap.end()) {
            outSize = 0;
            return nullptr;
        }
        outSize = it->second.size();
        return it->second.data();
    }

    IntIteratorPtr getAllStoredObjects() override {
        return IntIteratorPtr(new DataMapKeyIterator(mDataMap));
    }

    const DataFields &getFieldDesc(void) override { return mFieldDesc; }

    size_t getDataSize(void) override { return mFixedSize; }

private:
    size_t mFixedSize;
    DataFields mFieldDesc;

    typedef std::map<int, std::vector<uint8_t>> DataMap;
    DataMap mDataMap;

    typedef MapKeyIterator<DataMap, int> DataMapKeyIterator;
};

} // namespace Darkness

#endif // __RAWDATASTORAGE_H

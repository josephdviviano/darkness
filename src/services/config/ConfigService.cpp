/******************************************************************************
 *
 *    This file is part of openDarkEngine project
 *    Copyright (C) 2005-2009 openDarkEngine team
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
 *	  $Id$
 *
 *****************************************************************************/

#include "ConfigService.h"
#include "FileGroup.h"
#include "OpdeException.h"
#include "OpdeServiceManager.h"
#include "logger.h"
#include "platform/PlatformService.h"

#include <fstream>
#include <string>

using namespace std;

namespace Opde {

/*----------------------------------------------------*/
/*-------------------- ConfigService -----------------*/
/*----------------------------------------------------*/
template <> const size_t ServiceImpl<ConfigService>::SID = __SERVICE_ID_CONFIG;

ConfigService::ConfigService(ServiceManager *manager, const std::string &name)
    : ServiceImpl<ConfigService>(manager, name), mConfigPathOverride("") {}

//------------------------------------------------------
ConfigService::~ConfigService() {}

//------------------------------------------------------
bool ConfigService::init() {
    mPlatformService = GET_SERVICE(PlatformService);
    return true;
}

//------------------------------------------------------
void ConfigService::shutdown() { mPlatformService.reset(); }

//------------------------------------------------------
void ConfigService::setParamDescription(const std::string &param,
                                        const std::string &desc) {
    mConfigKeyDescriptions[param] = desc;
}

//------------------------------------------------------
void ConfigService::setParam(const std::string &param,
                             const std::string &value) {
    Parameters::iterator it = mParameters.find(param);

    if (it != mParameters.end()) {
        it->second = value;
    } else {
        mParameters.insert(make_pair(param, value));
    }
}

//------------------------------------------------------
Variant ConfigService::getParam(const std::string &param,
                                 const Variant &dflt) {
    Parameters::const_iterator it = mParameters.find(param);

    Parameters::const_iterator dit = mConfigKeyDescriptions.find(param);

    // warn if param has no desc specified
    if (dit == mConfigKeyDescriptions.end()) {
        LOG_ERROR("ConfigService: Warning: Config key '%s' has no description "
                  "specified",
                  param.c_str());
    }

    if (it != mParameters.end()) {
        return Variant(it->second);
    } else {
        return dflt;
    }
}

//------------------------------------------------------
bool ConfigService::getParam(const std::string &param, Variant &tgt) {
    Parameters::const_iterator it = mParameters.find(param);

    if (it != mParameters.end()) {
        tgt = it->second;
        return true;
    } else {
        return false;
    }
}

//------------------------------------------------------
bool ConfigService::hasParam(const std::string &param) {
    Parameters::const_iterator it = mParameters.find(param);

    if (it != mParameters.end()) {
        return true;
    } else {
        return false;
    }
}

//------------------------------------------------------
bool ConfigService::loadParams(const std::string &cfgfile) {
    // do we have an override set?
    if (mConfigPathOverride != "") {
        return loadFromFile(mConfigPathOverride +
                            mPlatformService->getDirectorySeparator() +
                            cfgfile);
    } else {
        // first the global config is loaded
        bool globalok =
            loadFromFile(mPlatformService->getGlobalConfigPath() +
                         mPlatformService->getDirectorySeparator() + cfgfile);

        // then overriden by local one
        bool userok =
            loadFromFile(mPlatformService->getUserConfigPath() +
                         mPlatformService->getDirectorySeparator() + cfgfile);

        return userok || globalok;
    }
}

//------------------------------------------------------
void ConfigService::setConfigPathOverride(const std::string &cfgpath) {
    mConfigPathOverride = cfgpath;
}

//------------------------------------------------------
ConfigService::GameType ConfigService::getGameType() {
    GameType gt = GAME_TYPE_INVALID;

    Variant val;

    if (getParam("game_type", val)) {
        if (val.toString() == "t1")
            gt = GAME_TYPE_T1;
        else if (val.toString() == "t2")
            gt = GAME_TYPE_T2;
        else if (val.toString() == "ss2")
            gt = GAME_TYPE_SS2;
    }

    return gt;
}

//------------------------------------------------------
std::string ConfigService::getLanguage() {
    Variant val = "english";

    // a trick - if not found, will use the previous
    // otherwise it will replace.
    getParam("language", val);

    return val.toString();
}

//------------------------------------------------------
std::string
ConfigService::getLocalisedResourcePath(const std::string &origPath) {
    std::string path, fname;

    // Split filename from path
    size_t pos = origPath.find_last_of("/\\");
    if (pos != std::string::npos) {
        path = origPath.substr(0, pos + 1);
        fname = origPath.substr(pos + 1);
    } else {
        path = "";
        fname = origPath;
    }

    return path + getLanguage() + mPlatformService->getDirectorySeparator() +
           fname;
}

//------------------------------------------------------
void ConfigService::logAllParameters() {
    Parameters::const_iterator dit = mConfigKeyDescriptions.begin();

    LOG_INFO("ConfigService: Configuration parameters:");

    while (dit != mConfigKeyDescriptions.end()) {
        LOG_INFO("ConfigService:\t%s - %s", dit->first.c_str(),
                 dit->second.c_str());
    }
}

//------------------------------------------------------
bool ConfigService::loadFromFile(const std::string &cfgfile) {
    LOG_INFO("ConfigService: Loading config file from '%s'",
             cfgfile.c_str());

    std::ifstream file(cfgfile);
    if (!file.is_open()) {
        LOG_ERROR("Config file '%s' was not found", cfgfile.c_str());
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#' || line[0] == ';')
            continue;

        // Skip section headers [section]
        if (line[0] == '[')
            continue;

        // Find the = separator
        size_t eqPos = line.find('=');
        if (eqPos == std::string::npos)
            continue;

        std::string key = line.substr(0, eqPos);
        std::string val = line.substr(eqPos + 1);

        // Trim whitespace from key and value
        auto trim = [](std::string &s) {
            size_t start = s.find_first_not_of(" \t\r\n");
            size_t end = s.find_last_not_of(" \t\r\n");
            if (start == std::string::npos) {
                s.clear();
            } else {
                s = s.substr(start, end - start + 1);
            }
        };

        trim(key);
        trim(val);

        if (!key.empty())
            setParam(key, val);
    }

    return true;
}

//-------------------------- Factory implementation
std::string ConfigServiceFactory::mName = "ConfigService";

ConfigServiceFactory::ConfigServiceFactory() : ServiceFactory(){};

const std::string &ConfigServiceFactory::getName() { return mName; }

const uint ConfigServiceFactory::getMask() { return SERVICE_CORE; }

const size_t ConfigServiceFactory::getSID() { return ConfigService::SID; }

Service *ConfigServiceFactory::createInstance(ServiceManager *manager) {
    return new ConfigService(manager, mName);
}

} // namespace Opde

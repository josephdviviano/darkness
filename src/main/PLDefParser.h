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

#ifndef __PLDEFPARSER_H
#define __PLDEFPARSER_H

#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace Opde {

struct PropertyDef {
    std::string name;
    std::string label;
    std::string dtype;
    std::string inheritor = "always";
    uint32_t verMaj = 2;
    uint32_t verMin = 4;
    bool isVarStr = false;
};

struct RelationDef {
    std::string name;
    bool hidden = false;
    bool noData = false;
    uint32_t lVerMaj = 2;
    uint32_t lVerMin = 0;
    uint32_t dVerMaj = 2;
    uint32_t dVerMin = 0;
    int fakeSize = -1;
};

struct PLDefResult {
    std::vector<PropertyDef> properties;
    std::vector<RelationDef> relations;
};

/** Minimal parser for .pldef files used by Thief 2.
 *
 * Handles:
 *   p_ver MAJ.MIN       — default property chunk version
 *   l_ver MAJ.MIN       — default link chunk version
 *   d_ver MAJ.MIN       — default link data chunk version
 *   rel_ver MAJ.MIN     — relation chunk version (ignored here)
 *   property NAME { ... }
 *   relation NAME { ... }
 *
 * Inside property blocks: label, p_ver, type varstr, inherit, dtype
 * Inside relation blocks: l_ver, d_ver, hidden, no_data, fake_size
 */
inline PLDefResult parsePLDef(const std::string &filename) {
    PLDefResult result;

    std::ifstream in(filename);
    if (!in.is_open())
        return result;

    // Global defaults
    uint32_t defPVerMaj = 2, defPVerMin = 4;
    uint32_t defLVerMaj = 2, defLVerMin = 0;
    uint32_t defDVerMaj = 2, defDVerMin = 0;

    auto parseVersion = [](const std::string &s, uint32_t &maj, uint32_t &min) {
        auto dot = s.find('.');
        if (dot != std::string::npos) {
            maj = static_cast<uint32_t>(std::stoul(s.substr(0, dot)));
            min = static_cast<uint32_t>(std::stoul(s.substr(dot + 1)));
        }
    };

    // Read all tokens, stripping comments
    std::vector<std::string> tokens;
    std::string line;
    while (std::getline(in, line)) {
        // Strip // comments
        auto cpos = line.find("//");
        if (cpos != std::string::npos)
            line.erase(cpos);

        std::istringstream ls(line);
        std::string tok;
        while (ls >> tok) {
            // Handle quoted strings: "some name"
            if (!tok.empty() && tok.front() == '"') {
                if (tok.back() == '"' && tok.size() > 1) {
                    // Complete quoted token — strip quotes
                    tokens.push_back(tok.substr(1, tok.size() - 2));
                } else {
                    // Read until closing quote
                    std::string quoted = tok.substr(1);
                    std::string next;
                    while (ls >> next) {
                        quoted += " " + next;
                        if (!next.empty() && next.back() == '"') {
                            quoted.pop_back();
                            break;
                        }
                    }
                    tokens.push_back(quoted);
                }
            } else {
                tokens.push_back(tok);
            }
        }
    }

    size_t i = 0;
    while (i < tokens.size()) {
        const auto &tok = tokens[i];

        if (tok == "p_ver" && i + 1 < tokens.size()) {
            parseVersion(tokens[i + 1], defPVerMaj, defPVerMin);
            i += 2;
        } else if (tok == "l_ver" && i + 1 < tokens.size()) {
            parseVersion(tokens[i + 1], defLVerMaj, defLVerMin);
            i += 2;
        } else if (tok == "d_ver" && i + 1 < tokens.size()) {
            parseVersion(tokens[i + 1], defDVerMaj, defDVerMin);
            i += 2;
        } else if (tok == "rel_ver" && i + 1 < tokens.size()) {
            // Relation chunk version — skip (handled by LinkService)
            i += 2;
        } else if (tok == "property" && i + 1 < tokens.size()) {
            PropertyDef prop;
            prop.name = tokens[i + 1];
            prop.verMaj = defPVerMaj;
            prop.verMin = defPVerMin;
            i += 2;

            // Expect '{'
            if (i < tokens.size() && tokens[i] == "{") {
                ++i;
                while (i < tokens.size() && tokens[i] != "}") {
                    if (tokens[i] == "label" && i + 1 < tokens.size()) {
                        prop.label = tokens[i + 1];
                        i += 2;
                    } else if (tokens[i] == "p_ver" && i + 1 < tokens.size()) {
                        parseVersion(tokens[i + 1], prop.verMaj, prop.verMin);
                        i += 2;
                    } else if (tokens[i] == "type" && i + 1 < tokens.size()) {
                        if (tokens[i + 1] == "varstr")
                            prop.isVarStr = true;
                        i += 2;
                    } else if (tokens[i] == "inherit" && i + 1 < tokens.size()) {
                        prop.inheritor = tokens[i + 1];
                        i += 2;
                    } else if (tokens[i] == "dtype" && i + 1 < tokens.size()) {
                        prop.dtype = tokens[i + 1];
                        i += 2;
                    } else if (tokens[i] == "cached") {
                        ++i; // skip
                    } else {
                        ++i; // skip unknown
                    }
                }
                if (i < tokens.size()) ++i; // skip '}'
            }

            result.properties.push_back(std::move(prop));
        } else if (tok == "relation" && i + 1 < tokens.size()) {
            RelationDef rel;
            rel.name = tokens[i + 1];
            rel.lVerMaj = defLVerMaj;
            rel.lVerMin = defLVerMin;
            rel.dVerMaj = defDVerMaj;
            rel.dVerMin = defDVerMin;
            i += 2;

            // Expect '{'
            if (i < tokens.size() && tokens[i] == "{") {
                ++i;
                while (i < tokens.size() && tokens[i] != "}") {
                    if (tokens[i] == "hidden") {
                        rel.hidden = true;
                        ++i;
                    } else if (tokens[i] == "no_data") {
                        rel.noData = true;
                        ++i;
                    } else if (tokens[i] == "l_ver" && i + 1 < tokens.size()) {
                        parseVersion(tokens[i + 1], rel.lVerMaj, rel.lVerMin);
                        i += 2;
                    } else if (tokens[i] == "d_ver" && i + 1 < tokens.size()) {
                        parseVersion(tokens[i + 1], rel.dVerMaj, rel.dVerMin);
                        i += 2;
                    } else if (tokens[i] == "fake_size" && i + 1 < tokens.size()) {
                        rel.fakeSize = std::stoi(tokens[i + 1]);
                        i += 2;
                    } else {
                        ++i; // skip unknown
                    }
                }
                if (i < tokens.size()) ++i; // skip '}'
            }

            result.relations.push_back(std::move(rel));
        } else {
            ++i;
        }
    }

    return result;
}

} // namespace Opde

#endif // __PLDEFPARSER_H

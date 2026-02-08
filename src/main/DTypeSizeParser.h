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

#ifndef __DTYPESIZEPARSER_H
#define __DTYPESIZEPARSER_H

#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace Opde {

/** Minimal .dtype parser that computes struct sizes.
 *
 * Parses the `namespace links` section and any global helper structs
 * to determine byte sizes of link data structures.
 *
 * Returns a map from struct name → size in bytes.
 */
inline std::map<std::string, int> parseDTypeSizes(const std::string &filename) {
    std::map<std::string, int> sizes;

    std::ifstream in(filename);
    if (!in.is_open())
        return sizes;

    // Tokenize the whole file
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
            // Handle quoted strings
            if (!tok.empty() && tok.front() == '"') {
                if (tok.back() == '"' && tok.size() > 1) {
                    tokens.push_back(tok.substr(1, tok.size() - 2));
                } else {
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

    // Type name → size in bytes
    auto typeSize = [](const std::string &t) -> int {
        if (t == "int32" || t == "uint32" || t == "float" || t == "bool32")
            return 4;
        if (t == "int16" || t == "uint16")
            return 2;
        if (t == "int8" || t == "uint8")
            return 1;
        if (t == "vector")
            return 12;
        if (t == "shortvec")
            return 6;
        return 0;
    };

    // Forward declaration: compute struct size from token position
    // We do two passes: first collect all struct boundaries, then compute sizes

    // Parse all struct definitions and alias declarations
    // We need to handle:
    //   struct NAME { fields... }
    //   struct NAME [N] { fields... }      (inline arrays inside structs)
    //   alias OTHER_STRUCT NAME
    //   alias OTHER_STRUCT NAME [N]
    //
    // Inside structs, field lines:
    //   TYPE [use ENUM] NAME [= "default"]
    //   TYPE [use ENUM] NAME [N]
    //   char [N] [use ENUM] NAME [M]
    //   alias OTHER_STRUCT NAME [N]
    //   struct NAME [N] { ... }     (nested)
    //   union NAME { ... }          (size = max of alternatives)

    // Recursive struct size computation from tokens
    // Returns the number of tokens consumed and the computed size
    struct Parser {
        const std::vector<std::string> &tokens;
        std::map<std::string, int> &sizes;
        std::function<int(const std::string &)> typeSize;

        // Parse a struct body (after the '{') and return (tokens consumed, size)
        std::pair<size_t, int> parseStructBody(size_t start) {
            size_t i = start;
            int totalSize = 0;

            while (i < tokens.size() && tokens[i] != "}") {
                if (tokens[i] == "struct") {
                    // Nested struct: struct NAME [N] { ... }
                    ++i;
                    if (i >= tokens.size()) break;
                    // skip name
                    ++i;
                    int count = 1;
                    // Check for [N]
                    if (i < tokens.size() && tokens[i].front() == '[') {
                        count = parseArrayCount(tokens[i]);
                        ++i;
                    }
                    // Expect '{'
                    if (i < tokens.size() && tokens[i] == "{") {
                        ++i;
                        auto [consumed, sz] = parseStructBody(i);
                        i += consumed;
                        totalSize += sz * count;
                        // skip '}'
                        if (i < tokens.size() && tokens[i] == "}")
                            ++i;
                    }
                } else if (tokens[i] == "union") {
                    // union NAME { alternatives... }
                    // Size = max of all alternatives
                    ++i;
                    if (i >= tokens.size()) break;
                    // skip name
                    ++i;
                    // Expect '{'
                    if (i < tokens.size() && tokens[i] == "{") {
                        ++i;
                        int maxSize = 0;
                        // Parse union body — contains struct/alias alternatives
                        while (i < tokens.size() && tokens[i] != "}") {
                            if (tokens[i] == "struct") {
                                ++i;
                                if (i >= tokens.size()) break;
                                ++i; // name
                                if (i < tokens.size() && tokens[i] == "{") {
                                    ++i;
                                    auto [consumed, sz] = parseStructBody(i);
                                    i += consumed;
                                    if (sz > maxSize) maxSize = sz;
                                    if (i < tokens.size() && tokens[i] == "}")
                                        ++i;
                                }
                            } else if (tokens[i] == "alias") {
                                ++i;
                                if (i + 1 >= tokens.size()) break;
                                std::string srcName = tokens[i]; ++i;
                                // skip dest name
                                ++i;
                                int aliasSize = lookupSize(srcName);
                                if (aliasSize > maxSize)
                                    maxSize = aliasSize;
                            } else {
                                ++i; // skip unknown
                            }
                        }
                        totalSize += maxSize;
                        if (i < tokens.size() && tokens[i] == "}")
                            ++i;
                    }
                } else if (tokens[i] == "alias") {
                    // alias OTHER NAME [N]
                    ++i;
                    if (i + 1 >= tokens.size()) break;
                    std::string srcName = tokens[i]; ++i;
                    // skip dest name
                    ++i;
                    int count = 1;
                    if (i < tokens.size() && tokens[i].front() == '[') {
                        count = parseArrayCount(tokens[i]);
                        ++i;
                    }
                    totalSize += lookupSize(srcName) * count;
                } else if (tokens[i] == "char") {
                    // char [N] [use ENUM] NAME [M]
                    ++i;
                    int charSize = 1;
                    if (i < tokens.size() && tokens[i].front() == '[') {
                        charSize = parseArrayCount(tokens[i]);
                        ++i;
                    }
                    // skip optional 'use ENUM'
                    if (i < tokens.size() && tokens[i] == "use") {
                        i += 2;
                    }
                    // skip field name
                    if (i < tokens.size()) ++i;
                    // Check for array [M]
                    int count = 1;
                    if (i < tokens.size() && tokens[i].front() == '[') {
                        count = parseArrayCount(tokens[i]);
                        ++i;
                    }
                    // skip optional = "default"
                    if (i < tokens.size() && tokens[i] == "=") {
                        i += 2;
                    }
                    totalSize += charSize * count;
                } else {
                    // Basic field: TYPE [use ENUM] NAME [N] [= "default"]
                    int sz = typeSize(tokens[i]);
                    if (sz > 0) {
                        ++i;
                        // skip optional 'use ENUM'
                        if (i < tokens.size() && tokens[i] == "use") {
                            i += 2;
                        }
                        // skip field name
                        if (i < tokens.size()) ++i;
                        // Check for array [N]
                        int count = 1;
                        if (i < tokens.size() && tokens[i].front() == '[') {
                            count = parseArrayCount(tokens[i]);
                            ++i;
                        }
                        // skip optional = "default"
                        if (i < tokens.size() && tokens[i] == "=") {
                            i += 2;
                        }
                        totalSize += sz * count;
                    } else {
                        // Skip unrecognized tokens (enum, bitfield, key, etc.)
                        ++i;
                    }
                }
            }

            return {i - start, totalSize};
        }

        int lookupSize(const std::string &name) {
            auto it = sizes.find(name);
            if (it != sizes.end())
                return it->second;
            return 0;
        }

        static int parseArrayCount(const std::string &tok) {
            // Parse "[N]" — extract N
            if (tok.size() >= 3 && tok.front() == '[' && tok.back() == ']') {
                return std::stoi(tok.substr(1, tok.size() - 2));
            }
            return 1;
        }
    };

    Parser parser{tokens, sizes, typeSize};

    // Two-pass approach: first pass collects all struct sizes at global scope
    // and inside namespace links. We skip enums, bitfields, etc.

    size_t i = 0;
    while (i < tokens.size()) {
        if (tokens[i] == "namespace") {
            ++i;
            if (i < tokens.size())
                ++i; // skip namespace name
            if (i < tokens.size() && tokens[i] == "{") {
                ++i; // enter namespace — structs inside are parsed the same way
            }
        } else if (tokens[i] == "struct") {
            ++i;
            if (i >= tokens.size()) break;
            std::string name = tokens[i];
            ++i;
            // Check for [N] on the struct itself (shouldn't happen at top level)
            if (i < tokens.size() && tokens[i] == "{") {
                ++i;
                auto [consumed, sz] = parser.parseStructBody(i);
                i += consumed;
                sizes[name] = sz;
                if (i < tokens.size() && tokens[i] == "}")
                    ++i;
            }
        } else if (tokens[i] == "alias") {
            // Top-level alias: alias SRC DST
            ++i;
            if (i + 1 >= tokens.size()) break;
            std::string srcName = tokens[i]; ++i;
            std::string dstName = tokens[i]; ++i;
            auto it = sizes.find(srcName);
            if (it != sizes.end())
                sizes[dstName] = it->second;
        } else if (tokens[i] == "enum" || tokens[i] == "bitfield") {
            // Skip enum/bitfield blocks
            ++i;
            // Skip to '{'
            while (i < tokens.size() && tokens[i] != "{") ++i;
            if (i < tokens.size()) {
                ++i;
                int depth = 1;
                while (i < tokens.size() && depth > 0) {
                    if (tokens[i] == "{") ++depth;
                    else if (tokens[i] == "}") --depth;
                    ++i;
                }
            }
        } else {
            ++i;
        }
    }

    return sizes;
}

} // namespace Opde

#endif // __DTYPESIZEPARSER_H

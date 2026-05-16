/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
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

// Hand-written recursive descent parser for Dark Engine .sch/.spc/.arc
// schema files. Replaces the original engine's YACC/Lex-based parser.
//
// The parser handles all three file types (they share the same grammar):
//   .spc — tag definitions, voice/concept hierarchies
//   .arc — archetype schema definitions (parameter defaults)
//   .sch — actual schema definitions with samples
//
// Loading order matters: .spc first (defines tags), .arc second (defines
// archetypes), .sch last (references both).

#pragma once

#include "SchemaTypes.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Darkness {

class SchemaParser {
public:
    SchemaParser() = default;

    /// Load all schema files from a directory (.spc first, .arc, then .sch)
    bool loadDirectory(const std::string &dirPath);

    /// Load and parse a single schema file
    bool loadFile(const std::string &filePath);

    /// Parse schema text directly (for testing)
    bool parseString(const std::string &text,
                     const std::string &filename = "<string>");

    // ── Query API ──

    /// Find a schema by name (case-insensitive)
    const SchemaEntry *findSchema(const std::string &name) const;

    /// Mutable schema lookup. Used by code that loads per-schema property
    /// overrides from the game database (P$SchAttFac, P$SchPlayPa, etc.)
    /// after the .sch files are parsed. Same case-insensitive lookup as
    /// findSchema. Returns null if the name is unknown.
    SchemaEntry *findSchemaMutable(const std::string &name);

    /// Find all schemas whose env_tags match the given query tags.
    /// A schema matches if ALL of its env_tags are satisfied by the query.
    std::vector<const SchemaEntry *>
    findByEnvTags(const std::vector<SchemaTagValue> &queryTags) const;

    /// Get a tag definition by name
    const TagDefinition *findTag(const std::string &name) const;

    // ── Stats ──

    size_t schemaCount() const { return mSchemas.size(); }
    size_t tagCount() const { return mTags.size(); }
    size_t voiceCount() const { return mVoices.size(); }
    size_t conceptCount() const { return mConcepts.size(); }
    size_t defineCount() const { return mDefines.size(); }

    const std::unordered_map<std::string, SchemaEntry> &schemas() const {
        return mSchemas;
    }

    const std::vector<std::string> &errors() const { return mErrors; }
    const std::vector<std::string> &warnings() const { return mWarnings; }

    // ── Serialization ──

    /// Serialize all parsed data back to .sch text format.
    /// Output is structurally equivalent to the input but may differ in
    /// whitespace, comment content, and ordering. Comments from the original
    /// source are preserved as "// [original comment]" where available.
    /// Suitable for round-trip testing: parse → serialize → reparse should
    /// produce identical data structures.
    std::string serialize() const;

    // ── Accessors for tags/voices/concepts/defines (for serialization & testing) ──

    const std::unordered_map<std::string, TagDefinition> &tags() const {
        return mTags;
    }
    const std::unordered_map<std::string, VoiceDefinition> &voices() const {
        return mVoices;
    }
    const std::unordered_map<std::string, ConceptDefinition> &concepts() const {
        return mConcepts;
    }
    const std::unordered_map<std::string, int> &defines() const {
        return mDefines;
    }
    const std::unordered_set<std::string> &requiredEnvTags() const {
        return mRequiredEnvTags;
    }

private:
    // ── Token types ──
    enum class TokenType {
        // Keywords
        Schema, Archetype, Volume, Delay, Pan, PanRange, Priority,
        Fade, Flags, AudioClass, MonoLoop, PolyLoop, LoopCount,
        Message, NoRepeat, NoCache, Stream, PlayOnce, NoCombat,
        NetAmbient, LocalSpatial, Freq,
        Voice, Concept, SchemaVoice, EnvTag,
        Tag, TagInt, EnvTagRequired,
        Include, Define,
        // Literals
        Identifier, Integer, String,
        // Punctuation
        LeftParen, RightParen,
        // Special
        EndOfFile
    };

    struct Token {
        TokenType type = TokenType::EndOfFile;
        std::string text;
        int intValue = 0;
        int line = 0;
    };

    // ── Tokenizer ──
    class Tokenizer {
    public:
        Tokenizer(const std::string &source, const std::string &filename,
                  const std::unordered_map<std::string, int> &defines,
                  std::vector<std::string> *errors = nullptr);
        Token next();
        Token peek();
        const std::string &filename() const { return mFilename; }
        int line() const { return mLine; }

    private:
        void skipWhitespaceAndComments();
        Token readToken();
        static TokenType lookupKeyword(const std::string &lower);

        const std::string &mSource;
        const std::string mFilename;
        const std::unordered_map<std::string, int> &mDefines;
        std::vector<std::string> *mParserErrors;  // borrowed from SchemaParser
        size_t mPos = 0;
        int mLine = 1;
        Token mPeeked;
        bool mHasPeeked = false;
    };

    // ── Parser methods ──
    void parseTopLevel(Tokenizer &tok, const std::string &baseDir);
    void parseSchemaEntry(Tokenizer &tok);
    void parseVoiceDef(Tokenizer &tok);
    void parseConceptDef(Tokenizer &tok);
    void parseSchemaVoiceDef(Tokenizer &tok);
    void parseEnvTagDef(Tokenizer &tok);
    void parseTagDef(Tokenizer &tok);
    void parseTagIntDef(Tokenizer &tok);
    void parseEnvTagRequiredDef(Tokenizer &tok);
    void parseInclude(Tokenizer &tok, const std::string &baseDir);
    void parseDefineDef(Tokenizer &tok);

    // Parse sub-components
    bool parseSchemaParams(Tokenizer &tok, SchemaEntry &entry);
    void parseSamples(Tokenizer &tok, SchemaEntry &entry);
    std::vector<SchemaTagValue> parseTagList(Tokenizer &tok);

    // Archetype inheritance
    void resolveArchetype(SchemaEntry &entry) const;

    // Utility
    static std::string toLower(const std::string &s);
    static bool isParamKeyword(TokenType t);
    static bool isTopLevelKeyword(TokenType t);
    static SchemaAudioClass parseAudioClassName(const std::string &name);
    void error(const Tokenizer &tok, const std::string &msg);
    void warn(const Tokenizer &tok, const std::string &msg);

    // ── Storage ──
    std::unordered_map<std::string, SchemaEntry> mSchemas;
    std::unordered_map<std::string, TagDefinition> mTags;
    std::unordered_map<std::string, VoiceDefinition> mVoices;
    std::unordered_map<std::string, ConceptDefinition> mConcepts;
    std::unordered_map<std::string, int> mDefines;
    std::unordered_set<std::string> mRequiredEnvTags;

    // Last schema defined (env_tag/schema_voice apply to it)
    std::string mLastSchemaName;

    // Included file tracking (prevent infinite recursion)
    std::unordered_set<std::string> mIncludedFiles;

    // Parse diagnostics
    std::vector<std::string> mErrors;
    std::vector<std::string> mWarnings;
};

} // namespace Darkness

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

#include "SchemaParser.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace Darkness {

// ════════════════════════════════════════════════════════════════════════════
// Tokenizer
// ════════════════════════════════════════════════════════════════════════════

SchemaParser::Tokenizer::Tokenizer(
    const std::string &source, const std::string &filename,
    const std::unordered_map<std::string, int> &defines,
    std::vector<std::string> *errors)
    : mSource(source), mFilename(filename), mDefines(defines),
      mParserErrors(errors) {}

SchemaParser::Token SchemaParser::Tokenizer::next()
{
    if (mHasPeeked) {
        mHasPeeked = false;
        return mPeeked;
    }
    return readToken();
}

SchemaParser::Token SchemaParser::Tokenizer::peek()
{
    if (!mHasPeeked) {
        mPeeked = readToken();
        mHasPeeked = true;
    }
    return mPeeked;
}

void SchemaParser::Tokenizer::skipWhitespaceAndComments()
{
    while (mPos < mSource.size()) {
        char c = mSource[mPos];

        // Whitespace
        if (c == ' ' || c == '\t' || c == '\r') {
            mPos++;
            continue;
        }
        if (c == '\n') {
            mPos++;
            mLine++;
            continue;
        }

        // Comment: // to end of line
        if (c == '/' && mPos + 1 < mSource.size() && mSource[mPos + 1] == '/') {
            mPos += 2;
            while (mPos < mSource.size() && mSource[mPos] != '\n')
                mPos++;
            continue;
        }

        break;
    }
}

SchemaParser::Token SchemaParser::Tokenizer::readToken()
{
    skipWhitespaceAndComments();

    if (mPos >= mSource.size())
        return {TokenType::EndOfFile, "", 0, mLine};

    char c = mSource[mPos];
    int tokenLine = mLine;

    // Parentheses
    if (c == '(') { mPos++; return {TokenType::LeftParen, "(", 0, tokenLine}; }
    if (c == ')') { mPos++; return {TokenType::RightParen, ")", 0, tokenLine}; }

    // Preprocessor directives: #include, #define
    if (c == '#') {
        mPos++;
        // Read directive name
        std::string directive;
        while (mPos < mSource.size() && std::isalpha(mSource[mPos]))
            directive += mSource[mPos++];

        std::string lower = directive;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

        if (lower == "include")
            return {TokenType::Include, directive, 0, tokenLine};
        if (lower == "define")
            return {TokenType::Define, directive, 0, tokenLine};

        // Unknown preprocessor directive — skip rest of line
        while (mPos < mSource.size() && mSource[mPos] != '\n')
            mPos++;
        return readToken();
    }

    // Quoted string
    if (c == '"') {
        mPos++;
        std::string text;
        while (mPos < mSource.size() && mSource[mPos] != '"') {
            if (mSource[mPos] == '\n') mLine++;
            text += mSource[mPos++];
        }
        if (mPos < mSource.size()) {
            mPos++; // skip closing quote
        } else if (mParserErrors) {
            // Unterminated string — reached EOF without closing quote
            std::ostringstream ss;
            ss << mFilename << ":" << tokenLine << ": error: unterminated string literal";
            mParserErrors->push_back(ss.str());
        }
        return {TokenType::String, text, 0, tokenLine};
    }

    // Integer (possibly negative)
    if (std::isdigit(c) || (c == '-' && mPos + 1 < mSource.size() &&
                            std::isdigit(mSource[mPos + 1]))) {
        std::string num;
        num += mSource[mPos++];
        while (mPos < mSource.size() && std::isdigit(mSource[mPos]))
            num += mSource[mPos++];
        return {TokenType::Integer, num, std::stoi(num), tokenLine};
    }

    // Identifier or keyword
    if (std::isalpha(c) || c == '_') {
        std::string ident;
        while (mPos < mSource.size() &&
               (std::isalnum(mSource[mPos]) || mSource[mPos] == '_'))
            ident += mSource[mPos++];

        // Check #define substitution first
        auto defIt = mDefines.find(ident);
        if (defIt != mDefines.end()) {
            return {TokenType::Integer, ident, defIt->second, tokenLine};
        }

        // Check for keyword
        std::string lower = ident;
        std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
        TokenType kw = lookupKeyword(lower);
        if (kw != TokenType::Identifier)
            return {kw, ident, 0, tokenLine};

        return {TokenType::Identifier, ident, 0, tokenLine};
    }

    // Unknown character — skip it
    mPos++;
    return readToken();
}

SchemaParser::TokenType
SchemaParser::Tokenizer::lookupKeyword(const std::string &lower)
{
    // Static keyword table (all lowercase)
    static const std::unordered_map<std::string, TokenType> keywords = {
        {"schema",           TokenType::Schema},
        {"archetype",        TokenType::Archetype},
        {"volume",           TokenType::Volume},
        {"delay",            TokenType::Delay},
        {"pan",              TokenType::Pan},
        {"pan_range",        TokenType::PanRange},
        {"priority",         TokenType::Priority},
        {"fade",             TokenType::Fade},
        {"flags",            TokenType::Flags},
        {"audio_class",      TokenType::AudioClass},
        {"mono_loop",        TokenType::MonoLoop},
        {"poly_loop",        TokenType::PolyLoop},
        {"loop_count",       TokenType::LoopCount},
        {"message",          TokenType::Message},
        {"no_repeat",        TokenType::NoRepeat},
        {"no_cache",         TokenType::NoCache},
        {"stream",           TokenType::Stream},
        {"play_once",        TokenType::PlayOnce},
        {"no_combat",        TokenType::NoCombat},
        {"net_ambient",      TokenType::NetAmbient},
        {"local_spatial",    TokenType::LocalSpatial},
        {"freq",             TokenType::Freq},
        {"voice",            TokenType::Voice},
        {"concept",          TokenType::Concept},
        {"schema_voice",     TokenType::SchemaVoice},
        {"env_tag",          TokenType::EnvTag},
        {"tag",              TokenType::Tag},
        {"tag_int",          TokenType::TagInt},
        {"env_tag_required", TokenType::EnvTagRequired},
    };

    auto it = keywords.find(lower);
    return (it != keywords.end()) ? it->second : TokenType::Identifier;
}

// ════════════════════════════════════════════════════════════════════════════
// Parser — top-level
// ════════════════════════════════════════════════════════════════════════════

bool SchemaParser::parseString(const std::string &text,
                               const std::string &filename)
{
    Tokenizer tok(text, filename, mDefines, &mErrors);
    parseTopLevel(tok, ".");
    return mErrors.empty();
}

bool SchemaParser::loadFile(const std::string &filePath)
{
    // Prevent re-including the same file
    std::string canonical = fs::weakly_canonical(filePath).string();
    if (mIncludedFiles.count(canonical))
        return true;
    mIncludedFiles.insert(canonical);

    std::ifstream ifs(filePath);
    if (!ifs.is_open()) {
        mErrors.push_back("Failed to open: " + filePath);
        return false;
    }

    std::ostringstream ss;
    ss << ifs.rdbuf();
    std::string contents = ss.str();

    std::string baseDir = fs::path(filePath).parent_path().string();
    if (baseDir.empty()) baseDir = ".";

    Tokenizer tok(contents, filePath, mDefines, &mErrors);
    parseTopLevel(tok, baseDir);
    return mErrors.empty();
}

bool SchemaParser::loadDirectory(const std::string &dirPath)
{
    if (!fs::is_directory(dirPath)) {
        mErrors.push_back("Not a directory: " + dirPath);
        return false;
    }

    // Collect files by extension, then load in order: .spc, .arc, .sch
    std::vector<std::string> spcFiles, arcFiles, schFiles;

    for (const auto &entry : fs::directory_iterator(dirPath)) {
        if (!entry.is_regular_file()) continue;
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == ".spc") spcFiles.push_back(entry.path().string());
        else if (ext == ".arc") arcFiles.push_back(entry.path().string());
        else if (ext == ".sch") schFiles.push_back(entry.path().string());
    }

    // Sort each group for deterministic load order
    std::sort(spcFiles.begin(), spcFiles.end());
    std::sort(arcFiles.begin(), arcFiles.end());
    std::sort(schFiles.begin(), schFiles.end());

    for (const auto &f : spcFiles) loadFile(f);
    for (const auto &f : arcFiles) loadFile(f);
    for (const auto &f : schFiles) loadFile(f);

    std::fprintf(stderr, "SchemaParser: loaded %zu schemas, %zu tags, "
                 "%zu voices, %zu concepts from %s\n",
                 mSchemas.size(), mTags.size(), mVoices.size(),
                 mConcepts.size(), dirPath.c_str());

    return mErrors.empty();
}

void SchemaParser::parseTopLevel(Tokenizer &tok, const std::string &baseDir)
{
    while (tok.peek().type != TokenType::EndOfFile) {
        switch (tok.peek().type) {
        case TokenType::Schema:       parseSchemaEntry(tok);          break;
        case TokenType::Voice:        parseVoiceDef(tok);             break;
        case TokenType::Concept:      parseConceptDef(tok);           break;
        case TokenType::SchemaVoice:  parseSchemaVoiceDef(tok);       break;
        case TokenType::EnvTag:       parseEnvTagDef(tok);            break;
        case TokenType::Tag:          parseTagDef(tok);               break;
        case TokenType::TagInt:       parseTagIntDef(tok);            break;
        case TokenType::EnvTagRequired: parseEnvTagRequiredDef(tok);  break;
        case TokenType::Include:      parseInclude(tok, baseDir);     break;
        case TokenType::Define:       parseDefineDef(tok);            break;
        default:
            // Skip unknown tokens (forward compatibility)
            warn(tok, "unexpected token: " + tok.peek().text);
            tok.next();
            break;
        }
    }
}

// ════════════════════════════════════════════════════════════════════════════
// Parser — schema definitions
// ════════════════════════════════════════════════════════════════════════════

void SchemaParser::parseSchemaEntry(Tokenizer &tok)
{
    tok.next(); // consume 'schema'

    Token nameToken = tok.next();
    if (nameToken.type != TokenType::Identifier &&
        nameToken.type != TokenType::Integer) {
        error(tok, "expected schema name after 'schema'");
        return;
    }

    SchemaEntry entry;
    entry.name = nameToken.text;

    // Parse parameters (known keywords)
    while (isParamKeyword(tok.peek().type)) {
        parseSchemaParams(tok, entry);
    }

    // Parse samples (identifiers that aren't top-level or parameter keywords)
    while (tok.peek().type == TokenType::Identifier ||
           tok.peek().type == TokenType::Freq) {
        parseSamples(tok, entry);
    }

    // Resolve archetype inheritance
    if (!entry.archetypeName.empty()) {
        resolveArchetype(entry);
    }

    // Store (case-insensitive key)
    std::string key = toLower(entry.name);
    mSchemas[key] = std::move(entry);
    mLastSchemaName = key;
}

bool SchemaParser::isParamKeyword(TokenType t)
{
    switch (t) {
    case TokenType::Archetype: case TokenType::Volume: case TokenType::Delay:
    case TokenType::Pan: case TokenType::PanRange: case TokenType::Priority:
    case TokenType::Fade: case TokenType::Flags: case TokenType::AudioClass:
    case TokenType::MonoLoop: case TokenType::PolyLoop:
    case TokenType::LoopCount: case TokenType::Message:
    case TokenType::NoRepeat: case TokenType::NoCache: case TokenType::Stream:
    case TokenType::PlayOnce: case TokenType::NoCombat:
    case TokenType::NetAmbient: case TokenType::LocalSpatial:
        return true;
    default:
        return false;
    }
}

bool SchemaParser::isTopLevelKeyword(TokenType t)
{
    switch (t) {
    case TokenType::Schema: case TokenType::Voice: case TokenType::Concept:
    case TokenType::SchemaVoice: case TokenType::EnvTag:
    case TokenType::Tag: case TokenType::TagInt:
    case TokenType::EnvTagRequired:
    case TokenType::Include: case TokenType::Define:
        return true;
    default:
        return false;
    }
}

bool SchemaParser::parseSchemaParams(Tokenizer &tok, SchemaEntry &entry)
{
    Token t = tok.next();
    switch (t.type) {
    case TokenType::Archetype: {
        Token name = tok.next();
        entry.archetypeName = name.text;
        break;
    }
    case TokenType::Volume: {
        Token val = tok.next();
        entry.playParams.volume = val.intValue;
        entry.playParams.fieldsSet |= SCH_SET_VOLUME;
        break;
    }
    case TokenType::Delay: {
        Token val = tok.next();
        entry.playParams.initialDelay = val.intValue;
        entry.playParams.fieldsSet |= SCH_SET_DELAY;
        break;
    }
    case TokenType::Pan: {
        Token val = tok.next();
        entry.playParams.pan = val.intValue;
        entry.playParams.flags |= SCH_PAN_POS;
        entry.playParams.fieldsSet |= SCH_SET_PAN;
        break;
    }
    case TokenType::PanRange: {
        Token val = tok.next();
        entry.playParams.pan = val.intValue;
        entry.playParams.flags |= SCH_PAN_RANGE;
        entry.playParams.fieldsSet |= SCH_SET_PAN;
        break;
    }
    case TokenType::Priority: {
        Token val = tok.next();
        entry.playParams.priority = val.intValue;
        entry.playParams.fieldsSet |= SCH_SET_PRIORITY;
        break;
    }
    case TokenType::Fade: {
        Token val = tok.next();
        entry.playParams.fade = val.intValue;
        entry.playParams.fieldsSet |= SCH_SET_FADE;
        break;
    }
    case TokenType::Flags: {
        Token val = tok.next();
        entry.playParams.flags = static_cast<uint32_t>(val.intValue);
        break;
    }
    case TokenType::AudioClass: {
        Token name = tok.next();
        entry.playParams.audioClass = parseAudioClassName(name.text);
        entry.playParams.fieldsSet |= SCH_SET_AUDIO_CLASS;
        break;
    }
    case TokenType::MonoLoop: {
        Token minVal = tok.next();
        Token maxVal = tok.next();
        entry.loopParams.isLooping = true;
        entry.loopParams.isPoly = false;
        entry.loopParams.maxSamples = 1;
        entry.loopParams.intervalMin = static_cast<uint16_t>(minVal.intValue);
        entry.loopParams.intervalMax = static_cast<uint16_t>(maxVal.intValue);
        break;
    }
    case TokenType::PolyLoop: {
        Token maxSamp = tok.next();
        Token minVal = tok.next();
        Token maxVal = tok.next();
        entry.loopParams.isLooping = true;
        entry.loopParams.isPoly = true;
        entry.loopParams.maxSamples = static_cast<uint8_t>(maxSamp.intValue);
        entry.loopParams.intervalMin = static_cast<uint16_t>(minVal.intValue);
        entry.loopParams.intervalMax = static_cast<uint16_t>(maxVal.intValue);
        break;
    }
    case TokenType::LoopCount: {
        Token val = tok.next();
        entry.loopParams.count = static_cast<uint16_t>(val.intValue);
        break;
    }
    case TokenType::Message: {
        Token name = tok.next();
        entry.message = name.text;
        break;
    }
    case TokenType::NoRepeat:    entry.playParams.flags |= SCH_NO_REPEAT;  break;
    case TokenType::NoCache:     entry.playParams.flags |= SCH_NO_CACHE;   break;
    case TokenType::Stream:      entry.playParams.flags |= SCH_STREAM;     break;
    case TokenType::PlayOnce:    entry.playParams.flags |= SCH_PLAY_ONCE;  break;
    case TokenType::NoCombat:    entry.playParams.flags |= SCH_NO_COMBAT;  break;
    case TokenType::NetAmbient:  entry.playParams.flags |= SCH_NET_AMBIENT; break;
    case TokenType::LocalSpatial: entry.playParams.flags |= SCH_LOC_SPATIAL; break;
    default:
        return false;
    }
    return true;
}

void SchemaParser::parseSamples(Tokenizer &tok, SchemaEntry &entry)
{
    // A sample line is: IDENTIFIER ["text"] [freq INT]
    // "freq" can also appear before the sample name as a modifier
    if (tok.peek().type == TokenType::Freq) {
        // freq keyword appearing standalone — skip it, it will be read
        // as part of the next sample's freq modifier
    }

    if (tok.peek().type != TokenType::Identifier)
        return;

    SchemaSample sample;
    Token nameToken = tok.next();
    sample.name = nameToken.text;

    // Optional quoted text
    if (tok.peek().type == TokenType::String) {
        sample.text = tok.next().text;
    }

    // Optional freq modifier
    if (tok.peek().type == TokenType::Freq) {
        tok.next(); // consume 'freq'
        Token val = tok.next();
        sample.frequency = static_cast<uint8_t>(val.intValue);
    }

    if (entry.samples.size() < 20) { // SCHEMA_SAMPLES_MAX
        entry.samples.push_back(std::move(sample));
    }
}

// ════════════════════════════════════════════════════════════════════════════
// Parser — tag/voice/concept definitions
// ════════════════════════════════════════════════════════════════════════════

void SchemaParser::parseVoiceDef(Tokenizer &tok)
{
    tok.next(); // consume 'voice'
    Token nameToken = tok.next();

    VoiceDefinition voice;
    voice.name = nameToken.text;

    // Optional archetype
    if (tok.peek().type == TokenType::Archetype) {
        tok.next(); // consume 'archetype'
        voice.archetypeName = tok.next().text;
    }

    mVoices[toLower(voice.name)] = std::move(voice);
}

void SchemaParser::parseConceptDef(Tokenizer &tok)
{
    tok.next(); // consume 'concept'
    Token nameToken = tok.next();
    Token priorityToken = tok.next();

    ConceptDefinition concept;
    concept.name = nameToken.text;
    concept.priority = priorityToken.intValue;

    mConcepts[toLower(concept.name)] = std::move(concept);
}

void SchemaParser::parseSchemaVoiceDef(Tokenizer &tok)
{
    tok.next(); // consume 'schema_voice'

    Token voiceToken = tok.next();   // voice name
    Token weightToken = tok.next();  // weight
    Token conceptToken = tok.next(); // concept name

    // Parse tag list
    auto tags = parseTagList(tok);

    // Apply to last schema
    if (!mLastSchemaName.empty()) {
        auto it = mSchemas.find(mLastSchemaName);
        if (it != mSchemas.end()) {
            it->second.voiceName = voiceToken.text;
            it->second.voiceWeight = weightToken.intValue;
            it->second.conceptName = conceptToken.text;
            it->second.voiceTags = std::move(tags);
        }
    }
}

void SchemaParser::parseEnvTagDef(Tokenizer &tok)
{
    tok.next(); // consume 'env_tag'

    auto tags = parseTagList(tok);

    // Apply to last schema
    if (!mLastSchemaName.empty()) {
        auto it = mSchemas.find(mLastSchemaName);
        if (it != mSchemas.end()) {
            it->second.envTags = std::move(tags);
        }
    } else {
        warn(tok, "env_tag without preceding schema definition");
    }
}

void SchemaParser::parseTagDef(Tokenizer &tok)
{
    tok.next(); // consume 'tag'
    Token nameToken = tok.next();

    TagDefinition tag;
    tag.name = nameToken.text;
    tag.isIntTag = false;

    // Read enum values until next keyword or EOF
    while (tok.peek().type == TokenType::Identifier) {
        tag.values.push_back(tok.next().text);
    }

    mTags[toLower(tag.name)] = std::move(tag);
}

void SchemaParser::parseTagIntDef(Tokenizer &tok)
{
    tok.next(); // consume 'tag_int'
    Token nameToken = tok.next();

    TagDefinition tag;
    tag.name = nameToken.text;
    tag.isIntTag = true;

    mTags[toLower(tag.name)] = std::move(tag);
}

void SchemaParser::parseEnvTagRequiredDef(Tokenizer &tok)
{
    tok.next(); // consume 'env_tag_required'
    Token nameToken = tok.next();

    std::string key = toLower(nameToken.text);
    mRequiredEnvTags.insert(key);

    // Also mark in the tag definition if it exists
    auto it = mTags.find(key);
    if (it != mTags.end()) {
        it->second.isRequired = true;
    }
}

void SchemaParser::parseInclude(Tokenizer &tok, const std::string &baseDir)
{
    tok.next(); // consume 'include'

    // Expect a quoted string filename
    Token fileToken = tok.next();
    if (fileToken.type != TokenType::String &&
        fileToken.type != TokenType::Identifier) {
        error(tok, "expected filename after #include");
        return;
    }

    // Resolve relative to the including file's directory
    std::string includePath = baseDir + "/" + fileToken.text;
    loadFile(includePath);
}

void SchemaParser::parseDefineDef(Tokenizer &tok)
{
    tok.next(); // consume 'define'

    Token nameToken = tok.next();
    Token valueToken = tok.next();

    if (nameToken.type == TokenType::Identifier &&
        valueToken.type == TokenType::Integer) {
        mDefines[nameToken.text] = valueToken.intValue;
    }
}

// ════════════════════════════════════════════════════════════════════════════
// Tag list parsing — (TagName val1 val2) (TagName min max) ...
// ════════════════════════════════════════════════════════════════════════════

std::vector<SchemaTagValue> SchemaParser::parseTagList(Tokenizer &tok)
{
    std::vector<SchemaTagValue> tags;

    while (tok.peek().type == TokenType::LeftParen) {
        tok.next(); // consume '('

        SchemaTagValue tag;

        // First token in parens is the tag name
        Token nameToken = tok.next();
        tag.tagName = nameToken.text;

        // Read values until ')'
        std::vector<Token> values;
        while (tok.peek().type != TokenType::RightParen &&
               tok.peek().type != TokenType::EndOfFile) {
            values.push_back(tok.next());
        }

        if (tok.peek().type == TokenType::RightParen)
            tok.next(); // consume ')'

        // Classify: all integers with exactly 2 = int range, otherwise enum
        if (values.size() == 2 &&
            values[0].type == TokenType::Integer &&
            values[1].type == TokenType::Integer) {
            tag.isIntRange = true;
            tag.rangeMin = values[0].intValue;
            tag.rangeMax = values[1].intValue;
        } else {
            tag.isIntRange = false;
            for (const auto &v : values) {
                tag.enumValues.push_back(v.text);
            }
        }

        tags.push_back(std::move(tag));
    }

    return tags;
}

// ════════════════════════════════════════════════════════════════════════════
// Archetype inheritance
// ════════════════════════════════════════════════════════════════════════════

void SchemaParser::resolveArchetype(SchemaEntry &entry) const
{
    // Walk the archetype chain (supports multi-level: A → B → C).
    // Uses a visited set to detect and break circular references.
    std::unordered_set<std::string> visited;
    std::string parentKey = toLower(entry.archetypeName);

    while (!parentKey.empty() && visited.find(parentKey) == visited.end()) {
        visited.insert(parentKey);

        auto it = mSchemas.find(parentKey);
        if (it == mSchemas.end())
            break;

        const SchemaEntry &parent = it->second;

        // Inherit play params only where the child hasn't explicitly set them.
        // Uses fieldsSet bitmask so "set to default" is distinguishable from "not set".
        if (!(entry.playParams.fieldsSet & SCH_SET_VOLUME) &&
            (parent.playParams.fieldsSet & SCH_SET_VOLUME))
            entry.playParams.volume = parent.playParams.volume;
        if (!(entry.playParams.fieldsSet & SCH_SET_DELAY) &&
            (parent.playParams.fieldsSet & SCH_SET_DELAY))
            entry.playParams.initialDelay = parent.playParams.initialDelay;
        if (!(entry.playParams.fieldsSet & SCH_SET_PAN) &&
            (parent.playParams.fieldsSet & SCH_SET_PAN))
            entry.playParams.pan = parent.playParams.pan;
        if (!(entry.playParams.fieldsSet & SCH_SET_PRIORITY) &&
            (parent.playParams.fieldsSet & SCH_SET_PRIORITY))
            entry.playParams.priority = parent.playParams.priority;
        if (!(entry.playParams.fieldsSet & SCH_SET_FADE) &&
            (parent.playParams.fieldsSet & SCH_SET_FADE))
            entry.playParams.fade = parent.playParams.fade;
        if (!(entry.playParams.fieldsSet & SCH_SET_AUDIO_CLASS) &&
            (parent.playParams.fieldsSet & SCH_SET_AUDIO_CLASS))
            entry.playParams.audioClass = parent.playParams.audioClass;

        // Propagate inherited fieldsSet so grandchild sees what was inherited
        entry.playParams.fieldsSet |= parent.playParams.fieldsSet;

        // Inherit flags (OR with parent, preserving child's explicit flags)
        entry.playParams.flags |= parent.playParams.flags;

        // Inherit loop params if child has no loop
        if (!entry.loopParams.isLooping && parent.loopParams.isLooping)
            entry.loopParams = parent.loopParams;

        // Inherit message if child has none
        if (entry.message.empty() && !parent.message.empty())
            entry.message = parent.message;

        // Continue up the chain
        parentKey = toLower(parent.archetypeName);
    }
}

// ════════════════════════════════════════════════════════════════════════════
// Query API
// ════════════════════════════════════════════════════════════════════════════

const SchemaEntry *SchemaParser::findSchema(const std::string &name) const
{
    auto it = mSchemas.find(toLower(name));
    return (it != mSchemas.end()) ? &it->second : nullptr;
}

const TagDefinition *SchemaParser::findTag(const std::string &name) const
{
    auto it = mTags.find(toLower(name));
    return (it != mTags.end()) ? &it->second : nullptr;
}

std::vector<const SchemaEntry *>
SchemaParser::findByEnvTags(const std::vector<SchemaTagValue> &queryTags) const
{
    std::vector<const SchemaEntry *> matches;

    for (const auto &[key, schema] : mSchemas) {
        if (!schema.hasEnvTags()) continue;

        // Check if ALL of the schema's env_tags match the query
        bool allMatch = true;
        for (const auto &reqTag : schema.envTags) {
            bool tagFound = false;
            for (const auto &qTag : queryTags) {
                if (toLower(reqTag.tagName) != toLower(qTag.tagName))
                    continue;

                // Tag name matches — check values
                if (reqTag.enumValues.empty() && !reqTag.isIntRange) {
                    // Wildcard match (no values specified)
                    tagFound = true;
                } else if (reqTag.isIntRange) {
                    // Integer range: query must be in range
                    if (qTag.isIntRange) {
                        tagFound = (qTag.rangeMin >= reqTag.rangeMin &&
                                    qTag.rangeMax <= reqTag.rangeMax);
                    }
                } else {
                    // Enum match: any query value must match any schema value
                    for (const auto &qVal : qTag.enumValues) {
                        for (const auto &rVal : reqTag.enumValues) {
                            if (toLower(qVal) == toLower(rVal)) {
                                tagFound = true;
                                break;
                            }
                        }
                        if (tagFound) break;
                    }
                }
                if (tagFound) break;
            }
            if (!tagFound) {
                allMatch = false;
                break;
            }
        }

        if (allMatch) {
            matches.push_back(&schema);
        }
    }

    return matches;
}

// ════════════════════════════════════════════════════════════════════════════
// Utility
// ════════════════════════════════════════════════════════════════════════════

std::string SchemaParser::toLower(const std::string &s)
{
    std::string result = s;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

SchemaAudioClass SchemaParser::parseAudioClassName(const std::string &name)
{
    std::string lower = toLower(name);

    // Accept string names
    if (lower == "noise")       return SchemaAudioClass::Noise;
    if (lower == "speech")      return SchemaAudioClass::Speech;
    if (lower == "ambient")     return SchemaAudioClass::Ambient;
    if (lower == "music")       return SchemaAudioClass::Music;
    if (lower == "metaui")      return SchemaAudioClass::MetaUI;
    if (lower == "player_feet") return SchemaAudioClass::PlayerFeet;
    if (lower == "other_feet")  return SchemaAudioClass::OtherFeet;
    if (lower == "collisions")  return SchemaAudioClass::Collisions;
    if (lower == "weapons")     return SchemaAudioClass::Weapons;
    if (lower == "monsters")    return SchemaAudioClass::Monsters;

    // Accept numeric values (0-9)
    if (lower.size() == 1 && lower[0] >= '0' && lower[0] <= '9')
        return static_cast<SchemaAudioClass>(lower[0] - '0');

    return SchemaAudioClass::Noise;
}

void SchemaParser::error(const Tokenizer &tok, const std::string &msg)
{
    std::ostringstream ss;
    ss << tok.filename() << ":" << tok.line() << ": error: " << msg;
    mErrors.push_back(ss.str());
}

void SchemaParser::warn(const Tokenizer &tok, const std::string &msg)
{
    std::ostringstream ss;
    ss << tok.filename() << ":" << tok.line() << ": warning: " << msg;
    mWarnings.push_back(ss.str());
}

} // namespace Darkness

-------------------------------------------------------------------

Changes in alpha release 0.0.2 (15-Dec-2005)

* Initial version

Changes in alpha release 0.0.3 (29-Dec-2005)

* Added OSM file manipulation base - DLL loader for linux, and some COM interfaces and implementation (IMalloc, IUnknown), plus some basic interfaces used by DLL loading
* Added console (without input now)
* Added ability to load D2 missions (tested on SS2, shall load Thief2 missions too - untested)
* Converted to DarkLib from class misfile (this will be used for loader in whole)
* Fixes

Changes in alpha release 0.1.0 (04-Jun-2006)
* Rewritten the whole thing to use Ogre3D
* CVS reinitialized

Changes in alpha release 0.1.1 (18-Jul-2006)
* added meshconvert application used to prepare the binary meshes to XML format used by Ogre3D importer
* added a trivial support for Widgets, and a not-yet-working console
* Fixes

Changes in alpha release 0.1.2 (29-Jul-2006)
* Rewritten the core renderer visibility testing routine - now more effective stack based approach
* fixed a bug with water portals
* Applied a telliamed's patch

Changes in alpha release 0.1.3 (11-Aug-2006)
* Darklib was replaced with DarkDatabase class
* Fixed to work on AMD64 platform

Changes in alpha release 0.1.4 (20-Aug-2006)
* Added the ability to do lightmap switching
* Added a console command "light ID Intensity". Note that the ID differes from the object ID.

Changes in alpha release 0.1.5 (02-Sep-2006)
* The water portals are really fixed now. The rendering of water works, but needs to be finished.
* Updated the documentation of the sources. It should be pretty complete now.
* The material scripts can be used (and are searched) prior to the simple texture-only default materials. Not too complete for now.
* Water material script added. If not found, water should be invisible.
* Added -Werror compiler flag.

Changes in alpha release 0.1.6 (17-Sep-2006)
* SceneManager is nearly cleaned up (needs finishing though)
* Level geometry is loaded outside the SceneManager class, in a new (and first) Service called worldrep
* Water rendering finally works, the material should be tweaked to look better
* SkyHack seems to work well now
* The levels are currently non-lit (No lightmaps), as the lightmapping needs final fix.

Changes in alpha release 0.1.7 (2-Oct-2006)
* Water portals rendering works even with the lightmaps. The materials are bad though - not looking as they should.
* Added/restructuralised the materials
* Added DarknessService, DarknessServiceFactory and some more. These will hopefully be a base classes for the services.
* Lightmaps are partially fixed. Swiched on by default now.
* Pre-add of the StructInfo sources (Thanks telliamed!). These should form the binary service later on (for universal binary file access using binary templates)
* Some sky box work

Changes in alpha release 0.1.8 (7-Oct-2006)
* Header files put to the place where cpp files are, in hoping that this wil simplify orientation and building
* Converted from automake/autoconf to cmake
* Fixes to make compatible with VC++ and Mingw (Still some problems can/will emerge though)
* Restructuralisation of the directory tree. I hope this will be more meaningful

Changes in alpha release 0.1.9 (03-Dec-2006)
* Updated cmake scripts for both ogre and cegui, now the cmake will end up with an error when it does not find one of those
* Rewritten the renderer to use hardware buffers for static geometry and entities for water (major speed improvement)
* Attaching meshes to the scene now works (this means the object could be displayed if loading would be coded)
* added a logger
* minor fixes and changes
- skyhack does not work correctly for now, the skybox needs to be rendered before the geometry in order to work. Will be fixed

Changes in alpha release 0.2.0 (28-Jan-2007)
* Lightmap atlas can now be rectangular (not square only)
* Console now understands 'load_mission MISSION_FILE' command
* Console now understands 'show_portals X' command (X==1 shows visible portal geometry)
* RaySceneQuery and CollisionSceneQuery were implemented (the latter was not tested yet, may be malfunctioning or incomplete)
* fixed the loading of the world geometry (mission geometry can now be loaded online several times)
* added an cmake option to compile the DARKNESS_DEBUG code in (including LOG_DEBUG statements)
* Light map atlassing rewritten a bit. Now atlassing is done after all lightmaps are inserted, using a binary tree approach.

Changes in alpha release 0.2.1 (23-May-2007)
* removed OpdeGUI and OpdeMission temporary classes
* Fixed water material loading (still needs a texture manager service though)
* Removed the old DarkDB loading code.
* Started implementing endianness independence.
* Added File/FileGroup abstract classes, and thei're implementations for std, Ogre::DataStream, Memory
* Modified the WorldRep service to work agains the new FileGroup class
* Added the RefCounted class, used to implement reference counted classes.
* Added a chunk extraction and listing tool.
* Some small fixes in CMake scripts.
* Implemented a binary service, which is a catalogue of dynamically defined structured types
* Implemented a compiler for dtype scripts
* Implemented a ScriptLoader for the dtype scripts
* Added dtype scripts for all 3 games
* Restructured the scripts directory
* Removed the junk code from main
* Implemented the DTypeDef and DVariant classes
* Made a small improvement in the DarkSceneManager (performance - wise)
* Implemented game's state manager.
* Moved the code to Ogre3D 1.4
* Added dependency to OIS
* Various small fixes
* Removed dependency on CeGUI

Changes in alpha release 0.2.9 (10-Oct-2008)
* Implemented a propper static geometry rendering class
* Added a image loading override that supports palletized transparency
* Implemented python bindings code, great part of Opde can now be used from python
* Implemented vis. blocking
* Implemented lightning
* Built-in properties now use built-in property declaration



TODO for openDarkEngine
------------------------

Pre-0.3 release:

    - cleanup of WorldRep service. New style of struct reading using the signatures should be used
      - partially done. The struct signatures are pain because of the alignment needed. Rewritten to use bitshift-like operators insted
    - new main.cpp - preferably only a small wrapper for python scripts to be executed with
      - this will be done differently. We have a testing viewer ready, the next task is to come up with gui and simulation. Python side code with opde module import.
    + installation scripts / win32 installer - should also prepare opde's config files to be ready to rock after install
    - small clarifications of the methods working with property/link data - what the data flow will be? getData can return archetype's data, so changes should be disallowed on that returned object
      - getData was removed. The functionality of data storage now lies in data storages
    - Testing: for example how to correctly (and legally) transfer python's lib together with opde, bug fixes for opde if it does not find data it needs
      - Python lib instalation works now on linux
    - Example scripts to the SVN - resources.cfg, opde.cfg, plugins.cfg, etc should be placed in SVN to ease the pain for people trying to build from source
      - We'll have a propper platform settings system written in 0.3 instead
    + optional: Optimization. The new SM for example does not perform so well one could be satisfied with it.
    + Built-in properties: To stop the opde from failing fatally if the core sripts are corrupted/missing
    - DTypeDef cleanup: It now seems we can safely swap endianness of DTypeDef supported data. This means we can do all the byte mumbo-jumbo whilst loading, and have a cleaner get/set code.
	- need to be 100% sure here - there are some unions present
	(Also an option: Getting rid of the DType structures altogether, using linear array of field lists with type. Bit more work, result would be much more simmilar to Dark's original impl.)
	- Data storages are used now. This gets obsolete thus. The unions will be handled internally in the questioned storage though...
    + SceneNode should be moved to RenderSystem again. The current position in the object service was too much a shortcut.
	* Here are some thoughts: both Position property setting and call to the ObjectService::setPosition/Orientation should end up calling position listeners(only once)!
	* some internal caching of the object->position and object->orientation can be done in object service, but not needed (too much data duplication)
	* Physics service has the ability to stop controling certain parts of the object's orientation/position. Need more info here - what does setPosition in obj. service do with it
	* AND how the physics service should update the position of the object (in fact any service)?
	    - seems the best way is that phys. service is a listener to the position/orientation of the object. when needing to set the position/orientation, it will call ObjService::setPos/Ori
    + Decoupling of data representation and access. No more should the code be based on DType and DTypeDef classes (can be, but only internally).
    + DTypeDef should have the variable length data option removed. There is property definition parameter designed just for that, and a dedicated variable length string data storage as well
    + Redesign and reimplementation of some core properties. Those should be marked internal and be hardcoded.
    + Possibility of property storage and implementation to be separate (for tools). This is done partially using optionality of the renderer listeners, but blind option for property group/data storage should do it better. That means core properties can reflect the values if blind is false as well.
    + Basic type data storages - float data storage, integer data storage, bool data storage (binary array based)
    + Advanced - structured templated data storage - something to be easilly personalised for the use with internal properties/link data that are structured (Property position for example).


0.3 - 0.4:
    * Object Scripts. The whole infrastructure - Script Service, script message struct. Need more info/ideas here.
        - It becomes clear that we need a full blown testing framework for scripts. We can omit the engine testing (not entirelly possible anyway),
	  but we need a framework to easily test the scripts - how those respond to messages, etc. We will probably need a set of hooks in the script service in order to do that and some simplification API.
    * in parallel to that (to test the system on something):
    	- Room service (probably the easiest, so should be the first)
	- A/R system
	- Physics
    * Sound system - need a propper resource system for this one, lots of data will travel because of this one
    * Full-Bright mode for DromEd users
    * Full support for multiple instances of Opde, also engine shutdown/restart (at least one of these has to work in order to do a fully working prop/link documentation generator)
      - not sure here yet - problems with ogre singletons. And after all this should not be needed besides object system comaparisons (can do a GAM<->XML importer/exporter instead)
    + DB load/save interfacing change. The old "one message covers all" adds code to services where it should rather save space. Rewrite to DBListener approach (onLoad, onSave...)
    + Platform service to take care of configuration storage directory, registry handling, per user save directories and other specifics (preparation was done by allowing platform dependent source code inclusion/compilation with cmake).
  From 0.2:
  * Code cleanup
  * Finalization of the core services bindings, coupled with Doc strings to make coder's life easier (This will continue till beta release of the project)
  * better handling of missing data
  * propper Big/Little endian handling for DType/DTypeDef classes (and OPDE in general). This (together with some other fixes) should enable the code to run on non-intel macs and other architectures
    - should be done now. Needs to be tested. The old handling of structure loads/saves needs to be removed (only one path to a problem solution should exist)
  * ogre's (and others) resource system management. How to organize resource unloading? How to classify loaded data? Those need to be answered
  * Other example scripts/documentation, including FAQ - How do I create a custom material replacement? How do I create a mesh replacement? (...New movements for AI?...)
  * Cleanup in the error handling - how to report errors, how to handle exceptions to Python
  * Object construction has some pending changes: Link definition should get clone/instantinate style attribute (clone target as well, clone link/leave archetype target id, discard link)
    - see the Thief1 (2?) Torch. In fact any archetype with particleAttachment. Then see T2's Flinderize link. Both have non-standard object instantination behavior.
  * On-the-fly max/min object id resets - should be partially implemented, but binary arrays need to be reallocated/repopulated then, this has to work (at least for editor mode)
  * Documentation - how to implement code that makes the property live. How to implement contstraints between various properties.
  * Fog rendering - the one of the last rendering problems to be solved. Fog lives in the scenemanager, but we need to render our geometry twice - once the part that is fogged, once the part that is not fogged.
	How this works in original dark remains to be revealed - non-fogged geometry seen through a fogged area is a small example. Maybe this uses some kind of volumetric fog. The author of DDFix - TimeSlip - should know more.
  * remove ODE, we will use in-house physics to comply with the original engine (too sensitive to change)
  * rendering optimizations
  * dev assistant script - find thief/ss2 game files, setup config files accordingly next to opde executable (install-less developement)
  * Redo the console using draw service to avoid the usage of ogre3d overlays
  * Cleanup
    - replace tabs by 4 spaces. Update formatting guide, astyle config
    - convert pimpl where possible
    - forward pointer types, include in cpp files as needed

----------------------------------
- Some random side-project ideas -
----------------------------------

Documentation
-------------
We have the doxy system running (needs updating the comments though). Texinfo file with data flow descriptions, coding style descriptions, how-to guide, etc
should be written (partially is already, more work is needed).


Documentation generator for Properties, links and chunk formats
---------------------------------------------------------------
STATUS: Prototype code present written in C++, should be rewritten in Python and use some smart templating.
Preferably generating texinfo files. Should run game's core without any graphics, collect data on properties/links/chunks
A detail holding text file should be possible to load, so there could be more than just the basic look of property or link described.

Example of such doc extension format (formal format specification needed):

Prop.ModelName {
    Holds the name of object's model. Implemented in RenderService.

    Changing this property will result in object's model change.

    ... etc ...
}


The documentation generator should then produce a texinfo file that could be compiled into info, html and other formats.
Up to three different structure descriptions would be introduced, one for each different game type (or a collored diff).

Currently a prototype code exists in opdeDocGen, but should be probably rewritten into a python script.


Preprocessor for .pldef and .dtype script files
-----------------------------------------------
STATUS: Not yet started.
A preprocessor with a C style syntax (simple is sufficient) could be introduced to help with merging of the 3 different versions of property, link and data
description scripts. It would be feeded with a preprepared set of variables (integer variables should be enough). Should handle simple condition evaluation.
File processing would then be done using a stack based machine, that would evaluate #if, #else, #endif lines.

VERSION variable should be introduced, that would be the base for the preprocessor. #define statement could be used to create new variables for the preprocessor,
#ifdef, #ifndef statements the same as in C could also be used if needed.

VERSION variable should be based on the internal (not in existance so far) config variable that should change the behavior of the whole engine. Values:

VERSION - MEANING
1         TDP/TG
2         TMA
3         SS2


The whole idea is to create a single set of scripts with the differences wrapped up in the #if, #endif conditions:

#if VERSION>=2

// include some stuff that is only
#else
#endif

Other statements could be also introduced, but I believe these will be more than sufficient:
#if CONDITION
 - tests for condition (allowed are: rounded braces, variables, spaces, logical expressions - &&,||, numerical >,<,>=,<=)
#else
 - else part for the condition
#endif
 - ends the conditionally included code
#define VAR VAL
 - will set a value VAL to a variable VAR, creating it if it did not exist (no macro style replacement will be allowed - too complex to do I believe)
#ifdef VAR
 - same as if, only test for variable's existance itself
#ifndef VAR
 - same as if, only test for variable's inexistance itself
#undef
 - undefines a variable


Editor side-Project
-------------------
STATUS: Not yet started

An editor (codename CamelEd ;) ) could be created. The basic operations should not be a problem to implement, as there already is the whole object system
roughly prepared for that. Basing it on WxPython could be a good idea.

TO BE SPECIFIED

some ideas:
*   Logger could get a brother - error reporter. Slightly more complex, conditionally compiled code present in whole OPDE would enable an existence of Error
    console, which could be even used for finding what went wrong in game mode - Message broadcasts, etc. Let it be 64 different data sources, and create a
    filter for the messages, so user could choose which messages are interesting.
*   Custom BSP builder and lighter should enable us to introduce better graphics. Some brushes could be marked as detail brushes for example - meaning those
    would not be used for occlusion
*   World geometry import - a special kind of brush maybe?
*   Custom WRRGB variant to implement this all

Need inspiration for other ideas. These listed are just some random dumb ideas that I came with. DromEd users should know better what they need
(if they need a DromEd replacement, that is)

OPDE as a python extension
--------------------------
STATUS: In progress

Ok, the 0.3 version release should include a python script using opde python module to implement the particular game. The principle is to use python's
library with our binary. We could also (and it would be a good idea) introduce the whole OPDE as a python Extension, meaning it would install into the
pythons extensions, and an ordinary python editor/debugger could then be used to work on OPDE.

Should not be so much work:
 * completely give the initialization of OPDE to the Python's side (meaning all the resource paths, etc would be configured by python script, including the
    call to bootstrapFinished). This even means the whole OPDE could be initialized and shutted down numerous times per one script execution.
 * wrap it all up as a python extension compiled as a shared library
 * custom compiler flags remain an unknown here - as well as cmake as an alternative to setup.py ordinary extension build tool - INFO NEEDED here

# darkness

darkness, a fork of the openDarkEngine project, aims to be an extensible, cross platform implementation of the [Dark Engine](https://en.wikipedia.org/wiki/Dark_Engine). It aims to support all previously-generated official and fan content, and admit modular access to modernizing the graphics and AI stacks.

![don't be so sure](doc/orly.png)

openDarkEngine (OPDE) was created as a heroic effort to build a multiplatform engine supporting the LookingGlass'es (TM) data files from the original DarkEngine(TM) games.

Efforts were previously started to bring in simulation code and scripting support, but after a long hiatus this project is now undergoing a *refactoring/cleanup* phase, that will continue with removal of Ogre3d library and dependencies, to be replaced with a thin, rendering only library (f.ex. bgfx). This means resource/scene management will have to be written. Bulky python script bindings will probably get replaced with a more manageable Squirrel language bindings.

## License
All the files, if not specified otherwise, are released under the GNU/GPL license.

## Building
There is some partially obsolete info with build instructions in the doc/DEVELOPERS file. This will be updated after the project cleanup is finished, which includes a transition to another rendering library.

## Help needed
This project would obviously benefit from more developers. Anyone willing to participate is welcome, please start by diving in the project, look at some issues that are in the tracker, maybe try fixing something or improving something.

## Thanks

* TomNHarris (telliamed) - for the all the work he has done understanding the Dark Engine and its data formats. Also for the irreplaceable help in the past.
* ShadowSpawn - For the BIN mesh format and Movement database format descriptions.
* ataricom - For helping out with the (now defunct) sourceforge Wiki
* TTLG community
* ...and others not mentioned


# Disclaimer
`darkness` is an independent reimplementation of the Dark Engine and is not affiliated with, endorsed by, or derived from the original software or its rights holders.

This project does not contain, distribute, or incorporate any proprietary source code, binaries, assets, or data files from the original software. All code in this repository has been written by the project's contributors through independent development efforts. Users must supply their own legally obtained copies of any original data files required to operate this software. This project does not facilitate, encourage, or provide access to pirated or unauthorized copies of any proprietary material.

### Intellectual Property
All original code in this repository is released under the GPLv3 license. See LICENSE for full terms.

The Dark Engine and any associated trademarks are the property of their respective owners. The use of these names within this project is solely for purposes of identification and interoperability, and does not imply any claim of ownership or affiliation.

### Purpose
This project exists to preserve access to The Dark Engine-based games on modern hardware and operating systems, in a context where the original software is no longer commercially available, maintained, or supported by its rights holders. It also aims to provide the opportunity for the fan community to extend the core functionality of the game to take advantage of modern hardware for non commercial purposes only.

### Good Faith
This project is developed and distributed in good faith, on a non-commercial basis, by volunteer contributors. Should any rights holder have concerns regarding this project, we welcome direct communication at [joseph at viviano dot ca] and are committed to addressing any legitimate concerns promptly and in good faith.


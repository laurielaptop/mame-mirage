# Ensoniq Mirage emulator (MAME with an updated `enmirage` driver)

This is a fork of [MAME](https://github.com/mamedev/mame) with changes to the Ensoniq Mirage driver. It is **not** the official MAME, it is not endorsed by the MAME team, and nothing here has been merged into MAME. Everything outside the files listed below is upstream MAME as of 2026-10-09 (commit 7804dd1c). MAME's own README follows after this section.

**To run it you need a Mirage boot ROM image (`mirage.bin`, 4096 bytes). It is not included and not provided; use a dump of your own machine's ROM.**

## What is different from MAME

The branch `mirage` is MAME at that commit plus these changes to `src/mame/ensoniq/enmirage.cpp` and two shared devices (`6522via`, `imagedev/midiin`):

| Change | Effect |
|---|---|
| CPU clock 1 MHz, VIA clock 2 MHz | Machine speed follows my reading of the board's clocks |
| Two comment corrections | Filter address order, port B bit 4 |
| `6522via` option plus the Mirage using it | The OS can clock its MIDI port: MIDI in and out work in the emulation |
| `-kbdin file.mid` | Plays a MIDI file through a stand-in for the keyboard controller, so notes arrive the way a played key's do |
| Filter model | An approximation of the CEM3328 voice filters |
| ADC read-back | The OS's start-up filter calibration reads that model |
| Copyright line | Adds the contributor to `copyright-holders` of `enmirage.cpp` |

The filter model and ADC read-back are approximations tuned by hand, with constants that are assumed or taken from a datasheet. They change the sound for everyone and I have not compared them with a real instrument, so I do not claim they are right. With them on there is also a tone about 5 to 7 emulated seconds into an OS 3.2 boot that is not there without them.

## Status: what has and has not been checked

Checked, in the emulator only: it builds with no new warnings in the changed files; `-validate`; headless boots; runs over five OS disk images and four MIDI files; builds of every driver file that reaches the two shared devices.

**Not checked: anything on a real Mirage.** The clock values, the keyboard protocol timings and the filter numbers are my readings and assumptions. Tested on macOS (Apple clang, arm64) only. Do not rely on this for anything where accuracy matters.

## Build and run

Build as MAME does (see MAME's documentation for the prerequisites on your system), in a path **without spaces**. For this driver only:

```
make SOURCES=src/mame/ensoniq/enmirage.cpp SUBTARGET=enmirage TOOLS=0 -j8
```

Put your ROM at `<romdir>/enmirage/mirage.bin` and run:

```
./enmirage enmirage -rompath <romdir>
./enmirage enmirage -rompath <romdir> -flop1 <os-disk.img> -kbdin <notes.mid>
```

`-kbdin` refuses its file if the boot ROM does not have the velocity tables it expects; that is a message, not a crash. The Mirage OS and its disk images are Ensoniq's and are not included either.

## What is not here

No ROM, no disk image, no OS binary, no schematic and no text from Ensoniq documentation.

## AI disclosure

Claude (several models over time) assisted with the original patches and with turning them into this series. Every series commit says so in an `AI disclosure` line.

## Licence

Changes to `enmirage.cpp`, `midiin.cpp/.h` and `6522via.cpp/.h` are offered under those files' existing licences (BSD-3-Clause). The rest is MAME, under MAME's licences.

---

# MAME

## What is MAME?

MAME is a multi-purpose emulation framework.

MAME's purpose is to preserve decades of software history. As electronic technology continues to rush forward, MAME prevents this important "vintage" software from being lost and forgotten. This is achieved by documenting the hardware and how it functions. The source code to MAME serves as this documentation. The fact that the software is usable serves primarily to validate the accuracy of the documentation (how else can you prove that you have recreated the hardware faithfully?). Over time, MAME (originally stood for Multiple Arcade Machine Emulator) absorbed the sister-project MESS (Multi Emulator Super System), so MAME now documents a wide variety of (mostly vintage) computers, video game consoles and calculators, in addition to the arcade video games that were its initial focus.

## Where can I find out more?

* [Official MAME Development Team Site](https://www.mamedev.org/) (includes binary downloads, wiki, forums, and more)
* [MAME Testers](https://mametesters.org/) (official bug tracker for MAME)

### Community

* [r/MAME](https://www.reddit.com/r/MAME/) on Reddit
* [MAMEdev Forum](https://forum.mamedev.org/)
* [MAMEdev Discussions](https://github.com/orgs/mamedev/discussions) on GitHub

## Development

![Alt](https://repobeats.axiom.co/api/embed/8461d8ae4630322dafc736fc25782de214b49630.svg "Repobeats analytics image")

### CI status and code scanning

[![CI (Linux)](https://github.com/mamedev/mame/workflows/CI%20(Linux)/badge.svg)](https://github.com/mamedev/mame/actions/workflows/ci-linux.yml) [![CI (Windows](https://github.com/mamedev/mame/workflows/CI%20(Windows)/badge.svg)](https://github.com/mamedev/mame/actions/workflows/ci-windows.yml) [![CI (macOS)](https://github.com/mamedev/mame/workflows/CI%20(macOS)/badge.svg)](https://github.com/mamedev/mame/actions/workflows/ci-macos.yml) [![Compile UI translations](https://github.com/mamedev/mame/workflows/Compile%20UI%20translations/badge.svg)](https://github.com/mamedev/mame/actions/workflows/language.yml) [![Build documentation](https://github.com/mamedev/mame/workflows/Build%20documentation/badge.svg)](https://github.com/mamedev/mame/actions/workflows/docs.yml)  [![Coverity Scan Status](https://scan.coverity.com/projects/5727/badge.svg?flat=1)](https://scan.coverity.com/projects/mame-emulator)

### How to compile?

If you're on a UNIX-like system (including Linux and macOS), it could be as easy as typing

```
make
```

for a full build,

```
make SUBTARGET=tiny
```

for a build including a small subset of supported systems.

See the [Compiling MAME](http://docs.mamedev.org/initialsetup/compilingmame.html) page on our documentation site for more information, including prerequisites for macOS and popular Linux distributions.

For recent versions of macOS you need to install [Xcode](https://developer.apple.com/xcode/) including command-line tools and [SDL 2.0](https://github.com/libsdl-org/SDL/releases/latest).

For Windows users, we provide a ready-made [build environment](http://www.mamedev.org/tools/) based on MinGW-w64.

Visual Studio builds are also possible, but you still need [build environment](http://www.mamedev.org/tools/) based on MinGW-w64.
In order to generate solution and project files just run:

```
make vs2022
```
or use this command to build it directly using msbuild

```
make vs2022 MSBUILD=1
```

### Coding standard

MAME source code should be viewed and edited with your editor set to use four spaces per tab. Tabs are used for initial indentation of lines, with one tab used per indentation level. Spaces are used for other alignment within a line.

Some parts of the code follow [Allman style](https://en.wikipedia.org/wiki/Indent_style#Allman_style); some parts of the code follow [K&R style](https://en.wikipedia.org/wiki/Indent_style#K.26R_style) -- mostly depending on who wrote the original version. **Above all else, be consistent with what you modify, and keep whitespace changes to a minimum when modifying existing source.** For new code, the majority tends to prefer Allman style, so if you don't care much, use that.

All contributors need to either add a standard header for license info (on new files) or inform us of their wishes regarding which of the following licenses they would like their code to be made available under: the [BSD-3-Clause](http://opensource.org/licenses/BSD-3-Clause) license, the [LGPL-2.1](http://opensource.org/licenses/LGPL-2.1), or the [GPL-2.0](http://opensource.org/licenses/GPL-2.0).

See more specific [C++ Coding Guidelines](https://docs.mamedev.org/contributing/cxx.html) on our documentation web site.

## License

The MAME project as a whole is made available under the terms of the
[GNU General Public License, version 2](http://opensource.org/licenses/GPL-2.0)
or later (GPL-2.0+), since it contains code made available under multiple
GPL-compatible licenses.  A great majority of the source files (over 90%
including core files) are made available under the terms of the
[3-clause BSD License](http://opensource.org/licenses/BSD-3-Clause), and we
would encourage new contributors to make their contributions available under the
terms of this license.

Please note that MAME is a registered trademark of Gregory Ember, and permission
is required to use the "MAME" name, logo, or wordmark.

<a href="http://opensource.org/licenses/GPL-2.0" target="_blank">
<img align="right" width="100" src="https://opensource.org/wp-content/uploads/2009/06/OSIApproved.svg">
</a>

    Copyright (c) 1997-2026  MAMEdev and contributors

    This program is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License version 2, as provided in
    docs/legal/GPL-2.0.

    This program is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details.

Please see [COPYING](COPYING) for more details.

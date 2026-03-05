# psx-reverb

Command line tool to process a raw audio file using the original PlayStation™ reverb algorithm as described at [psx-spx](https://psx-spx.consoledev.net/soundprocessingunitspu/).

## Usage

```
Usage: psx-reverb [options] <input file>
Options:
  --preset <name>      Set reverb preset (case-insensitive, default: Room)

Valid preset names are:
    Room
    StudioSmall
    StudioMedium
    StudioLarge
    Hall
    HalfEcho
    SpaceEcho
    ChaosEcho
    Delay
    Off
```

The input file should be a raw signed 16-bit PCM, 2 channel file, preferably 44100 Hz.

The presets are the set of Reverb and SPU register values taken from psx-psx.

## Building

CMakeLists.txt is provided as well as several helper scripts.

### Windows

Run gensln.bat from a Developer Command Prompt for Visual Studio. Then either open build\psx-reverb.sln in Visual Studio and build from there, or run build.bat

### Linux

Use cmake to configure the project, or run the provied build.sh script to build debug and release using GCC and Clang.

## References

Much of the information, and some of the comments, are taken directly from https://psx-spx.consoledev.net/soundprocessingunitspu/

## Known issues

It has been reported that this produces slightly too much reverb.

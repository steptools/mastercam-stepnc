mastercam-stepnc
=======

STEP-NC Export Chook for Mastercam
--

This project build a plugin for [Mastercam](http://www.mastercam.com)
that exports a machining program as STEP-NC.  The program uses the
"Chooks" plugin API to access Mastercam data and the STEP-NC Machine
API to create the STEP-NC data.   

## Building

This package contains a Visual Studio 2012 project file for building.
Mastercam X8 and X9 requires x64 code, so the project only has platforms
for "Release x64" and "Debug x64".   The master branch of this package
supports the latest version of Mastercam (X9) and earlier code can be 
found on the mcx8 branch.

The package requires a Mastercam installation with the Mastercam X8
SDK installed.  You also need the STEP-NC Machine DLL to create the
STEP-NC data.  A downloadable version that is free for personal use
can be found at:

STEP-NC Machine (which contains the STEP-NC DLL)

 - Download - http://www.steptools.com/products/stepncmachine/download/
 - API Docs - http://www.steptools.com/support/stepnc_docs/stepncdll/

The project will build a DLL called `ap238export.dll` and will output
it to the `C:\Program Files\mcamx8\chooks\` directory.  Then you can
call the plugin by starting Mastercam, opening your file, clicking
`Alt-C` and selecting the ap238.export.dll in the chooks dialog box.

## Extending

See [CONTRIBUTING.md](CONTRIBUTING.md) if you would like to send a
pull request with your changes.

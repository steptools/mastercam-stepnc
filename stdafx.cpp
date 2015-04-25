// stdafx.cpp : source file that includes just the standard includes
// ap238export.pch will be the pre-compiled header
// stdafx.obj will contain the pre-compiled type information

#include "stdafx.h"


// DLL stuff for Mastercam to STEP-NC translator
//
#include "stdafx.h"
#include "m_core.h"
#include "m_mastercam.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


extern "C" __declspec(dllexport) int m_version (int version)
{
    int ret = C_H_VERSION;

    // Allow chook to run in any version of mastercam that has
    // the same major version
    if ( (version / 100) == (C_H_VERSION / 100) )
	    ret = version;

    return ret;
}

void ap238export();

extern "C" __declspec(dllexport) int m_main (int not_used)
{
//    MessageBox (0, "Big Hello from Mastercam!", "", MB_OK);
//    SetDllDirectory("C:\\Program Files (x86)\\STEP Tools\\STEP-NC Machine");
    ap238export();
//    SetDllDirectory(NULL);

    return MC_NOERROR;
}
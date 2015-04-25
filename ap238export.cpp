/*
 * Copyright (c) 1991-2015 by STEP Tools Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// main routine for AP-238 export

#include "stdafx.h"
#include <comutil.h>
#include <stdio.h>

#include "m_core.h"
#include "m_mastercam.h"
#include "m_mill.h"

#import <mscorlib.tlb>
#import <System.tlb>

#import "c:\\Program Files (x86)\\STEP Tools\\STEP-NC Machine\\stepnc_x64.tlb" 
using namespace std;


// COM Smart pointer for the move checker.  You can also do it the
// old-school way using CoCreateInstance
stepnc_x64::_AptStepMakerPtr apt;
stepnc_x64::_ProcessPtr process;
stepnc_x64::_FeaturePtr feature;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define MY_DEBUG 1

#ifdef MY_DEBUG
FILE* pfLog;
#define MY_TRACE fprintf
#else
void* pfLog;
#define MY_TRACE void(0)
#endif

bool cc1 = true;
bool cc3 = true;
bool use_tp_orientation = true;
bool compound = false;	    // this flag not fully implemented
bool pattern = false;

double hole_bot;

#define EPSILON 0.01

BOOL VEq(p_3d v1, p_3d v2)
{
    return (v1[0] == v2[0] &&
	    v1[1] == v2[1] &&
	    v1[2] == v2[2]);
}

// database of chains converted to pockets
// eptr_type pChain_db[1024];
int feature_id_db[1024];
int ws_id_db[1024];
int chain_db_count = 0;

// used to store tool direction and compute CCW vs CW
BOOL tool_direction[1024];

// main functions
void LoadAllTools();
void MakePlanarFace(operation op, p_3d ptCenter, double dLength, double dWidth, _int64 ws_id);
void MakeGeneralOutsideProfile(operation op, CHAIN* pChain, _int64 ws_id);
void MakeGeneralPocket(operation op, CHAIN* pChain, _int64 ws_id, int nChains);
void MakeRoundHole(operation op, p_3d ptCenter, _int64 ws_id);
void MakeRoundPocket(operation op, p_3d ptCenter, double dDiameter, double dDepth, _int64 ws_id);


// helper functions
void MakeChainProfile(
    CHAIN* pChain,
    double z_value,
    BOOL closed,
    BOOL reverse_contour_direction
);

void MakeProfileGeometry (
    ent seg1, ent seg2,
    BOOL first,
    BOOL boss_or_open,
    double z_value,
    BOOL last_in_open,
    BOOL reverse_contour_direction
);

BOOL counter_clockwise (p_2d p1, p_2d p2, p_2d center);
void GetChainBBox(
    CHAIN* pChain, int nView, double& dLength, double& dWidth, p_3d center
);



// code given to us by Mastercam
// Retrieve the 'tree branches' of items above the Operation
// Used to make nested workplans
void GetOpParents(operation *opl, CStringArray *names);

extern void ap238export()
{
    MC_BOOL bResult, bChainResult;
    CString strMessage;
    nci_bin n;
    long ltcode = 0;		// lathe tool number
    long fpos;
    p_3d ptStart, ptEnd, ptBot;
    p_3d vecDir, vecRef, vecOldDir = { 0, 0, 1 };
    x_matrix mXform1;		// wcs change (for origin) apply first
    x_matrix mXform2;		// tool plane change (for axis) apply second
    unsigned nOldOpID = -1;
    int nOldGcode = -1;
    double nOldRPM = 0;
    double dOldFeed = 0.0;
    BOOL bAxisChgd = FALSE;
    double dRad;
    CString strLabel;
    BOOL bFirstMove = TRUE;
    BOOL bInRapidMode = FALSE;
    short nPrevCComp = 0;
    p_3d vecSurfNorm;
    BOOL bNewOpCheckCComp = FALSE;
    chain_manager_info cmi;
    int nChains;
    short nPock;		// decode pocket and island chains
    pock_bound* pPB;
    CString cmnt;		// current comment
    long main_q;		// question each export or not

    double hole_z1, hole_z2, hole_z3;	// depth for holes when hole pattern
    double hole_speed, hole_feed, hole_plunge, hole_retract;


    CString fnme = "export.stpnc";

    CFileDialog dlgSave(
	FALSE, ".stpnc", fnme, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
	"STEP-NC Files (*.stpnc)|*.stpnc|STEP Files (*.stp;*.step;*.p21)|*.stp;*.step;*.p21|All Files (*.*)|*.*||", NULL
    );
    try
    {
	if (IDOK != dlgSave.DoModal())
	{
	    return;
	}

#ifdef MY_DEBUG
	pfLog = fopen("c:\\Users/Hardwick/Documents/mc8_trace.txt", "w+t");
#endif
	// Initialize COM.
	HRESULT hr = CoInitialize(NULL);
	if (!SUCCEEDED (hr))
	{
	    MY_TRACE(pfLog, "COM initialization failed - call STEP Tools \n");
	    MessageBox (0, "COM initialization failed - call STEP Tools", "", MB_OK);
	    fflush(pfLog);
	    return;
	}
	hr = apt.CreateInstance(__uuidof(stepnc_x64::AptStepMaker));
	if (!SUCCEEDED (hr))
	{
	    MY_TRACE(pfLog, "APT initialization failed - call STEP Tools \n");
	    MessageBox (0, "APT initialization failed - call STEP Tools", "", MB_OK);
	    fflush(pfLog);
	    return;
	}
	hr = feature.CreateInstance(__uuidof(stepnc_x64::Feature));
	if (!SUCCEEDED (hr))
	{
	    MY_TRACE(pfLog, "Feature initialization failed - call STEP Tools \n");
	    MessageBox (0, "Feature initialization failed - call STEP Tools", "", MB_OK);
	    fflush(pfLog);
	    return;
	}
	hr = process.CreateInstance(__uuidof(stepnc_x64::Process));
	if (!SUCCEEDED (hr))
	{
	    MY_TRACE(pfLog, "Process initialization failed - call STEP Tools \n");
	    MessageBox (0, "Process initialization failed - call STEP Tools", "", MB_OK);
	    fflush(pfLog);
	    return;
	}

	chain_db_count = 0;		    // database of pocket chains


	// get name of the machine group
	operation *op0 = TpMainOpMgr.GetMainOpList()[0];
	op_group *op_parent = TpMainGrpMgr.GetMainGrpList().GroupByID(op0->cmn.grp_idn);
	op_group *op_mach = TpMainGrpMgr.GetMainGrpList().RetrieveRootMachineGroup(op_parent->grp_idn);

	CString main_name = op_mach->name;

	apt->PartNo("Mastercam Export");
	feature->OpenNewWorkpiece ("Mastercam Export");

	// if on then use bounded curve geometry
//		apt->SetDefineArcUsingViaOff ();
//		apt->SetDebuggingNamesOn();

	if (IsEnglish())
	{
	    MY_TRACE(pfLog, "Setting units to Inches\n");
	    apt->Inches();
	}
	else
	{
	    MY_TRACE(pfLog, "Setting units to Millimeters\n");
	    apt->Millimeters();
	}

	apt->CamModeOn ();
	apt->MultaxOn();

	INT_PTR GrpListSize = TpMainGrpMgr.GetMainGrpList().GetSize ();
	CString tmsg;
	/*		op_group* group0 = TpMainGrpMgr.m_MainGrpList[0];
			op_group* group1 = TpMainGrpMgr.m_MainGrpList[1];
			tmsg.Format ("List size is %d contains %s and %s", GrpListSize, group0->name, group1->name);
			AfxMessageBox(tmsg, MB_YESNO | MB_ICONINFORMATION);*/

	op_group* group = NULL;
	if (GrpListSize > 0)
	{
	    op_group* group = TpMainGrpMgr.GetMainGrpList()[0];
	    MY_TRACE(pfLog, "Job setup material = %s at (%f, %f, %f) \n\tlength = %f, width = %f, height = %f\n",
		     group->ogi.pg2.matl_name,
		     group->ogi.pg3.cpt[X], group->ogi.pg3.cpt[Y], group->ogi.pg3.cpt[Z],
		     group->ogi.pg3.x, group->ogi.pg3.y, group->ogi.pg3.z);

	    if (group->ogi.pg3.x != 0 && group->ogi.pg3.y != 0)
	    {
		process->BlockRawpiece (
		    "Mastercam block",
		    group->ogi.pg3.cpt[X],
		    group->ogi.pg3.cpt[Y],
		    group->ogi.pg3.cpt[Z] - group->ogi.pg3.z,
		    group->ogi.pg3.z, group->ogi.pg3.x, group->ogi.pg3.y
		);
	    }
	}

	LoadAllTools ();

	INT_PTR OpListSize = TpMainOpMgr.GetMainOpList().GetSize();
	if (OpListSize == 0)
	    MY_TRACE(pfLog, "Empty Operation list\n");
	else
	{
	    operation *opl = NULL;
	    op_information info;
	    opl = TpMainOpMgr.GetMainOpList()[0];
	    op_info (opl->op_idn, TpMainOpMgr.GetMainOpList(), &info);

	    if (info.is_lathe)
	    {
		MY_TRACE(pfLog, "LATHE\n");
		apt->SetModeTurn ();
	    }
	    else if (info.is_mill)
	    {
		MY_TRACE(pfLog, "MILL\n");
		apt->SetModeMill ();
	    }

	    CString message;
	    CString caption = "STEP-NC Export";
	    message.Format ("Export all data with no more questions");
	    main_q = AfxMessageBox(message, MB_YESNO | MB_ICONINFORMATION);
	    if (main_q == IDYES)
	    {
		MY_TRACE(pfLog, "Exporting all data with no questions asked\n");
	    }

	    memset(&mXform1, 0, sizeof(x_matrix));
	    memset(&mXform2, 0, sizeof(x_matrix));

	    memset(&ptStart, 0, 3 * sizeof(double));
	    memset(&vecSurfNorm, 0, 3 * sizeof(double));

	    // Rewind to the beginning of the NCI file
	    nci_manager(opl->op_idn, NCIMGR_FSTART, &n, &fpos, &bResult);

//			for (INT_PTR OpCount = 3; OpCount < 4; ++OpCount)
	    CStringArray old_names, new_names;
	    for (INT_PTR OpCount = 0; OpCount < OpListSize; ++OpCount)
	    {
		MY_TRACE (pfLog, "TOP OF THE LOOP OPCount = %d, OpListSize = %d\n", OpCount, OpListSize);
		opl = TpMainOpMgr.GetMainOpList()[OpCount];
		char opDescription[80] = "";
		op_make_description (opl, FALSE, opDescription, 80);

		// workplan hierarchy for the next operation
		GetOpParents(opl, &new_names); // Get the "parents" of this operation
		if (OpCount == 0)
		{
		    INT_PTR NamesSize = new_names.GetSize ();
		    MY_TRACE(pfLog, "Mastercam operation stack has height %d\n", NamesSize);
		    old_names.Add (new_names[0]);
		    // Edited on July 2 2012 to eliminate "Toolpath Group" nested workplan - was NameSize - 2 now NameSize - 3
		    // ON Jan 15 2014 changed back to NameSize -2 to get workplan names for Sandvik process data
		    for (INT_PTR i = NamesSize - 2; i >= 0; i--)
		    {
			MY_TRACE(pfLog, "Making nested workplan called %s\n", new_names[i]);
			apt->NestWorkplan (new_names[i].AllocSysString());
		    }
		}
		else
		{
		    INT_PTR i, j;
		    INT_PTR NamesSize = new_names.GetSize ();
		    INT_PTR OldNamesSize = old_names.GetSize ();
		    MY_TRACE(pfLog, "Mastercam operation stack: new height %d old height %d\n", NamesSize, OldNamesSize);
		    // stacks are in reverse order
		    // Number changed on July 2 2012 to try to be consistent with above - NO TESTING - was -1 now - 2
		    INT_PTR ii = NamesSize - 2;
		    INT_PTR jj = OldNamesSize - 2;
		    // start at end of each and look for first difference
		    while (ii >= 0 && jj >= 0 && new_names[ii] == old_names[jj])
		    {
			ii--; jj--;
		    }
		    // pop off remainder of previous
		    for (j = 0; j <= jj && j < OldNamesSize - 1; j++)
		    {
			MY_TRACE(pfLog, "Ending workplan called %s\n", old_names[j]);
			apt->EndWorkplan ();
		    }
		    // ALSO CHANGED DO NOT FEEL GOOD ABOUT THIS ONE
		    if (ii == NamesSize - 2) i--;
		    // push new - last is machine group and is ignored
		    for (i = ii; i >= 0; i--)
		    {
			MY_TRACE(pfLog, "Making nested workplan called %s\n", new_names[i]);
			apt->NestWorkplan (new_names[i].AllocSysString());
		    }
		}
		old_names.RemoveAll();
		INT_PTR NamesSize = new_names.GetSize ();
		for (INT_PTR i = 0; i < NamesSize; i++)
		{
		    old_names.Add (new_names[i]);
		}

		if (main_q == IDNO)
		{
		    CString message;
		    CString caption = "STEP-NC Export";
		    message.Format ("Export Operation '%s' (%d of %d)", opDescription, OpCount + 1, OpListSize);
		    long res = AfxMessageBox(message, MB_YESNO | MB_ICONINFORMATION);
		    if (res == IDNO)
		    {
			MY_TRACE(pfLog, "Skipping operation %d of %d at user request\n", OpCount + 1, OpListSize);
			apt->Workingstep(_com_util::ConvertStringToBSTR (opDescription));
			char buf[100];
			sprintf (buf, "mc%d.stpnc", OpCount);
			apt->ExternalOperation (_com_util::ConvertStringToBSTR (buf));
			continue;
		    }
		}

		// Create new workingstep (MCAM OP ==> STEP WS)
		MY_TRACE(pfLog, "Making new workingstep called %s with comment %s\n", opDescription, opl->comment);
		if (strlen (opDescription) > 0)
		    apt->Workingstep(_com_util::ConvertStringToBSTR (opDescription));
		else if (opl->opcode == TP_DRILL)
		    apt->Workingstep("drill");
		else
		    apt->Workingstep("mill");


		// Move to the beginning of the operation stream
		nci_manager(opl->op_idn, NCIMGR_RD_SOS, &n, &fpos, &bResult);

		// --------------------------------------------------------
		// BEGIN reading operations in a loop from NCI data
		// --------------------------------------------------------
		while (bResult)
		{

		    // Read from the operation stream
		    nci_manager(opl->op_idn, NCIMGR_RD, &n, &fpos, &bResult);
		    MY_TRACE(pfLog, "HERE IS THE CURRENT GCODE: %d\n", n.gcode);
		    if (!bResult || n.op_idn != opl->op_idn)
		    {
			MY_TRACE(pfLog, "Breaking loop\n");
			break;
		    }

		    cmnt = opl->comment;

		    if (n.gcode == 20100)
		    {
			ltcode = n.u.l20100.lToolNum;
			MY_TRACE(pfLog, "LATHE TOOL SLOT: %d Number %d\n", n.u.l20100.uSlot, ltcode);
		    }

//						dlgProg.m_ctrlProgress.SetPos(opo.slot);

//					tp_get_all(n.op_idn, &op, &toolpaths, &ntoolpaths, &bIgnore);
		    if (n.op_idn != nOldOpID)
		    {
			CString msg;
//						msg.Format ("Exporting operation %d (%d:%d)) - %s.", n.op_idn, op.u.op.op_idn, op.u.op.slot, op.u.op.comment);
//						AfxMessageBox(msg);
			MY_TRACE (pfLog, "Exporting operation %d\n", n.op_idn);
		    }

		    /*******************************************************************/
		    /*	Workingstep stuff                                              */
		    /*******************************************************************/
		    // Make new workingstep
		    if (n.op_idn != nOldOpID)
		    {
			int fe_id = 0;
			if (ltcode == 0)
			{
			    apt->LoadTool(opl->tl.tlno);			// changed from slot to tlno for KTH then back for UTA - KTH almost definitely better
			    MY_TRACE(pfLog, "Loading tool %d\n", opl->tl.tlno);
			    bFirstMove = TRUE;
			}
			else  	// assume lathe tool
			{
			    apt->LoadTool(ltcode);
			    MY_TRACE(pfLog, "Lathe Loading tool %d\n", ltcode);
			    bFirstMove = TRUE;
			}

// --------------------------------------------------------
// BEGIN feature extraction
// --------------------------------------------------------

			// Now write feature information
			MY_TRACE(pfLog, "New op id = %d old op id = %d\n", n.op_idn, nOldOpID);
			if (n.op_idn != nOldOpID)
			{
			    // Extract chain
			    cmi.op_idn = n.op_idn;
			    cmi.mode = CHNMGR_GET;
			    cmi.bnd_n_start = 1;
			    cmi.bnd_n_end = 2;
			    cmi.chns = NULL;
			    nChains = 0;

			    chain_manager(&cmi, -1, &bChainResult);

			    if (!bChainResult)
			    {
				chain_manager(&cmi, 0, &bChainResult);
				nChains = number_of_chains(cmi.chns);
			    }

			    if (opl->opcode == TP_POCKET && cc3)
			    {

				// Call after calling "chain_manager" with "CHNMGR_GET"
				// This call yields a linked list of "pock_bound",
				// which (I believe) parallels the list of chains in "cmi.chns"
				// Unfortunately it does not set the "p_b" member in each CHAIN.
				// The list contains all pocket bounds, but no island/pocket flags yet.
//							chains_to_pock_bounds(cmi.chns, FALSE, FALSE, FALSE, 1.0, &pPB);

				// To find out which is apocket and which is an island --
//							sort_pock_bounds(NULL, FALSE, FALSE, 1.0, &pPB, &nPock);
				MY_TRACE(pfLog, "Number of Pockets %d pb = %x\n", nPock, pPB);

				nPock = 1;
				if (bChainResult && (nPock == 1 || nPock == 0))
				{
				    MakeGeneralPocket(*opl, cmi.chns, apt->GetCurrentWorkingstep(), nChains);
				}
				else if (bChainResult)
				{
				    // nFeatureID = MakeCompoundPocket (op.u.op, cmi.chns, apt->GetCurrentWorkingstep(), nPock, pPB);
				}
			    }
			    else if (opl->opcode == TP_CONTOUR && cc3)
			    {
				if (bChainResult)
				{
				    MakeGeneralOutsideProfile(*opl, cmi.chns, apt->GetCurrentWorkingstep());
				}
			    }
			    else if (opl->opcode == TP_FACE && cc3)
			    {
				double dLength, dWidth;
				p_3d ptCenter;

				if (bChainResult)  // Chain boundary -- use bounding box
				{
				    GetChainBBox(cmi.chns, opl->cpln.view_n, dLength, dWidth, ptCenter);
				}
				else if (group != NULL)  	// Stock boundary (ASSUMED the face is on the top of the stock)
				{
				    MY_TRACE(pfLog, "WARNING: Making planar face using stock dimensions\n");
				    dLength = group->ogi.pg3.x;
				    dWidth = group->ogi.pg3.y;

				    ptCenter[0] = group->ogi.pg3.cpt[X];
				    ptCenter[1] = group->ogi.pg3.cpt[Y];
				    ptCenter[2] = group->ogi.pg3.cpt[Z];
				}
				else
				{
				    dLength = dWidth = ptCenter[0]= ptCenter[1] = ptCenter[2] = 0;
				}

				MakePlanarFace(*opl, ptCenter, dLength, dWidth, apt->GetCurrentWorkingstep());
				if (nChains > 1)
				{
				    // ChainsToBosses(/*nFeatureID,*/ cmi.chns->next); // starting at the second chain in the CMI
				}
			    }
			}
		    }

// --------------------------------------------------------
// END feature extraction
// --------------------------------------------------------

#ifdef MY_DEBUG
		    fflush(pfLog);
#endif
		    // should not be doing the below every time
		    /*						if (op.u.op.tl.coolant == 1) {
		    						apt->CoolantOn();
		    						}
		    						else {
		    						apt->CoolantOff();
		    						}

		    						if (tool_direction[op.u.op.tl.slot])
		    						apt->SpindleSpeed(op.u.op.tl.rpm);
		    						else
		    						apt->SpindleSpeed(-op.u.op.tl.rpm);*/

		    bNewOpCheckCComp = TRUE;
		    // add operation to instantiate drill cycle
		    // this used to key off gcode == 0 which lead to issues
		    if (opl->opcode == TP_DRILL && cc1 && n.gcode == 81)
		    {
			MakeRoundHole(*opl, n.u.m0.ep1, apt->GetCurrentWorkingstep());

			// ptStart is probably redundant and currently wrong (not used)
			// When switched to code = 81 was found to be at bottom
			memcpy(ptStart, n.u.m0.ep1, 3 * sizeof(double));
			memcpy(ptEnd, n.u.m0.ep1, 3 * sizeof(double));
			memcpy(ptBot, ptEnd, 3 * sizeof(double));
			if (opl->cmn.clearance_on)
			    ptEnd[Z] = ptEnd[Z] - opl->cmn.clearance_pln;
			else if (opl->cmn.retract_on)  // seems to be same as clearance_plane and  not the value called retract in the UI
			{
			    if (opl->cmn.retract_inc)
				ptEnd[Z] = ptEnd[Z] - opl->cmn.retract_pln;
			    else
				ptEnd[Z] = ptEnd[Z];
			}

			memcpy(ptBot, ptEnd, 3 * sizeof(double));
			double depth = fabs(opl->cmn.top_stock - opl->cmn.depth);
			MY_TRACE(pfLog, "Drilling Depth = %lf, break through = %lf\n", depth, opl->u.prm_drl.brk_thru);
			MY_TRACE(pfLog, "z = %lf Clearance = %lf, Retract = %lf feed_pln = %f\n", ptEnd[Z], opl->cmn.clearance_pln,
				 opl->cmn.retract_pln, opl->cmn.feed_pln);
//							depth = depth + op.u.op.u.prm_drl.brk_thru;	// KTH magnus says ignore
			if (opl->cmn.depth_inc)
			    ptBot[Z] = ptBot[Z] - depth;
			else
			    ptBot[Z] = opl->cmn.depth;

			// As per magnus feed/speed must change at the feed plane point
			if (opl->cmn.feed_inc)
			    ptEnd[Z] = ptEnd[Z] + opl->cmn.feed_pln;	// this is the value called retract
			else
			    ptEnd[Z] = opl->cmn.feed_pln;			// this is the value called retract

			// remember values for other points in pattern
			hole_z1 = ptStart[Z];
			hole_z2 = ptEnd[Z];
			hole_z3 = ptBot[Z];
			if (tool_direction[opl->tl.tlno])
			    hole_speed = opl->tl.rpm;
			else
			    hole_speed = -opl->tl.rpm;
			hole_feed = opl->tl.feed;
			hole_plunge = opl->tl.plunge;
			hole_retract = opl->tl.retract;

			// calculate the orientation
			if (!use_tp_orientation)
			{
			    xform_pt(ptStart, &mXform1);
			    xform_pt(ptEnd, &mXform1);
			    xform_pt(ptBot, &mXform1);

			    xform_pt(ptStart, &mXform2);
			    xform_pt(ptEnd, &mXform2);
			    xform_pt(ptBot, &mXform2);
			}

			if (nOldRPM != hole_speed)
			{
			    apt->SpindleSpeed (hole_speed);		// as per KTH
			    nOldRPM = hole_speed;
			}

			MY_TRACE(pfLog, "Retract point at (%lf, %lf, %lf)\n", ptEnd[X], ptEnd[Y], ptEnd[Z]);
			// should already be here for first point
//							apt->GoToXYZ(cmnt, ptEnd[X], ptEnd[Y], ptEnd[Z]);

			apt->Feedrate(hole_feed);
			bInRapidMode = FALSE;
			apt->GoToXYZ(_com_util::ConvertStringToBSTR (cmnt), ptBot[X], ptBot[Y], ptBot[Z]);
			MY_TRACE(pfLog, "Bottom point at (%lf, %lf, %lf)\n", ptBot[X], ptBot[Y], ptBot[Z]);

			apt->Feedrate(hole_retract);
			apt->GoToXYZ(_com_util::ConvertStringToBSTR (cmnt), ptEnd[X], ptEnd[Y], ptEnd[Z]);
			MY_TRACE(pfLog, "Retract point at (%lf, %lf, %lf)\n", ptEnd[X], ptEnd[Y], ptEnd[Z]);

			MY_TRACE(pfLog, "Depth = %f, spindle = %f feed = %f, plunge = %f, retract = %f\n",
				 depth, hole_speed, hole_feed, hole_plunge, hole_retract);

		    }
		    else if (opl->opcode == TP_CIRCMILL && cc3 && n.gcode == 0)
		    {
			// this code worked much better here than up above with other features
			// consider moving them all to this logic PLEASE
			p_3d ptCenter;
			double dDiameter = opl->u.circmill.diameter;
			memcpy(ptCenter, n.u.m0.ep1, 3 * sizeof(double));

			double dDepth;
			if (opl->cmn.depth_inc)
			{
			    dDepth = fabs (opl->cmn.depth);
			    ptCenter[Z] = opl->cmn.top_stock -dDepth;
			}
			else
			{
			    dDepth = fabs(opl->cmn.top_stock - opl->cmn.depth);
			    ptCenter[Z] = opl->cmn.depth;
			}

			MY_TRACE(pfLog, "Circular milling found diameter = %f, depth = %f\n", dDiameter, dDepth);
			MY_TRACE(pfLog, "Center = (%f, %f, %f)\n", ptCenter[0], ptCenter[1], ptCenter[2]);
			MakeRoundPocket (*opl, ptCenter, dDiameter, dDepth, apt->GetCurrentWorkingstep());
		    }


		    // Data now gathered for new operation
		    if (n.op_idn != nOldOpID && n.gcode < 1000)
			nOldOpID = n.op_idn;

// --------------------------------------------------------
// BEGIN toolpath extraction
// --------------------------------------------------------

		    if (n.gcode == 1027)
		    {
			MY_TRACE(pfLog, "Origin found (%f, %f, %f)\n", n.u.m1027.wcs_origin[0], n.u.m1027.wcs_origin[1], n.u.m1027.wcs_origin[2]);
			MY_TRACE(pfLog, "X found (%f, %f, %f)\n", n.u.m1027.wcs_m[0][0], n.u.m1027.wcs_m[0][1], n.u.m1027.wcs_m[0][2]);
			MY_TRACE(pfLog, "Y found (%f, %f, %f)\n", n.u.m1027.wcs_m[1][0], n.u.m1027.wcs_m[1][1], n.u.m1027.wcs_m[1][2]);
			MY_TRACE(pfLog, "Z found (%f, %f, %f)\n", n.u.m1027.wcs_m[2][0], n.u.m1027.wcs_m[2][1], n.u.m1027.wcs_m[2][2]);
		    }

		    // Now act based on the operaton type (G-code)
		    if (n.gcode == 0 && cc1)  // rapid
		    {

			if (nOldRPM != opl->tl.rpm)   // as per KTH need to set spindle speed even for rapid
			{
			    if (tool_direction[opl->tl.tlno])
				apt->SpindleSpeed(opl->tl.rpm);
			    else
				apt->SpindleSpeed(-opl->tl.rpm);
			    nOldRPM = opl->tl.rpm;
			}

			if (n.u.m0.ccomp > 0)
			{
			    bNewOpCheckCComp = FALSE;

			    if (n.u.m0.ccomp != 0 && cc1)
			    {
				if (n.u.m0.ccomp == 41)
				{
				    MY_TRACE(pfLog, "Left Cutter Contact\n");
				    apt->Left();
				}
				else if (n.u.m0.ccomp == 42)
				{
				    MY_TRACE(pfLog, "Right Cutter Contact\n");
				    apt->Right();
				}
				else if (n.u.m0.ccomp == COMP_CANCEL)
				{
				    MY_TRACE(pfLog, "Cutter Contact Off\n");
				    apt->CenterOn();
				}
				else
				    MY_TRACE(pfLog, "Cutter Contact type unknown\n");

				MY_TRACE(pfLog, "Cutter compensation contact G%d (in control)\n", n.u.m0.ccomp);
			    }
			    else if (cc1)
			    {
				apt->CenterOn();
				MY_TRACE(pfLog, "Cutter compensation center (in computer)\n");
			    }

			    nPrevCComp = n.u.m0.ccomp;
			}

			if (opl->opcode == TP_POINT)  // Assume it's probing (NIST used this type of op)
			{
			    MY_TRACE(pfLog, "Warning TP Point found and ignored\n");
			}
			else   // used to guard this with code to stop if drilling
			{

			    if (!bInRapidMode)
			    {
				apt->MultaxOff();

				MY_TRACE(pfLog, "OP %d, Rapid, multax_off, oldcode = %d, newcode = %d\n", n.op_idn, nOldGcode, n.gcode);

				apt->Rapid();
				bInRapidMode = TRUE;
			    }
			    // Geometry
			    // calculate the orientation of use the toolpath orientation defined in STEP-NC?
			    if (!use_tp_orientation)
			    {
				xform_pt(n.u.m0.ep1, &mXform1);
				xform_pt(n.u.m0.ep1, &mXform2);
			    }

			    if (VEq(vecDir, vecOldDir))
			    {
				if (bFirstMove)
				{
				    apt->FirstPathStartPoint (n.u.m0.ep1[0], n.u.m0.ep1[1], n.u.m0.ep1[2]);
				    bFirstMove = FALSE;
				}
				// need to generate a G0 code to the first point
				apt->GoToXYZ(_com_util::ConvertStringToBSTR (cmnt), n.u.m0.ep1[0], n.u.m0.ep1[1], n.u.m0.ep1[2]);
			    }
			    else
			    {
				apt->MultaxOn();
				if (bFirstMove)
				{
				    apt->FirstPathStartPoint (n.u.m0.ep1[0], n.u.m0.ep1[1], n.u.m0.ep1[2]);
				    apt->FirstPathStartAxis (vecDir[0], vecDir[1], vecDir[2]);
				    bFirstMove = FALSE;
				}
				apt->GoToXYZ_IJK(_com_util::ConvertStringToBSTR (cmnt), n.u.m0.ep1[0], n.u.m0.ep1[1], n.u.m0.ep1[2],
						 vecDir[0], vecDir[1], vecDir[2]);
				apt->MultaxOff();
				//				memcpy(vecOldDir, vecDir, 3 * sizeof(double));
			    }

			    MY_TRACE(pfLog, "S%d, F%f\n", opl->tl.rpm, n.u.m0.feed);
			    MY_TRACE(pfLog, "G0 X%f Y%f Z%f I%f J%f K%f\n", n.u.m0.ep1[0], n.u.m0.ep1[1], n.u.m0.ep1[2], vecDir[0], vecDir[1], vecDir[2]);
			    dOldFeed = fabs(n.u.m0.feed);
			}

			memcpy(ptStart, n.u.m0.ep1, 3 * sizeof(double));
		    }
		    else if (n.gcode == 1 && cc1)  // line feed
		    {
			bInRapidMode = FALSE;

			if (n.gcode != nOldGcode)
			{
			    MY_TRACE(pfLog, "Old code %d, new code %d multax_off\n", nOldGcode, n.gcode);
			    apt->MultaxOff();
			}

			if (nOldRPM != opl->tl.rpm)
			{
			    if (tool_direction[opl->tl.tlno])
				apt->SpindleSpeed(opl->tl.rpm);
			    else
				apt->SpindleSpeed(-opl->tl.rpm);
			    nOldRPM = opl->tl.rpm;
			}

			if (dOldFeed != n.u.m1.feed)
			{
			    if (opl->tl.use_css)
			    {
				apt->FeedrateCSS (fabs (n.u.m1.feed), opl->tl.max_ss);
				MY_TRACE(pfLog, "apt - css feed %f max %d\n", fabs(n.u.m1.feed), opl->tl.max_ss);
			    }
			    else
			    {
				apt->Feedrate(fabs (n.u.m1.feed));
				MY_TRACE(pfLog, "apt - feed %f\n", fabs(n.u.m1.feed));
			    }
			    dOldFeed = n.u.m1.feed;
			}

			// Cutter compensation (0 => control)
			if (n.u.m1.ccomp > 0)
			{
			    bNewOpCheckCComp = FALSE;

			    if (n.u.m1.ccomp != 0)
			    {
				if (n.u.m1.ccomp == 41)
				{
				    MY_TRACE(pfLog, "Cutter Contact Left\n");
				    apt->Left();
				}
				else if (n.u.m1.ccomp == 42)
				{
				    MY_TRACE(pfLog, "Cutter Contact Right\n");
				    apt->Right();
				}
				else if (n.u.m1.ccomp == COMP_CANCEL)
				{
				    MY_TRACE(pfLog, "Cutter Contact Off\n");
				    apt->CenterOn();
				}
				else
				    MY_TRACE(pfLog, "Cutter Contact unknown\n");

				MY_TRACE(pfLog, "Cutter compensation contact G%d (in control)\n", n.u.m1.ccomp);
			    }
			    else
			    {
				apt->CenterOn();
				MY_TRACE(pfLog, "Cutter compensation center (in computer)\n");
			    }

			    nPrevCComp = n.u.m1.ccomp;
			}

			// Geometry
			// calculate the orientation of use the toolpath orientation defined in STEP-NC?
			if (!use_tp_orientation)
			{
			    xform_pt(n.u.m1.ep1, &mXform1);
			    xform_pt(n.u.m1.ep1, &mXform2);
			}

			if (VEq(vecDir, vecOldDir))
			{
			    if (bFirstMove)
			    {
				apt->FirstPathStartPoint (n.u.m0.ep1[0], n.u.m0.ep1[1], n.u.m0.ep1[2]);
				bFirstMove = FALSE;
			    }
			    apt->GoToXYZ(_com_util::ConvertStringToBSTR (cmnt), n.u.m1.ep1[0], n.u.m1.ep1[1], n.u.m1.ep1[2]);
			}
			else
			{
			    apt->MultaxOn();
			    if (bFirstMove)
			    {
				apt->FirstPathStartPoint (n.u.m0.ep1[0], n.u.m0.ep1[1], n.u.m0.ep1[2]);
				apt->FirstPathStartAxis (vecDir[0], vecDir[1], vecDir[2]);
				bFirstMove = FALSE;
			    }

			    apt->GoToXYZ_IJK(_com_util::ConvertStringToBSTR (cmnt), n.u.m1.ep1[0], n.u.m1.ep1[1], n.u.m1.ep1[2],
					     vecDir[0], vecDir[1], vecDir[2]);
			    apt->MultaxOff();
			    //			    memcpy(vecOldDir, vecDir, 3 * sizeof(double));
			}

			MY_TRACE(pfLog, "Comp = %d\n", n.u.m1.ccomp);
			MY_TRACE(pfLog, "S%d, F%f\n", opl->tl.rpm, n.u.m1.feed);
			MY_TRACE(pfLog, "G1 X%f Y%f Z%f I%f J%f K%f\n", n.u.m1.ep1[0], n.u.m1.ep1[1], n.u.m1.ep1[2], vecDir[0], vecDir[1], vecDir[2]);
			memcpy(ptStart, n.u.m1.ep1, 3 * sizeof(double));
		    }
		    else if ((n.gcode == 2 || n.gcode == 3) && cc1)   // CW/CCW arc
		    {
			bInRapidMode = FALSE;

			if (n.gcode != nOldGcode)
			{
			    MY_TRACE(pfLog, "Old code %d, new code %d multax_off\n", nOldGcode, n.gcode);
			    apt->MultaxOff();
			}

			if (nOldRPM != opl->tl.rpm)
			{
			    if (tool_direction[opl->tl.tlno])
				apt->SpindleSpeed(opl->tl.rpm);
			    else
				apt->SpindleSpeed(-opl->tl.rpm);
			    nOldRPM = opl->tl.rpm;
			}

			if (dOldFeed != n.u.m2.feed)
			{
			    if (opl->tl.use_css)
			    {
				apt->FeedrateCSS (fabs (n.u.m2.feed), opl->tl.max_ss);
				MY_TRACE(pfLog, "apt - css feed %f max %d\n", fabs(n.u.m2.feed), opl->tl.max_ss);
			    }
			    else
			    {
				apt->Feedrate(fabs (n.u.m2.feed));
				MY_TRACE(pfLog, "apt - feed %f\n", fabs(n.u.m2.feed));
			    }
			    dOldFeed = n.u.m2.feed;
			}

			// Cutter compensation (change only if it's new operation or "CANCEL"
			if (n.u.m2.ccomp > 0)  //bNewOpCheckCComp || (n.u.m2.ccomp == COMP_CANCEL))
			{
			    bNewOpCheckCComp = FALSE;

			    if (n.u.m2.ccomp != COMP_OFF)
			    {
				if (n.u.m2.ccomp == COMP_LEFT)
				{
				    MY_TRACE(pfLog, "Cutter contact left\n");
				    apt->Left();
				}
				else if (n.u.m2.ccomp == COMP_RIGHT)
				{
				    MY_TRACE(pfLog, "Cutter contact right\n");
				    apt->Right();
				}
				else if (n.u.m2.ccomp == COMP_CANCEL)
				{
				    MY_TRACE(pfLog, "Cutter contact off\n");
				    apt->CenterOn();
				}
				else
				    MY_TRACE(pfLog, "Cutter Contact unknown\n");

				MY_TRACE(pfLog, "Cutter compensation contact G%d (in control)\n", n.u.m2.ccomp);
			    }
			    else
			    {
				apt->CenterOn();
				MY_TRACE(pfLog, "Cutter compensation center (in computer)\n");
			    }

			    nPrevCComp = n.u.m2.ccomp;
			}

			// Geometry
			// calculate the orientation of use the toolpath orientation defined in STEP-NC?
			if (!use_tp_orientation)
			{
			    xform_pt(n.u.m2.ep1, &mXform1);
			    xform_pt(n.u.m2.ep1, &mXform2);
			    xform_pt(n.u.m2.cpt, &mXform1);
			    xform_pt(n.u.m2.cpt, &mXform2);
			}

			dRad = sqrt(pow(n.u.m2.ep1[0] - n.u.m2.cpt[0], 2) +
				    pow(n.u.m2.ep1[1] - n.u.m2.cpt[1], 2) +
				    pow(n.u.m2.ep1[2] - n.u.m2.cpt[2], 2));

			MY_TRACE(pfLog, "Comp = %d Plane = %d circle = %d position = %d\n", n.u.m2.ccomp, n.u.m2.pln, n.u.m2.circle, n.u.m2.position);
			MY_TRACE(pfLog, "S%d, F%f\n", opl->tl.rpm, n.u.m2.feed);
			MY_TRACE(pfLog, "%s X%f Y%f Z%f CENTER(X%f Y%f Z%f) R%f\n", n.gcode == 2 ? "G2" : "G3", n.u.m2.ep1[0], n.u.m2.ep1[1], n.u.m2.ep1[2], n.u.m2.cpt[0], n.u.m2.cpt[1], n.u.m2.cpt[2], dRad);

//						if (n.u.m2.circle)
//							AfxMessageBox("Mastercam FULL CIRCLE found", MB_YESNO | MB_ICONINFORMATION);

			if (n.u.m2.pln == 0)
			{
			    int ccw;
			    // if arc is in plane then simple calculation
			    if (use_tp_orientation)
			    {
				if (n.gcode == 3)
				    ccw = 1;
				else
				    ccw = 0;
			    }
			    else  	// else need to worry about current normal
			    {
				if (n.gcode == 3 && vecDir[2] == 1)
				    ccw = 1;
				else if (n.gcode == 2 && vecDir[2] == -1)
				    ccw = 1;
				else
				    ccw = 0;
			    }

			    apt->ArcXYPlane(_com_util::ConvertStringToBSTR (cmnt), n.u.m2.ep1[0], n.u.m2.ep1[1], n.u.m2.ep1[2],
					    n.u.m2.cpt[0], n.u.m2.cpt[1], n.u.m2.cpt[2],
					    dRad, ccw);
			    MY_TRACE(pfLog, "(XY) CENTER AXIS \n");
			}
			else if (n.u.m2.pln == 1)
			{
			    int ccw;
			    if (use_tp_orientation)
			    {
				if (n.gcode == 3)
				    ccw = 1;
				else
				    ccw = 0;
			    }
			    else
			    {
				if (n.gcode == 3 && vecDir[0] == 1)
				    ccw = 1;
				else if (n.gcode == 2 && vecDir[0] == -1)
				    ccw = 1;
				else
				    ccw = 0;
			    }

			    apt->ArcYZPlane(_com_util::ConvertStringToBSTR (cmnt), n.u.m2.cpt[2], n.u.m2.ep1[0], n.u.m2.ep1[1],
					    n.u.m2.cpt[2], n.u.m2.cpt[0], n.u.m2.cpt[1],
					    dRad, ccw);
			    /*							apt->ArcYZPlane(cmnt, n.u.m2.ep1[0], n.u.m2.ep1[1], n.u.m2.ep1[2],
			    							n.u.m2.cpt[0], n.u.m2.cpt[1], n.u.m2.cpt[2],
			    							dRad, n.gcode == 3);*/
			    MY_TRACE(pfLog, "(YZ) CENTER AXIS\n");
			}
			else if (n.u.m2.pln == 2)
			{
			    int ccw;
			    if (use_tp_orientation)
			    {
				if (n.gcode == 3)
				    ccw = 1;
				else
				    ccw = 0;
			    }
			    else
			    {
				if (n.gcode == 3 && vecDir[1] == 1)
				    ccw = 1;
				else if (n.gcode == 2 && vecDir[1] == -1)
				    ccw = 1;
				else
				    ccw = 0;
			    }

			    apt->ArcZXPlane(_com_util::ConvertStringToBSTR (cmnt), n.u.m2.ep1[0], -n.u.m2.cpt[2], n.u.m2.ep1[1],
					    n.u.m2.cpt[0], -n.u.m2.cpt[2], n.u.m2.cpt[1],
					    dRad, ccw);
			    /*							apt->ArcZXPlane(cmnt, n.u.m2.ep1[0], n.u.m2.ep1[1], n.u.m2.ep1[2],
			    							n.u.m2.cpt[0], n.u.m2.cpt[1], n.u.m2.cpt[2],
			    							dRad, n.gcode == 3);*/
			    MY_TRACE(pfLog, "(ZX) CENTER AXIS\n");
			}
			else
			{
			    CString msg;
			    msg.Format ("Unknown arc plane value = %d", n.u.m2.pln);
			    AfxMessageBox(msg, MB_YESNO | MB_ICONINFORMATION);//
			}

			dOldFeed = n.u.m2.feed;
		    }
		    else if (n.gcode == 11 && cc1)  // 5-axis move
		    {
			bInRapidMode = FALSE;

			if (nOldGcode != n.gcode)
			{
			    MY_TRACE(pfLog, "Old code %d, new code %d\n", nOldGcode, n.gcode);
			    apt->MultaxOn();
			}

			if (nOldRPM != opl->tl.rpm)
			{
			    if (tool_direction[opl->tl.tlno])
				apt->SpindleSpeed(opl->tl.rpm);
			    else
				apt->SpindleSpeed(-opl->tl.rpm);
			    nOldRPM = opl->tl.rpm;
			}

			if (dOldFeed != fabs(n.u.m11.feed))
			{
			    if (opl->tl.use_css)
			    {
				apt->FeedrateCSS (fabs (n.u.m11.feed), opl->tl.max_ss);
				MY_TRACE(pfLog, "apt - css feed %f max %d\n", fabs(n.u.m11.feed), opl->tl.max_ss);
			    }
			    else
			    {
				apt->Feedrate(fabs (n.u.m11.feed));
				MY_TRACE(pfLog, "apt - feed %f\n", fabs(n.u.m11.feed));
			    }
			    dOldFeed = fabs(n.u.m11.feed);
			}

			// Geometry
			apt->GoToXYZ_IJK(_com_util::ConvertStringToBSTR (cmnt), n.u.m11.ep1[0], n.u.m11.ep1[1], n.u.m11.ep1[2],
					 n.u.m11.snvec[0], n.u.m11.snvec[1], n.u.m11.snvec[2]);

			MY_TRACE(pfLog, "S%d, F%f\n", opl->tl.rpm, n.u.m11.feed);
			MY_TRACE(pfLog, "G11 X%f Y%f Z%f I%f J%f K%f\n", n.u.m11.ep1[0], n.u.m11.ep1[1], n.u.m11.ep1[2],n.u.m11.snvec[0], n.u.m11.snvec[1], n.u.m11.snvec[2]);

			dOldFeed = fabs(n.u.m11.feed);
		    }
		    else if ((n.gcode == 81 || n.gcode == 82 || n.gcode == 83) && cc3)  // Drilling canned cycle
		    {
			// implement as TP_DRILL operation
			MY_TRACE(pfLog, "G%d not implemented S%d, F%f\n", n.gcode, opl->tl.rpm, n.u.m11.feed);
			;// Not implemented yet
		    }
		    else if (n.gcode == 100)  // Drilling position
		    {
			if (cc1)  // Add goto code for Drill to bottom
			{
			    ptStart[X] = n.u.m100.x; ptStart[Y] = n.u.m100.y; ptStart[Z] = hole_z1;
			    ptEnd[X] = n.u.m100.x; ptEnd[Y] = n.u.m100.y; ptEnd[Z] = hole_z2;
			    ptBot[X] = n.u.m100.x; ptBot[Y] = n.u.m100.y; ptBot[Z] = hole_z3;

			    // calculate the orientation of use the toolpath orientation defined in STEP-NC?
			    if (!use_tp_orientation)
			    {
				xform_pt(ptStart, &mXform1);
				xform_pt(ptEnd, &mXform1);
				xform_pt(ptBot, &mXform1);

				xform_pt(ptStart, &mXform2);
				xform_pt(ptEnd, &mXform2);
				xform_pt(ptBot, &mXform2);
			    }

			    if (hole_speed != nOldRPM)
			    {
				apt->SpindleSpeed (hole_speed);
				nOldRPM = hole_speed;
			    }

			    if (!bInRapidMode)
			    {
				apt->Rapid();
				bInRapidMode = TRUE;
			    }

			    MY_TRACE(pfLog, "GOTO Drilling Next point at (%lf, %lf, %lf)\n", ptEnd[X], ptEnd[Y], ptEnd[Z]);

			    // need this for second and subsequent but not first
			    apt->GoToXYZ(_com_util::ConvertStringToBSTR (cmnt), ptEnd[X], ptEnd[Y], ptEnd[Z]);

			    apt->Feedrate(hole_feed);
			    apt->GoToXYZ(_com_util::ConvertStringToBSTR (cmnt), ptBot[X], ptBot[Y], ptBot[Z]);
			    MY_TRACE(pfLog, "Bot point at (%lf, %lf, %lf)\n", ptBot[X], ptBot[Y], ptBot[Z]);

			    apt->Feedrate(hole_retract);
			    apt->GoToXYZ(_com_util::ConvertStringToBSTR (cmnt), ptEnd[X], ptEnd[Y], ptEnd[Z]);

			    bInRapidMode = FALSE;
			    MY_TRACE(pfLog, "spindle = %f feed = %f, plunge = %f, retract = %f\n",
				     hole_speed, hole_feed, hole_plunge, hole_retract);
			}
			if (cc3)
			{
			    if (pattern || !pattern)
			    {
				MY_TRACE(pfLog, "New pattern drilling point at (%lf, %lf, %lf)\n",  ptBot[X], ptBot[Y], ptBot[Z]);
				process->DrillPointAdd (apt->GetCurrentWorkingstep(),  ptBot[X], ptBot[Y], ptBot[Z]);
				MY_TRACE(pfLog, "New pattern drilling Made it\n");
			    }
			    else
			    {
				MY_TRACE(pfLog, "New drilling Workingstep at (%lf, %lf, %lf)\n",  ptBot[X], ptBot[Y], ptBot[Z]);
				process->DrillWorkingstepAdd (apt->GetCurrentWorkingstep(),   ptBot[X], ptBot[Y], ptBot[Z]);
				MY_TRACE(pfLog, "New drilling Made it\n");
			    }
			}

		    }
		    else if (n.gcode == 1014)  	// Toolplane change (axis rotation)
		    {
			memcpy(mXform2.r, n.u.m1014.tpln_33, 9 * sizeof(double));
			memcpy(vecDir, n.u.m1014.tpln_33[2], 3 * sizeof(double));
			memcpy(vecRef, n.u.m1014.tpln_33[0], 3 * sizeof(double));
			MY_TRACE(pfLog, "tool plane (%lf, %lf, %lf) ref (%lf, %lf, %lf)\n", vecDir[X], vecDir[Y], vecDir[Z],
				 vecRef[X], vecRef[Y], vecRef[Z]);
			// Define orientation in STEP-NC
			if (use_tp_orientation)
			{
			    MY_TRACE(pfLog, "setting ws_toolpath_orientation to (0, 0, 0) dir (%lf, %lf, %lf) ref (%lf, %lf, %lf)\n", vecDir[X], vecDir[Y], vecDir[Z],
				     vecRef[X], vecRef[Y], vecRef[Z]);
			    apt->WorkingstepToolpathOrientation (0, 0, 0, vecDir[X], vecDir[Y], vecDir[Z], vecRef[X], vecRef[Y], vecRef[Z]);

			    // tool plane change is not a reason to go to 5 axis machining
			    memcpy(vecOldDir, vecDir, 3 * sizeof(double));
			}
			else
			    bAxisChgd = TRUE;

		    }
		    else if (n.gcode == 1027)  	// WCS change (axis rotation)
		    {
			memcpy(mXform1.t, n.u.m1027.wcs_origin, 3 * sizeof(double));
			mXform1.r[0][0] = mXform1.r[1][1] = mXform1.r[2][2] = 1;

			MY_TRACE(pfLog, "wcs move (%lf, %lf, %lf)\n", mXform1.t[0], mXform1.t[1], mXform1.t[2]);
			if (use_tp_orientation)
			{
			    p_3d pt_origin;
			    memcpy (pt_origin, n.u.m1027.wcs_origin, 3 * sizeof(double));
			    xform_pt(pt_origin, &mXform2);
			    MY_TRACE(pfLog, "setting ws_toolpath_orientation to (%lf, %lf, %lf) dir (%lf, %lf, %lf) ref (%lf, %lf, %lf)\n",
				     pt_origin[X], pt_origin[Y], pt_origin[Z],
				     vecDir[X], vecDir[Y], vecDir[Z],
				     vecRef[X], vecRef[Y], vecRef[Z]);
			    apt->WorkingstepToolpathOrientation (pt_origin[X], pt_origin[Y], pt_origin[Z], vecDir[X], vecDir[Y], vecDir[Z], vecRef[X], vecRef[Y], vecRef[Z]);
			}
		    }

// --------------------------------------------------------
// END toolpath extraction
// --------------------------------------------------------
		    nOldOpID = n.op_idn;
		}

		nOldGcode = n.gcode;
#ifdef MY_DEBUG
		fflush(pfLog);
#endif

		// Move to the next op
//			operation_manager(&opo, OPMGR_GET_NEXT_LIST, &dop, &bResult);

	    }
#ifdef MY_DEBUG
	    fflush(pfLog);
#endif
	}

#ifdef MY_DEBUG
	fflush(pfLog);
#endif

//	    dlgProg.DestroyWindow();
	apt->CoolantOff();

//	    long res = AfxMessageBox("Export complete. Run Configuration File?", MB_YESNO | MB_ICONINFORMATION);

	MY_TRACE(pfLog, "Saving data in %s\n", dlgSave.GetPathName());
	apt->SaveAsModules(_com_util::ConvertStringToBSTR (dlgSave.GetPathName()));
//	    apt->SaveFastAsModules(dlgSave.GetPathName());

	AfxMessageBox("Mastercam STEP-NC export complete", MB_OK);
	MY_TRACE(pfLog, "Mastercam STEP-NC E\export complete\n");

	MY_TRACE(pfLog, "Shutting down\n");
	feature->Reset();
	process->Reset();
	apt->Reset();

    }
    /*	else
    	{
    	AfxMessageBox("Nothing to export.", MB_OK | MB_ICONINFORMATION);
    	}*/
    catch (...)
    {
	//	dlgProg.DestroyWindow();
	feature->Reset();
	process->Reset();
	apt->Shutdown();
	//	ShowMessage("Can't export toolpaths.", MSG_ERROR);
	AfxMessageBox("Can't export toolpaths.", MB_OK | MB_ICONSTOP);
    }

#ifdef MY_DEBUG
    fclose(pfLog);
#endif

    return;
}

// load all the tools
void LoadAllTools()
{
    tp_tool tl;
    MC_BOOL bResult = true;
    boolean foundOne = false;		// used to decide if need to search lathe tools
    DB_LIST_ENT_PTR ptrTool;
//	db_ptr_type ptrTool;
    int i;
    CString strToolName;
    CString a; // eliminate leading ' '

    // Enumerate tools
    for (i = 1; i < 256; i++)
    {
	memset(&tl, 0, sizeof(tp_tool));

	tl.op.slot = i;
	tool_manager(&tl, TLMGR_GET, &ptrTool, &bResult);

	if (!bResult)
	{
	    continue;
	}
	else
	{
	    foundOne = TRUE;
	}

	tool_direction[i] = tl.spindle_rot;

	/*		if (tl.op.comment[0] == ' ')
			a = tl.op.comment+1;
			else
			a = tl.op.comment;*/

	if (tl.op.type == 10 || tl.op.type == 19 || tl.op.type == 11 )   // end mill flat | bullnose | ball
	{
	    apt->DefineToolEndmill (tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length, tl.op.n_flutes, 0);
	    apt->SELCTLTool(tl.op.tlno);
	    MY_TRACE(pfLog, "\nSaved Endmill Tool %d MCAM id = %d with identifier %s and type = %d\n", tl.op.slot, tl.op.tlno, tl.op.comment, tl.op.type);
	    MY_TRACE(pfLog, "Diameter = %f, Overall length = %f, functional length = %f, flute length = %f\n", tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length);
	    MY_TRACE(pfLog, "Num flutes = %d Taper angle (saved as 0) = %f\n", tl.op.n_flutes, tl.op.tip_angle);
	    char buf[30];
	    char buf2[30];
	    sprintf (buf, "%d", tl.op.tlno);
	    if (tl.op.type == 10)
	    {
		sprintf (buf2, "Endmill-%d", tl.op.tlno);
		apt->SetToolIdentifier (_com_util::ConvertStringToBSTR (buf), buf2);
	    }
	    else if (tl.op.type == 19)
	    {
		sprintf (buf2, "Bullnose-%d", tl.op.tlno);
		apt->SetToolIdentifier (_com_util::ConvertStringToBSTR (buf), buf2);
	    }
	    else
	    {
		sprintf (buf2, "Ball-%d", tl.op.tlno);
		apt->SetToolIdentifier (_com_util::ConvertStringToBSTR (buf), buf2);
	    }
	}
	else if (tl.op.type == 12)   // chamfer tool (tool tip angle not 0)
	{
	    apt->DefineToolEndmill (tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length, tl.op.n_flutes, tl.op.tip_angle);
	    apt->SELCTLTool(tl.op.tlno);
	    MY_TRACE(pfLog, "\nSaved Chamfer Endmill Tool %d MCAM id = %d with identifier %s and type = %d\n", tl.op.slot, tl.op.tlno, tl.op.comment, tl.op.type);
	    MY_TRACE(pfLog, "Diameter = %f, Overall length = %f, functional length = %f, flute length = %f\n", tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length);
	    MY_TRACE(pfLog, "Num flutes = %d Taper angle = %f\n", tl.op.n_flutes, tl.op.tip_angle);
	    char buf[30];
	    char buf2[30];
	    sprintf (buf, "%d", tl.op.tlno);
	    sprintf (buf2, "Chamfer-%d", tl.op.tlno);
	    apt->SetToolIdentifier (_com_util::ConvertStringToBSTR (buf), buf2);
	}
	else if (tl.op.type == 13)
	{
	    apt->DefineToolFacemill (tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length, tl.op.n_flutes, tl.op.tip_angle);
	    apt->SELCTLTool(tl.op.tlno);
	    MY_TRACE(pfLog, "\nSaved Facemill Tool %d MCAM id = %d with identifier %s and type = %d\n", tl.op.slot, tl.op.tlno, tl.op.comment, tl.op.type);
	    MY_TRACE(pfLog, "Diameter = %f, Overall length = %f, functional length = %f, flute length = %f\n", tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length);
	    MY_TRACE(pfLog, "Num flutes = %d Taper angle = %f\n", tl.op.n_flutes, tl.op.tip_angle);
	    char buf[30];
	    char buf2[30];
	    sprintf (buf, "%d", tl.op.tlno);
	    sprintf (buf2, "Facemill-%d", tl.op.tlno);
	    apt->SetToolIdentifier (_com_util::ConvertStringToBSTR (buf), buf2);
	}
	else if (tl.op.type == 2)
	{
	    apt->DefineToolSpotDrill(tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length, tl.op.tip_angle);
	    apt->SELCTLTool(tl.op.tlno);
	    MY_TRACE(pfLog, "\nSaved Spot Drill Tool %d MCAM id = %d with identifier %s and type = %d\n", tl.op.slot, tl.op.tlno, tl.op.comment, tl.op.type);
	    MY_TRACE(pfLog, "Diameter = %f, Overall length = %f, functional length = %f, flute length = %f\n", tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length);
	    MY_TRACE(pfLog, "tip angle = %f\n", tl.op.tip_angle);
	    char buf[30];
	    char buf2[30];
	    sprintf (buf, "%d", tl.op.tlno);
	    sprintf (buf2, "Spotdrill-%d", tl.op.tlno);
	    apt->SetToolIdentifier (_com_util::ConvertStringToBSTR (buf), buf2);
	}
	else if (tl.op.type == 3)
	{
	    apt->DefineToolDrill(tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length, tl.op.tip_angle);
	    apt->SELCTLTool(tl.op.tlno);
	    MY_TRACE(pfLog, "\nSaved (Twist) Drill Tool %d MCAM id = %d with identifier %s and type = %d\n", tl.op.slot, tl.op.tlno, tl.op.comment, tl.op.type);
	    MY_TRACE(pfLog, "Diameter = %f, Overall length = %f, functional length = %f, flute length = %f\n", tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length);
	    MY_TRACE(pfLog, "tip angle = %f\n", tl.op.tip_angle);
	    char buf[30];
	    char buf2[30];
	    sprintf (buf, "%d", tl.op.tlno);
	    sprintf (buf2, "Drill-%d", tl.op.tlno);
	    apt->SetToolIdentifier (_com_util::ConvertStringToBSTR (buf), buf2);
	}
	else if (tl.op.type == 4)
	{
	    apt->DefineToolTapping(tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length, "metric", tl.op.thds, tl.op.dia -tl.op.thds);
	    apt->DefineToolHandOfCut ("right");
	    apt->SELCTLTool(tl.op.tlno);
	    MY_TRACE(pfLog, "\nSaved Tapping RH Tool %d MCAM id = %d with identifier %s and type = %d\n", tl.op.slot, tl.op.tlno, tl.op.comment, tl.op.type);
	    MY_TRACE(pfLog, "Diameter = %f, Overall length = %f, functional length = %f, flute length = %f\n", tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length);
	    MY_TRACE(pfLog, "thread_pitch = %f\n", tl.op.thds);
	    char buf[30];
	    char buf2[30];
	    sprintf (buf, "%d", tl.op.tlno);
	    sprintf (buf2, "Right_tap-%d", tl.op.tlno);
	    apt->SetToolIdentifier (_com_util::ConvertStringToBSTR (buf), buf2);
	}
	else if (tl.op.type == 5)
	{
	    apt->DefineToolTapping(tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length, "metric", tl.op.thds, tl.op.dia -tl.op.thds);
	    apt->DefineToolHandOfCut ("left");
	    apt->SELCTLTool(tl.op.tlno);
	    MY_TRACE(pfLog, "\nSaved Tapping LH Tool %d MCAM id = %d with identifier %s and type = %d\n", tl.op.slot, tl.op.tlno, tl.op.comment, tl.op.type);
	    MY_TRACE(pfLog, "Diameter = %f, Overall length = %f, functional length = %f, flute length = %f\n", tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length);
	    MY_TRACE(pfLog, "thread_pitch = %f\n", tl.op.thds);
	    char buf[30];
	    char buf2[30];
	    sprintf (buf, "%d", tl.op.tlno);
	    sprintf (buf2, "Left_tap-%d", tl.op.tlno);
	    apt->SetToolIdentifier (_com_util::ConvertStringToBSTR (buf), buf2);
	}
	else if (tl.op.type == 7)
	{
	    apt->DefineToolRotatingBoringCuttingTool(tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length);
	    apt->SELCTLTool(tl.op.tlno);
	    MY_TRACE(pfLog, "\nSaved Bore Tool %d MCAM id = %d with identifier %s and type = %d\n", tl.op.slot, tl.op.tlno, tl.op.comment, tl.op.type);
	    MY_TRACE(pfLog, "Diameter = %f, Overall length = %f, functional length = %f, flute length = %f\n", tl.op.dia, tl.holder_length + tl.oa_length, tl.oa_length, tl.flute_length);
	    char buf[30];
	    char buf2[30];
	    sprintf (buf, "%d", tl.op.tlno);
	    sprintf (buf2, "Boring-%d", tl.op.tlno);
	    apt->SetToolIdentifier (_com_util::ConvertStringToBSTR (buf), buf2);
	}
	else
	{
	    // as per note from magnus overall assembly length in MCam is holder length + oa_length
	    apt->DefineTool(tl.op.dia, tl.op.crad, 0, 0, 0, 0, tl.holder_length + tl.oa_length);
	    MY_TRACE(pfLog, "\nSaved APT Tool %d MCAM id = %d with identifier %s and type = %d and tip = %f\n", tl.op.slot, tl.op.tlno, tl.op.comment, tl.op.type, tl.op.tip_angle);
	}

	if (tl.op.rad_type && (tl.op.type == 10 || tl.op.type == 13 || tl.op.type == 19 || tl.op.type  == 11))
	{
	    apt->DefineToolRadius (tl.op.crad);
	    MY_TRACE(pfLog, "\nCorner radius = %f, rad_type = %d\n", tl.op.crad, tl.op.rad_type);
	}
	if (tl.op.matl == 1)
	{
	    apt->DefineToolMaterial ("Mastercam X4", "HSS");
	    MY_TRACE(pfLog, "Material code = %d  value = HSS ", tl.op.matl);
	}
	else if (tl.op.matl == 2)
	{
	    apt->DefineToolMaterial ("Mastercam X4", "Carbide");
	    MY_TRACE(pfLog, "Material code = %d  value = Carbide ", tl.op.matl);
	}
	else if (tl.op.matl == 3)
	{
	    apt->DefineToolMaterial ("Mastercam X4", "TiCoated");
	    MY_TRACE(pfLog, "Material code = %d  value = TiCoated ", tl.op.matl);
	}
	else if (tl.op.matl == 3)
	{
	    apt->DefineToolMaterial ("Mastercam X4", "Ceramic");
	    MY_TRACE(pfLog, "Material code = %d  value = Ceramic ", tl.op.matl);
	}
	else
	    MY_TRACE(pfLog, "Material code = %d  value = Unknown ", tl.op.matl);

	if (tl.op.feed != 0 || tl.op.rpm != 0)
	{
	    apt->DefineToolFeedAndSpeed (tl.op.feed, tl.op.rpm);
	    MY_TRACE(pfLog, "Feed = %f Speed = %d\n", tl.op.feed, tl.op.rpm);
	}
	else
	    MY_TRACE(pfLog, "No feed/speed recommendations set (Feed = %f Speed = %d)\n", tl.op.feed, tl.op.rpm);

//		long tool_id = apt->GetCurrentTool ();
//		process->SetToolLength (tool_id, tl.oa_length);
	if (tool_direction[i])
	    MY_TRACE(pfLog, "Tool is CCW\n");
	else
	    MY_TRACE(pfLog, "Tool is CW\n");

    }

    if (foundOne)
	return;

    // try lathe tools
    MY_TRACE(pfLog, "Trying lathe tools\n");

    LATHETOOL ltl;

    // Enumerate tools
    for (i = 1; i < 256; i++)
    {
	memset(&ltl, 0, sizeof(tp_tool));

	//	ltl.op.slot = i;
	ltl.uSlot = i;

	ltool_manager(&ltl, TLMGR_GET, &ptrTool, &bResult);

	if (!bResult)
	{
	    continue;
	}
	else
	{
	    foundOne = TRUE;
	}

	tool_direction[i] = ltl.fCCW;

	apt->DefineTool(0, 0, 0, 0, 0, 0, 0);
	apt->SELCTLTool(ltl.lToolNum);
	MY_TRACE(pfLog, "Found lathe tool in slot %d toolnum = %d\n", i, ltl.lToolNum);
	if (tool_direction[i])
	    MY_TRACE(pfLog, "Tool is CCW\n");
	else
	    MY_TRACE(pfLog, "Tool is CW\n");
    }

    if (foundOne)
	return;

    MY_TRACE(pfLog, "NO lathe tools\n");

}

// code given to use by Mastercam
// Retrieve the 'tree branches' of items above the Operation
void GetOpParents(operation *opl, CStringArray *names)
{
    names->RemoveAll();

    //Get the immediate parent (a Toolpath Group) of this Operation
    op_group *op_parent = TpMainGrpMgr.GetMainGrpList().GroupByID(opl->cmn.grp_idn);
    CString parent(op_parent->name);
    names->Add(op_parent->name);  // Add the name of the Toolpath Group

    // Get the Machine Group that contains this Toolpath Group
    op_group *op_mach = TpMainGrpMgr.GetMainGrpList().RetrieveRootMachineGroup(op_parent->grp_idn);

    int subLevel = 0;
    CString parentSub[6];
    long nextParent = op_parent->parent_grp_idn;
    while (nextParent != op_mach->grp_idn)
    {
	op_parent = TpMainGrpMgr.GetMainGrpList().GroupByID(nextParent);
	names->Add(op_parent->name); // Add the name(s) of any Toolpath Groups that we're nested under
	parentSub[subLevel++] = op_parent->name;
	nextParent = op_parent->parent_grp_idn;
    }
    names->Add(op_mach->name); // Add the Mchine Group name
}


void MakeRoundHole(operation op, p_3d ptCenter, _int64 ws_id)
{
    short  cycle = op.u.prm_drl.cycle;
    double peck1 = op.u.prm_drl.peck1;
    double peck2 = op.u.prm_drl.peck2;
    double breakthrough_amount = op.u.prm_drl.brk_thru;
    unsigned char through = op.u.prm_drl.drill_tip;
    double retract_pln = op.cmn.retract_pln;
    int retract_on = op.cmn.retract_on;

    // feed plane not used for drilling in MCAM

    double dwell = op.u.prm_drl.dwell;
    double retract_feed;
    if (op.tl.feed != 0)
	retract_feed = op.tl.retract / op.tl.feed;
    else
	retract_feed = 1;

    double retract_distance = op.u.prm_drl.peck_clr;
    double dwell_step = dwell;				// NO VALUE IN MASTERCAM?

    if (peck2 == 0 && peck1 != 0)
	peck2 = peck1;

    double tip_angle = op.tl.tip_angle;
    double tip_radius = op.tl.crad;

    CString cmnt;
    cmnt.Format ("%s", op.comment);

    _int64 hole_id;
    double hole_depth, drill_depth;

    ws_id = apt->GetCurrentWorkingstep();

    MY_TRACE(pfLog, "MakeFeature: Round Hole for %s ws_id = %d retract = %f retract_on = %d retract_inc = %d\n", op.comment, ws_id, retract_pln, retract_on, op.cmn.retract_inc);
    MY_TRACE(pfLog, "Breakthrough = %f Tool tip angle = %f Tool tip radius = %f\n", breakthrough_amount, tip_angle, tip_radius);

    feature->SetDirection(-op.tpln.m[2][0], -op.tpln.m[2][1], -op.tpln.m[2][2],
			  op.tpln.m[0][0], op.tpln.m[0][1], op.tpln.m[0][2]);

    if (op.cmn.top_stock_inc == TRUE)
    {
	MY_TRACE(pfLog, "MakeFeature: Round Hole WARNING top of stock incremental\n");
//		AfxMessageBox("Incremental Top of Stock.");
    }

    if (op.cmn.depth_inc == TRUE)
	hole_depth = fabs (op.cmn.depth);
    else
	hole_depth = fabs (op.cmn.depth - op.cmn.top_stock);

    if (op.cmn.retract_inc == FALSE)
	retract_pln = fabs (op.cmn.top_stock - retract_pln);

    if (op.cmn.depth_inc == FALSE)
	hole_bot = op.cmn.depth;
    else
	hole_bot = op.cmn.top_stock - op.cmn.depth;

    feature->SetLocation(ptCenter[0], ptCenter[1], hole_bot);

    if (through)
    {
	hole_id = feature->RoundHole(ws_id, _com_util::ConvertStringToBSTR (cmnt), hole_depth, op.tl.dia); // Hole diameter is the drill diameter
	drill_depth = hole_depth + fabs(breakthrough_amount);
    }
    else
    {
	hole_id = feature->RoundHole(ws_id, _com_util::ConvertStringToBSTR (cmnt), hole_depth, op.tl.dia); // Hole diameter is the drill diameter
	drill_depth = hole_depth;
	feature->HoleConicalBottom(hole_id, tip_angle, tip_radius);
    }

    MY_TRACE(pfLog, "Round Hole depth = %f, dia = %f at (%f, %f, %f)\n", hole_depth, op.tl.dia, ptCenter[0], ptCenter[1], hole_bot);
//    MY_TRACE(pfLog, "Round Hole depth = %f, dia = %f at (%f, %f, %f)\n", hole_depth, op.tl.dia, ptCenter[0], ptCenter[1], ptCenter[2] - retract_pln);

    if (cycle == 0)
    {
	MY_TRACE(pfLog, "Drilling dwell = %f, depth = %f retract feed = %f\n", dwell, drill_depth, retract_feed);
	process->Drilling (ws_id, drill_depth, dwell, retract_feed, 0);
    }
    else if (cycle == 1)
    {
	MY_TRACE(pfLog, "Multistep Drilling dwell = %f, depth = %f retract feed = %f\n", dwell, drill_depth, retract_feed);
	MY_TRACE(pfLog, "Multistep Drilling peck1 = %f peck2 = %f retract distance = %f, dwell step = %f\n", peck1, peck2, retract_distance, dwell_step);
	process->MultistepDrilling (ws_id, drill_depth, peck2, peck1, retract_distance, dwell, retract_feed, dwell_step, 0);
    }
    else if (cycle == 3)
    {
	MY_TRACE(pfLog, "Tapping dwell = %f, depth = %f retract feed = %f\n",  dwell, drill_depth, retract_feed);
	process->Tapping (ws_id, drill_depth, 0, dwell, retract_feed, 0);
    }
    else   // 4 is common what is it?
    {
	MY_TRACE(pfLog, "Unprocessed Drilling cycle = %d dwell = %f, depth = %f\n", cycle, dwell, op.cmn.top_stock - op.cmn.depth);
	process->Drilling (ws_id, drill_depth, dwell, retract_feed, 0);
    }

    if (retract_pln != 0)
    {
	process->RetractPlane (ws_id, retract_pln);
    }

    apt->Rapid ();	    // next move is rapid unless otherwise defined
    return;
}


void MakePlanarFace(operation op, p_3d ptCenter, double dLength, double dWidth, _int64 ws_id)
{
    MY_TRACE(pfLog, "Making planar face for workingstep at %d with sizes (%f, %f)\n", ws_id, dLength, dWidth);
    feature->SetDirection(-op.tpln.m[2][0], -op.tpln.m[2][1], -op.tpln.m[2][2],
			  op.tpln.m[0][0], op.tpln.m[0][1], op.tpln.m[0][2]);
    feature->SetLocation(ptCenter[0] - dLength / 2, ptCenter[1] - dWidth/2, ptCenter[2]);
    feature->PlanarFace(ws_id, "planar face", op.cmn.top_stock - op.cmn.depth, dLength, dWidth);
    process->PlaneRoughMilling (ws_id, op.dcuts.fin_amt, op.dcuts.rgh_amt);
    return;
}

void MakeRoundPocket(operation op, p_3d ptCenter, double dDiameter, double dDepth, _int64 ws_id)
{
    MY_TRACE(pfLog, "Making circular pocket for workingstep at %d with diameter = %f and depth = %f\n", ws_id, dDiameter, dDepth);
    feature->SetDirection(op.tpln.m[2][0], op.tpln.m[2][1], op.tpln.m[2][2],
			  -op.tpln.m[0][0], -op.tpln.m[0][1], -op.tpln.m[0][2]);
    feature->SetLocation(ptCenter[0], ptCenter[1], ptCenter[2]);
    feature->ClosedCircularPocket (ws_id, "circular pocket", dDepth, dDiameter);
    // parameters need to be fixed
    process->BottomSideRoughMilling (ws_id, op.dcuts.fin_amt, op.dcuts.rgh_amt, op.dcuts.fin_amt, op.dcuts.rgh_amt);
    return;
}


void MakeGeneralOutsideProfile(operation op, CHAIN* pChain, _int64 ws_id)
{
    double xy_allow = op.cmn.stk_remain;
    double z_allow = op.dcuts.fin_amt * op.dcuts.fin_n;

    int nFeatureID = 0;
    int old_feature_id = 0;

    MC_BOOL ok;
    double depth, cdepth, abs_depth, abs_top_of_stock;

    cdepth = -999;
    //  in incremental mode pocket depth and top of stock are relative to chain
    if (op.cmn.depth_inc || op.cmn.top_stock_inc)
	get_chain_depth (pChain, &cdepth, &ok);

    if (op.cmn.top_stock_inc)
	abs_top_of_stock = cdepth + op.cmn.top_stock;
    else
	abs_top_of_stock = op.cmn.top_stock;

    if (op.cmn.depth_inc)
	abs_depth = cdepth;
    else
	abs_depth = op.cmn.depth;

    depth = fabs (abs_depth - abs_top_of_stock);

    MY_TRACE(pfLog, "Chain depth = %f, abs_depth = %f, abs_top_of_stock = %d\n", cdepth, abs_depth, abs_top_of_stock);
    MY_TRACE(pfLog, "cmn.top_stock = %f, cmn.top_stock_inc = %d\n", op.cmn.top_stock, op.cmn.top_stock_inc);
    MY_TRACE(pfLog, "cmn.depth = %f, cmn.dpeth_inc = %d\n", op.cmn.depth, op.cmn.depth_inc);

    // in incremental mode clearance, retract and feed are relative to top of stock
    double abs_clear = 0;
    if (op.cmn.clearance_on)
    {
	if (op.cmn.clearance_inc)
	    abs_clear = op.cmn.clearance_pln + abs_top_of_stock;
	else
	    abs_clear = op.cmn.clearance_pln;
    }

    // need process clearance plane
    if (abs_clear > 0)
	apt->ClearancePlane (abs_clear);

    double abs_retract = 0;
    double abs_feed = 0;
    // STEP-NC only has a retract plane, MCAM has retract and feed
    if (op.cmn.retract_on)
    {
	if (op.cmn.retract_inc)
	    abs_retract = op.cmn.retract_pln + abs_top_of_stock;
	else
	    abs_retract = op.cmn.retract_pln;
    }

    // feed plane is not optional in MCAM
    if (op.cmn.feed_inc)
	abs_feed = op.cmn.feed_pln + abs_top_of_stock;
    else
	abs_feed = op.cmn.feed_pln;

    // pick the largest
    if (abs_feed > abs_retract)
	abs_retract = abs_feed;

    if (abs_retract > 0)
	apt->RetractPlane (abs_retract);

    MY_TRACE(pfLog, "Retract = %f, feed = %f, clearance = %f\n", abs_retract, abs_feed, abs_clear);

    // do we need to define a profile for this chain?
//    find_id_for_chain (pChain, old_feature_id, ws_id);

    // define new workingstep with new feature
    if (old_feature_id == 0)
    {
	MY_TRACE(pfLog, "Operation needs new feature\n");

	ws_id = apt->GetCurrentWorkingstep();
	feature->SetLocation (0, 0, 0);
	feature->SetDirection(-op.cpln.m[2][0], -op.cpln.m[2][1], -op.cpln.m[2][2],
			      op.cpln.m[0][0], op.cpln.m[0][1], op.cpln.m[0][2]);


	// check for clockwise or anit-clockwise chain
	short direction;
	MC_BOOL succf;
	MC_BOOL view_match;
	long n_pts;
	ctour_rec ctour[500];
	double l_tol = 0;		// linearization tolerance?
	short nView = 0;		// chain view?
	double rel_depth;
	chain_to_ctour_array (pChain, ctour, &rel_depth, nView, 500, 0, &n_pts, &view_match, 1,
			      l_tol, 1, &succf);

	find_direction (ctour, n_pts, &direction, &succf);

	if (pChain->closed)
	{
	    MY_TRACE(pfLog, "Contour Chain is closed\n");
	    MY_TRACE(pfLog, "Contour Chain direction is %s num points is %d\n", direction?"true":"false", n_pts);
	    MakeChainProfile(pChain, abs_depth, TRUE, direction);
	}
	else
	{
	    MY_TRACE(pfLog, "Contour Chain is open\n");
	    MY_TRACE(pfLog, "Contour Chain direction is %s num points is %d\n", direction?"true":"false", n_pts);
	    MakeChainProfile(pChain, abs_depth, FALSE, direction);
	}
    }
    else  	// reuse old feature
    {
	MY_TRACE(pfLog, "Operation reusing old feature id = %d\n", old_feature_id);
	ws_id = process->SecondWorkingstep (ws_id);
    }

    // create process data
    double rgh_side_allow = 0;
    double rgh_radial = 0;
    double axial_depth = 0;
    int rgh_passes = 1;

    if (op.mcuts.on)
    {
	rgh_radial = op.mcuts.rgh_amt;
	rgh_side_allow = op.mcuts.fin_amt * op.mcuts.fin_n;
	rgh_passes = op.mcuts.rgh_n;
    }

    if (op.dcuts.on)
    {
	axial_depth = op.dcuts.rgh_amt;
	if (op.dcuts.fin_n != 0)
	    MY_TRACE(pfLog, "WARNING finish depth cuts for contour\n");
    }
    else
	axial_depth = depth;

    if (old_feature_id == 0 && pChain->closed)
    {
	MY_TRACE(pfLog, "Closed general outside profile\n");
	feature->ClosedGeneralOutsideProfile (ws_id,  "closed contour", depth);
//		add_to_chain_db (pChain, nFeatureID, ws_id);
    }
    else if (old_feature_id == 0)
    {
	MY_TRACE(pfLog, "Open general outside profile\n");
	feature->OpenGeneralOutsideProfile (ws_id,  "open contour", depth);
//		add_to_chain_db (pChain, nFeatureID, ws_id);
    }
    else
	nFeatureID = old_feature_id;

    MY_TRACE(pfLog, "S Rough Milling ws_id = %d side_allow = %f, radial = %f number = &d\n",
	     ws_id, rgh_side_allow, rgh_radial, rgh_passes);
    process->SideRoughMilling (ws_id, rgh_side_allow, rgh_radial, axial_depth);

    // set number using new number_of_passes attribute

    if (op.mcuts.on && op.mcuts.fin_amt > 0 && op.mcuts.fin_n > 0)
    {
	MY_TRACE(pfLog, "S Finish Milling passes = %d fin step = %f\n", op.mcuts.fin_n, op.mcuts.fin_amt);
	ws_id = process->SecondWorkingstep (ws_id);
	// make axial_depth conditional on if finishing at every depth
	process->SideFinishMilling (ws_id, op.cmn.stk_remain, op.mcuts.fin_amt, axial_depth);
    }

    return;
}

void MakeChainProfile(CHAIN* pChain, double z_value, BOOL closed, BOOL reverse_contour_direction)
{
    ent seg1, seg2;
    CHAIN_ENT *ce1, *ce2;

    p_3d ptCenView = { 0, 0, 0 };
    p_3d ptCen3d = { 0, 0, 0 };
    BOOL first = true;

    // changed for Boxy - added code to process single line outside profiles
    for (ce1 = first_chain_curve(pChain); ce1 != NULL; ce1 = next_chain_curve(pChain, ce1, FALSE))
    {
	get_ent_from_eptr(ce1->e_ptr, &seg1);
	ce2 = next_chain_curve(pChain, ce1, FALSE);
	if (ce2 != NULL)
	{
	    MY_TRACE(pfLog, "ce1 = %x ce2 = %x\n", ce1, ce2);
	    get_ent_from_eptr(ce2->e_ptr, &seg2);
	    MakeProfileGeometry (seg1, seg2, first, !closed,
				 z_value, FALSE, reverse_contour_direction);
	}
	else if (closed == TRUE)
	{
	    ce2 = first_chain_curve(pChain);
	    MY_TRACE(pfLog, "closing: ce2 = %x\n", ce2);
	    get_ent_from_eptr(ce2->e_ptr, &seg2);
	    MakeProfileGeometry (seg1, seg2, first, FALSE,
				 z_value, FALSE, reverse_contour_direction);
	    break;
	}
	else if (first)  // one line open profile?
	{
	    ce2 = first_chain_curve(pChain);
	    MY_TRACE(pfLog, "One line open profile: ce1 = %x ce2 = %x\n", ce1, ce2);
	    get_ent_from_eptr(ce2->e_ptr, &seg2);
	    MakeProfileGeometry (seg1, seg2, first, FALSE,
				 z_value, FALSE, reverse_contour_direction);
	    break;
	}

	first = false;
    }
}

void MakeGeneralPocket(operation op, CHAIN* pChain, _int64 ws_id, int nChains)
{
    double xy_allow = op.cmn.stk_remain;
    double z_allow = op.dcuts.fin_amt * op.dcuts.fin_n;

    _int64 nFeatureID = 0;
    _int64 old_feature_id;

    MC_BOOL ok;
    double depth, cdepth, abs_depth, abs_top_of_stock;

    if (op.u.prm_pkt.pock_type != POCK_OPEN)
	sort_outside_to_inside (&pChain);	    // make sure pocket is first chain

    cdepth = -999;
    //  in incremental mode pocket depth and top of stock are relative to chain
    if (op.cmn.depth_inc || op.cmn.top_stock_inc)
	get_chain_depth (pChain, &cdepth, &ok);

    if (op.cmn.top_stock_inc)
	abs_top_of_stock = cdepth + op.cmn.top_stock;
    else
	abs_top_of_stock = op.cmn.top_stock;

    if (op.cmn.depth_inc)
	abs_depth = cdepth;
    else
	abs_depth = op.cmn.depth;

    depth = fabs (abs_depth - abs_top_of_stock);

    MY_TRACE(pfLog, "Chain depth = %f, abs_depth = %f, abs_top_of_stock = %d\n", cdepth, abs_depth, abs_top_of_stock);
    MY_TRACE(pfLog, "cmn.top_stock = %f, cmn.top_stock_inc = %d\n", op.cmn.top_stock, op.cmn.top_stock_inc);
    MY_TRACE(pfLog, "cmn.depth = %f, cmn.dpeth_inc = %d\n", op.cmn.depth, op.cmn.depth_inc);

    // in incremental mode clearance, retract and feed are relative to top of stock
    double abs_clear = 0;
    if (op.cmn.clearance_on)
    {
	if (op.cmn.clearance_inc)
	    abs_clear = op.cmn.clearance_pln + abs_top_of_stock;
	else
	    abs_clear = op.cmn.clearance_pln;
    }

    // need process clearance plane
    if (abs_clear > 0)
	apt->ClearancePlane (abs_clear);

    double abs_retract = 0;
    double abs_feed = 0;
    // STEP-NC only has a retract plane, MCAM has retract and feed
    if (op.cmn.retract_on)
    {
	if (op.cmn.retract_inc)
	    abs_retract = op.cmn.retract_pln + abs_top_of_stock;
	else
	    abs_retract = op.cmn.retract_pln;
    }

    // feed plane is not optional in MCAM
    if (op.cmn.feed_inc)
	abs_feed = op.cmn.feed_pln + abs_top_of_stock;
    else
	abs_feed = op.cmn.feed_pln;

    // pick the largest
    if (abs_feed > abs_retract)
	abs_retract = abs_feed;

    if (abs_retract > 0)
	apt->RetractPlane (abs_retract);

    MY_TRACE(pfLog, "Retract = %f, feed = %f, clearance = %f\n", abs_retract, abs_feed, abs_clear);

    // do we need to define a profile for this chain?
//    find_id_for_chain (pChain, old_feature_id, ws_id);

    old_feature_id = 0;
    // define new workingstep with new feature
    if (old_feature_id == 0)
    {
	MY_TRACE(pfLog, "Operation needs new feature\n");
	ws_id = apt->GetCurrentWorkingstep();
	feature->SetLocation (0, 0, 0);
	feature->SetDirection(-op.cpln.m[2][0], -op.cpln.m[2][1], -op.cpln.m[2][2],
			      op.cpln.m[0][0], op.cpln.m[0][1], op.cpln.m[0][2]);

	if (op.u.prm_pkt.pock_type != POCK_OPEN)
	    MakeChainProfile(pChain, abs_depth, TRUE, FALSE);
	else
	    MakeChainProfile(pChain, abs_depth, FALSE, FALSE);
    }
    else  	// reuse old feature
    {
	MY_TRACE(pfLog, "Operation reusing old feature id = %d\n", old_feature_id);
	ws_id = process->SecondWorkingstep (ws_id);
    }

    // create process data
    double fin_radial = 0;
    double axial_depth = 0;
    double radial_depth = 0;

    if (op.u.prm_pkt.finish)
    {
	fin_radial = op.u.prm_pkt.fin_n * op.u.prm_pkt.fin_step;
    }

    if (op.u.prm_pkt.pock_type == POCK_OPEN)
    {
	if (old_feature_id == 0)
	{
	    nFeatureID = feature->OpenGeneralPocket(ws_id, "open pocket", depth);
//			add_to_chain_db (pChain, nFeatureID, ws_id);
	}
	else
	    nFeatureID = old_feature_id;

	if (op.u.prm_pkt.rough == TRUE)
	{
	    if (op.dcuts.on)
		axial_depth = op.dcuts.rgh_amt;
	    else
		axial_depth = depth;

	    MY_TRACE(pfLog, "B&S Rough Milling ws_id = %d side_allow = %f, bott_allow = %f, radial = %f, axial = %f fin_radial = %f\n",
		     ws_id, op.cmn.stk_remain, op.dcuts.stock_t_l, axial_depth, radial_depth, fin_radial);
	    process->BottomSideRoughMilling (ws_id, op.cmn.stk_remain, op.dcuts.stock_t_l, radial_depth, axial_depth);
	    // values before KTH
	    //			process->BottomSideRoughMilling (ws_id, op.cmn.stk_remain + fin_radial, op.dcuts.stock_t_l, radial_depth, axial_depth);
	}
	MY_TRACE(pfLog, "MakeFeature: Open Pocket\n");
    }
    else   // all others assumed to be closed // if (op.u.prm_pkt.pock_type == POCK_STD) {
    {
	if (old_feature_id == 0)
	{
	    nFeatureID = feature->ClosedGeneralPocket(ws_id, "closed pocket", depth);
//			add_to_chain_db (pChain, nFeatureID, ws_id);
	}
	else
	    nFeatureID = old_feature_id;

	if (op.u.prm_pkt.rough == TRUE)
	{
	    radial_depth = op.u.prm_pkt.rgh_step;
	    if (op.dcuts.on)
		axial_depth = op.dcuts.rgh_amt;
	    else
		axial_depth = depth;

	    MY_TRACE(pfLog, "B&S Rough Milling ws_id = %d side_allow = %f, bott_allow = %f, radial = %f, axial = %f fin_radial = %f\n",
		     ws_id, op.cmn.stk_remain, op.dcuts.stock_t_l, axial_depth, radial_depth, fin_radial);
	    process->BottomSideRoughMilling (ws_id, op.cmn.stk_remain, op.dcuts.stock_t_l, radial_depth, axial_depth);
	    // Before KTH
	    //			process->BottomSideRoughMilling (ws_id, op.cmn.stk_remain + fin_radial, op.dcuts.stock_t_l, radial_depth, axial_depth);
	}
	MY_TRACE(pfLog, "MakeFeature: Closed Pocket\n");
    }
//    else {
//		MY_TRACE(pfLog, "Unsupported pocket type (neither open nor closed)\n");
//    }

//    if (nChains > 1 && old_feature_id == 0)
//		ChainsToBosses(nFeatureID, pChain->next, abs_depth, depth, op.dcuts.island_depths); // starting at the second chain in the CMI

    /* Taken out because looks stupid for Boxy
       if (op.u.prm_pkt.finish) {
       MY_TRACE(pfLog, "B&S Finish Milling passes = %d fin step = %f\n", op.u.prm_pkt.fin_n, op.u.prm_pkt.fin_step);
       if (op.u.prm_pkt.fr_override_on)
       apt->Feedrate (op.u.prm_pkt.fr_override);
       if (op.u.prm_pkt.ss_override_on)
       apt->SpindleSpeed (op.u.prm_pkt.ss_override);

       ws_id = process->SecondWorkingstep (ws_id);

       if (op.u.prm_pkt.cp.finish_all)
       process->SideFinishMilling (ws_id, op.cmn.stk_remain, op.u.prm_pkt.fin_step, axial_depth);
       else
       process->SideFinishMilling (ws_id, op.cmn.stk_remain, op.u.prm_pkt.fin_step, depth);

       // add one side mill op for each spring back with radial_depth = 0
       int count = op.u.prm_pkt.n_spring_cuts;
       for (int j = 0; j < count; j++) {
       ws_id = process->SecondWorkingstep (ws_id);
       process->SideFinishMilling (ws_id, op.cmn.stk_remain, 0, 0);
       }
       }
    */

    return;
}


int arc_count = 0;
// seg1 is the current segement, seg2 is the next segment
// use seg2 to determine order for points in seg1
void MakeProfileGeometry (ent seg1, ent seg2, BOOL first, BOOL boss_or_open, double z_value,
			  BOOL last_in_open, BOOL reverse_contour_direction)
{
    p_3d pt1End = { 0, 0, 0 };
    p_3d pt1Strt = { 0, 0, 0 };
    p_3d pt2End = { 0, 0, 0 };
    p_3d pt2Strt = { 0, 0, 0 };
    p_3d ptCenView = { 0, 0, 0 };
    p_3d ptCen3d = { 0, 0, 0 };

    // make a line
    if (seg1.id == L_ID)
    {
	copy_p_3d(pt1Strt, seg1.u.li.e1);
	copy_p_3d(pt1End, seg1.u.li.e2);

	if (seg2.id == L_ID)
	{
	    copy_p_3d(pt2Strt, seg2.u.li.e1);
	    copy_p_3d(pt2End, seg2.u.li.e2);
	}
	else if (seg2.id == A_ID)
	{
	    copy_p_3d(pt2Strt, seg2.u.ar.ep1);
	    copy_p_3d(pt2End, seg2.u.ar.ep2);
	}
	else
	{
	    MY_TRACE(pfLog, "Unknown type of profile chain segment = %d\n", seg1.id);
	    return;
	}

	// determine if need to swap
	if (dist_pp_3d (pt1Strt, pt2End) < EPSILON || dist_pp_3d (pt1Strt, pt2Strt) < EPSILON)
	{
	    MY_TRACE(pfLog, "Swapping line point order\n");
	    copy_p_3d(pt1Strt, seg1.u.li.e2);
	    copy_p_3d(pt1End, seg1.u.li.e1);
	}

	if (first)
	{
	    feature->LineTo("First point", pt1Strt[X], pt1Strt[Y], z_value);
	    MY_TRACE(pfLog, "First point in chain for line\n");
	}

	feature->LineTo("Line point", pt1End[X], pt1End[Y], z_value);
	MY_TRACE(pfLog, "Line: (%f, %f, %f)->(%f, %f, %f)\n", pt1Strt[X], pt1Strt[Y], z_value,
		 pt1End[X], pt1End[Y], z_value);
    }

    else if (seg1.id == A_ID)
    {
	copy_p_3d(pt1Strt, seg1.u.ar.ep1);
	copy_p_3d(pt1End, seg1.u.ar.ep2);

	if (seg2.id == L_ID)
	{
	    copy_p_3d(pt2Strt, seg2.u.li.e1);
	    copy_p_3d(pt2End, seg2.u.li.e2);
	}
	else if (seg2.id == A_ID)
	{
	    copy_p_3d(pt2Strt, seg2.u.ar.ep1);
	    copy_p_3d(pt2End, seg2.u.ar.ep2);
	}
	else
	{
	    MY_TRACE(pfLog, "Unknown type of profile chain segment = %d\n", seg1.id);
	    return;
	}

	BOOL ccw = TRUE;
	// determine if need to swap
	if (dist_pp_3d (pt1Strt, pt2End) < EPSILON || dist_pp_3d (pt1Strt, pt2Strt) < EPSILON)
	{
	    MY_TRACE(pfLog, "Swapping arc point order\n");
	    copy_p_3d(pt1Strt, seg1.u.ar.ep2);
	    copy_p_3d(pt1End, seg1.u.ar.ep1);
	    ccw = FALSE;
	}

	if (first)
	{
	    feature->LineTo("First point", pt1Strt[X], pt1Strt[Y], z_value);
	    MY_TRACE(pfLog, "First point in chain for arc\n");
	}

	copy_p_3d (ptCenView, seg1.u.ar.c);
	view_to_world (ptCenView, seg1.u.ar.view, ptCen3d);
	if (boss_or_open)
	{
	    p_2d p1, p2, cen;
	    copy_pt (p1, pt1Strt);		// points may be swapped for open chain
	    copy_pt (p2, pt1End);
	    copy_pt (cen, ptCen3d);
	    // Function assumes all BOSS arcs are less than 180
	    ccw = counter_clockwise (p1, p2, cen);
	}


	// negative sweep means reverse direction?
	// tested with several pockets including one with bosses
	if (seg1.u.ar.sw < 0)
	    ccw = !ccw;

	// Otherwise arcs are CCW in MCAM, and start/end points control the actual shape
	feature->Arc("arc", pt1End[X], pt1End[Y], z_value,
		     ptCen3d[X], ptCen3d[Y], z_value, seg1.u.ar.r, ccw);

	MY_TRACE(pfLog, "Arc %d: C(%f, %f, %f), \n\t(%f, %f, %f)->(%f, %f, %f), \n\tr=%f, %s\n",
		 arc_count++, ptCen3d[X], ptCen3d[Y], z_value,
		 pt1Strt[X], pt1Strt[Y], z_value,
		 pt1End[X], pt1End[Y], z_value, seg1.u.ar.r, ccw?"ccw":"cw");

	MY_TRACE(pfLog, "Arc %d: start angle = %f, sweep angle = %f\n",
		 arc_count - 1, seg1.u.ar.sa, seg1.u.ar.sw);

    }
    else
    {
	MY_TRACE(pfLog, "Unknown type of profile chain segment = %d\n", seg1.id);
	return;
    }

    if (last_in_open)
    {
	if (seg2.id == L_ID)
	{
	    if (dist_pp_3d (pt2End, seg2.u.li.e1) < EPSILON)
	    {
		feature->LineTo("Last Open Line point", seg2.u.li.e2[X], seg2.u.li.e2[Y], z_value);
		MY_TRACE(pfLog, "Last Open Line point: (%f, %f, %f)->(%f, %f, %f)\n", pt1End[X], pt1End[Y], z_value,
			 seg2.u.li.e2[X], seg2.u.li.e2[Y], z_value);
	    }
	    else if (dist_pp_3d (pt2End, seg2.u.li.e2) < EPSILON)
	    {
		feature->LineTo("Last Open Line point", seg2.u.li.e1[X], seg2.u.li.e1[Y], z_value);
		MY_TRACE(pfLog, "Last Open Line point: (%f, %f, %f)->(%f, %f, %f)\n",
			 pt1End[X], pt1End[Y], z_value,
			 seg2.u.li.e1[X], seg2.u.li.e1[Y], z_value);
	    }
	}
	if (seg2.id == A_ID)
	{
	    copy_p_3d (ptCenView, seg2.u.ar.c);
	    view_to_world (ptCenView, seg2.u.ar.view, ptCen3d);

	    if (dist_pp_3d (pt2End, seg2.u.ar.ep1) < EPSILON)
	    {
		feature->Arc("arc", seg2.u.ar.ep2[X], seg2.u.ar.ep2[Y], z_value,
			     ptCen3d[X], ptCen3d[Y], z_value, seg2.u.ar.r, TRUE);

		MY_TRACE(pfLog, "Arc %d: C(%f, %f, %f), \n\t(%f, %f, %f)->(%f, %f, %f), \n\tr=%f, %s\n",
			 arc_count++, ptCen3d[X], ptCen3d[Y], z_value,
			 pt1End[X], pt1End[Y], z_value,
			 seg2.u.ar.ep2[X], seg2.u.ar.ep2[Y], z_value, seg1.u.ar.r, TRUE?"ccw":"cw");
	    }

	    else if (dist_pp_3d (pt2End, seg2.u.ar.ep2) < EPSILON)
	    {
		feature->Arc("arc", seg2.u.ar.ep1[X], seg2.u.ar.ep1[Y], z_value,
			     ptCen3d[X], ptCen3d[Y], z_value, seg2.u.ar.r, FALSE);

		MY_TRACE(pfLog, "Arc %d: C(%f, %f, %f), \n\t(%f, %f, %f)->(%f, %f, %f), \n\tr=%f, %s\n",
			 arc_count++, ptCen3d[X], ptCen3d[Y], z_value,
			 pt1End[X], pt1End[Y], z_value,
			 seg2.u.ar.ep1[X], seg2.u.ar.ep1[Y], z_value, seg1.u.ar.r, FALSE?"ccw":"cw");
	    }
	}
    }
    return;
}
// used to calculate the arc direction for BOSS arcs
// assumes all BOSS arcs are less than 180

// could  not find anything in Mastercam to indicate BOSS arc direction
// possible clues might be large start_angles or sweeps
// also some of the functions seem to make coordinates go negative sometimes

// Mastercam arc's in boundary chains are always clockwise so do not use this
// algorithm except for bosses

// This algorithm is strong on the case where arc's are 90 degrees vertical or
// horizontal because that seems to happen a lot in practice for Bosses
BOOL counter_clockwise (p_2d p1, p_2d p2, p_2d center)
{
    double dx1, dy1, dx2, dy2;
    dx1 = center[X] - p1[X];
    dy1 = center[Y] - p1[Y];
    dx2 = p2[X] - center[X];
    dy2 = p2[Y] - center[Y];

    BOOL ccw;
    if (fabs (dx1) > EPSILON && fabs (dy2) > EPSILON && fabs (dy1) > EPSILON && fabs (dx2) > EPSILON)
    {
	if (dy2 * dx1 - dy1 * dx2 > 0)
	    ccw = FALSE;
	else if (dy2 * dx1 - dy1 * dx2 < 0)
	    ccw = TRUE;
	MY_TRACE (pfLog, "FORMULA 1 ccw = %d\n", ccw);
    }
    else if (fabs (dy1) > EPSILON && fabs (dx2) > EPSILON)
    {
	if (dx2 * dy1 < 0)
	    ccw = FALSE;
	else
	    ccw = TRUE;
	MY_TRACE (pfLog, "FORMULA 2 dy1 = %f dx2 = %f ccw = %d\n", dy1, dx2, ccw);
    }
    else if (fabs (dy2) > EPSILON && fabs (dx1) > EPSILON)
    {
	if (dx1 * dy2 < 0)
	    ccw = TRUE;
	else
	    ccw = FALSE;
	MY_TRACE (pfLog, "FORMULA 3 dx1 = %f dy2 = %f ccw=%d\n", dx1, dy2, ccw);
    }
    else
	MY_TRACE (pfLog, "FORMULA TROUBLE\n");

    MY_TRACE(pfLog, "BOSS Arc calculation: C(%f, %f),\n (%f, %f)->(%f, %f),\n %s\n",
	     center[X], center[Y], p1[X], p1[Y], p2[X], p2[Y], ccw? "counter-clockwise" : "clockwise");

    return ccw;
}



/////////////////////////////////////////////////////////////////////////////
//
//	FUNCTION:	GetChainBBox
//
//	PURPOSE:	Compute the bounding box of a chain.
//
//	ARGUMENTS:	CHAIN* pChain	    - MCAM chain entity
//			int nView	    - MCAM view (coordinate system)
//			double& dLength	    - Length of the bbox [out]
//			double& dWitdth	    - Width of the bbox [out]
//			p3_d center	    - Center point of the box [out]
//
//	RETURN VALUE:	void
//
/////////////////////////////////////////////////////////////////////////////
void GetChainBBox(CHAIN* pChain, int nView, double& dLength, double& dWidth, p_3d center)
{
    double ME_DBL_MAX = 100000;
    double dXMin = ME_DBL_MAX;
    double dXMax = -ME_DBL_MAX;
    double dYMin = ME_DBL_MAX;
    double dYMax = -ME_DBL_MAX;

    gt ent2D;
    p_2d bbox[2];

    for (CHAIN_ENT* ce = first_chain_curve(pChain); ce != NULL; ce = next_chain_curve(pChain, ce, FALSE))
    {
	ent seg;
	get_ent_from_eptr(ce->e_ptr, &seg);
	ent_to_gt(&ent2D, nView, &seg);
	bbox_gt(&ent2D, bbox);

	dXMin = min(dXMin, bbox[0][0]);
	dXMax = max(dXMax, bbox[1][0]);

	dYMin = min(dYMin, bbox[0][1]);
	dYMax = max(dYMax, bbox[1][1]);
    }

    dLength = dXMax - dXMin;
    dWidth = dYMax - dYMin;

    center[0] = dXMin + dLength / 2;
    center[1] = dYMin + dWidth / 2;
    center[2] = 0;			// Don't set it here

    dLength = fabs(dLength);
    dWidth = fabs(dWidth);
}



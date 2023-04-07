////////////////////////////////////////////////////////////////////////////////
//                                            __ _      _     _               //
//                                           / _(_)    | |   | |              //
//                __ _ _   _  ___  ___ _ __ | |_ _  ___| | __| |              //
//               / _` | | | |/ _ \/ _ \ '_ \|  _| |/ _ \ |/ _` |              //
//              | (_| | |_| |  __/  __/ | | | | | |  __/ | (_| |              //
//               \__, |\__,_|\___|\___|_| |_|_| |_|\___|_|\__,_|              //
//                  | |                                                       //
//                  |_|                                                       //
//                                                                            //
//                                                                            //
//              VHDL to Verilog                                               //
//              HDL Translator                                                //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

/* Copyright (c) 202X-202X by the author(s)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * =============================================================================
 * Author(s):
 *   Francisco Javier Reina Campo <pacoreinacampo@queenfield.tech>
 */

#ifndef VHD2V_H

//////////////////////////////////////////////////////////////////////
// Change below if necessary
//////////////////////////////////////////////////////////////////////

//#define WINDOWS
#define UNIX

//////////////////////////////////////////////////////////////////////
// Warning messages
//////////////////////////////////////////////////////////////////////

#define  WARN_0_PUTFOR      "0\0Please put the for-statement into a new always-statement manually."
#define  WARN_1_PUTLOOP     "1\0Please move the above loop-variable declaration to outside always-statement."
#define  WARN_2_RETWIDTH    "2\0Please specify signal width of return value."
#define  WARN_3_PROCPARAM   "3\0Please check the order of function/procedure parameters."
#define  WARN_4_CLKRST      "4\0Can not analyze clock/rst condition of process statement."
#define  WARN_5_ARRAYTYPE   "5\0Unsupported data type: too complicated array."
#define  WARN_6_SIGWIDTH    "6\0Please check signal width (converted from integer)."
#define  WARN_7_INTTYPE     "7\0Unsupported data type: too long bit-width of integer type."
#define  WARN_8_DATATYPE    "8\0Unsupported data type."
#define  WARN_9_SIGCOPY     "9\0Please set signal width and/or copy times manually."
#define  WARN_10_SIGNAME    "10\0The signal name was different from its declaration (Verilog is case sensitive)."

//////////////////////////////////////////////////////////////////////
// Type/Constant declarations
//////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#define TITLE "verilog2vhdl - QueenField\n"

typedef enum {
  True, False
} Boolean;

#define NULLSTR    (char *)NULL

// TOKEN cell
typedef struct _tree_cell {
  int    linenum;    // line number in source code
  int    typ;        // token type
  char  *str;        // ID/digit value
  int    *siglist;   // signal list
  int    *typelist;  // type list
  struct _tree_cell  *next;
  struct _tree_cell  *info0;
  struct _tree_cell  *info1;
  struct _tree_cell  *info2;
  struct _tree_cell  *info3;
  struct _tree_cell  *info4;
  struct _tree_cell  *info5;
} TREECELL;

typedef TREECELL  *TCELLPNT;

#define  NULLCELL       (TREECELL *)NULL
#define  CellLine(x)    ((x)->linenum)
#define  CellType(x)    ((x)->typ)
#define  CellStr(x)     ((x)->str)
#define  NextCell(x)    ((x)->next)
#define  CellInfo0(x)   ((x)->info0)
#define  CellInfo1(x)   ((x)->info1)
#define  CellInfo2(x)   ((x)->info2)
#define  CellInfo3(x)   ((x)->info3)
#define  CellInfo4(x)   ((x)->info4)
#define  CellInfo5(x)   ((x)->info5)
#define  CellSig(x)     ((SIGLIST *)((x)->siglist))
#define  CellTypList(x) ((TYPELIST *)((x)->typelist))

#define  SetType(d,s)    (d)->typ   = (s);
#define  SetLine(d,s)    (d)->linenum = (s);
#define  SetNext(d,s)    (d)->next  = (s);
#define  SetInfo0(d,s)   (d)->info0 = (s);
#define  SetInfo1(d,s)   (d)->info1 = (s);
#define  SetInfo2(d,s)   (d)->info2 = (s);
#define  SetInfo3(d,s)   (d)->info3 = (s);
#define  SetInfo4(d,s)   (d)->info4 = (s);
#define  SetInfo5(d,s)   (d)->info5 = (s);
#define  SetSig(d,s)     (d)->siglist = (int *)(s);
#define  SetTypList(d,t) (d)->typelist = (int *)(t);

// Signal list
typedef struct _signal_list  {
  char    *str;        // signal name
  TCELLPNT  typ;       // signal type
  Boolean    isreg;    // reg or wire
  Boolean    isinp;    // input flag
  Boolean    isout;    // output flag
  Boolean    isfunc;   // function/procedure flag
  struct _signal_list  *next;
} SIGLIST;

#define  NULLSIG       (SIGLIST *)NULL
#define  SigStr(x)     ((x)->str)
#define  SigType(x)    ((x)->typ)
#define  SigIsReg(x)   ((x)->isreg)
#define  SigIsInp(x)   ((x)->isinp)
#define  SigIsOut(x)   ((x)->isout)
#define  SigIsFunc(x)  ((x)->isfunc)
#define  NextSig(x)    ((x)->next)

// Comment list
typedef struct _comment_list {
  int    linenum;    // comment line number
  char  *str;        // string
  Boolean  printed;  // flag to indicate the comment is already printed or not
  struct _comment_list  *next;
} COMMENTLIST;

#define  CommentLine(x)  ((x)->linenum)
#define  CommentStr(x)   ((x)->str)
#define  CommentPrn(x)   ((x)->printed)
#define  NextComment(x)  ((x)->next)

// Type def list
typedef struct _type_list {
  char    *name;   // type name
  TCELLPNT  info;  // type declaration
  struct _type_list  *next;
} TYPELIST;

#define  NULLTYPE       (TYPELIST *)NULL
#define  TypeName(x)    ((x)->name)
#define  TypeInfo(x)    ((x)->info)
#define  NextType(x)    ((x)->next)

// for-generate/loop extraction list
typedef struct _extract_list {
  int      substnum;  // loop value (will be substituted to loop variable)
  TCELLPNT id;        // loop variable
  struct _extract_list  *next;
} EXTRACTLIST;

#define  ExtractId(x)    ((x)->id)
#define  ExtractNum(x)   ((x)->substnum)
#define  NextExtract(x)  ((x)->next)

//////////////////////////////////////////////////////////////////////
// variable/function declarations
//////////////////////////////////////////////////////////////////////

// for lex
extern FILE     *yyin, *yyout;
extern int      yylex();
extern void     yyrestart(FILE *);
extern int      yylex_linenum;

// for parser
#define YYSTYPE TCELLPNT
#include "vhdlparse.tab.h"    // this sentence must be after the above definition of YYSTYPE
extern int      yyparse();
extern int      yylexlinenum;
#define alloca  malloc
extern void     yyerror(char *);

// for debug
#define YYDEBUG    0

// global variables
extern FILE         *fpout;            // output file pointer
extern int          Sysid;             // system internal counter value
extern COMMENTLIST  *CommentListTop;   // comment list
extern TYPELIST     *TypeListTop;      // type list
extern EXTRACTLIST  *ExtractListTop;   // extract list
extern TCELLPNT     ParseTreeTop;      // parse tree
extern SIGLIST      *ParseSigListTop;  // signal list
extern Boolean      ParseError;        // parse flag

// prototype
extern TCELLPNT     MallocTcell(int, char *, int);
extern void         FreeTcell(TCELLPNT);
extern void         FreeTree(TCELLPNT);
extern SIGLIST      *MakeNewSigList(void);
extern void         FreeSigList(SIGLIST *);
extern Boolean      AppendSigList(SIGLIST *, char *, TCELLPNT, Boolean, Boolean, Boolean, Boolean);
extern char         *GetSigName(TCELLPNT);
extern void         RegisterIoSignals(SIGLIST *, TCELLPNT);
extern SIGLIST      *SearchSigList(SIGLIST *, char *);
extern char         *SearchSimilarSig(SIGLIST *, char *);
extern COMMENTLIST  *MakeNewCommentList(void);
extern void         RegisterComment(COMMENTLIST *, char *, int *);
extern TYPELIST     *MakeNewTypeList(void);
extern void         FreeTypeList(TYPELIST *);
extern void         AppendTypeList(TYPELIST *, TCELLPNT);
extern TYPELIST     *SearchTypeList(TYPELIST *, char *);
extern EXTRACTLIST  *MakeNewExtractList(void);
extern void         FreeExtractList(EXTRACTLIST *);

#define VHD2V_H
#endif

// end of file

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
 *   Francisco Javier Reina Campo <frareicam@gmail.com>
 */

#include "vhdl2verilog.h"
#define TMPFNAME "vhdl2verilog.tmp"

FILE     *fpout;
int      Sysid;
COMMENTLIST  *CommentListTop;
TYPELIST  *TypeListTop;
EXTRACTLIST  *ExtractListTop;
TCELLPNT  ParseTreeTop;
SIGLIST    *ParseSigListTop;
Boolean    ParseError;

//////////////////////////////////////////////////////////////////////
// Memory allocation/free
//////////////////////////////////////////////////////////////////////

////////////////// TREECELL type (parse tree NODE) ///////////////////

// malloc NODE
// parameter:
//     typ:    Type of token
//     str:    ID or number value (if exists)
//     linenum:  HDL line number
// return:  Node cell
TCELLPNT MallocTcell(int typ, char *str, int linenum)
{
  register TCELLPNT  retval;
  if ((retval  = (TCELLPNT)malloc(sizeof (TREECELL))) == NULLCELL) {
    fprintf(stderr, "INTERNAL ERROR: malloc failure in MallocTcell()\n");
    return (NULLCELL);
  }

  CellLine(retval)  = linenum;
  CellType(retval)  = typ;
  if (str == NULLSTR)
    CellStr(retval)  = NULLSTR;
  else {
    if ((CellStr(retval)  = (char *)malloc(strlen(str) + 1)) == NULLSTR) {
      fprintf(stderr, "INTERNAL ERROR: malloc failure in MallocTcell()\n");
      free(retval);
      return (NULLCELL);
    }
    else
      strcpy(CellStr(retval), str);
  }

  SetNext(retval, NULLCELL);
  SetInfo0(retval, NULLCELL);
  SetInfo1(retval, NULLCELL);
  SetInfo2(retval, NULLCELL);
  SetInfo3(retval, NULLCELL);
  SetInfo4(retval, NULLCELL);
  SetInfo5(retval, NULLCELL);
  SetSig(retval, NULLSIG);
  SetTypList(retval, NULLTYPE);

  return (retval);
}

// free NODE
// parameter:
//    cell:  node cell
void FreeTcell(TCELLPNT cell)
{
  if (cell == NULLCELL)
    return;

  if (CellStr(cell) != NULLSTR)
    free(CellStr(cell));
  FreeSigList(CellSig(cell));
  FreeTypeList(CellTypList(cell));
  free(cell);
}

// free parse-tree
// parameter:
//    top:  top of parse tree
void FreeTree(TCELLPNT top)
{
  if (top == NULLCELL)
    return;

  FreeTree(NextCell(top));
  FreeTree(CellInfo0(top));
  FreeTree(CellInfo1(top));
  FreeTree(CellInfo2(top));
  FreeTree(CellInfo3(top));
  FreeTree(CellInfo4(top));
  FreeTree(CellInfo5(top));

  FreeTcell(top);
}

///////////////////////// SIGLIST type /////////////////////////

// make new signal list
// return:  Top of signal list
SIGLIST *MakeNewSigList(void)
{
  register SIGLIST  *retval = (SIGLIST *)malloc(sizeof (SIGLIST));
  if (retval != NULLSIG) {
    NextSig(retval)  = NULLSIG;
    SigStr(retval)  = NULLSTR;
  }
  else {
    fprintf(stderr, "INTERNAL ERROR: malloc failure in MakeNewSigList()\n");
  }
  return (retval);
}

// free signal list
// parameter:
//    top:  top of signal list
void  FreeSigList(SIGLIST *top)
{
  register  SIGLIST *ptr, *nextptr;
  if (top == NULLSIG)
    return;
  for (ptr = top; ptr != NULLSIG; ptr = nextptr) {
    nextptr  = NextSig(ptr);
    if (SigStr(ptr) != NULLSTR)
      free(SigStr(ptr));
    free(ptr);
  }
}

// append new signal/function/procedure name to signal list
// parameter:
//    top:  Top of signal list
//    str:  Signal/function/procedure name
//    typ:  Signal type
//    isreg:  True if the signal type is 'reg'
//    isinp:  True if the signal direction is input
//    isout:  True if the signal direction is output
//    isfunc:  True if the 'str' is function/procedure name
// return:  False if a signal with the same name is already in signal list
Boolean AppendSigList(SIGLIST *top, char *str, TCELLPNT typ, Boolean isreg, Boolean isinp, Boolean isout, Boolean isfunc)
{
  register SIGLIST *ptr;
  if (top == NULLSIG)
    return (False);

  for (ptr = top; NextSig(ptr) != NULLSIG; ptr = NextSig(ptr)) {
#ifdef UNIX
    if (!strcasecmp(str, SigStr(ptr)))
      return (False);
#endif
#ifdef WINDOWS
    if (!stricmp(str, SigStr(ptr)))
      return (False);
#endif
  }

  SigStr(ptr)    = (char *)malloc(strlen(str) + 1);
  strcpy(SigStr(ptr), str);
  SigType(ptr)  = typ;
  SigIsReg(ptr)  = isreg;
  SigIsInp(ptr)  = isinp;
  SigIsOut(ptr)  = isout;
  SigIsFunc(ptr)  = isfunc;

  NextSig(ptr)  = MakeNewSigList();

  return (True);
}

// search a signal/function/procedure in signal list (CASE sensitive)
// parameter:
//    top:  Top of signal list
//    str:  Signal/function/procedure name
// return:   If the given signal name is registered in the list, pointer to signal list cell.  Otherwise NULL.
SIGLIST *SearchSigList(SIGLIST *top, char *str)
{
  register SIGLIST *ptr;
  if (top == NULLSIG || str == NULLSTR)
    return (NULLSIG);

  for (ptr = top; NextSig(ptr) != NULLSIG; ptr = NextSig(ptr)) {
    if (!strcmp(str, SigStr(ptr)))
      return (ptr);
  }
  return (NULLSIG);
}

// search a signal/function/procedure in signal list (CASE insensitive)
// parameter:
//    top:  Top of signal list
//    str:  Signal/function/procedure name
// return:  If a similar (yet CASE is different) signal is registered in the list,
//      the corresponding signal name in the declaration part.  Otherwise NULL.
//      Note that NULL will be returned if the SAME signal name is found.
char *SearchSimilarSig(SIGLIST *top, char *str)
{
  register SIGLIST *ptr;
  if (top == NULLSIG || str == (char *)NULL)
    return ((char *)NULL);

  for (ptr = top; NextSig(ptr) != NULLSIG; ptr = NextSig(ptr)) {
    if (!strcmp(str, SigStr(ptr)))
      return ((char *)NULL);      // IGNORE the same signal

#ifdef UNIX
    if (!strcasecmp(str, SigStr(ptr)))
      return (SigStr(ptr));
#endif
#ifdef WINDOWS
    if (!stricmp(str, SigStr(ptr)))
      return (SigStr(ptr));
#endif
  }
  return ((char *)NULL);
}

// get signal name from a NODE
// parameter:
//    top:  Node cell
// return:  signal name
char *GetSigName(TCELLPNT top)
{
  if (top == NULLCELL)
    return ((char *)NULL);

  switch (CellType(top)) {
  case T_ID:
  case N_IDLIST:
    return (CellStr(top));

  case N_STDVECTOR:
  case N_STDELEMENT:
    return (CellStr(CellInfo0(top)));

  default:
    break;
  }
  return ((char *)NULL);
}

// register I/O signals into signal list (called in parser part)
// parameter:
//    top:  Top of signal list
//    port:  Top of parse tree of I/O declarations (N_SIGDEF)
void RegisterIoSignals(SIGLIST *top, TCELLPNT port)
{
  register TCELLPNT  ioitem, idlist;
  if (top == (SIGLIST *)NULL || port == NULLCELL)
    return;
  if (CellType(port) != N_PORTDEF)
    return;

  // register signals
  for (ioitem = CellInfo0(port); ioitem != NULLCELL; ioitem = NextCell(ioitem)) {  // io_item
    if (CellType(ioitem) != N_SIGDEF)
      continue;
    /* signal item: N_SIGDEF info0=idlist, info1=sigdir, info2=sigtype */
    for (idlist = CellInfo0(ioitem); idlist != NULLCELL; idlist = NextCell(idlist)) {  // id_list
      if (CellType(CellInfo1(ioitem)) == T_IN) {
        if (AppendSigList(ParseSigListTop,CellStr(idlist),CellInfo2(ioitem),False,True,False,False) == False) {  /* (input) */
        //  ParseError = True;
        //  fprintf(stderr, "ERROR: in line %d, Duplicate signal declaration (Verilog is case sensitive).\n", yylexlinenum);
          yyerror("$Duplicate signal declaration (Verilog is case sensitive).\n");
        }
      }
      else if (CellType(CellInfo1(ioitem)) == T_OUT) {
        if (AppendSigList(ParseSigListTop,CellStr(idlist),CellInfo2(ioitem),False,False,True,False) == False) {  /* (output) */
        //  ParseError = True;
        //  fprintf(stderr, "ERROR: in line %d, Duplicate signal declaration (Verilog is case sensitive).\n", yylexlinenum);
          yyerror("$Duplicate signal declaration (Verilog is case sensitive).\n");
        }
      }
      else if (CellType(CellInfo1(ioitem)) == T_INOUT) {
        if (AppendSigList(ParseSigListTop,CellStr(idlist),CellInfo2(ioitem),False,True,True,False) == False) {  /* (inout) */
        //  ParseError = True;
        //  fprintf(stderr, "ERROR: in line %d, Duplicate signal declaration (Verilog is case sensitive).\n", yylexlinenum);
          yyerror("$Duplicate signal declaration (Verilog is case sensitive).\n");
        }
      }
    }
  }
}

///////////////////////// COMMENTLIST type /////////////////////////

// make new comment list
// return:  top of comment list
COMMENTLIST *MakeNewCommentList(void)
{
  register  COMMENTLIST  *retval;
  retval  = (COMMENTLIST *)malloc(sizeof (COMMENTLIST));
  if (retval != (COMMENTLIST *)NULL) {
    NextComment(retval)  = (COMMENTLIST *)NULL;
    CommentStr(retval)  = NULLSTR;
  }
  else {
    fprintf(stderr, "INTERNAL ERROR: malloc failure in MakeNewCommentList()\n");
  }
  return (retval);
}

// free comment list
// parameter:
//    top:  top of comment list
void  FreeCommentList(COMMENTLIST *top)
{
  register  COMMENTLIST *ptr, *nextptr;
  if (top == (COMMENTLIST *)NULL)
    return;
  for (ptr = top; ptr != (COMMENTLIST *)NULL; ptr = nextptr) {
    nextptr  = NextComment(ptr);
    if (CommentStr(ptr) != NULLSTR)
      free(CommentStr(ptr));
    free(ptr);
  }
}

// append new comment to comment list
// parameter:
//    top:    top of comment list
//    str:    comment
//    linenum:  HDL line number (NOTE: pointer to int value)
void RegisterComment(COMMENTLIST *top, char *str, int *linenum)
{
  register COMMENTLIST *ptr;
  if (top == (COMMENTLIST *)NULL)
    return;

  // set pointer to the last of the comment list
  for (ptr = top; NextComment(ptr) != (COMMENTLIST *)NULL; ptr = NextComment(ptr))
    ;

  CommentLine(ptr)  = *linenum;
  CommentStr(ptr)    = (char *)malloc(strlen(str) + 10);
  if (*str == '-') {  // (if comment (+ empty lines))
    char *ptr2;
    int  i;
    strcpy(CommentStr(ptr), str);
    *CommentStr(ptr)    = '/';
    *(CommentStr(ptr) + 1)  = '/';
    for (ptr2 = CommentStr(ptr), i = 0; *ptr2 != '\0'; ptr2++) {
      if (*ptr2 == '\n')
        i++;
    }
    (*linenum) += i;
  }
  else {        // (if empty lines)
    strcpy(CommentStr(ptr), str);
    (*linenum) += strlen(str);
  }

  CommentPrn(ptr)    = False;
  NextComment(ptr)  = MakeNewCommentList();
}

///////////////////////// TYPELIST type /////////////////////////

// make new type-list
// return:  top of type-list
TYPELIST *MakeNewTypeList(void)
{
  register TYPELIST  *retval = (TYPELIST *)malloc(sizeof (TYPELIST));
  if (retval != NULLTYPE) {
    NextType(retval)  = NULLTYPE;
    TypeName(retval)  = NULLSTR;
  }
  else {
    fprintf(stderr, "INTERNAL ERROR: malloc failure in MakeNewTypeList()\n");
  }
  return (retval);
}

// free type list
// parameter:
//    top:  top of type-list
void  FreeTypeList(TYPELIST *top)
{
  register  TYPELIST *ptr, *nextptr;
  if (top == NULLTYPE)
    return;
  for (ptr = top; ptr != NULLTYPE; ptr = nextptr) {
    nextptr  = NextType(ptr);
    if (TypeName(ptr) != NULLSTR)
      free(TypeName(ptr));
    free(ptr);
  }
}

// append new type definition to type-list
// parameter:
//    top:  top of type-list
//    typ:  top of parse-tree of a type declaration (T_TYPE)
void AppendTypeList(TYPELIST *top, TCELLPNT typ)
{
  register TYPELIST *ptr;
  register char    *name;
  if (top == (TYPELIST *)NULL || typ == NULLCELL)
    return;

  name  = CellStr(CellInfo0(typ));
  for (ptr = top; NextType(ptr) != (TYPELIST *)NULL; ptr = NextType(ptr)) {
    if (!strcmp(name, TypeName(ptr)))
      return;
  }

  TypeName(ptr)  = (char *)malloc(strlen(name) + 1);
  strcpy(TypeName(ptr), name);
  TypeInfo(ptr)  = CellInfo1(typ);
  NextType(ptr)  = MakeNewTypeList();
}


// search a type info in type-list
// parameter:
//    top:  Top of type-list
//    name:  type name
// return:   If the given type name is registered in the list, pointer to type-list cell.  Otherwise NULL.
TYPELIST *SearchTypeList(TYPELIST *top, char *name)
{
  register TYPELIST *ptr;
  if (top == (TYPELIST *)NULL || name == (char *)NULL)
    return ((TYPELIST *)NULL);

  for (ptr = top; NextType(ptr) != (TYPELIST *)NULL; ptr = NextType(ptr)) {
    if (!strcmp(name, TypeName(ptr)))
      return (ptr);
  }

  return ((TYPELIST *)NULL);
}

// (function for evaluating enum type)
// count # of enum type element
// parameter:
//    top:  Top of enum declaration (N_ENUMTYPE)
// return:  # of enum type element
int EnumItemNum(TCELLPNT top)
{
  register TCELLPNT  ptr;
  register int    itemnum;
  if (top == NULLCELL)
    return (0);
  if (CellType(top) != N_ENUMTYPE)
    return (0);
  for (ptr = CellInfo0(top), itemnum = 0; ptr != NULLCELL; ptr = NextCell(ptr), itemnum++)
    ;
  return (itemnum);
}

// (function for evaluating type definitions)
// evaluate integer value
// parameter:
//    top:  Top of parse tree
//    ret:  Pointer to result variable
// return:  True if evaluating value is successful without overflow.
Boolean EvalValue(TCELLPNT top, int *ret)
{
  if (top == NULLCELL)
    return (False);
  switch (CellType(top)) {
  case T_PLUS:
    {
      int  lval, rval;
      if (EvalValue(CellInfo0(top),&lval) == True && EvalValue(CellInfo1(top),&rval) == True) {
        (*ret)  = lval + rval;
        return (True);
      }
      return (False);
    }

  case T_MINUS:
    {
      int  lval, rval;
      if (EvalValue(CellInfo0(top),&lval) == True && EvalValue(CellInfo1(top),&rval) == True) {
        (*ret)  = lval - rval;
        return (True);
      }
      return (False);
    }

  case T_MULT:
    {
      int  lval, rval;
      if (EvalValue(CellInfo0(top),&lval) == True && EvalValue(CellInfo1(top),&rval) == True) {
        (*ret)  = lval * rval;
        return (True);
      }
      return (False);
    }

  case N_DUMMY:
  case N_PAREN:
    return (EvalValue(CellInfo0(top), ret));

  case T_BINDIGIT:
    {
      register int  bit;
      register char  *ptr;
      ptr = CellStr(top);
      ptr++;  // '''
      for (*ret = 0, bit = 1; *ptr != '\0' && *ptr != '\'' && *ptr != '"' && bit < 32; ptr++, bit++) {
        (*ret)  = (*ret) * 2 + (*ptr - '0');
      }
      if (bit >= 32)
        return (False);    // overflow
      return (True);
    }

  case T_HEXDIGIT:
    {
      register int  bit;
      register char  *ptr;
      ptr = CellStr(top);
      ptr++;  // 'X'
      ptr++;  // '"'
      for (*ret = 0, bit = 4; *ptr != '\0' && *ptr != '"' && bit < 32; ptr++, bit += 4) {
        if ('0' <= *ptr && *ptr <= '9')      (*ret)  = (*ret) * 16 + (*ptr - '0');
        else if ('a' <= *ptr && *ptr <= 'f')  (*ret)  = (*ret) * 16 + (*ptr - 'a' + 10);
        else if ('A' <= *ptr && *ptr <= 'F')  (*ret)  = (*ret) * 16 + (*ptr - 'A' + 10);
      }
      if (bit >= 32)
        return (False);    // overflow
      return (True);
    }

  case T_DECDIGIT:
    {
      register int  bit;
      register char  *ptr;
      register Boolean flag = False;  // plus
      ptr = CellStr(top);
      if (*ptr == '-') {
        ptr++;
        if (flag == False)
          flag = True;  // minus
        else
          flag = False;
      }
      for (*ret = 0, bit = 4; *ptr != '\0' && bit < 32; ptr++, bit += 4) {
        (*ret)  = (*ret) * 10 + (*ptr - '0');
      }
      if (bit >= 32)
        return (False);    // overflow
      if (flag == True)
        *ret *= (-1);
      return (True);
    }

  default:
    break;
  }
  return (False);
}

// (function for evaluating type definitions)
// evaluate integer value range
// parameter:
//    top:  Top of parse tree (T_TO or T_DOWNTO or NULLCELL)
// return:  eval result value (positive).  -1 if a error happens.
int ChkIntegerRange(TCELLPNT top)
{
  int  from, to;
  if (top == NULLCELL)
    return (-1);
  if (CellType(top) == T_TO) {
    if (EvalValue(CellInfo0(top), &from) == True && EvalValue(CellInfo1(top), &to) == True)
      return (to - from + 1);
    else 
      return (-1);
  }
  else if (CellType(top) == T_DOWNTO) {
    if (EvalValue(CellInfo0(top), &from) == True && EvalValue(CellInfo1(top), &to) == True)
      return (from - to + 1);
    else
      return (-1);
  }
  return (-1);
}

// (function for evaluating type definitions)
// evaluate integer value bit width
// parameter:
//    top:  integer value range
// return:  eval result value (positive).  -1 if a error happens.
int ChkIntegerBitLength(int range)
{
  register  int  len, cmpval;
  if (range <= 0)
    return (-1);
  for (cmpval = 2, len = 1; range > cmpval && len < 31; cmpval *= 2, len++)
    ;
  if (len < 32)
    return (len);
  return (-1);  // overflow
}

///////////////////////// EXTRACTLIST type /////////////////////////

// make new loop extract list
// return:  top of list
EXTRACTLIST *MakeNewExtractList(void)
{
  register  EXTRACTLIST  *retval;
  retval  = (EXTRACTLIST *)malloc(sizeof (EXTRACTLIST));
  if (retval != (EXTRACTLIST *)NULL) {
    NextExtract(retval)  = (EXTRACTLIST *)NULL;
  }
  else {
    fprintf(stderr, "INTERNAL ERROR: malloc failure in MakeNewExtractList()\n");
  }
  return (retval);
}

// free loop extract list
// parameter:
//    top:  top of list
void  FreeExtractList(EXTRACTLIST *top)
{
  register  EXTRACTLIST *ptr, *nextptr;
  if (top == (EXTRACTLIST *)NULL)
    return;
  for (ptr = top; ptr != (EXTRACTLIST *)NULL; ptr = nextptr) {
    nextptr  = NextType(ptr);
    free(ptr);
  }
}

// append new loop-extract info to extract-list
// parameter:
//    top:  top of loop extract list
//    id:    loop variable (a number will be substituted)
//    num:  loop value
void AppendExtractList(EXTRACTLIST *top, TCELLPNT id, int num)
{
  register EXTRACTLIST  *ptr;
  if (top == (EXTRACTLIST *)NULL || id == NULLCELL)
    return;
  if (CellType(id) != T_ID)
    return;
  for (ptr = top; NextExtract(ptr) != (EXTRACTLIST *)NULL; ptr = NextExtract(ptr))
    ;
  ExtractId(ptr)    = id;
  ExtractNum(ptr)    = num;
  NextExtract(ptr)  = MakeNewExtractList();
}

// remove a loop-extract info from extract-list
// parameter:
//    top:  top of loop extract list
//    id:    loop variable (a number will be substituted)
void RemoveExtractList(EXTRACTLIST *top, TCELLPNT id)
{
  register EXTRACTLIST  *ptr;
  if (top == (EXTRACTLIST *)NULL || id == NULLCELL)
    return;
  if (CellType(id) != T_ID)
    return;
  for (ptr = top; NextExtract(ptr) != (EXTRACTLIST *)NULL; ptr = NextExtract(ptr)) {
    if (ExtractId(ptr) == id) {
      register EXTRACTLIST  *nextptr;
      nextptr  = NextExtract(ptr);
      ExtractId(ptr)    = ExtractId(nextptr);
      ExtractNum(ptr)    = ExtractNum(nextptr);
      NextExtract(ptr)  = NextExtract(nextptr);
      free(nextptr);
      return;
    }
  }
}

// determine substitute a value or not
// parameter:
//    top:  top of loop extract list
//    nd:    Node of signal name
//    num:  pointer to subst value buffer
// return:  True if the given signal name must be replaced by a number
Boolean ChkSubstVal(EXTRACTLIST *top, TCELLPNT nd, int *num)
{
  register EXTRACTLIST  *ptr;
  if (top == (EXTRACTLIST *)NULL || nd == NULLCELL)
    return (False);
  if (CellType(nd) != T_ID)
    return (False);
  for (ptr = top; NextExtract(ptr) != (EXTRACTLIST *)NULL; ptr = NextExtract(ptr)) {
    if (!strcmp(CellStr(ExtractId(ptr)), CellStr(nd))) {
      *num  = ExtractNum(ptr);
      return (True);
    }
  }
  return (False);
}


//////////////////////////////////////////////////////////////////////
// Write output file
//////////////////////////////////////////////////////////////////////

// print tabs
// parameter:
//    fp:    Output file pointer
//    id:    Number of tabs
static void fprintfTab(FILE *fp, int indent)
{
  register int  i;
  for (i = 0; i < indent; i++)
    fprintf(fp, "\t");
}

// print warnings
// parameter:
//    fp:    Output file pointer
//    line:  HDL line number
//    msg:  Warning message. Format: "%s\0%s" where the first string is warning number and the second is message body.
static void fprintfWarning(FILE *fp, int line, char *msg)
{
  char  *msg_top, *id;
  id  = msg;
  for (msg_top = msg; *msg_top != '\0'; msg_top++)
    ;
  msg_top++;

  if (line > 0) {
    fprintf(fp, "\n// WARNING(%s) in line %d: %s\n", id, line, msg_top);
    fprintf(stderr, "WARNING(%s) in line %d: %s\n", id, line, msg_top);
  }
  else {
    fprintf(fp, "\n// WARNING(%s): %s\n", id, msg_top);
    fprintf(stderr, "WARNING(%s): %s\n", id, msg_top);
  }
}

// print comments
// parameter:
//    fp:      Output file pointer
//    top:    Top of comment list
//    linenum:  HDL line number (NOTE: comments UNTIL this line will be printed)
//    pretab:    Tab number (usually 0)
// return:  True if some comments are printed out.
static Boolean fprintfComment(FILE *fp, COMMENTLIST *top, int linenum, int pretab)
{
  register COMMENTLIST *ptr;
  register Boolean  retval;
  if (top == (COMMENTLIST *)NULL || linenum <= 0)
    return (False);
  for (ptr = top, retval = False; NextComment(ptr) != (COMMENTLIST *)NULL; ptr = NextComment(ptr)) {
    register int i;
    if (CommentPrn(ptr) == True)
      continue;
    if (linenum > 0 && linenum < CommentLine(ptr))
      return (retval);
    if (*(CommentStr(ptr)) != '\n') {
      for (i = 0; i < pretab; i++)
        fprintf(fp, "\t");
    }
    fprintf(fp, "%s", CommentStr(ptr));
    CommentPrn(ptr)  = True;
    retval = True;
  }
  return (retval);
}

// print enum declaration
// parameter:
//    fp:      Output file pointer
//    top:    Top of parse tree of enum declaration (N_ENUMTYPE)
//    com:    Top of comment list
//    indent:    Tab number
static void fprintfEnumDef(FILE *fp, TCELLPNT top, COMMENTLIST *com, int indent)
{
  register TCELLPNT  ptr;
  register int    itemnum, item;
  if (top == NULLCELL)
    return;
  if (CellType(top) != N_ENUMTYPE)
    return;
  itemnum  = EnumItemNum(top);
  for (ptr = CellInfo0(top), item = 1; ptr != NULLCELL; ptr = NextCell(ptr), item++) {
    register int  bit;
    fprintfTab(fp, indent);
    fprintf(fp, "parameter %s\t= %d'b", CellStr(ptr), itemnum);  // (T_ID)
    for (bit = 1; bit <= itemnum; bit++) {
      if (item == bit)  fprintf(fp, "1");
      else        fprintf(fp, "0");
    }
    fprintf(fp, ";");

    if (fprintfComment(fp, com, CellLine(ptr), indent) == False)
      fprintf(fp, "\n");
  }
}


// print verilog output
// parameter:
//    fp:      Output file pointer
//    sw:      print sw: False to disable print (only to traverse parse tree)
//    top:    Top of parse tree
//    sig:    Top of signal list
//    com:    Top of comment list
//    typ:    Top of type list
//    ext:    Top of extract list
//    indent:    Tab number
//    flag0:    Pointer to (int)flag0, experession type: arithmetic or logical
//    flag1:    Pointer to (int)flag1, general purpose variable:
//            1. N_CONDASSIGN,N_WHENCOND    pointer to last condition
//            2. N_PORTMAP          port map type flag (0:"sig,.." 1:"port=>sig,...")
//    flag2:    Pointer to (int)flag2, general purpose variable: port map 1st data flag
//    flag3:    Pointer to (int)flag3, general purpose variable: pointer to parse tree of L-value in substitution statement
//    flag4:    Pointer to (int)flag4, general purpose variable: pointer to array-type definition (point range of array)
//    flag5:    Pointer to (int)flag5, uncheck case (when 1)
static void fprintfVerilog(FILE *fp, Boolean sw, TCELLPNT top, SIGLIST *sig, COMMENTLIST *com, TYPELIST *typ, EXTRACTLIST *ext, int indent, int *flag0, int *flag1, int *flag2, int *flag3, int *flag4, int *flag5)
{
  if (top == NULLCELL)
    return;

  switch (CellType(top)) {
  case N_DUMMY:
    fprintfVerilog(fp, sw, NextCell(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
    break;

///////////////////////////////////////////////////////////////////////////////
// ALL
///////////////////////////////////////////////////////////////////////////////
  case N_MODULE:
    *flag5 = 0;    // check CASE

    /* one design unit: N_MODULE flag0=header, flag1=entity, flag2=arch */
    fprintfVerilog(fp, sw, CellInfo1(top), CellSig(top), com, CellTypList(top), ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);    // entity
    fprintfVerilog(fp, sw, CellInfo2(top), CellSig(top), com, CellTypList(top), ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // architecture

    if (sw == True && NextCell(top) != NULLCELL) fprintf(fp, "\n");
    fprintfVerilog(fp, sw, NextCell(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);    // (next module)
    break;

  case T_ARCHITECTURE:
    /* architecture part: T_ARCHITECTURE flag0=declaration, flag1=body */
    {
      register TCELLPNT  ptr;
      // declaration
      for (ptr = CellInfo0(top); ptr != NULLCELL; ptr = NextCell(ptr)) {
        if (sw == True) fprintfComment(fp, com, CellLine(ptr) - 1, indent);  // print remaining comments
        fprintfVerilog(fp, sw, ptr, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
      }
      if (sw == True) fprintf(fp, "\n");

      // body
      for (ptr = CellInfo1(top); ptr != NULLCELL; ptr = NextCell(ptr)) {
        if (sw == True) fprintfComment(fp, com, CellLine(ptr) - 1, indent);  // print remaining comments
        fprintfVerilog(fp, sw, ptr, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
      }

      if (sw == True) fprintf(fp, "endmodule\n");
    }
    break;

  case T_BLOCK:
    /* block: T_BLOCK flag0=ID(label), flag1=declaration, flag2=body */
    {
      register TCELLPNT  ptr;
      // declaration
      for (ptr = CellInfo1(top); ptr != NULLCELL; ptr = NextCell(ptr)) {
        if (sw == True) fprintfComment(fp, com, CellLine(ptr) - 1, indent);  // print remaining comments
        fprintfVerilog(fp, sw, ptr, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
      }
      if (sw == True) fprintf(fp, "\n");

      // body
      for (ptr = CellInfo2(top); ptr != NULLCELL; ptr = NextCell(ptr)) {
        if (sw == True) fprintfComment(fp, com, CellLine(ptr) - 1, indent);  // print remaining comments
        fprintfVerilog(fp, sw, ptr, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
      }
    }
    break;

  case N_FORGENERATE:
    /* for generate: N_FORGENERATE flag0=ID(label), flag1=ID(variable), flag2=width, info3=body */
    if (sw == True) {
      register TCELLPNT  width;
      register TCELLPNT  ptr;
      register int    i;
      int          from, to;

      width = CellInfo2(top);
      if (EvalValue(CellInfo0(width), &from) == True && EvalValue(CellInfo1(width), &to) == True) {  // (can extract)
        // extract loop
        if (to < from) {
          register int  tmp;
          tmp    = from;
          from  = to;
          to    = tmp;
        }
        for (i = from; i <= to; i++) {
          AppendExtractList(ext, CellInfo1(top), i);
          for (ptr = CellInfo3(top); ptr != NULLCELL; ptr = NextCell(ptr)) {
            fprintfVerilog(fp, sw, ptr, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write body
          }
          fprintf(fp, "\n");
          RemoveExtractList(ext, CellInfo1(top));
        }
      }
      else {  // (can not extract loop: write for statement)
fprintfWarning(fp, CellLine(top), WARN_0_PUTFOR);
        fprintfComment(fp, com, CellLine(top) - 1, indent);

        fprintfTab(fp, indent);
        fprintf(fp, "integer %s;\n", CellStr(CellInfo1(top)));

        fprintfTab(fp, indent);
        fprintf(fp, "for (%s = ", CellStr(CellInfo1(top)));
        fprintfVerilog(fp, sw, CellInfo0(CellInfo2(top)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
        if (CellType(CellInfo2(top)) == T_TO) {
          fprintf(fp, "; %s <= ", CellStr(CellInfo1(top)));
          fprintfVerilog(fp, sw, CellInfo1(CellInfo2(top)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
          fprintf(fp, "; %s = %s + 1) begin\n", CellStr(CellInfo1(top)), CellStr(CellInfo1(top)));
        }
        else {  // T_DOWNTO
          fprintf(fp, "; %s >= ", CellStr(CellInfo1(top)));
          fprintfVerilog(fp, sw, CellInfo1(CellInfo2(top)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
          fprintf(fp, "; %s = %s - 1) begin\n", CellStr(CellInfo1(top)), CellStr(CellInfo1(top)));
        }

        for (ptr = CellInfo3(top); ptr != NULLCELL; ptr = NextCell(ptr)) {
          fprintfVerilog(fp, sw, ptr, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);    // write body
        }

        fprintfTab(fp, indent);
        fprintf(fp, "end\n");
      }
    }
    break;

  case N_FORLOOP:
    /* for loop: T_FOR info0=ID(label,NULL), info1=ID(variable), info2=width, info3=body */
    if (sw == True) {
      register TCELLPNT  width;
      register TCELLPNT  ptr;
      register int    i;
      int          from, to;

      width = CellInfo2(top);
      if (EvalValue(CellInfo0(width), &from) == True && EvalValue(CellInfo1(width), &to) == True) {
        // extract loop
        if (to < from) {
          register int  tmp;
          tmp    = from;
          from  = to;
          to    = tmp;
        }
        for (i = from; i <= to; i++) {
          AppendExtractList(ext, CellInfo1(top), i);
          for (ptr = CellInfo3(top); ptr != NULLCELL; ptr = NextCell(ptr)) {
            fprintfVerilog(fp, sw, ptr, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write body
          }
          fprintf(fp, "\n");
          RemoveExtractList(ext, CellInfo1(top));
        }
      }
      else {
        fprintfComment(fp, com, CellLine(top) - 1, indent);

        fprintfTab(fp, indent );
        fprintf(fp, "integer %s;\n", CellStr(CellInfo1(top)));
fprintfWarning(fp, -1, WARN_1_PUTLOOP);

        fprintfTab(fp, indent);
        fprintf(fp, "for (%s = ", CellStr(CellInfo1(top)));
        fprintfVerilog(fp, sw, CellInfo0(CellInfo2(top)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
        if (CellType(CellInfo2(top)) == T_TO) {
          fprintf(fp, "; %s <= ", CellStr(CellInfo1(top)));
          fprintfVerilog(fp, sw, CellInfo1(CellInfo2(top)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
          fprintf(fp, "; %s = %s + 1) begin\n", CellStr(CellInfo1(top)), CellStr(CellInfo1(top)));
        }
        else {  // T_DOWNTO
          fprintf(fp, "; %s >= ", CellStr(CellInfo1(top)));
          fprintfVerilog(fp, sw, CellInfo1(CellInfo2(top)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
          fprintf(fp, "; %s = %s - 1) begin\n", CellStr(CellInfo1(top)), CellStr(CellInfo1(top)));
        }

        for (ptr = CellInfo3(top); ptr != NULLCELL; ptr = NextCell(ptr)) {
          fprintfVerilog(fp, sw, CellInfo3(top), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // write body
        }

        fprintfTab(fp, indent);
        fprintf(fp, "end\n");
      }
    }
    break;

  case T_FUNCTION:
    /* function def: T_FUNCTION info0=ID, info1=port, info2=type, info3=variable, info4=body, info5=ret */
    if (sw == True) {
      register TCELLPNT  item;

      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
      fprintf(fp, "function");
      fprintf(fp, " [?:?] ");
      fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);    // func name
      fprintf(fp, ";");
fprintfWarning(fp, CellLine(top), WARN_2_RETWIDTH);

      fprintfComment(fp, com, CellLine(top), indent);
      for (item = CellInfo1(top); item != NULLCELL; item = NextCell(item)) {
        /* parameter item: N_PARAMDEF info0=idlist, info1=sigtype, info2=defaultval */
        fprintfComment(fp, com, CellLine(item) - 1, indent);
        fprintfTab(fp, indent + 1);
        fprintf(fp, "input\t");

        // write type
        fprintfVerilog(fp, sw, CellInfo1(item), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
        if (sw == True) fprintf(fp, "\t");
        // write idlist
        fprintfVerilog(fp, sw, CellInfo0(item), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);

        if (sw == True) {
          fprintf(fp, ";");
          if (fprintfComment(fp, com, CellLine(item), 1) == False)
            fprintf(fp, "\n");
        }
      }

      for (item = CellInfo3(top); item != NULLCELL; item = NextCell(item)) {  /* variables */
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // variable
      }

      fprintfTab(fp, indent);
      fprintf(fp, "begin\n");

      for (item = CellInfo4(top); item != NULLCELL; item = NextCell(item)) {  /* body */
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // body
      }

      fprintf(fp, "\n");

      /* function return: T_RETURN info0=exp */
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent + 1);
      fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);    // func name
      fprintf(fp, " = ");
      fprintfVerilog(fp, sw, CellInfo0(CellInfo5(top)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);    // func retvalue
      fprintf(fp, ";");
      if (fprintfComment(fp, com, CellLine(CellInfo5(top)), 1) == False)
        fprintf(fp, "\n");

      fprintfTab(fp, indent);
      fprintf(fp, "end\n");
      fprintfTab(fp, indent);
      fprintf(fp, "endfunction\n");
    }
    break;

  case N_SIGDEF:
    /* (none) */
    break;

  case N_CALLPROC:
    /* N_CALLPROC flag0=ID, flag1=port */
    if (sw == True) {
      register TCELLPNT  item;
      fprintfComment(fp, com, CellLine(top) - 1, indent);

fprintfWarning(fp, CellLine(top), WARN_3_PROCPARAM);
      fprintfTab(fp, indent);
      fprintf(fp, "%s(", CellStr(CellInfo0(top)));    // procedure name

      for (item = CellInfo1(top); item != NULLCELL; item = NextCell(item)) {
        if (CellType(item) == N_PORTITEMNAME)
          fprintfVerilog(fp, sw, CellInfo1(item), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // name
        else
          fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // name

        if (NextCell(item) != NULLCELL)
          fprintf(fp, ",");
      }

      fprintf(fp, ");");

      if (fprintfComment(fp, com, CellLine(top), indent) == False)
        fprintf(fp, "\n");
    }
    *flag0 = 0;  // arithmetic
    break;

  case T_PROCEDURE:
    /* procedure def: T_PROCEDURE info0=ID, info1=port, info2=variale, info3=body */
    if (sw == True) {
      register TCELLPNT  item;

      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
      fprintf(fp, "task ");
      fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);    // proc name
      fprintf(fp, ";\n");

      fprintfComment(fp, com, CellLine(top), indent);
      for (item = CellInfo1(top); item != NULLCELL; item = NextCell(item)) {
        /* parameter item: N_PARAMDEF info0=idlist, info1=sigtype, info2=defaultval */
        fprintfComment(fp, com, CellLine(item) - 1, indent);
        fprintfTab(fp, indent + 1);

        // write type
        fprintfVerilog(fp, sw, CellInfo1(item), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
        if (sw == True) fprintf(fp, "\t");
        // write idlist
        fprintfVerilog(fp, sw, CellInfo0(item), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);

        if (sw == True) {
          fprintf(fp, ";");
          if (fprintfComment(fp, com, CellLine(item), 1) == False)
            fprintf(fp, "\n");
        }
      }

      for (item = CellInfo2(top); item != NULLCELL; item = NextCell(item)) {  /* variables */
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // variable
      }

      fprintfTab(fp, indent);
      fprintf(fp, "begin\n");

      for (item = CellInfo3(top); item != NULLCELL; item = NextCell(item)) {  /* body */
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // body
      }

      fprintf(fp, "\n");

      fprintfTab(fp, indent);
      fprintf(fp, "end\n");
      fprintfTab(fp, indent);
      fprintf(fp, "endtask\n");
    }

    break;

///////////////////////////////////////////////////////////////////////////////
// entity part
///////////////////////////////////////////////////////////////////////////////
  case T_ENTITY:
    /* entity part: T_ENTITY flag0=ID, flag1=generic, flag2=port */
    if (sw == True) {
      fprintfComment(fp, com, CellLine(top), indent);
      fprintf(fp, "module %s", CellStr(CellInfo0(top)));
    }

    // write port name
    if (CellInfo2(top) != NULLCELL) {
      register TCELLPNT  parameter;
      if (sw == True) fprintf(fp, "(");
      for (parameter = CellInfo0(CellInfo2(top)); parameter != NULLCELL;) {  // CellInfo0(CellInfo2(top)) == parameter_item
        fprintfVerilog(fp, sw, CellInfo0(parameter), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write id_list
        if ((parameter = NextCell(parameter)) != NULLCELL) {
          if (sw == True) fprintf(fp, ", ");
        }
      }
      if (sw == True) fprintf(fp, ");\n");
    }

    // write generic parameters
    if (CellInfo1(top) != NULLCELL) {
      fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);
      if (sw == True) fprintf(fp, "\n");  // (region separator)
    }

    // write signal list
    if (CellInfo2(top) != NULLCELL) {
      fprintfVerilog(fp, sw, CellInfo2(top), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);
      if (sw == True) fprintf(fp, "\n");  // (region separator)
    }
    break;

  case N_GENERICDEF:
    /* generic part: N_GENERICDEF flag0=parameter */
    {
      register TCELLPNT  parameter;
      for (parameter = CellInfo0(top); parameter != NULLCELL; parameter = NextCell(parameter)) {
        fprintfVerilog(fp, sw, parameter, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
      }
    }
    break;

  case N_PARAMDEF:
    /* parameter item: N_PARAMDEF flag0=idlist, flag1=sigtype, flag2=defaultval */
    if (sw == True) {
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
      fprintf(fp, "parameter\t");
    }
    // write id_list
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
    // write default value
    if (CellInfo2(top) != NULLCELL) {
      if (sw == True) fprintf(fp, "\t= ");
      fprintfVerilog(fp, sw, CellInfo2(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
      if (sw == True) fprintf(fp, ";");
      if (fprintfComment(fp, com, CellLine(CellInfo2(top)), 1) == False)
        fprintf(fp, "\n");
    }
    else
      fprintf(fp, "\n");
    break;

  case N_PORTDEF:
    /* port part: N_PORTDEF flag0=io_item */
    {
      register TCELLPNT  item, id;
      for (item = CellInfo0(top); item != NULLCELL; item = NextCell(item)) {
        if (sw == True) {
          fprintfComment(fp, com, CellLine(item) - 1, indent);
          fprintfTab(fp, indent);
        }
        // write direction
        fprintfVerilog(fp, sw, CellInfo1(item), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
        if (sw == True) fprintf(fp, "\t");
        // write type
        fprintfVerilog(fp, sw, CellInfo2(item), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
        if (sw == True) fprintf(fp, "\t");
        // write idlist
        fprintfVerilog(fp, sw, CellInfo0(item), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);

        if (sw == True) {
          fprintf(fp, ";");
          if (fprintfComment(fp, com, CellLine(item), 1) == False)
            fprintf(fp, "\n");
        }

      // write additional "reg" definition for output signals
        if (sw == True) {
          id  = CellInfo0(item);    // idlist
          for (; id != NULLCELL; id = NextCell(id)) {  // for each signal
            register SIGLIST  *ptr;
            if ((ptr = SearchSigList(sig, GetSigName(id)) ) != NULLSIG) {
              if (SigIsReg(ptr) == True && SigIsOut(ptr) == True) {
                fprintfTab(fp, indent);
                fprintf(fp, "reg\t");
                // write type
                fprintfVerilog(fp, sw, CellInfo2(item), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
                fprintf(fp, "\t%s;", GetSigName(id));
                fprintf(fp, "\t\t// appended automatically by vhdl2verilog.\n");
              }
            }
          } // end of for loop;
        }

      }
    }
    break;

  case N_IDLIST:
    {  /* signal/parameter names: N_IDLIST */
      register TCELLPNT  ptr;
      for (ptr = top; ptr != NULLCELL; ptr = NextCell(ptr)) {
        if (sw == True) fprintf(fp, "%s", CellStr(ptr));
        if (NextCell(ptr) != NULLCELL) {
          if (sw == True) fprintf(fp, ", ");
        }
      }
    }
    break;

///////////////////////////////////////////////////////////////////////////////
// declaration part
///////////////////////////////////////////////////////////////////////////////
  case T_CONSTANT:
    if (sw == True) {
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
      fprintf(fp, "parameter\t%s\t= ", CellStr(CellInfo0(top)));
    }

    // write value
    fprintfVerilog(fp, sw, CellInfo2(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
    if (sw == True) {
      fprintf(fp, ";");
      if (fprintfComment(fp, com, CellLine(CellInfo2(top)), 1) == False)
        fprintf(fp, "\n");
    }
    break;

  case T_SIGNAL:
    if (sw == True) {
      TCELLPNT  id;
      Boolean    is_first;
      Boolean    last_reg_flag;

      fprintfComment(fp, com, CellLine(top) - 1, indent);

      is_first  = True;
      id      = CellInfo0(top);    // idlist

      for (; id != NULLCELL; id = NextCell(id)) {  // for each signal
        register SIGLIST  *ptr;
        if ((ptr = SearchSigList(sig, GetSigName(id)) ) != NULLSIG) {
          if (is_first == True) {
            is_first    = False;
            last_reg_flag  = SigIsReg(ptr);

            fprintfTab(fp, indent);

            if (SigIsReg(ptr) == True)
              fprintf(fp, "reg \t");
            else
              fprintf(fp, "wire\t");

            // write type
            *flag4  = (int)(NULLCELL);
            fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
            fprintf(fp, "\t");

            // write name
            fprintf(fp, "%s", CellStr(id));

            // write array range (if exist)
            if (*flag4 != (int)NULLCELL && sw == True) {
              TCELLPNT  range;
              range  = (TCELLPNT)(*flag4, flag5);
              // write width
              fprintf(fp, "\t");
              fprintf(fp, "[");
              fprintfVerilog(fp, sw, CellInfo0(range), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write range value
              fprintf(fp, ":");
              fprintfVerilog(fp, sw, CellInfo1(range), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write range value
              fprintf(fp, "]");
            }
          }
          else if (last_reg_flag != SigIsReg(ptr)) {
            is_first    = False;
            last_reg_flag  = SigIsReg(ptr);

            fprintf(fp, ";\n");
            fprintfTab(fp, indent);

            if (SigIsReg(ptr) == True)
              fprintf(fp, "reg \t");
            else
              fprintf(fp, "wire\t");

            // write type
            *flag4  = (int)(NULLCELL);
            fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
            fprintf(fp, "\t");

            // write name
            fprintf(fp, "%s", CellStr(id));

            // write array range (if exist)
            if (*flag4 != (int)NULLCELL && sw == True) {
              TCELLPNT  range;
              range  = (TCELLPNT)(*flag4, flag5);
              // write width
              fprintf(fp, "\t");
              fprintf(fp, "[");
              fprintfVerilog(fp, sw, CellInfo0(range), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write range value
              fprintf(fp, ":");
              fprintfVerilog(fp, sw, CellInfo1(range), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write range value
              fprintf(fp, "]");
            }
          }
          else {
            is_first    = False;
            last_reg_flag  = SigIsReg(ptr);

            fprintf(fp, ", ");

            // write name
            fprintf(fp, "%s", CellStr(id));

            // write array range (if exist)
            if (*flag4 != (int)NULLCELL && sw == True) {
              TCELLPNT  range;
              range  = (TCELLPNT)(*flag4, flag5);
              // write width
              fprintf(fp, "\t");
              fprintf(fp, "[");
              fprintfVerilog(fp, sw, CellInfo0(range), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write range value
              fprintf(fp, ":");
              fprintfVerilog(fp, sw, CellInfo1(range), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write range value
              fprintf(fp, "]");
            }
          }
        }  // end of SearchSigList
        else {
          fprintf(stderr, "INTERNAL ERROR: T_SIG\n");
          fprintf(fp, "INTERNAL ERROR: T_SIG\n");
          exit(0);
        }
      } // end of for loop;

      fprintf(fp, ";");
      if (fprintfComment(fp, com, CellLine(CellInfo0(top)), 1) == False)
        fprintf(fp, "\n");
    }
    break;

  case T_VARIABLE:
    if (sw == True) {
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
    }

    if (sw == True) {
      fprintf(fp, "reg \t");
    }

    // write type
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
    if (sw == True) fprintf(fp, "\t");
    // write id_list
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);

    if (sw == True) {
      fprintf(fp, ";");
      if (fprintfComment(fp, com, CellLine(CellInfo0(top)), 1) == False)
        fprintf(fp, "\n");
    }
    break;


///////////////////////////////////////////////////////////////////////////////
// body part
///////////////////////////////////////////////////////////////////////////////
  case T_PROCESS:
    /* process: T_PROCESS flag0=ID(label), flag1=sensitivity, flag2=variable, info3=body */

    // variable
    if (CellInfo2(top) != NULLCELL) {
      register TCELLPNT  item;
      for (item = CellInfo2(top); item != NULLCELL; item = NextCell(item)) {  // CellInfo3(top) == body
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
      }
      if (sw == True) fprintf(fp, "\n");  // (separator)
    }

    // sensitivity list
    if (sw == True) {
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
      fprintf(fp, "always @(");
    }
    {
      register TCELLPNT  item;
      for (item = CellInfo1(top); item != NULLCELL;) {  // CellInfo1(top) == sensesig_list
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write item
        if ((item = NextCell(item)) != NULLCELL) {
          if (sw == True) fprintf(fp, " or ");
        }
      }
    }
    if (sw == True) fprintf(fp, ") begin\n");

    // body
    {
      register TCELLPNT  item;
      for (item = CellInfo3(top); item != NULLCELL; item = NextCell(item)) {  // CellInfo3(top) == body
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);
      }
    }

    if (sw == True) {
      fprintfTab(fp, indent);
      fprintf(fp, "end\n");
    }
    break;

  case N_SYNCPROCESS:
    /* process: T_PROCESS flag0=ID(label), flag1=sensitivity, flag2=variable, info3=body */

    // variable
    if (CellInfo2(top) != NULLCELL) {
      register TCELLPNT  item;
      for (item = CellInfo2(top); item != NULLCELL; item = NextCell(item)) {  // CellInfo3(top) == body
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
      }
      if (sw == True) fprintf(fp, "\n");  // (separator)
    }

    // sensitivity list (clock)
    if (sw == True) {
      register TCELLPNT  edgecond;

      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
      fprintf(fp, "always @(");

      /* if: N_SYNCIF info0=cond, info1=then, info2=else(NULL) */
      /* event: T_EVENT info0=exp */
      edgecond = CellInfo0(CellInfo0(CellInfo3(top)));  // (exp of T_EVENT)
      while (CellType(edgecond) == N_PAREN)
        edgecond = CellInfo0(edgecond);
      if (CellType(edgecond) == T_EQUAL) {
        if (CellType(CellInfo0(edgecond)) == T_BINDIGIT) {
          if (!strcmp(CellStr(CellInfo0(edgecond)),"'1'"))    fprintf(fp, "posedge ");
          else if (!strcmp(CellStr(CellInfo0(edgecond)),"'0'"))  fprintf(fp, "negedge ");
          else {
fprintfWarning(fp, CellLine(top), WARN_4_CLKRST);
          }

          fprintfVerilog(fp, sw, CellInfo1(edgecond), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
        }
        else {
          if (!strcmp(CellStr(CellInfo1(edgecond)),"'1'"))    fprintf(fp, "posedge ");
          else if (!strcmp(CellStr(CellInfo1(edgecond)),"'0'"))  fprintf(fp, "negedge ");
          else {
fprintfWarning(fp, CellLine(top), WARN_4_CLKRST);
          }

          fprintfVerilog(fp, sw, CellInfo0(edgecond), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
        }
      }
      else {
fprintfWarning(fp, CellLine(top), WARN_4_CLKRST);
      }

      fprintf(fp, ") begin\n");
    }

    // body
    {
      register TCELLPNT  item;
      for (item = CellInfo3(top); item != NULLCELL; item = NextCell(item)) {  // CellInfo3(top) == body
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);
      }
    }

    if (sw == True) {
      fprintfTab(fp, indent);
      fprintf(fp, "end\n");
    }
    break;

  case N_ASYNCPROCESS:
    /* process: T_PROCESS flag0=ID(label), flag1=sensitivity, flag2=variable, info3=body */

    // variable
    if (CellInfo2(top) != NULLCELL) {
      register TCELLPNT  item;
      for (item = CellInfo2(top); item != NULLCELL; item = NextCell(item)) {  // CellInfo3(top) == body
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
      }
      if (sw == True) fprintf(fp, "\n");  // (separator)
    }

    // sensitivity list (clock,rst)
    if (sw == True) {
      register TCELLPNT  edgecond, rstcond;

      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
      fprintf(fp, "always @(");

      /* if: N_ASYNCIF info0=exp, info1=body1, info2=edge_cond, info3=body2 */
      /* event: T_EVENT info0=exp */
      edgecond = CellInfo0(CellInfo2(CellInfo3(top)));  // (exp of T_EVENT)
      while (CellType(edgecond) == N_PAREN)
        edgecond = CellInfo0(edgecond);
      if (CellType(edgecond) == T_EQUAL) {
        if (CellType(CellInfo0(edgecond)) == T_BINDIGIT) {
          if (!strcmp(CellStr(CellInfo0(edgecond)),"'1'"))    fprintf(fp, "posedge ");
          else if (!strcmp(CellStr(CellInfo0(edgecond)),"'0'"))  fprintf(fp, "negedge ");
          else {
fprintfWarning(fp, CellLine(top), WARN_4_CLKRST);
          }

          fprintfVerilog(fp, sw, CellInfo1(edgecond), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
        }
        else {
          if (!strcmp(CellStr(CellInfo1(edgecond)),"'1'"))    fprintf(fp, "posedge ");
          else if (!strcmp(CellStr(CellInfo1(edgecond)),"'0'"))  fprintf(fp, "negedge ");
          else {
fprintfWarning(fp, CellLine(top), WARN_4_CLKRST);
          }

          fprintfVerilog(fp, sw, CellInfo0(edgecond), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
        }
      }
      else {
fprintfWarning(fp, CellLine(top), WARN_4_CLKRST);
      }

      fprintf(fp, " or ");

      rstcond = CellInfo0(CellInfo3(top));        // (exp of reset cond)
      while (CellType(rstcond) == N_PAREN)
        rstcond = CellInfo0(rstcond);

      if (CellType(rstcond) == T_EQUAL) {
        if (CellType(CellInfo0(rstcond)) == T_BINDIGIT) {
          if (!strcmp(CellStr(CellInfo0(rstcond)),"'1'"))      fprintf(fp, "posedge ");
          else if (!strcmp(CellStr(CellInfo0(rstcond)),"'0'"))  fprintf(fp, "negedge ");

          fprintfVerilog(fp, sw, CellInfo1(rstcond), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
        }
        else {
          if (!strcmp(CellStr(CellInfo1(rstcond)),"'1'"))      fprintf(fp, "posedge ");
          else if (!strcmp(CellStr(CellInfo1(rstcond)),"'0'"))  fprintf(fp, "negedge ");

          fprintfVerilog(fp, sw, CellInfo0(rstcond), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
        }
      }
      else {
fprintfWarning(fp, CellLine(top), WARN_4_CLKRST);
      }

      fprintf(fp, ") begin\n");
    }

    // body
    {
      register TCELLPNT  item;
      for (item = CellInfo3(top); item != NULLCELL; item = NextCell(item)) {  // CellInfo3(top) == body
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);
      }
    }

    if (sw == True) {
      fprintfTab(fp, indent);
      fprintf(fp, "end\n");
    }
    break;

  case T_SIGSUBST:
    /* subst: T_SIGSUBST/N_ASSIGN flag0=ID, flag1=signal, flag2=exp */
    if (sw == True)  {
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
    }
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // signal
    if (sw == True) fprintf(fp, "\t<= ");
    *flag3  = (int)CellInfo1(top);  // flag3...pointer to L value
    fprintfVerilog(fp, sw, CellInfo2(top), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // exp
    if (sw == True) {
      fprintf(fp, ";");
      if (fprintfComment(fp, com, CellLine(CellInfo2(top)), 1) == False)
        fprintf(fp, "\n");
    }
    break;

  case T_VARSUBST:
    /* subst: T_VARSUBST flag0=NULL, flag1=variable, flag2=exp */
    if (sw == True)  {
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
    }
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // variable
    if (sw == True) fprintf(fp, "\t= ");
    *flag3  = (int)CellInfo1(top);  // flag3...pointer to L value
    fprintfVerilog(fp, sw, CellInfo2(top), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // exp
    if (sw == True) {
      fprintf(fp, ";");
      if (fprintfComment(fp, com, CellLine(CellInfo2(top)), 1) == False)
        fprintf(fp, "\n");
    }
    break;

  case N_SYNCIF:
    /* if: N_SYNCIF info0=cond, info1=then, info2=else(NULL) */
    {
      register TCELLPNT  item;
      for (item = CellInfo1(top); item != NULLCELL; item = NextCell(item)) {  // CellInfo1(top) == then
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // body
      }
    }
    break;

  case N_ASYNCIF:
    /* if: N_ASYNCIF info0=exp, info1=body1, info2=edge_cond, info3=body2 */
    if (sw == True) {
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
      fprintf(fp, "if ");
    }
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // cond
    if (sw == True) {
      fprintf(fp, " begin");
      if (fprintfComment(fp, com, CellLine(top), 1) == False)
        fprintf(fp, "\n");
    }

    {
      register TCELLPNT  item;
      for (item = CellInfo1(top); item != NULLCELL; item = NextCell(item)) {  // CellInfo1(top) == then
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // body
      }
    }

    if (sw == True) {
      fprintfTab(fp, indent);
      fprintf(fp, "end else begin\n");
    }
    {
      register TCELLPNT  item;
      for (item = CellInfo3(top); item != NULLCELL; item = NextCell(item)) {  // CellInfo3(top) == else
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // body
      }
    }
    if (sw == True) {
      fprintfTab(fp, indent);
      fprintf(fp, "end\n");
    }
    break;

  case T_IF:
    /* if: T_IF flag0=cond, flag1=then, flag2=else */
    if (sw == True) {
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
      fprintf(fp, "if ");
      if (CellType(CellInfo0(top)) != N_PAREN)  fprintf(fp, "(");
    }
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // cond
    if (sw == True) {
      if (CellType(CellInfo0(top)) != N_PAREN)  fprintf(fp, ")");
      fprintf(fp, " begin");
      if (fprintfComment(fp, com, CellLine(top), 1) == False)
        fprintf(fp, "\n");
    }

    {
      register TCELLPNT  item;
      for (item = CellInfo1(top); item != NULLCELL; item = NextCell(item)) {  // CellInfo1(top) == then
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // body
      }
    }

    if (sw == True) {
      fprintfTab(fp, indent);
      fprintf(fp, "end");
    }
    if (CellInfo2(top) != NULLCELL)
      fprintfVerilog(fp, sw, CellInfo2(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // else
    else {
      if (sw == True) fprintf(fp, "\n");
    }
    break;

  case T_ELSIF:
    /* elsif: T_ELSIF flag0=cond, flag1=then, flag2=else */
    if (sw == True) {
      fprintf(fp, " else if ");
      if (CellType(CellInfo0(top)) != N_PAREN)  fprintf(fp, "(");
    }
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // cond
    if (sw == True) {
      if (CellType(CellInfo0(top)) != N_PAREN)  fprintf(fp, ")");
      fprintf(fp, " begin");
      if (fprintfComment(fp, com, CellLine(top), 1) == False)
        fprintf(fp, "\n");
    }

    {
      register TCELLPNT  item;
      for (item = CellInfo1(top); item != NULLCELL; item = NextCell(item)) {  // CellInfo1(top) == then
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // body
      }
    }

    if (sw == True) {
      fprintfTab(fp, indent);
      fprintf(fp, "end");
    }
    if (CellInfo2(top) != NULLCELL)
      fprintfVerilog(fp, sw, CellInfo2(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // else
    else {
      if (sw == True) fprintf(fp, "\n");
    }
    break;

  case T_ELSE:
    /* else: T_ELSE flag0=else */
    if (sw == True) {
      fprintf(fp, " else begin");
      if (fprintfComment(fp, com, CellLine(top), 1) == False)
        fprintf(fp, "\n");
    }

    {
      register TCELLPNT  item;
      for (item = CellInfo0(top); item != NULLCELL; item = NextCell(item)) {  // CellInfo0(top) == else
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // body
      }
    }

    if (sw == True) {
      fprintfTab(fp, indent);
      fprintf(fp, "end\n");
    }
    break;

  case T_CASE:
    /* case: T_CASE flag0=sig, flag1=condlist */
    if (sw == True) {
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
      fprintf(fp, "case ");
      if (CellType(CellInfo0(top)) != N_PAREN)  fprintf(fp, "(");
    }
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // sig
    if (sw == True) {
      if (CellType(CellInfo0(top)) != N_PAREN)  fprintf(fp, ")");
      if (fprintfComment(fp, com, CellLine(top), 1) == False)
        fprintf(fp, "\n");
    }

    // case items
    {
      register TCELLPNT  item;
      for (item = CellInfo1(top); item != NULLCELL; item = NextCell(item)) {
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
      }
    }

    if (sw == True) {
      fprintfTab(fp, indent);
      fprintf(fp, "endcase\n");
    }
    break;

  case N_CASECOND:
    /* case: N_CASECOND flag0=exp list, flag1=body */
    if (sw == True) {
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
    }
    if (CellType(CellInfo0(top)) != T_OTHERS) {
      TCELLPNT  ptr;
      for (ptr = CellInfo0(top); ptr != NULLCELL; ptr = NextCell(ptr)) {
        fprintfVerilog(fp, sw, ptr, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // exp
        if (NextCell(ptr) != NULLCELL && sw == True)
          fprintf(fp, ", ");
      }
    }
    else {
      if (sw == True) fprintf(fp, "default");
    }
    if (sw == True) fprintf(fp, ": begin\n");

    // body
    {
      register TCELLPNT  item;
      for (item = CellInfo1(top); item != NULLCELL; item = NextCell(item)) {  // CellInfo1(top) == body
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);
      }
    }

    if (sw == True) {
      fprintfTab(fp, indent);
      fprintf(fp, "end\n");
    }
    break;

  case N_ASSIGN:
    /* subst: T_SIGSUBST/N_ASSIGN flag0=ID, flag1=signal, flag2=exp */
    if (sw == True) {
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
      fprintf(fp, "assign\t");
    }
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // signal
    if (sw == True) fprintf(fp, "\t= ");
    *flag3  = (int)CellInfo1(top);  // flag3...pointer to L value
    fprintfVerilog(fp, sw, CellInfo2(top), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // exp
    if (sw == True) {
      fprintf(fp, ";");
      if (fprintfComment(fp, com, CellLine(CellInfo2(top)), 1) == False)
        fprintf(fp, "\n");
    }
    break;

  case N_CONDASSIGN:
    /* when: N_CONDASSIGN flag0=ID(label), flag1=sig, flag2=body */
    if (sw == True) {
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
      fprintf(fp, "assign\t");
    }
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // signal
    if (sw == True) fprintf(fp, "\t= ");

    *flag1  = (int)CellInfo2(top);  // (pointer to last condition)
    *flag3  = (int)CellInfo1(top);  // flag3...pointer to L value
    fprintfVerilog(fp, sw, CellInfo2(top), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // body
    if (sw == True) {
      fprintf(fp, ";");
      if (fprintfComment(fp, com, CellLine((TCELLPNT)(*flag1)), 1) == False)
        fprintf(fp, "\n");
    }
    break;

  case N_WHENCOND:
    /* when: N_WHENCOND flag0=substexp, flag1=condexp, flag2=else */
    if (CellInfo1(top) != NULLCELL) {
      if (sw == True) fprintf(fp, "(");
      fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // cond
      if (sw == True) fprintf(fp, ") ? (");
      fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // exp
      if (sw == True) {
        fprintf(fp, ")");
        if (fprintfComment(fp, com, CellLine(CellInfo0(top)), 1) == False)
          fprintf(fp, "\n");

        fprintfTab(fp, indent + 2);
        fprintf(fp, ": (");
      }
      *flag1 = (int)CellInfo2(top);  // (pointer to last condition)
      fprintfVerilog(fp, sw, CellInfo2(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // else
      if (sw == True) fprintf(fp, ")");
    }
    else {
      *flag1 = (int)CellInfo0(top);  // (pointer to last condition)
      fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // else
    }
    break;

  case N_SELASSIGN:
    /* when: N_SELASSIGN info0=ID(label), info1=exp, info2=sig, info3=body */
    if (sw == True) {
      register TCELLPNT  item;

      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
      fprintf(fp, "assign\t");
      fprintfVerilog(fp, sw, CellInfo2(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // module name
      fprintf(fp, " =\t");

      for (item = CellInfo3(top); item != NULLCELL; item = CellInfo2(item)) {
      /* when: N_SELCOND info0=substexp, info1=condexp, info2=else */
        if (CellInfo1(item) != NULLCELL) {
          register TCELLPNT  ptr;
          fprintf(fp, "(");

          for (ptr = CellInfo1(item); ptr != NULLCELL; ptr = NextCell(ptr)) {
            fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // cond(L)
            fprintf(fp, " == ");
            fprintfVerilog(fp, sw, ptr, sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // cond(R)

            if (NextCell(ptr) != NULLCELL)
              fprintf(fp, " || ");
          }

          fprintf(fp, ") ? (");

          fprintfVerilog(fp, sw, CellInfo0(item), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // substexp

          fprintf(fp, ")");
          if (fprintfComment(fp, com, CellLine(item), 1) == False)
            fprintf(fp, "\n");
          fprintfTab(fp, indent + 3);
          fprintf(fp, " : ");
        }
        else {  // (last else)
          fprintf(fp, "(");
          fprintfVerilog(fp, sw, CellInfo0(item), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // substexp
          fprintf(fp, ")");
          fprintf(fp, ";");
          if (fprintfComment(fp, com, CellLine(item), 1) == False)
            fprintf(fp, "\n");
        }
      }
    }
    break;

  case N_PORTMAP:
    /* portmap: N_PORTMAP flag0=ID(label), flag1=ID(module), flag2=generic, info3=port */
    if (sw == True) {
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
    }

    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // module name
    if (CellInfo2(top) != NULLCELL) {  // generic
      if (sw == True) {
        register TCELLPNT  item;
        fprintf(fp, " #(\n");

        for (item = CellInfo2(top); item != NULLCELL; item = NextCell(item)) {
          fprintfTab(fp, indent + 1);
          if (CellType(item) == N_PORTITEMNAME)
            fprintfVerilog(fp, sw, CellInfo1(item), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);
          else
            fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);
          if (NextCell(item) != NULLCELL)
            fprintf(fp, ",");
          if (fprintfComment(fp, com, CellLine(item), 1) == False)
            fprintf(fp, "\n");
        }

        fprintfTab(fp, indent);
        fprintf(fp, ") ");
      }
    }
    else {
      if (sw == True) fprintf(fp, "\t");
    }

    // label
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
    if (sw == True && ext != (EXTRACTLIST *)NULL && NextExtract(ext) != (EXTRACTLIST *)NULL) {  // change label
      EXTRACTLIST  *ptr;
      for (ptr = ext; NextExtract(ptr) != (EXTRACTLIST *)NULL; ptr = NextExtract(ptr)) {
        fprintf(fp, "_%d", ExtractNum(ptr));
      }
    }

    // port
    if (sw == True) {
      fprintf(fp, " (");
    }
    {
      register TCELLPNT  item;
      *flag2 = 1;    // (first item)
      for (item = CellInfo3(top); item != NULLCELL; item = NextCell(item), *flag2 = 0) {
        *flag1 = 0;  // (if N_PORTITEMNAME then non-zero value will return)
        *flag5 = 1;  // unchack CASE (for port name)
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // port
        *flag5 = 0;  // check CASE
        if (NextCell(item) != NULLCELL) {
          if (sw == True) fprintf(fp, ",");
        }
        if (sw == True && *flag1 != 0) {
          if (fprintfComment(fp, com, CellLine(item), 1) == False)
            fprintf(fp, "\n");
        }
      }
    }

    if (sw == True) {
      if (*flag1 != 0) fprintfTab(fp, indent);
      fprintf(fp, ");");
      if (fprintfComment(fp, com, CellLine(CellInfo3(top)), 1) == False)
        fprintf(fp, "\n");
    }
    break;

  case N_PORTITEMNAME:
    if (sw == True) {
      if (*flag2 != 0)  fprintf(fp, "\n");    // (first item)
      fprintfComment(fp, com, CellLine(top) - 1, indent);
      fprintfTab(fp, indent);
      fprintf(fp, ".");
    }
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // sig
    if (sw == True) fprintf(fp, "(");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // exp
    if (sw == True) fprintf(fp, ")");
    *flag1 = 1;    // N_PORTITEMNAME
    break;


///////////////////////////////////////////////////////////////////////////////
// signal type
///////////////////////////////////////////////////////////////////////////////
  case T_STDLOGIC:
    /* none */
    if (sw == True) fprintf(fp, "\t");
    break;

  case T_STDLOGICVEC:
    if (CellInfo0(top) != NULLCELL) {
      // write width
      if (sw == True) fprintf(fp, "[");
      fprintfVerilog(fp, sw, CellInfo0(CellInfo0(top)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write range value
      if (sw == True) fprintf(fp, ":");
      fprintfVerilog(fp, sw, CellInfo1(CellInfo0(top)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write range value
      if (sw == True) fprintf(fp, "]");
    }
    break;

  case N_ENUMTYPE:
    if (sw == True) {
      fprintf(fp, "[%d:0]", EnumItemNum(top) - 1);  // write signal width
    }
    break;

  case T_ARRAY:
    if (sw == True) {
      if (CellType(CellInfo1(top)) == T_STDLOGIC)
        fprintf(fp, "\t");
      else if (CellType(CellInfo1(top)) == T_STDLOGICVEC) {
        TCELLPNT  range;
        range  = CellInfo0(CellInfo1(top));
        // write width
        fprintf(fp, "[");
        fprintfVerilog(fp, sw, CellInfo0(range), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write range value
        fprintf(fp, ":");
        fprintfVerilog(fp, sw, CellInfo1(range), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write range value
        fprintf(fp, "]");

        *flag4  = (int)(CellInfo0(top));
      }
      else {
fprintfWarning(fp, CellLine(top), WARN_5_ARRAYTYPE);
      }
    }
    break;

  case T_INTEGER:
    if (sw == True) {
      register int  len;
      if ((len = ChkIntegerBitLength(ChkIntegerRange(CellInfo0(top)))) > 0) {
        fprintf(fp, "[%d:0]", len - 1);  // write signal width
fprintfWarning(fp, CellLine(top), WARN_6_SIGWIDTH);
      }
      else {
fprintfWarning(fp, CellLine(top), WARN_7_INTTYPE);
      }
    }
    break;

  case N_TYPEID:
    {
      register TYPELIST  *typptr;
      typptr  = SearchTypeList(typ, CellStr(top));
      if (typptr == (TYPELIST *)NULL) {  // undeclared type
        if (sw == True) {
fprintfWarning(fp, CellLine(top), WARN_8_DATATYPE);
          fprintf(fp, "%s", CellStr(top));
        }
      }
      else {  // found type def
        fprintfVerilog(fp, sw, TypeInfo(typptr), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write signal type
      }
    }
    break;

///////////////////////////////////////////////////////////////////////////////
// type def
///////////////////////////////////////////////////////////////////////////////
  case T_TYPE:
    if (CellType(CellInfo1(top)) == N_ENUMTYPE)
      fprintfEnumDef(fp, CellInfo1(top), com, indent);    // (write parameter definitions)
    break;

///////////////////////////////////////////////////////////////////////////////
// signal dir
///////////////////////////////////////////////////////////////////////////////
  case T_IN:
    if (sw == True) fprintf(fp, "input");
    break;

  case T_INOUT:
    if (sw == True) fprintf(fp, "inout");
    break;

  case T_OUT:
    if (sw == True) fprintf(fp, "output");
    break;

///////////////////////////////////////////////////////////////////////////////
// expression
///////////////////////////////////////////////////////////////////////////////
  case T_AND:
    *flag0 = 0;  // arithmetic
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (*flag0 == 0) {
      if (sw == True) fprintf(fp, " & ");    // arithmetic
    }
    else {
      if (sw == True) fprintf(fp, " && ");  // logical
    }
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    break;

  case T_OR:
    *flag0 = 0;  // arithmetic
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (*flag0 == 0) {
      if (sw == True) fprintf(fp, " | ");    // arithmetic
    }
    else {
      if (sw == True) fprintf(fp, " || ");  // logical
    }
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    break;

  case T_XOR:
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " ^ ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    *flag0 = 0;  // arithmetic
    break;

  case T_NAND:
    if (sw == True) fprintf(fp, " ~(");
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " & ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    if (sw == True) fprintf(fp, ")");
    *flag0 = 0;  // arithmetic
    break;

  case T_NOR:
    if (sw == True) fprintf(fp, " ~(");
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " | ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    if (sw == True) fprintf(fp, ")");
    *flag0 = 0;  // arithmetic
    break;

  case T_EQUAL:
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " == ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    *flag0 = 1;  // logical
    break;

  case T_NOTEQUAL:
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " != ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    *flag0 = 1;  // logical
    break;

  case N_GE:
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " <= ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    *flag0 = 1;  // logical
    break;

  case T_LE:
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " >= ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    *flag0 = 1;  // logical
    break;

  case T_GT:
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " < ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    *flag0 = 1;  // logical
    break;

  case T_LS:
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " > ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    *flag0 = 1;  // logical
    break;

  case T_SLL:
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " << ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    *flag0 = 0;  // arithmetic
    break;

  case T_SRL:
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " >> ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    *flag0 = 0;  // arithmetic
    break;

  case T_PLUS:
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " + ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    *flag0 = 0;  // arithmetic
    break;

  case T_MINUS:
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " - ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    *flag0 = 0;  // arithmetic
    break;

  case T_CONCAT:
    if (sw == True) fprintf(fp, "{");
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, ", ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    if (sw == True) fprintf(fp, "}");
    *flag0 = 0;    // arithmetic
    break;

  case T_MULT:
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " * ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    *flag0 = 0;  // arithmetic
    break;

  case T_DIV:
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " / ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    *flag0 = 0;  // arithmetic
    break;

  case T_MOD:
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write L
    if (sw == True) fprintf(fp, " % ");
    fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // write R
    *flag0 = 0;  // arithmetic
    break;

  case T_NOT:
    *flag0 = 0;  // arithmetic
    fprintfVerilog(fp, False, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // NO-PRINT (for only checking flag0)
    if (*flag0 == 0) {
      if (sw == True) fprintf(fp, " ~");    // arithmetic
    }
    else {
      if (sw == True) fprintf(fp, " !");    // logical
    }
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);    // PRINT
    break;

  case T_BINDIGIT :
    {
      register char  *str;
      register int  num = strlen(CellStr(top));
      str = (char *)malloc(num + 1);
      strcpy(str, CellStr(top));
      num -= 2;
      *(str + (num + 1)) = '\0';
      if (sw == True) fprintf(fp, "%d'b%s", num, str + 1);
      free(str);
    }
    *flag0 = 0;  // arithmetic
    break;

  case T_HEXDIGIT :
    {
      register char  *str;
      register int  num = strlen(CellStr(top));
      str = (char *)malloc(num + 1);
      strcpy(str, CellStr(top));
      num -= 3;
      *(str + (num + 2)) = '\0';
      if (sw == True) fprintf(fp, "%d'h%s", num * 4, str + 2);
      free(str);
    }
    *flag0 = 0;  // arithmetic
    break;

  case T_DECDIGIT :
    if (sw == True) fprintf(fp, "%s", CellStr(top));
    *flag0 = 0;  // arithmetic
    break;

  case N_OTHERS:
    /* N_OTHERS flag0=value */
    if (sw == True) {
      register TCELLPNT  substval, typptr;
      register SIGLIST  *sigptr;

      fprintf(fp, "{");

      substval  = (TCELLPNT)(*flag3);    // flag3...pointer to L value
      switch (CellType(substval)) {
      case T_ID:
      case N_IDLIST:
        // look up signal declarations
        if ((sigptr = SearchSigList(sig, GetSigName(substval)) ) != NULLSIG) {
          typptr = SigType(sigptr);
          switch (CellType(typptr)) {
          case T_STDLOGICVEC:
            fprintf(fp, "(");
            if (CellType(CellInfo0(typptr)) == T_TO) {
              fprintfVerilog(fp, sw, CellInfo1(CellInfo0(typptr)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // value
              fprintf(fp, " - ");
              fprintfVerilog(fp, sw, CellInfo0(CellInfo0(typptr)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // value
            }
            else {  // T_DOWNTO
              fprintfVerilog(fp, sw, CellInfo0(CellInfo0(typptr)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // value
              fprintf(fp, " - ");
              fprintfVerilog(fp, sw, CellInfo1(CellInfo0(typptr)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // value
            }
            fprintf(fp, " + 1)");
            break;

          case T_STDLOGIC:
            fprintf(fp, "1");
            break;

          default:
            if (sw == True) {
              fprintf(fp, "UNKNOWN_WIDTH");
fprintfWarning(fp, CellLine(top), WARN_9_SIGCOPY);
            }
            break;
          }
        }
        else {
          if (sw == True) {
            fprintf(fp, "UNKNOWN_WIDTH");
fprintfWarning(fp, CellLine(top), WARN_9_SIGCOPY);
          }
        }
        break;

      case N_STDVECTOR:
        fprintf(fp, "(");
        if (CellType(CellInfo1(substval)) == T_TO) {
          fprintfVerilog(fp, sw, CellInfo1(CellInfo1(substval)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // value
          fprintf(fp, " - ");
          fprintfVerilog(fp, sw, CellInfo0(CellInfo1(substval)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // value
        }
        else {  // T_DOWNTO
          fprintfVerilog(fp, sw, CellInfo0(CellInfo1(substval)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // value
          fprintf(fp, " - ");
          fprintfVerilog(fp, sw, CellInfo1(CellInfo1(substval)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // value
        }
        fprintf(fp, " + 1)");
        break;

      case N_STDELEMENT:
        fprintf(fp, "1");
        break;

      case N_LISTARRAY:
        if (sw == True) {
          fprintf(fp, "UNKNOWN_WIDTH");
fprintfWarning(fp, CellLine(top), WARN_9_SIGCOPY);
        }
        break;
      default:
        if (sw == True) {
          fprintf(fp, "UNKNOWN_WIDTH");
fprintfWarning(fp, CellLine(top), WARN_9_SIGCOPY);
        }
        break;
      }

      fprintf(fp, "{");
      fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // value
      fprintf(fp, "}}");
    }
    *flag0 = 0;  // arithmetic
    break;

  case N_CALLFUNC:
    /* N_CALLFUNC flag0=ID, flag1=port */
    if (sw == True) {
      register TCELLPNT  item;
      fprintf(fp, "%s(", CellStr(CellInfo0(top)));    // function name

      for (item = CellInfo1(top); item != NULLCELL; item = NextCell(item)) {
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // name
        if (NextCell(item) != NULLCELL)
          fprintf(fp, ",");
      }

      fprintf(fp, ")");
    }
    *flag0 = 0;  // arithmetic
    break;

  case N_STDVECTOR:
    if (sw == True) {
      char  *name;
      // (flag5...CHECK CASE iff 0)
      if (*flag5 == 0 && (name = SearchSimilarSig(sig, CellStr(CellInfo0(top)))) != (char *)NULL) {
fprintfWarning(fp, CellLine(top), WARN_10_SIGNAME);
      }
      else
        name = CellStr(CellInfo0(top));

      fprintf(fp, "%s[", name);
      fprintfVerilog(fp, sw, CellInfo0(CellInfo1(top)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // index value (L)
      fprintf(fp, ":");
      fprintfVerilog(fp, sw, CellInfo1(CellInfo1(top)), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // index value (R)
      fprintf(fp, "]");
    }
    *flag0 = 0;  // arithmetic
    break;

  case N_STDELEMENT:
    if (sw == True) {
      char  *name;
      SIGLIST  *sigptr;

      // (flag5...CHECK CASE iff 0)
      if (*flag5 == 0 && (name = SearchSimilarSig(sig, CellStr(CellInfo0(top)))) != (char *)NULL) {
fprintfWarning(fp, CellLine(top), WARN_10_SIGNAME);
      }
      else
        name = CellStr(CellInfo0(top));

      sigptr  = SearchSigList(sig, name);
      if (sigptr != NULLSIG && SigIsFunc(sigptr) == True)
        fprintf(fp, "%s(", name);    // function call
      else
        fprintf(fp, "%s[", name);
      fprintfVerilog(fp, sw, CellInfo1(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);  // index value
      if (sigptr != NULLSIG && SigIsFunc(sigptr) == True)
        fprintf(fp, ")");        // function call
      else
        fprintf(fp, "]");
    }
    *flag0 = 0;  // arithmetic
    break;

  case T_ID:
    if (sw == True) {
      int  substval;
      if (ChkSubstVal(ext, top, &substval) == True)  // replace
        fprintf(fp, "%d", substval);
      else {  // do not replace
        char  *name;
        // (flag5...CHECK CASE iff 0)
        if (*flag5 == 0 && (name = SearchSimilarSig(sig, CellStr(top))) != (char *)NULL) {
fprintfWarning(fp, CellLine(top), WARN_10_SIGNAME);
        } else
          name = CellStr(top);
        fprintf(fp, "%s", name);
      }
    }
    *flag0 = 0;  // arithmetic
    break;

  case N_LISTARRAY:
    /* N_LISTARRAY flag0=list */
    if (sw == True) {
      register TCELLPNT  item;
      fprintf(fp, "{", CellStr(CellInfo0(top)));    // function name

      for (item = CellInfo0(top); item != NULLCELL; item = NextCell(item)) {
        fprintfVerilog(fp, sw, item, sig, com, typ, ext, indent + 1, flag0, flag1, flag2, flag3, flag4, flag5);  // name
        if (NextCell(item) != NULLCELL)
          fprintf(fp, ",");
      }

      fprintf(fp, "}");
    }
    *flag0 = 0;  // arithmetic
    break;

  case N_PAREN:
    if (sw == True) fprintf(fp, "(");
    fprintfVerilog(fp, sw, CellInfo0(top), sig, com, typ, ext, indent, flag0, flag1, flag2, flag3, flag4, flag5);
    if (sw == True) fprintf(fp, ")");
    break;


  default:
    if (sw == True) fprintf(stderr, "INTERNAL ERROR: fprintfVerilog(): unknown node %d\n", CellType(top));
  }
}


//////////////////////////////////////////////////////////////////////
// Main
//////////////////////////////////////////////////////////////////////

void main(int argc, char *argv[])
{
  char  *outfname, *ptr;
  int    argctr;
  int    flag0, flag1, flag2, flag3, flag4, flag5;  // (for fprintfVerilog)

  fprintf(stderr, "%s\n", TITLE);

  if (argc < 2) {
    fprintf(stderr, "usage: vhdl2verilog inputfile1 inputfile2 ...\n");
#ifdef WINDOWS
    getchar();
#endif
    exit(1);
  }

  for (argctr = 1; argctr < argc; argctr++) {  // for each input files
  // open files
    if ((yyin = fopen(argv[argctr], "rt")) == (FILE *)NULL) {  // open input file
      fprintf(stderr, "Can not open input file %s\n", argv[argctr]);
      continue;
    }

    // (make output file name)
    outfname = (char *)malloc(strlen(argv[argctr]) + 100);
    strcpy(outfname, argv[argctr]);
    for (ptr = outfname + strlen(outfname) - 1; ptr != outfname; ptr--) {
      if (*ptr == '\\')
        break;
      if (*ptr == '.') {
        *ptr = '\0';
        break;
      }
    }
    strcat(outfname, ".v");

    if (!strcmp(outfname, argv[argctr])) {  // check input file == output file or not
      fprintf(stderr, "Can not overwrite input file %s by output file\n", outfname);
      continue;
    }
    if ((fpout = fopen(outfname, "wt")) == (FILE *)NULL) {  // open output file
      fprintf(stderr, "Can not create output file %s\n", outfname);
      continue;
    }

  // init data structure
    ParseTreeTop  = NULLCELL;
    ParseError    = False;

    ParseSigListTop  = MakeNewSigList();
    TypeListTop    = MakeNewTypeList();
    CommentListTop  = MakeNewCommentList();
    ExtractListTop  = MakeNewExtractList();

    fprintf(stderr, "\nInput file: %s\n", argv[argctr]);

    yylexlinenum  = 1;
    yyout      = fopen(TMPFNAME, "wt");

  // main
    yyrestart(yyin);  // init flex
    yyparse();      // parse

    fclose(yyin);
    fclose(yyout);
    unlink(TMPFNAME);

    if (ParseError == False) {
      Sysid = 0;

      fprintf(fpout, "// Converted from %s\n", argv[argctr]);
      fprintf(fpout, "// by %s\n\n", TITLE);

      // make output
      fprintfVerilog(fpout, True, ParseTreeTop, NULLSIG, CommentListTop, NULLTYPE, ExtractListTop, 0, &flag0, &flag1, &flag2, &flag3, &flag4, &flag5);
      fclose(fpout);
    }
    else {
      fclose(fpout);
      unlink(outfname);
    }

  // release memory
    free(outfname);
    FreeTree(ParseTreeTop);
    FreeSigList(ParseSigListTop);
    FreeTypeList(TypeListTop);
    FreeCommentList(CommentListTop);
    FreeExtractList(ExtractListTop);
  }

  fprintf(stderr, "\nComplete.\n");
#ifdef WINDOWS
  getchar();
#endif

  exit(0);
}

// end of file

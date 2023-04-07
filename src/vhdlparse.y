%{
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

#include "vhdl2verilog.h"
void yyerror();
static  TCELLPNT  ParseLastModule;

%}

%token    T_ALL
%token    T_ARCHITECTURE
%token    T_ARRAY
%token    T_BEGIN
%token    T_BLOCK
%token    T_CASE
%token    T_COMPONENT
%token    T_CONSTANT
%token    T_CONVINT
%token    T_CONVSTDVEC
%token    T_DOWNTO
%token    T_ELSE
%token    T_ELSIF
%token    T_END
%token    T_ENTITY
%token    T_EVENT
%token    T_FOR
%token    T_FUNCTION
%token    T_GENERATE
%token    T_GENERIC
%token    T_IF
%token    T_IN
%token    T_INOUT
%token    T_INTEGER
%token    T_IS
%token    T_LIBRARY
%token    T_LOOP
%token    T_MAP
%token    T_NULL
%token    T_OF
%token    T_OTHERS
%token    T_OUT
%token    T_PORT
%token    T_PROCEDURE
%token    T_PROCESS
%token    T_RANGE
%token    T_RETURN
%token    T_SELECT
%token    T_SIGNAL
%token    T_SIGNED
%token    T_STDLOGICVEC
%token    T_STDLOGIC
%token    T_THEN
%token    T_TO
%token    T_TOSTDLOGICVEC
%token    T_TYPE
%token    T_UNSIGNED
%token    T_USE
%token    T_VARIABLE
%token    T_WHEN
%token    T_WITH

%token    T_LPAREN
%token    T_RPAREN
%token    T_SIGSUBST
%token    T_VARSUBST
%token    T_PARSUBST
%token    T_SEMICOLON
%token    T_COLON
%token    T_COMMA
%token    T_COND_OR
%token    T_DECDIGIT
%token    T_HEXDIGIT
%token    T_BINDIGIT
%token    T_ID

%token    N_DUMMY
%token    N_GENERICDEF
%token    N_MODULE
%token    N_PARAMDEF
%token    N_PORTDEF
%token    N_SIGDEF
%token    N_ENUMTYPE
%token    N_SYNCPROCESS
%token    N_ASYNCPROCESS
%token    N_CALLPROC
%token    N_ASSIGN
%token    N_SYNCIF
%token    N_ASYNCIF
%token    N_CASECOND
%token    N_CONDASSIGN
%token    N_WHENCOND
%token    N_SELASSIGN
%token    N_SELCOND
%token    N_PORTMAP
%token    N_PORTITEMNAME
%token    N_IDLIST
%token    N_GE
%token    N_OTHERS
%token    N_CALLFUNC
%token    N_PAREN
%token    N_STDVECTOR
%token    N_STDELEMENT
%token    N_LISTARRAY
%token    N_TYPEID
%token    N_FORLOOP
%token    N_FORGENERATE

%left      T_AND
%left      T_OR
%nonassoc  T_NAND
%nonassoc  T_NOR
%left      T_XOR
%right     T_NOT
%nonassoc  T_LE
%nonassoc  T_GT
%nonassoc  T_LS
%nonassoc  T_NOTEQUAL
%nonassoc  T_EQUAL
%nonassoc  T_SLL
%nonassoc  T_SRL
%left      T_PLUS
%left      T_MINUS
%left      T_MULT
%left      T_DIV
%left      T_MOD
%left      T_CONCAT
%left      T_UPLUS
%left      T_UMINUS

%%

top
  : module
    { ParseTreeTop = $1; }
  ;

module    /* RTL top */
  : module header_part entity_part arch_part
    {  /* one design unit: N_MODULE info0=header, info1=entity, info2=arch */
      TCELLPNT newmod = MallocTcell(N_MODULE,NULLSTR,0);
      SetNext(ParseLastModule,newmod); ParseLastModule = newmod;        /* link module chain */
      SetLine(newmod,CellLine($4)); SetNext(newmod,NULLCELL); SetInfo0(newmod,$2); SetInfo1(newmod,$3); SetInfo2(newmod,$4);

      SetSig(newmod,ParseSigListTop); ParseSigListTop = MakeNewSigList();    /* reset signal list */
      SetTypList(newmod,TypeListTop); TypeListTop = MakeNewTypeList();    /* reset type list */
      /* reset comment list ... unnecessary (common to all modules) */
      /* reset extract list ... unnecessary (common to all modules) */
    }
  | header_part entity_part arch_part
    {  /* one design unit: N_MODULE info0=header, info1=entity, info2=arch */
      TCELLPNT newmod = MallocTcell(N_MODULE,NULLSTR,0);
      $$ = ParseLastModule = newmod;
      SetLine(newmod,CellLine($3)); SetNext(newmod,NULLCELL); SetInfo0(newmod,$1); SetInfo1(newmod,$2); SetInfo2(newmod,$3);

      SetSig(newmod,ParseSigListTop); ParseSigListTop = MakeNewSigList();    /* reset signal list */
      SetTypList(newmod,TypeListTop); TypeListTop = MakeNewTypeList();    /* reset type list */
      /* reset comment list ... unnecessary (common to all modules) */
      /* reset extract list ... unnecessary (common to all modules) */
    }
  ;

header_part
  : header_part header_item
    {  /* library/use definitions */
      $$ = NULLCELL;
      FreeTcell($1); FreeTcell($2);
    }
  | header_item
    {  /* library/use definitions */
      $$ = NULLCELL;
      FreeTcell($1);
    }
  ;

header_item
  : T_LIBRARY id_list T_SEMICOLON
    {  /* library/use definitions */
      $$ = NULLCELL;
      FreeTcell($1); FreeTree($2); FreeTcell($3);
    }
  | use_item T_SEMICOLON
    {  /* library/use definitions */
      $$ = NULLCELL;
      FreeTcell($1);
      FreeTcell($2);
    }
  ;

use_item
  : use_item T_COMMA T_ID
    {  /* library/use definitions */
      $$ = NULLCELL;
      FreeTcell($1);
      FreeTcell($2); FreeTcell($3);
    }
  | T_USE T_ID
    {  /* library/use definitions */
      $$ = NULLCELL;
      FreeTcell($1); FreeTcell($2);
    }
  ;

entity_part
  : T_ENTITY T_ID T_IS generic_part port_part T_END T_ENTITY T_ID T_SEMICOLON
    {  /* entity part: T_ENTITY info0=ID, info1=generic, info2=port */
      $$ = $1; SetLine($$,CellLine($3)); SetInfo0($$,$2); SetInfo1($$,$4); SetInfo2($$,$5);
      FreeTcell($3); FreeTcell($6); FreeTcell($7); FreeTcell($8); FreeTcell($9);
      RegisterIoSignals(ParseSigListTop, CellInfo2($$));
    }
  | T_ENTITY T_ID T_IS generic_part port_part T_END T_ID T_SEMICOLON
    {  /* entity part: T_ENTITY info0=ID, info1=generic, info2=port */
      $$ = $1; SetLine($$,CellLine($3)); SetInfo0($$,$2); SetInfo1($$,$4); SetInfo2($$,$5);
      FreeTcell($3); FreeTcell($6); FreeTcell($7); FreeTcell($8);
      RegisterIoSignals(ParseSigListTop, CellInfo2($$));
    }
  | T_ENTITY T_ID T_IS generic_part port_part T_END T_SEMICOLON
    {  /* entity part: T_ENTITY info0=ID, info1=generic, info2=port */
      $$ = $1; SetLine($$,CellLine($3)); SetInfo0($$,$2); SetInfo1($$,$4); SetInfo2($$,$5);
      FreeTcell($3); FreeTcell($6); FreeTcell($7);
      RegisterIoSignals(ParseSigListTop, CellInfo2($$));
    }
  | T_ENTITY T_ID T_IS port_part T_END T_ENTITY T_ID T_SEMICOLON
    {  /* entity part: T_ENTITY info0=ID, info1=generic(NULL), info2=port */
      $$ = $1; SetLine($$,CellLine($3)); SetInfo0($$,$2); SetInfo1($$,NULLCELL); SetInfo2($$,$4);
      FreeTcell($3); FreeTcell($5); FreeTcell($6); FreeTcell($7); FreeTcell($8);
      RegisterIoSignals(ParseSigListTop, CellInfo2($$));
    }
  | T_ENTITY T_ID T_IS port_part T_END T_ID T_SEMICOLON
    {  /* entity part: T_ENTITY info0=ID, info1=generic(NULL), info2=port */
      $$ = $1; SetLine($$,CellLine($3)); SetInfo0($$,$2); SetInfo1($$,NULLCELL); SetInfo2($$,$4);
      FreeTcell($3); FreeTcell($5); FreeTcell($6); FreeTcell($7);
      RegisterIoSignals(ParseSigListTop, CellInfo2($$));
    }
  | T_ENTITY T_ID T_IS port_part T_END T_SEMICOLON
    {  /* entity part: T_ENTITY info0=ID, info1=generic(NULL), info2=port */
      $$ = $1; SetLine($$,CellLine($3)); SetInfo0($$,$2); SetInfo1($$,NULLCELL); SetInfo2($$,$4);
      FreeTcell($3); FreeTcell($5); FreeTcell($6);
      RegisterIoSignals(ParseSigListTop, CellInfo2($$));
    }
  | T_ENTITY T_ID T_IS T_END T_ENTITY T_ID T_SEMICOLON
    {  /* entity part: T_ENTITY info0=ID, info1=generic(NULL), info2=port(NULL) */
      $$ = $1; SetLine($$,CellLine($3)); SetInfo0($$,$2); SetInfo1($$,NULLCELL); SetInfo2($$,NULLCELL);
      FreeTcell($3); FreeTcell($4); FreeTcell($5); FreeTcell($6); FreeTcell($7);
      RegisterIoSignals(ParseSigListTop, CellInfo2($$));
    }
  | T_ENTITY T_ID T_IS T_END T_ID T_SEMICOLON
    {  /* entity part: T_ENTITY info0=ID, info1=generic(NULL), info2=port(NULL) */
      $$ = $1; SetLine($$,CellLine($3)); SetInfo0($$,$2); SetInfo1($$,NULLCELL); SetInfo2($$,NULLCELL);
      FreeTcell($3); FreeTcell($4); FreeTcell($5); FreeTcell($6);
      RegisterIoSignals(ParseSigListTop, CellInfo2($$));
    }
  | T_ENTITY T_ID T_IS T_END T_SEMICOLON
    {  /* entity part: T_ENTITY info0=ID, info1=generic(NULL), info2=port(NULL) */
      $$ = $1; SetLine($$,CellLine($3)); SetInfo0($$,$2); SetInfo1($$,NULLCELL); SetInfo2($$,NULLCELL);
      FreeTcell($3); FreeTcell($4); FreeTcell($5);
      RegisterIoSignals(ParseSigListTop, CellInfo2($$));
    }
  ;

generic_part
  : T_GENERIC T_LPAREN parameter_part T_RPAREN T_SEMICOLON
    {  /* generic part: N_GENERICDEF info0=parameter */
      $$ = $1; SetLine($$,CellLine($2)); SetType($$,N_GENERICDEF); SetInfo0($$,$3);
      FreeTcell($2); FreeTcell($4); FreeTcell($5);
    }
  ;

parameter_part
  : parameter_item parameter_part
    {  /* parameters: */
      $$ = $1; SetNext($$,$2);
    }
  | parameter_item
    {  /* parameters: */
      $$ = $1; SetNext($$,NULLCELL);
    }
  ;

parameter_item
  : id_list T_COLON sigtype T_VARSUBST exp T_SEMICOLON
    {  /* parameter item: N_PARAMDEF info0=idlist, info1=sigtype, info2=defaultval */
      $$ = MallocTcell(N_PARAMDEF,NULLSTR,0); SetLine($$,CellLine($1)); SetInfo0($$,$1); SetInfo1($$,$3); SetInfo2($$,$5);
      FreeTcell($2); FreeTcell($4); FreeTcell($6);
    }
  | id_list T_COLON sigtype T_VARSUBST exp
    {  /* parameter item: N_PARAMDEF info0=idlist, info1=sigtype, info2=defaultval */
      $$ = MallocTcell(N_PARAMDEF,NULLSTR,0); SetLine($$,CellLine($1)); SetInfo0($$,$1); SetInfo1($$,$3); SetInfo2($$,$5);
      FreeTcell($2); FreeTcell($4);
    }
  | id_list T_COLON sigtype T_SEMICOLON
    {  /* parameter item: N_PARAMDEF info0=idlist, info1=sigtype, info2=defaultval(NULL) */
      $$ = MallocTcell(N_PARAMDEF,NULLSTR,0); SetLine($$,CellLine($1)); SetInfo0($$,$1); SetInfo1($$,$3); SetInfo2($$,NULLCELL);
      FreeTcell($2); FreeTcell($4);
    }
  | id_list T_COLON sigtype
    {  /* parameter item: N_PARAMDEF info0=idlist, info1=sigtype, info2=defaultval(NULL) */
      $$ = MallocTcell(N_PARAMDEF,NULLSTR,0); SetLine($$,CellLine($1)); SetInfo0($$,$1); SetInfo1($$,$3); SetInfo2($$,NULLCELL);
      FreeTcell($2);
    }
  ;

port_part
  : T_PORT io_part T_SEMICOLON
    {  /* port part: N_PORTDEF info0=parameter */
      $$ = $1; SetLine($$,CellLine($1)); SetType($$,N_PORTDEF); SetInfo0($$,$2);
      FreeTcell($3);
    }
  ;

io_part
  : T_LPAREN io_list T_RPAREN
    {
      $$ = $2;
      FreeTcell($1); FreeTcell($3);
    }
  ;

io_list
  : io_item io_list
    {  /* io defs: */
      $$ = $1; SetNext($$,$2);
    }
  | io_item
    {  /* io defs: */
      $$ = $1; SetNext($$,NULLCELL);
    }
  ;

io_item
  : T_SIGNAL io_item
    {  /* (procedure signal I/O) */
      $$ = $2;
      FreeTcell($1);
    }
  | id_list T_COLON sigdir sigtype T_SEMICOLON
    {  /* signal item: N_SIGDEF info0=idlist, info1=sigdir, info2=sigtype */
      $$ = MallocTcell(N_SIGDEF,NULLSTR,0); SetLine($$,CellLine($1)); SetInfo0($$,$1); SetInfo1($$,$3); SetInfo2($$,$4);
      FreeTcell($2); FreeTcell($5);
    }
  | id_list T_COLON sigdir sigtype
    {  /* signal item: N_SIGDEF info0=idlist, info1=sigdir, info2=sigtype */
      $$ = MallocTcell(N_SIGDEF,NULLSTR,0); SetLine($$,CellLine($1)); SetInfo0($$,$1); SetInfo1($$,$3); SetInfo2($$,$4);
      FreeTcell($2);
    }
  ;

id_list
  : T_ID T_COMMA id_list
    {  /* signal/parameter names: N_IDLIST */
      $$ = $1; SetType($$,N_IDLIST); SetNext($$,$3);
      FreeTcell($2);
    }
  | T_ID
    {  /* signal/parameter names: N_IDLIST */
      $$ = $1; SetType($$,N_IDLIST); SetNext($$,NULLCELL);
    }
  ;

sigdir
  : T_IN
    {
      $$ = $1;
    }
  | T_OUT
    {
      $$ = $1;
    }
  | T_INOUT
    {
      $$ = $1;
    }
  ;

sigtype
  : T_STDLOGIC
    {
      $$ = $1;
    }
  | T_STDLOGICVEC T_LPAREN sigwidth T_RPAREN
    {  /* stdlogicvec: T_STDLOGICVEC info0 = sigwidth */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$3);
      FreeTcell($2); FreeTcell($4);
    }
  | T_STDLOGICVEC
    {  /* stdlogicvec: T_STDLOGICVEC info0 = sigwidth(NULL) */
      $$ = $1; SetInfo0($$,NULLCELL);
    }
  | T_INTEGER T_RANGE sigwidth
    {  /* integer: T_INTEGER info0 = sigwidth */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$3);
      FreeTcell($2);
    }
  | T_INTEGER
    {  /* integer: T_INTEGER info0 = sigwidth(NULL) */
      $$ = $1; SetInfo0($$,NULLCELL);
    }
  | T_ID
    {
      $$ = $1; SetType($$,N_TYPEID);
    }
  ;

sigwidth
  : exp T_TO exp
    {  /* sigwidth: T_TO info0=exp_L, info1=exp_R */
      $$ = $2; SetLine($$,CellLine($1)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | exp T_DOWNTO exp
    {  /* sigwidth: T_DOWNTO info0=exp_L, info1=exp_R */
      $$ = $2; SetLine($$,CellLine($1)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  ;

arch_part
  : T_ARCHITECTURE T_ID T_OF T_ID T_IS declaration_part T_BEGIN body_part T_END T_ARCHITECTURE T_ID T_SEMICOLON
    {  /* architecture part: T_ARCHITECTURE info0=declaration, info1=body */
      $$ = $1; SetLine($$,CellLine($5)); SetInfo0($$,$6); SetInfo1($$,$8);
      FreeTcell($2); FreeTcell($3); FreeTcell($4); FreeTcell($5); FreeTcell($7);
      FreeTcell($9); FreeTcell($10); FreeTcell($11); FreeTcell($12);
    }
  | T_ARCHITECTURE T_ID T_OF T_ID T_IS declaration_part T_BEGIN body_part T_END T_ID T_SEMICOLON
    {  /* architecture part: T_ARCHITECTURE info0=declaration, info1=body */
      $$ = $1; SetLine($$,CellLine($5)); SetInfo0($$,$6); SetInfo1($$,$8);
      FreeTcell($2); FreeTcell($3); FreeTcell($4); FreeTcell($5); FreeTcell($7);
      FreeTcell($9); FreeTcell($10); FreeTcell($11);
    }
  | T_ARCHITECTURE T_ID T_OF T_ID T_IS declaration_part T_BEGIN body_part T_END T_SEMICOLON
    {  /* architecture part: T_ARCHITECTURE info0=declaration, info1=body */
      $$ = $1; SetLine($$,CellLine($5)); SetInfo0($$,$6); SetInfo1($$,$8);
      FreeTcell($2); FreeTcell($3); FreeTcell($4); FreeTcell($5); FreeTcell($7);
      FreeTcell($9); FreeTcell($10);
    }
  | T_ARCHITECTURE T_ID T_OF T_ID T_IS T_BEGIN body_part T_END T_ARCHITECTURE T_ID T_SEMICOLON
    {  /* architecture part: T_ARCHITECTURE info0=declaration(NULL), info1=body */
      $$ = $1; SetLine($$,CellLine($6)); SetInfo0($$,NULLCELL); SetInfo1($$,$7);
      FreeTcell($2); FreeTcell($3); FreeTcell($4); FreeTcell($5); FreeTcell($6);
      FreeTcell($8); FreeTcell($9); FreeTcell($10); FreeTcell($11);
    }
  | T_ARCHITECTURE T_ID T_OF T_ID T_IS T_BEGIN body_part T_END T_ID T_SEMICOLON
    {  /* architecture part: T_ARCHITECTURE info0=declaration(NULL), info1=body */
      $$ = $1; SetLine($$,CellLine($6)); SetInfo0($$,NULLCELL); SetInfo1($$,$7);
      FreeTcell($2); FreeTcell($3); FreeTcell($4); FreeTcell($5); FreeTcell($6);
      FreeTcell($8); FreeTcell($9); FreeTcell($10);
    }
  | T_ARCHITECTURE T_ID T_OF T_ID T_IS T_BEGIN body_part T_END T_SEMICOLON
    {  /* architecture part: T_ARCHITECTURE info0=declaration(NULL), info1=body */
      $$ = $1; SetLine($$,CellLine($6)); SetInfo0($$,NULLCELL); SetInfo1($$,$7);
      FreeTcell($2); FreeTcell($3); FreeTcell($4); FreeTcell($5); FreeTcell($6);
      FreeTcell($8); FreeTcell($9);
    }
  ;

declaration_part
  : component_part declaration_part
    {  /* (discard component declaration) */
      $$ = $2;
    }
  | component_part
    {  /* (discard component declaration) */
      $$ = NULLCELL;
    }
  | const_part declaration_part
    {
      $$ = $1; SetNext($$,$2);
    }
  | const_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | signal_part declaration_part
    {
      $$ = $1; SetNext($$,$2);
    }
  | signal_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | typedef_part declaration_part
    {
      $$ = $1; SetNext($$,$2);
    }
  | typedef_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | function_part declaration_part
    {
      $$ = $1; SetNext($$,$2);
    }
  | function_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | procedure_part declaration_part
    {
      $$ = $1; SetNext($$,$2);
    }
  | procedure_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  ;

component_part
  : T_COMPONENT T_ID generic_part port_part T_END T_COMPONENT T_SEMICOLON
    {  /* (discard component declaration) */
      $$ = NULLCELL;
      FreeTcell($1); FreeTcell($2); FreeTree($3); FreeTree($4); FreeTcell($5); FreeTcell($6); FreeTcell($7);
    }
  | T_COMPONENT T_ID port_part T_END T_COMPONENT T_SEMICOLON
    {  /* (discard component declaration) */
      $$ = NULLCELL;
      FreeTcell($1); FreeTcell($2); FreeTree($3); FreeTcell($4); FreeTcell($5); FreeTcell($6);
    }
  ;

const_part
  : T_CONSTANT T_ID T_COLON sigtype T_VARSUBST exp T_SEMICOLON
    {  /* constant def: T_CONSTANT info0=ID, info1=type, info2=value */
      $$ = $1; SetLine($$,CellLine($7)); SetInfo0($$,$2); SetInfo1($$,$4); SetInfo2($$,$6);
      if (AppendSigList(ParseSigListTop,CellStr($2),$4,False,False,False,False) == False)  /* (const) */
        yyerror("$Duplicate signal declaration (Verilog is case sensitive).");
      FreeTcell($3); FreeTcell($5); FreeTcell($7);
    }
  ;

signal_part
  : T_SIGNAL id_list T_COLON sigtype T_SEMICOLON
    {  /* signal def: T_SIGNAL info0=idlist, info1=type */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,$2); SetInfo1($$,$4);
      {
        register TCELLPNT  item;
        for (item = $2; item != NULLCELL; item = NextCell(item)) {
          if (AppendSigList(ParseSigListTop,CellStr(item),$4,False,False,False,False) == False)  /* (wire) */
            yyerror("$Duplicate signal declaration (Verilog is case sensitive).");
        }
      }
      FreeTcell($3); FreeTcell($5);
    }
  ;

typedef_part
  : T_TYPE T_ID T_IS type_item T_SEMICOLON
    {  /* type def: T_TYPE info0=ID, info1=type */
      $$ = $1; SetLine($$,CellLine($3)); SetInfo0($$,$2); SetInfo1($$,$4);
      AppendTypeList(TypeListTop, $$);
      FreeTcell($3); FreeTcell($5);
    }
  | T_TYPE T_ID T_IS sigtype T_SEMICOLON
    {  /* type def: T_TYPE info0=ID, info1=type */
      $$ = $1; SetLine($$,CellLine($3)); SetInfo0($$,$2); SetInfo1($$,$4);
      AppendTypeList(TypeListTop, $$);
      FreeTcell($3); FreeTcell($5);
    }
  ;

type_item
  : T_LPAREN id_list T_RPAREN
    {  /* type def: N_ENUMTYPE info0=list */
      $$ = $1; SetLine($$,CellLine($1)); SetType($$,N_ENUMTYPE); SetInfo0($$,$2);
      FreeTcell($3);
    }
  | T_ARRAY T_LPAREN sigwidth T_RPAREN T_OF sigtype
    {  /* type def: T_ARRAY info0=width, info1=type */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$3); SetInfo1($$,$6);
      FreeTcell($2); FreeTcell($4); FreeTcell($5);
    }
  ;

function_part
  : T_FUNCTION T_ID funcport_part T_RETURN sigtype T_IS processvar_part T_BEGIN process_body func_result T_END T_FUNCTION T_ID T_SEMICOLON
    {  /* function def: T_FUNCTION info0=ID, info1=port, info2=type, info3=variable, info4=body, info5=ret */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$2); SetInfo1($$,$3); SetInfo2($$,$5); SetInfo3($$,$7); SetInfo4($$,$9); SetInfo5($$,$10);
      if (AppendSigList(ParseSigListTop,CellStr($2),NULLCELL,False,False,False,True) == False)  /* (function/procedure) */
        yyerror("$Duplicate function declaration.");
      FreeTcell($4); FreeTcell($6); FreeTcell($8);
      FreeTcell($11); FreeTcell($12); FreeTcell($13); FreeTcell($14);
    }
  | T_FUNCTION T_ID funcport_part T_RETURN sigtype T_IS processvar_part T_BEGIN process_body func_result T_END T_ID T_SEMICOLON
    {  /* function def: T_FUNCTION info0=ID, info1=port, info2=type, info3=variable, info4=body, info5=ret */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$2); SetInfo1($$,$3); SetInfo2($$,$5); SetInfo3($$,$7); SetInfo4($$,$9); SetInfo5($$,$10);
      if (AppendSigList(ParseSigListTop,CellStr($2),NULLCELL,False,False,False,True) == False)  /* (function/procedure) */
        yyerror("$Duplicate function declaration.");
      FreeTcell($4); FreeTcell($6); FreeTcell($8);
      FreeTcell($11); FreeTcell($12); FreeTcell($13);
    }
  | T_FUNCTION T_ID funcport_part T_RETURN sigtype T_IS T_BEGIN process_body func_result T_END T_FUNCTION T_ID T_SEMICOLON
    {  /* function def: T_FUNCTION info0=ID, info1=port, info2=type, info3=variable(NULL), info4=body, info5=ret */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$2); SetInfo1($$,$3); SetInfo2($$,$5); SetInfo3($$,NULLCELL); SetInfo4($$,$8); SetInfo5($$,$9);
      if (AppendSigList(ParseSigListTop,CellStr($2),NULLCELL,False,False,False,True) == False)  /* (function/procedure) */
        yyerror("$Duplicate function declaration.");
      FreeTcell($4); FreeTcell($6); FreeTcell($7);
      FreeTcell($10); FreeTcell($11); FreeTcell($12); FreeTcell($13);
    }
  | T_FUNCTION T_ID funcport_part T_RETURN sigtype T_IS T_BEGIN process_body func_result T_END T_ID T_SEMICOLON
    {  /* function def: T_FUNCTION info0=ID, info1=port, info2=type, info3=variable(NULL), info4=body, info5=ret */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$2); SetInfo1($$,$3); SetInfo2($$,$5); SetInfo3($$,NULLCELL); SetInfo4($$,$8); SetInfo5($$,$9);
      if (AppendSigList(ParseSigListTop,CellStr($2),NULLCELL,False,False,False,True) == False)  /* (function/procedure) */
        yyerror("$Duplicate function declaration.");
      FreeTcell($4); FreeTcell($6); FreeTcell($7);
      FreeTcell($10); FreeTcell($11); FreeTcell($12);
    }
  | T_FUNCTION T_ID funcport_part T_RETURN sigtype T_IS processvar_part T_BEGIN func_result T_END T_FUNCTION T_ID T_SEMICOLON
    {  /* function def: T_FUNCTION info0=ID, info1=port, info2=type, info3=variable, info4=body(NULL), info5=ret */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$2); SetInfo1($$,$3); SetInfo2($$,$5); SetInfo3($$,$7); SetInfo4($$,NULLCELL); SetInfo5($$,$9);
      if (AppendSigList(ParseSigListTop,CellStr($2),NULLCELL,False,False,False,True) == False)  /* (function/procedure) */
        yyerror("$Duplicate function declaration.");
      FreeTcell($4); FreeTcell($6); FreeTcell($8);
      FreeTcell($10); FreeTcell($11); FreeTcell($12); FreeTcell($13);
    }
  | T_FUNCTION T_ID funcport_part T_RETURN sigtype T_IS processvar_part T_BEGIN func_result T_END T_ID T_SEMICOLON
    {  /* function def: T_FUNCTION info0=ID, info1=port, info2=type, info3=variable, info4=body(NULL), info5=ret */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$2); SetInfo1($$,$3); SetInfo2($$,$5); SetInfo3($$,$7); SetInfo4($$,NULLCELL); SetInfo5($$,$9);
      if (AppendSigList(ParseSigListTop,CellStr($2),NULLCELL,False,False,False,True) == False)  /* (function/procedure) */
        yyerror("$Duplicate function declaration.");
      FreeTcell($4); FreeTcell($6); FreeTcell($8);
      FreeTcell($10); FreeTcell($11); FreeTcell($12);
    }
  | T_FUNCTION T_ID funcport_part T_RETURN sigtype T_IS T_BEGIN func_result T_END T_FUNCTION T_ID T_SEMICOLON
    {  /* function def: T_FUNCTION info0=ID, info1=port, info2=type, info3=variable(NULL), info4=body(NULL), info5=ret */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$2); SetInfo1($$,$3); SetInfo2($$,$5); SetInfo3($$,NULLCELL); SetInfo4($$,NULLCELL); SetInfo5($$,$8);
      if (AppendSigList(ParseSigListTop,CellStr($2),NULLCELL,False,False,False,True) == False)  /* (function/procedure) */
        yyerror("$Duplicate function declaration.");
      FreeTcell($4); FreeTcell($6); FreeTcell($7);
      FreeTcell($9); FreeTcell($10); FreeTcell($11); FreeTcell($12);
    }
  | T_FUNCTION T_ID funcport_part T_RETURN sigtype T_IS T_BEGIN func_result T_END T_ID T_SEMICOLON
    {  /* function def: T_FUNCTION info0=ID, info1=port, info2=type, info3=variable(NULL), info4=body(NULL), info5=ret */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$2); SetInfo1($$,$3); SetInfo2($$,$5); SetInfo3($$,NULLCELL); SetInfo4($$,NULLCELL); SetInfo5($$,$8);
      if (AppendSigList(ParseSigListTop,CellStr($2),NULLCELL,False,False,False,True) == False)  /* (function/procedure) */
        yyerror("$Duplicate function declaration.");
      FreeTcell($4); FreeTcell($6); FreeTcell($7);
      FreeTcell($9); FreeTcell($10); FreeTcell($11);
    }
  ;

funcport_part
  : T_LPAREN parameter_part T_RPAREN
    {
      $$ = $2;
      FreeTcell($1); FreeTcell($3);
    }
  ;

func_result
  : T_RETURN exp T_SEMICOLON
    {  /* function return: T_RETURN info0=exp */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,$2);
      FreeTcell($3);
    }
  ;

procedure_part
  : T_PROCEDURE T_ID io_part T_IS processvar_part T_BEGIN process_body T_END T_PROCEDURE T_ID T_SEMICOLON
    {  /* procedure def: T_PROCEDURE info0=ID, info1=port, info2=variale, info3=body */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$2); SetInfo1($$,$3); SetInfo2($$,$5); SetInfo3($$,$7);
      if (AppendSigList(ParseSigListTop,CellStr($2),NULLCELL,False,False,False,True) == False)  /* (function/procedure) */
        yyerror("$Duplicate procedure declaration.");
      FreeTcell($4); FreeTcell($6); FreeTcell($8);
      FreeTcell($9); FreeTcell($10); FreeTcell($11);
    }
  | T_PROCEDURE T_ID io_part T_IS processvar_part T_BEGIN process_body T_END T_ID T_SEMICOLON
    {  /* procedure def: T_PROCEDURE info0=ID, info1=port, info2=variale, info3=body */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$2); SetInfo1($$,$3); SetInfo2($$,$5); SetInfo3($$,$7);
      if (AppendSigList(ParseSigListTop,CellStr($2),NULLCELL,False,False,False,True) == False)  /* (function/procedure) */
        yyerror("$Duplicate procedure declaration.");
      FreeTcell($4); FreeTcell($6); FreeTcell($8);
      FreeTcell($9); FreeTcell($10);
    }
  | T_PROCEDURE T_ID io_part T_IS T_BEGIN process_body T_END T_PROCEDURE T_ID T_SEMICOLON
    {  /* procedure def: T_PROCEDURE info0=ID, info1=port, info2=variale(NULL), info3=body */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$2); SetInfo1($$,$3); SetInfo2($$,NULLCELL); SetInfo3($$,$6);
      if (AppendSigList(ParseSigListTop,CellStr($2),NULLCELL,False,False,False,True) == False)  /* (function/procedure) */
        yyerror("$Duplicate procedure declaration.");
      FreeTcell($4); FreeTcell($5); FreeTcell($7);
      FreeTcell($8); FreeTcell($9); FreeTcell($10);
    }
  | T_PROCEDURE T_ID io_part T_IS T_BEGIN process_body T_END T_ID T_SEMICOLON
    {  /* procedure def: T_PROCEDURE info0=ID, info1=port, info2=variale(NULL), info3=body */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$2); SetInfo1($$,$3); SetInfo2($$,NULLCELL); SetInfo3($$,$6);
      if (AppendSigList(ParseSigListTop,CellStr($2),NULLCELL,False,False,False,True) == False)  /* (function/procedure) */
        yyerror("$Duplicate procedure declaration.");
      FreeTcell($4); FreeTcell($5); FreeTcell($7);
      FreeTcell($8); FreeTcell($9);
    }
  ;

body_part
  : process_part body_part
    {
      $$ = $1; SetNext($$,$2);
    }
  | process_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | subst_part body_part
    {
      $$ = $1; SetType($1,N_ASSIGN); SetNext($$,$2);
    }
  | subst_part
    {
      $$ = $1; SetType($1,N_ASSIGN); SetNext($$,NULLCELL);
    }
  | when_part body_part
    {
      $$ = $1; SetNext($$,$2);
    }
  | when_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | withsel_part body_part
    {
      $$ = $1; SetNext($$,$2);
    }
  | withsel_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | portmap_part body_part
    {
      $$ = $1; SetNext($$,$2);
    }
  | portmap_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | block_part body_part
    {
      $$ = $1; SetNext($$,$2);
    }
  | block_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | forgenerate_part body_part
    {
      $$ = $1; SetNext($$,$2);
    }
  | forgenerate_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | callproc_part body_part
    {
      $$ = $1; SetNext($$,$2);
    }
  | callproc_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  ;

forgenerate_part
  : T_ID T_COLON T_FOR T_ID T_IN sigwidth T_GENERATE body_part T_END T_GENERATE T_ID T_SEMICOLON
    {  /* for generate: T_FOR info0=ID(label), info1=ID(variable), info2=width, info3=body */
      $$ = $3; SetType($$,N_FORGENERATE); SetLine($$,CellLine($5)); SetInfo0($$,$1); SetInfo1($$,$4); SetInfo2($$,$6); SetInfo3($$,$8);
      FreeTcell($2); FreeTcell($5); FreeTcell($7);
      FreeTcell($9); FreeTcell($10); FreeTcell($11); FreeTcell($12);
    }
  | T_ID T_COLON T_FOR T_ID T_IN sigwidth T_GENERATE body_part T_END T_GENERATE T_SEMICOLON
    {  /* for generate: T_FOR info0=ID(label), info1=ID(variable), info2=width, info3=body */
      $$ = $3; SetType($$,N_FORGENERATE); SetLine($$,CellLine($5)); SetInfo0($$,$1); SetInfo1($$,$4); SetInfo2($$,$6); SetInfo3($$,$8);
      FreeTcell($2); FreeTcell($5); FreeTcell($7);
      FreeTcell($9); FreeTcell($10); FreeTcell($11);
    }
  | T_FOR T_ID T_IN sigwidth T_GENERATE body_part T_END T_GENERATE T_ID T_SEMICOLON
    {  /* for generate: T_FOR info0=ID(label,NULL), info1=ID(variable), info2=width, info3=body */
      $$ = $1; SetType($$,N_FORGENERATE); SetLine($$,CellLine($3)); SetInfo0($$,NULLCELL); SetInfo1($$,$2); SetInfo2($$,$4); SetInfo3($$,$6);
      FreeTcell($3); FreeTcell($5); FreeTcell($7);
      FreeTcell($8); FreeTcell($9); FreeTcell($10);
    }
  | T_FOR T_ID T_IN sigwidth T_GENERATE body_part T_END T_GENERATE T_SEMICOLON
    {  /* for generate: T_FOR info0=ID(label,NULL), info1=ID(variable), info2=width, info3=body */
      $$ = $1; SetType($$,N_FORGENERATE); SetLine($$,CellLine($3)); SetInfo0($$,NULLCELL); SetInfo1($$,$2); SetInfo2($$,$4); SetInfo3($$,$6);
      FreeTcell($3); FreeTcell($5); FreeTcell($7);
      FreeTcell($8); FreeTcell($9);
    }
  ;

process_part
  : T_ID T_COLON T_PROCESS T_LPAREN sensesig_list T_RPAREN processvar_part T_BEGIN process_body T_END T_PROCESS T_ID T_SEMICOLON
    {  /* process: T_PROCESS info0=ID(label), info1=sensitivity, info2=variable, info3=body */
      $$ = $3; SetLine($$,CellLine($4)); SetInfo0($$,$1); SetInfo1($$,$5); SetInfo2($$,$7); SetInfo3($$,$9);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_SYNCIF)  SetType($$,N_SYNCPROCESS);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_ASYNCIF) SetType($$,N_ASYNCPROCESS);
      FreeTcell($2); FreeTcell($4); FreeTcell($6); FreeTcell($8);
      FreeTcell($10); FreeTcell($11); FreeTcell($12); FreeTcell($13);
    }
  | T_ID T_COLON T_PROCESS T_LPAREN sensesig_list T_RPAREN processvar_part T_BEGIN process_body T_END T_PROCESS T_SEMICOLON
    {  /* process: T_PROCESS info0=ID(label), info1=sensitivity, info2=variable, info3=body */
      $$ = $3; SetLine($$,CellLine($4)); SetInfo0($$,$1); SetInfo1($$,$5); SetInfo2($$,$7); SetInfo3($$,$9);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_SYNCIF)  SetType($$,N_SYNCPROCESS);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_ASYNCIF) SetType($$,N_ASYNCPROCESS);
      FreeTcell($2); FreeTcell($4); FreeTcell($6); FreeTcell($8);
      FreeTcell($10); FreeTcell($11); FreeTcell($12);
    }
  | T_ID T_COLON T_PROCESS T_LPAREN sensesig_list T_RPAREN T_BEGIN process_body T_END T_PROCESS T_ID T_SEMICOLON
    {  /* process: T_PROCESS info0=ID(label), info1=sensitivity, info2=variable(NULL), info3=body */
      $$ = $3; SetLine($$,CellLine($4)); SetInfo0($$,$1); SetInfo1($$,$5); SetInfo2($$,NULLCELL); SetInfo3($$,$8);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_SYNCIF)  SetType($$,N_SYNCPROCESS);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_ASYNCIF) SetType($$,N_ASYNCPROCESS);
      FreeTcell($2); FreeTcell($4); FreeTcell($6); FreeTcell($7);
      FreeTcell($9); FreeTcell($10); FreeTcell($11); FreeTcell($12);
    }
  | T_ID T_COLON T_PROCESS T_LPAREN sensesig_list T_RPAREN T_BEGIN process_body T_END T_PROCESS T_SEMICOLON
    {  /* process: T_PROCESS info0=ID(label), info1=sensitivity, info2=variable(NULL), info3=body */
      $$ = $3; SetLine($$,CellLine($4)); SetInfo0($$,$1); SetInfo1($$,$5); SetInfo2($$,NULLCELL); SetInfo3($$,$8);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_SYNCIF)  SetType($$,N_SYNCPROCESS);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_ASYNCIF) SetType($$,N_ASYNCPROCESS);
      FreeTcell($2); FreeTcell($4); FreeTcell($6); FreeTcell($7);
      FreeTcell($9); FreeTcell($10); FreeTcell($11);
    }
  | T_PROCESS T_LPAREN sensesig_list T_RPAREN processvar_part T_BEGIN process_body T_END T_PROCESS T_ID T_SEMICOLON
    {  /* process: T_PROCESS info0=ID(label,NULL), info1=sensitivity, info2=variable, info3=body */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,NULLCELL); SetInfo1($$,$3); SetInfo2($$,$5); SetInfo3($$,$7);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_SYNCIF)  SetType($$,N_SYNCPROCESS);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_ASYNCIF) SetType($$,N_ASYNCPROCESS);
      FreeTcell($2); FreeTcell($4); FreeTcell($6); FreeTcell($8);
      FreeTcell($9); FreeTcell($10); FreeTcell($11);
    }
  | T_PROCESS T_LPAREN sensesig_list T_RPAREN processvar_part T_BEGIN process_body T_END T_PROCESS T_SEMICOLON
    {  /* process: T_PROCESS info0=ID(label,NULL), info1=sensitivity, info2=variable, info3=body */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,NULLCELL); SetInfo1($$,$3); SetInfo2($$,$5); SetInfo3($$,$7);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_SYNCIF)  SetType($$,N_SYNCPROCESS);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_ASYNCIF) SetType($$,N_ASYNCPROCESS);
      FreeTcell($2); FreeTcell($4); FreeTcell($6); FreeTcell($8);
      FreeTcell($9); FreeTcell($10);
    }
  | T_PROCESS T_LPAREN sensesig_list T_RPAREN T_BEGIN process_body T_END T_PROCESS T_ID T_SEMICOLON
    {  /* process: T_PROCESS info0=ID(label,NULL), info1=sensitivity, info2=variable(NULL), info3=body */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,NULLCELL); SetInfo1($$,$3); SetInfo2($$,NULLCELL); SetInfo3($$,$6);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_SYNCIF)  SetType($$,N_SYNCPROCESS);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_ASYNCIF) SetType($$,N_ASYNCPROCESS);
      FreeTcell($2); FreeTcell($4); FreeTcell($5); FreeTcell($7);
      FreeTcell($8); FreeTcell($9); FreeTcell($10);
    }
  | T_PROCESS T_LPAREN sensesig_list T_RPAREN T_BEGIN process_body T_END T_PROCESS T_SEMICOLON
    {  /* process: T_PROCESS info0=ID(label,NULL), info1=sensitivity, info2=variable(NULL), info3=body */
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,NULLCELL); SetInfo1($$,$3); SetInfo2($$,NULLCELL); SetInfo3($$,$6);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_SYNCIF)  SetType($$,N_SYNCPROCESS);
      if (CellInfo3($$) != NULLCELL && CellType(CellInfo3($$)) == N_ASYNCIF) SetType($$,N_ASYNCPROCESS);
      FreeTcell($2); FreeTcell($4); FreeTcell($5);
      FreeTcell($7); FreeTcell($8); FreeTcell($9);
    }
  ;

processvar_part
  : processvar_item processvar_part
    {
      $$ = $1; SetNext($$,$2);
    }
  | processvar_item
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  ;

processvar_item
  : T_VARIABLE id_list T_COLON sigtype T_SEMICOLON
    {  /* variable def: T_VARIABLE info0=idlist, info1=type */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,$2); SetInfo1($$,$4);
      FreeTcell($3); FreeTcell($5);
    }
  | const_part
    {
      $$ = $1;
    }
  ;

process_body
  : subst_part process_body
    {
      register SIGLIST  *sig;
      $$ = $1; SetNext($$,$2);
      if ((sig = SearchSigList(ParseSigListTop, GetSigName(CellInfo1($1))) ) != NULLSIG) {
        SigIsReg(sig)  = True;    /* (register) */
      }
    }
  | subst_part
    {
      register SIGLIST  *sig;
      $$ = $1; SetNext($$,NULLCELL);
      if ((sig = SearchSigList(ParseSigListTop, GetSigName(CellInfo1($1))) ) != NULLSIG) {
        SigIsReg(sig)  = True;    /* (register) */
      }
    }
  | if_part process_body
    {
      $$ = $1; SetNext($$,$2);
    }
  | if_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | case_part process_body
    {
      $$ = $1; SetNext($$,$2);
    }
  | case_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | forloop_part process_body
    {
      $$ = $1; SetNext($$,$2);
    }
  | forloop_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | callproc_part process_body
    {
      $$ = $1; SetNext($$,$2);
    }
  | callproc_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | syncif_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  | asyncif_part
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  ;

forloop_part
  : T_FOR T_ID T_IN sigwidth T_LOOP process_body T_END T_LOOP T_SEMICOLON
    {  /* for loop: T_FOR info0=ID(label,NULL), info1=ID(variable), info2=width, info3=body */
      $$ = $1; SetType($$,N_FORLOOP); SetLine($$,CellLine($3)); SetInfo0($$,NULLCELL); SetInfo1($$,$2); SetInfo2($$,$4); SetInfo3($$,$6);
      FreeTcell($3); FreeTcell($5); FreeTcell($7);
      FreeTcell($8); FreeTcell($9);
    }
  ; 

callproc_part
  : T_ID T_LPAREN portmap_body T_RPAREN T_SEMICOLON
    {  /* call proc: N_CALLPROC info0=ID, info1=port */
      $$ = $2; SetLine($$,CellLine($2)); SetType($$,N_CALLPROC); SetInfo0($$,$1); SetInfo1($$,$3);
      FreeTcell($4); FreeTcell($5);
    }
  ;

subst_part
  : T_ID T_COLON signame T_SIGSUBST exp T_SEMICOLON
    {  /* subst: T_SIGSUBST/N_ASSIGN info0=ID, info1=signal, info2=exp */
      $$ = $4; SetLine($$,CellLine($2)); SetInfo0($$,$1); SetInfo1($$,$3); SetInfo2($$,$5);
      FreeTcell($2); FreeTcell($6);
    }
  | signame T_SIGSUBST exp T_SEMICOLON
    {  /* subst: T_SIGSUBST/N_ASSIGN info0=ID(NULL), info1=signal, info2=exp */
      $$ = $2; SetLine($$,CellLine($1)); SetInfo0($$,NULLCELL); SetInfo1($$,$1); SetInfo2($$,$3);
      FreeTcell($4);
    }
  | signame T_VARSUBST exp T_SEMICOLON
    {  /* subst: T_VARSUBST info0=NULL, info1=signal, info2=exp */
      $$ = $2; SetLine($$,CellLine($1)); SetInfo0($$,NULLCELL); SetInfo1($$,$1); SetInfo2($$,$3);
      FreeTcell($4);
    }
  ;

syncif_part
  : T_IF edge_cond T_THEN process_body T_END T_IF T_SEMICOLON
    {  /* if: N_SYNCIF info0=cond, info1=then, info2=else(NULL) */
      $$ = $1; SetLine($$,CellLine($1)); SetType($$,N_SYNCIF); SetInfo0($$,$2); SetInfo1($$,$4); SetInfo2($$,NULLCELL);
      FreeTcell($3); FreeTcell($5); FreeTcell($6); FreeTcell($7);
    }
  | T_IF edge_cond T_THEN T_END T_IF T_SEMICOLON
    {  /* if: N_SYNCIF info0=cond, info1=then(NULL), info2=else(NULL) */
      $$ = $1; SetLine($$,CellLine($1)); SetType($$,N_SYNCIF); SetInfo0($$,$2); SetInfo1($$,NULLCELL); SetInfo2($$,NULLCELL);
      FreeTcell($3); FreeTcell($4); FreeTcell($5); FreeTcell($6);
    }
  ;

asyncif_part
  : T_IF exp T_THEN process_body T_ELSIF edge_cond T_THEN process_body T_END T_IF T_SEMICOLON
    {  /* if: N_ASYNCIF info0=exp, info1=body1, info2=edge_cond, info3=body2 */
      $$ = $1; SetLine($$,CellLine($1)); SetType($$,N_ASYNCIF); SetInfo0($$,$2); SetInfo1($$,$4); SetInfo2($$,$6); SetInfo3($$,$8);
      FreeTcell($3); FreeTcell($5); FreeTcell($7);
      FreeTcell($9); FreeTcell($10); FreeTcell($11);
    }
  | T_IF exp T_THEN T_ELSIF edge_cond T_THEN process_body T_END T_IF T_SEMICOLON
    {  /* if: N_ASYNCIF info0=exp, info1=body1(NULL), info2=edge_cond, info3=body2 */
      $$ = $1; SetLine($$,CellLine($1)); SetType($$,N_ASYNCIF); SetInfo0($$,$2); SetInfo1($$,NULLCELL); SetInfo2($$,$5); SetInfo3($$,$7);
      FreeTcell($3); FreeTcell($4); FreeTcell($6);
      FreeTcell($8); FreeTcell($9); FreeTcell($10);
    }
  | T_IF exp T_THEN process_body T_ELSIF edge_cond T_THEN T_END T_IF T_SEMICOLON
    {  /* if: N_ASYNCIF info0=exp, info1=body1, info2=edge_cond, info3=body2(NULL) */
      $$ = $1; SetLine($$,CellLine($1)); SetType($$,N_ASYNCIF); SetInfo0($$,$2); SetInfo1($$,$4); SetInfo2($$,$6); SetInfo3($$,NULLCELL);
      FreeTcell($3); FreeTcell($5); FreeTcell($7);
      FreeTcell($8); FreeTcell($9); FreeTcell($10);
    }
  | T_IF exp T_THEN T_ELSIF edge_cond T_THEN T_END T_IF T_SEMICOLON
    {  /* if: N_ASYNCIF info0=exp, info1=body1(NULL), info2=edge_cond, info3=body2(NULL) */
      $$ = $1; SetLine($$,CellLine($1)); SetType($$,N_ASYNCIF); SetInfo0($$,$2); SetInfo1($$,NULLCELL); SetInfo2($$,$5); SetInfo3($$,NULLCELL);
      FreeTcell($3); FreeTcell($4); FreeTcell($6);
      FreeTcell($7); FreeTcell($8); FreeTcell($9);
    }
  ;

if_part
  : T_IF exp T_THEN process_body ifelse_part
    {  /* if: T_IF info0=cond, info1=then, info2=else */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,$2); SetInfo1($$,$4); SetInfo2($$,$5);
      FreeTcell($3);
    }
  | T_IF exp T_THEN process_body T_END T_IF T_SEMICOLON
    {  /* if: T_IF info0=cond, info1=then, info2=else(NULL) */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,$2); SetInfo1($$,$4); SetInfo2($$,NULLCELL);
      FreeTcell($3); FreeTcell($5); FreeTcell($6); FreeTcell($7);
    }
  | T_IF exp T_THEN ifelse_part
    {  /* if: T_IF info0=cond, info1=then(NULL), info2=else */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,$2); SetInfo1($$,NULLCELL); SetInfo2($$,$4);
      FreeTcell($3);
    }
  | T_IF exp T_THEN T_END T_IF T_SEMICOLON
    {  /* if: T_IF info0=cond, info1=then(NULL), info2=else(NULL) */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,$2); SetInfo1($$,NULLCELL); SetInfo2($$,NULLCELL);
      FreeTcell($3); FreeTcell($4); FreeTcell($5); FreeTcell($6);
    }
  ;

ifelse_part
  : T_ELSIF exp T_THEN process_body ifelse_part
    {  /* elsif: T_ELSIF info0=cond, info1=then, info2=else */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,$2); SetInfo1($$,$4); SetInfo2($$,$5);
      FreeTcell($3);
    }
  | T_ELSIF exp T_THEN process_body T_END T_IF T_SEMICOLON
    {  /* elsif: T_ELSIF info0=cond, info1=then, info2=else(NULL) */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,$2); SetInfo1($$,$4); SetInfo2($$,NULLCELL);
      FreeTcell($3); FreeTcell($5); FreeTcell($6); FreeTcell($7);
    }
  | T_ELSIF exp T_THEN ifelse_part
    {  /* elsif: T_ELSIF info0=cond, info1=then(NULL), info2=else */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,$2); SetInfo1($$,NULLCELL); SetInfo2($$,$4);
      FreeTcell($3);
    }
  | T_ELSIF exp T_THEN T_END T_IF T_SEMICOLON
    {  /* elsif: T_ELSIF info0=cond, info1=then(NULL), info2=else(NULL) */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,$2); SetInfo1($$,NULLCELL); SetInfo2($$,NULLCELL);
      FreeTcell($3); FreeTcell($4); FreeTcell($5); FreeTcell($6);
    }
  | T_ELSE process_body T_END T_IF T_SEMICOLON
    {  /* else: T_ELSE info0=else */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,$2);
      FreeTcell($3); FreeTcell($4); FreeTcell($5);
    }
  | T_ELSE T_END T_IF T_SEMICOLON
    {  /* else: T_ELSE info0=else(NULL) */
      $$ = $1; SetLine($$,CellLine($4)); SetInfo0($$,NULLCELL);
      FreeTcell($2); FreeTcell($3); FreeTcell($4);
    }
  ;

edge_cond
  : T_LPAREN edge_cond T_RPAREN
    {
      $$ = $2; SetLine($$,CellLine($1)); 
      FreeTcell($1); FreeTcell($3);
    }
  | T_EVENT T_AND T_ID T_EQUAL T_BINDIGIT
    {  /* event: T_EVENT info0=exp */
      $$ = $1; SetLine($$,CellLine($5)); SetInfo0($$,$4); SetInfo0($4,$3); SetInfo1($4,$5);
      FreeTcell($2);
    }
  | exp T_AND T_EVENT
    {  /* event: T_EVENT info0=exp */
      $$ = $3; SetLine($$,CellLine($1)); SetInfo0($$,$1);
      FreeTcell($2);
    }
  ;

case_part
  : T_CASE signame T_IS case_body T_END T_CASE T_SEMICOLON
    {  /* case: T_CASE info0=sig, info1=caseitem */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,$2); SetInfo1($$,$4);
      FreeTcell($3); FreeTcell($5); FreeTcell($6); FreeTcell($7);
    }
  ;

case_body
  : case_item case_body
    {
      $$ = $1; SetNext($$,$2);
    }
  | case_item
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  ;

case_item
  : T_WHEN case_cond T_PARSUBST process_body
    {  /* case: N_CASECOND info0=exp, info1=body */
      $$ = $1; SetLine($$,CellLine($1)); SetType($$,N_CASECOND); SetInfo0($$,$2); SetInfo1($$,$4);
      FreeTcell($3);
    }
  | T_WHEN T_OTHERS T_PARSUBST process_body
    {  /* case: N_CASECOND info0=exp, info1=body */
      $$ = $1; SetLine($$,CellLine($3)); SetType($$,N_CASECOND); SetInfo0($$,$2); SetInfo1($$,$4);
      FreeTcell($3);
    }
  | T_WHEN case_cond T_PARSUBST
    {  /* case: N_CASECOND info0=exp, info1=body(NULL) */
      $$ = $1; SetLine($$,CellLine($1)); SetType($$,N_CASECOND); SetInfo0($$,$2); SetInfo1($$,NULLCELL);
      FreeTcell($3);
    }
  | T_WHEN T_OTHERS T_PARSUBST
    {  /* case: N_CASECOND info0=exp, info1=body(NULL) */
      $$ = $1; SetLine($$,CellLine($3)); SetType($$,N_CASECOND); SetInfo0($$,$2); SetInfo1($$,NULLCELL);
      FreeTcell($3);
    }
  ;

case_cond
  : exp T_COND_OR case_cond
    {
      $$ = $1; SetNext($$,$3);
      FreeTcell($2);
    }
  | exp
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  ;

when_part
  : T_ID T_COLON signame T_SIGSUBST whenclause
    {  /* when: N_CONDASSIGN info0=ID(label), info1=sig, info2=body */
      $$ = $4; SetLine($$,CellLine($2)); SetType($$,N_CONDASSIGN); SetInfo0($$,$1); SetInfo1($$,$3); SetInfo2($$,$5);
      FreeTcell($2);
    }
  | signame T_SIGSUBST whenclause
    {  /* when: N_CONDASSIGN info0=ID(label,NULL), info1=sig, info2=body */
      $$ = $2; SetLine($$,CellLine($1)); SetType($$,N_CONDASSIGN); SetInfo0($$,NULLCELL); SetInfo1($$,$1); SetInfo2($$,$3);
    }
  ;

whenclause
  : exp T_WHEN exp T_ELSE whenclause
    {  /* when: N_WHENCOND info0=substexp, info1=condexp, info2=else */
      $$ = $2; SetLine($$,CellLine($1)); SetType($$,N_WHENCOND); SetInfo0($$,$1); SetInfo1($$,$3); SetInfo2($$,$5);
      FreeTcell($4);
    }
  | exp T_SEMICOLON
    {  /* when: N_WHENCOND info0=substexp, info1=condexp(NULL), info2=else(NULL) */
      $$ = $2; SetLine($$,CellLine($1)); SetType($$,N_WHENCOND); SetInfo0($$,$1); SetInfo1($$,NULLCELL); SetInfo2($$,NULLCELL);
    }
  ;

withsel_part
  : T_ID T_COLON T_WITH exp T_SELECT signame T_SIGSUBST withbody
    {  /* when: N_SELASSIGN info0=ID(label), info1=exp, info2=sig, info3=body */
      $$ = $3; SetLine($$,CellLine($3)); SetType($$,N_SELASSIGN); SetInfo0($$,$1); SetInfo1($$,$4); SetInfo2($$,$6); SetInfo3($$,$8);
      FreeTcell($2); FreeTcell($5); FreeTcell($7);
    }
  | T_WITH exp T_SELECT signame T_SIGSUBST withbody
    {  /* when: N_SELASSIGN info0=ID(label,NULL), info1=exp, info2=sig, info3=body */
      $$ = $1; SetLine($$,CellLine($1)); SetType($$,N_SELASSIGN); SetInfo0($$,NULLCELL); SetInfo1($$,$2); SetInfo2($$,$4); SetInfo3($$,$6);
      FreeTcell($3); FreeTcell($5);
    }
  ;

withbody
  : exp T_WHEN case_cond T_COMMA withbody
    {  /* when: N_SELCOND info0=substexp, info1=condexp, info2=else */
      $$ = $2; SetLine($$,CellLine($1)); SetType($$,N_SELCOND); SetInfo0($$,$1); SetInfo1($$,$3); SetInfo2($$,$5);
      FreeTcell($4);
    }
  | exp T_WHEN T_OTHERS T_SEMICOLON
    {  /* when: N_SELCOND info0=substexp, info1=condexp(NULL), info2=else(NULL) */
      $$ = $2; SetLine($$,CellLine($1)); SetType($$,N_SELCOND); SetInfo0($$,$1); SetInfo1($$,NULLCELL); SetInfo2($$,NULLCELL);
      FreeTcell($3); FreeTcell($4);
    }
  ;

portmap_part
  : T_ID T_COLON T_ID T_GENERIC T_MAP T_LPAREN portmap_body T_RPAREN T_PORT T_MAP T_LPAREN portmap_body T_RPAREN T_SEMICOLON
    {  /* portmap: N_PORTMAP info0=ID(label), info1=ID(module), info2=generic, info3=port */
      $$ = $2; SetLine($$,CellLine($6)); SetType($$,N_PORTMAP); SetInfo0($$,$1); SetInfo1($$,$3); SetInfo2($$,$7); SetInfo3($$,$12);
      FreeTcell($4); FreeTcell($5); FreeTcell($6); FreeTcell($8);
      FreeTcell($9); FreeTcell($10); FreeTcell($11); FreeTcell($13); FreeTcell($14);
    }
  | T_ID T_COLON T_ID T_PORT T_MAP T_LPAREN portmap_body T_RPAREN T_SEMICOLON
    {  /* portmap: N_PORTMAP info0=ID(label), info1=ID(module), info2=generic(NULL), info3=port */
      $$ = $2; SetLine($$,CellLine($6)); SetType($$,N_PORTMAP); SetInfo0($$,$1); SetInfo1($$,$3); SetInfo2($$,NULLCELL); SetInfo3($$,$7);
      FreeTcell($4); FreeTcell($5); FreeTcell($6); FreeTcell($8);
      FreeTcell($9);
    }
  ;

portmap_body
  : portmap_item T_COMMA portmap_body
    {
      $$ = $1; SetLine($$,CellLine($1)); SetNext($$,$3);
      FreeTcell($2);
    }
  | portmap_item
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  ;

portmap_item
  : signame T_PARSUBST exp
    {  /* portmap: N_PORTITEMNAME info0=sig, info1=exp */
      $$ = $2; SetLine($$,CellLine($1)); SetType($$,N_PORTITEMNAME); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | signame
    {  /* portmap: (PORTITEMORDER) */
      $$ = $1;
    }
  ;

block_part
  : T_ID T_COLON T_BLOCK declaration_part T_BEGIN body_part T_END T_BLOCK T_ID T_SEMICOLON
    {  /* block: T_BLOCK info0=ID(label), info1=declaration, info2=body */
      $$ = $3; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$4); SetInfo2($$,$6);
      FreeTcell($2); FreeTcell($5); FreeTcell($7);
      FreeTcell($8); FreeTcell($9); FreeTcell($10);
    }
  | T_ID T_COLON T_BLOCK declaration_part T_BEGIN body_part T_END T_BLOCK T_SEMICOLON
    {  /* block: T_BLOCK info0=ID(label), info1=declaration, info2=body */
      $$ = $3; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$4); SetInfo2($$,$6);
      FreeTcell($2); FreeTcell($5); FreeTcell($7);
      FreeTcell($8); FreeTcell($9);
    }
  | T_BLOCK declaration_part T_BEGIN body_part T_END T_BLOCK T_ID T_SEMICOLON
    {  /* block: T_BLOCK info0=ID(label,NULL), info1=declaration, info2=body */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,NULLCELL); SetInfo1($$,$2); SetInfo2($$,$4);
      FreeTcell($3); FreeTcell($5); FreeTcell($6);
      FreeTcell($7); FreeTcell($8);
    }
  | T_BLOCK declaration_part T_BEGIN body_part T_END T_BLOCK T_SEMICOLON
    {  /* block: T_BLOCK info0=ID(label,NULL), info1=declaration, info2=body */
      $$ = $1; SetLine($$,CellLine($1)); SetInfo0($$,NULLCELL); SetInfo1($$,$2); SetInfo2($$,$4);
      FreeTcell($3); FreeTcell($5); FreeTcell($6);
      FreeTcell($7);
    }
  ;

sensesig_list
  : signame T_COMMA sensesig_list
    {  /* signal list */
      $$ = $1; SetLine($$,CellLine($1)); SetNext($$,$3);
      FreeTcell($2);
    }
  | signame
    {
      $$ = $1; SetNext($$,NULLCELL);
    }
  ;

exp
  : exp T_AND exp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | exp T_OR exp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | exp T_XOR exp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | exp T_NAND exp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | exp T_NOR exp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | eqexp
    {
      $$ = $1;
    }
  ;

eqexp
  : shiftexp T_EQUAL shiftexp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | shiftexp T_NOTEQUAL shiftexp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | shiftexp T_SIGSUBST shiftexp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetType($$,N_GE); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | shiftexp T_LE shiftexp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | shiftexp T_GT shiftexp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | shiftexp T_LS shiftexp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | shiftexp
    {
      $$ = $1;
    }
  ;

shiftexp
  : shiftexp T_SLL shiftexp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | shiftexp T_SRL shiftexp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | addexp
    {
      $$ = $1;
    }
  ;

addexp
  : addexp T_PLUS addexp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | addexp T_MINUS addexp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | addexp T_CONCAT addexp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | T_MINUS T_DECDIGIT  %prec T_UMINUS
    {
      char  *newstr;
      $$ = $2; FreeTcell($1);
      newstr = (char *)malloc(sizeof (char) * (strlen(CellStr($2)) + 2));
      sprintf(newstr, "-%s", CellStr($2));
      free(CellStr($2));
      CellStr($2) = newstr;
    }
  | T_PLUS T_DECDIGIT    %prec T_UPLUS
    {
      $$ = $2; FreeTcell($1);
    }
  | multexp
    {
      $$ = $1;
    }
  ;

multexp
  : multexp T_MULT multexp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | multexp T_DIV multexp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | multexp T_MOD multexp
    {
      $$ = $2; SetLine($$,CellLine($3)); SetInfo0($$,$1); SetInfo1($$,$3);
    }
  | notexp
    {
      $$ = $1;
    }
  ;

notexp
  : T_NOT literal
    {
      $$ = $1; SetLine($$,CellLine($2)); SetInfo0($$,$2);
    }
  | literal
    {
      $$ = $1;
    }
  ;

literal
  : T_BINDIGIT
    {
      $$ = $1;
    }
  | T_HEXDIGIT
    {
      $$ = $1;
    }
  | T_DECDIGIT
    {
      $$ = $1;
    }
  | T_LPAREN T_OTHERS T_PARSUBST T_BINDIGIT T_RPAREN
    {  /* N_OTHERS info0=value */
      $$ = $2; SetLine($$,CellLine($5)); SetType($$,N_OTHERS); SetInfo0($$,$4);
      FreeTcell($1); FreeTcell($3); FreeTcell($5);
    }
  | T_LPAREN T_OTHERS T_PARSUBST signame T_RPAREN
    {  /* N_OTHERS info0=value */
      $$ = $2; SetLine($$,CellLine($5)); SetType($$,N_OTHERS); SetInfo0($$,$4);
      FreeTcell($1); FreeTcell($3); FreeTcell($5);
    }
  | T_ID T_LPAREN portmap_body T_RPAREN
    {  /* N_CALLFUNC info0=ID, info1=port */
      $$ = $2; SetLine($$,CellLine($4)); SetType($$,N_CALLFUNC); SetInfo0($$,$1); SetInfo1($$,$3);
      FreeTcell($4);
    }
  | T_CONVINT T_LPAREN exp T_RPAREN
    {  /* (conv_integer) */
      $$ = $3; SetLine($$,CellLine($4)); 
      FreeTcell($1); FreeTcell($2); FreeTcell($4);
    }
  | T_UNSIGNED T_LPAREN exp T_RPAREN
    {  /* (unsigned) */
      $$ = $3; SetLine($$,CellLine($4)); 
      FreeTcell($1); FreeTcell($2); FreeTcell($4);
    }
  | T_TOSTDLOGICVEC T_LPAREN exp T_RPAREN
    {  /* (to_stdlogicvector) */
      $$ = $3; SetLine($$,CellLine($4)); 
      FreeTcell($1); FreeTcell($2); FreeTcell($4);
    }
  | T_CONVSTDVEC T_LPAREN exp T_COMMA exp T_RPAREN
    {  /* (conv_std_logicvector) */
      $$ = $3; SetLine($$,CellLine($6)); 
      FreeTcell($1); FreeTcell($2); FreeTcell($4);
      FreeTree($5); FreeTcell($6);
    }
  | T_SIGNED T_LPAREN exp T_RPAREN
    {  /* (conv_integer) */
      $$ = $3; SetLine($$,CellLine($4)); 
      FreeTcell($1); FreeTcell($2); FreeTcell($4);
    }
  | T_ID T_LPAREN sigwidth T_RPAREN
    {  /* N_STDVECTOR info0=ID, info1=width */
      $$ = $2; SetLine($$,CellLine($4)); SetType($$,N_STDVECTOR); SetInfo0($$,$1); SetInfo1($$,$3);
      FreeTcell($4);
    }
  | T_ID T_LPAREN exp T_RPAREN
    {  /* N_STDELEMENT info0=ID, info1=exp */
      $$ = $2; SetLine($$,CellLine($4)); SetType($$,N_STDELEMENT); SetInfo0($$,$1); SetInfo1($$,$3);
      FreeTcell($4);
    }
  | T_ID
    {
      $$ = $1;
    }
  | T_LPAREN sensesig_list T_RPAREN
    {  /* N_LISTARRAY info0=list */
      $$ = $1; SetLine($$,CellLine($3)); SetType($$,N_LISTARRAY); SetInfo0($$,$2);
      FreeTcell($3);
    }
  | T_LPAREN exp T_RPAREN
    {  /* N_PAREN info0=exp */
      $$ = $1; SetLine($$,CellLine($3)); SetType($$,N_PAREN); SetInfo0($$,$2);
      FreeTcell($3);
    }
  ;

signame
  : T_ID T_LPAREN sigwidth T_RPAREN
    {  /* N_STDVECTOR info0=ID, info1=width */
      $$ = $2; SetLine($$,CellLine($4)); SetType($$,N_STDVECTOR); SetInfo0($$,$1); SetInfo1($$,$3);
      FreeTcell($4);
    }
  | T_ID T_LPAREN exp T_RPAREN
    {  /* N_STDELEMENT info0=ID, info1=exp */
      $$ = $2; SetLine($$,CellLine($4)); SetType($$,N_STDELEMENT); SetInfo0($$,$1); SetInfo1($$,$3);
      FreeTcell($4);
    }
  | T_ID
    {
      $$ = $1;
    }
  | T_LPAREN sensesig_list T_RPAREN
    {  /* N_LISTARRAY info0=list */
      $$ = $1; SetLine($$,CellLine($3)); SetType($$,N_LISTARRAY); SetInfo0($$,$2);
      FreeTcell($3);
    }
  ;

%%

void yyerror(char *s)
{
  ParseError = True;
  if (*s == '$')
    fprintf(stderr, "ERROR: in line %d, %s\n", yylexlinenum, s + 1);
  else
    fprintf(stderr, "ERROR: parse error in line %d\n", yylexlinenum);
    return;
}

// end of file

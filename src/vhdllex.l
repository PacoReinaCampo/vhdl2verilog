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

int yylexlinenum;
int yywrap(void)
{
  return (1);
}

%}
%%
all                             {yylval = MallocTcell(T_ALL, NULLSTR, yylexlinenum); return(T_ALL);}
and                             {yylval = MallocTcell(T_AND, NULLSTR, yylexlinenum); return(T_AND);}
architecture                    {yylval = MallocTcell(T_ARCHITECTURE, NULLSTR, yylexlinenum); return(T_ARCHITECTURE);}
array                           {yylval = MallocTcell(T_ARRAY, NULLSTR, yylexlinenum); return(T_ARRAY);}
begin                           {yylval = MallocTcell(T_BEGIN, NULLSTR, yylexlinenum); return(T_BEGIN);}
block                           {yylval = MallocTcell(T_BLOCK, NULLSTR, yylexlinenum); return(T_BLOCK);}
case                            {yylval = MallocTcell(T_CASE, NULLSTR, yylexlinenum); return(T_CASE);}
component                       {yylval = MallocTcell(T_COMPONENT, NULLSTR, yylexlinenum); return(T_COMPONENT);}
constant                        {yylval = MallocTcell(T_CONSTANT, NULLSTR, yylexlinenum); return(T_CONSTANT);}
conv_integer                    {yylval = MallocTcell(T_CONVINT, NULLSTR, yylexlinenum); return(T_CONVINT);}
conv_std_logic_vector           {yylval = MallocTcell(T_CONVSTDVEC, NULLSTR, yylexlinenum); return(T_CONVSTDVEC);}
downto                          {yylval = MallocTcell(T_DOWNTO, NULLSTR, yylexlinenum); return(T_DOWNTO);}
else                            {yylval = MallocTcell(T_ELSE, NULLSTR, yylexlinenum); return(T_ELSE);}
elsif                           {yylval = MallocTcell(T_ELSIF, NULLSTR, yylexlinenum); return(T_ELSIF);}
end                             {yylval = MallocTcell(T_END, NULLSTR, yylexlinenum); return(T_END);}
entity                          {yylval = MallocTcell(T_ENTITY, NULLSTR, yylexlinenum); return(T_ENTITY);}
for                             {yylval = MallocTcell(T_FOR, NULLSTR, yylexlinenum); return(T_FOR);}
function                        {yylval = MallocTcell(T_FUNCTION, NULLSTR, yylexlinenum); return(T_FUNCTION);}
generate                        {yylval = MallocTcell(T_GENERATE, NULLSTR, yylexlinenum); return(T_GENERATE);}
generic                         {yylval = MallocTcell(T_GENERIC, NULLSTR, yylexlinenum); return(T_GENERIC);}
if                              {yylval = MallocTcell(T_IF, NULLSTR, yylexlinenum); return(T_IF);}
in                              {yylval = MallocTcell(T_IN, NULLSTR, yylexlinenum); return(T_IN);}
inout                           {yylval = MallocTcell(T_INOUT, NULLSTR, yylexlinenum); return(T_INOUT);}
integer                         {yylval = MallocTcell(T_INTEGER, NULLSTR, yylexlinenum); return(T_INTEGER);}
is                              {yylval = MallocTcell(T_IS, NULLSTR, yylexlinenum); return(T_IS);}
library                         {yylval = MallocTcell(T_LIBRARY, NULLSTR, yylexlinenum); return(T_LIBRARY);}
loop                            {yylval = MallocTcell(T_LOOP, NULLSTR, yylexlinenum); return(T_LOOP);}
map                             {yylval = MallocTcell(T_MAP, NULLSTR, yylexlinenum); return(T_MAP);}
mod                             {yylval = MallocTcell(T_MOD, NULLSTR, yylexlinenum); return(T_MOD);}
nand                            {yylval = MallocTcell(T_NAND, NULLSTR, yylexlinenum); return(T_NAND);}
nor                             {yylval = MallocTcell(T_NOR, NULLSTR, yylexlinenum); return(T_NOR);}
not                             {yylval = MallocTcell(T_NOT, NULLSTR, yylexlinenum); return(T_NOT);}
null;                           { /* NO ACTION */ }
of                              {yylval = MallocTcell(T_OF, NULLSTR, yylexlinenum); return(T_OF);}
or                              {yylval = MallocTcell(T_OR, NULLSTR, yylexlinenum); return(T_OR);}
others                          {yylval = MallocTcell(T_OTHERS, NULLSTR, yylexlinenum); return(T_OTHERS);}
out                             {yylval = MallocTcell(T_OUT, NULLSTR, yylexlinenum); return(T_OUT);}
port                            {yylval = MallocTcell(T_PORT, NULLSTR, yylexlinenum); return(T_PORT);}
procedure                       {yylval = MallocTcell(T_PROCEDURE, NULLSTR, yylexlinenum); return(T_PROCEDURE);}
process                         {yylval = MallocTcell(T_PROCESS, NULLSTR, yylexlinenum); return(T_PROCESS);}
range                           {yylval = MallocTcell(T_RANGE, NULLSTR, yylexlinenum); return(T_RANGE);}
return                          {yylval = MallocTcell(T_RETURN, NULLSTR, yylexlinenum); return(T_RETURN);}
select                          {yylval = MallocTcell(T_SELECT, NULLSTR, yylexlinenum); return(T_SELECT);}
signal                          {yylval = MallocTcell(T_SIGNAL, NULLSTR, yylexlinenum); return(T_SIGNAL);}
signed                          {yylval = MallocTcell(T_SIGNED, NULLSTR, yylexlinenum); return(T_SIGNED);}
sll                             {yylval = MallocTcell(T_SLL, NULLSTR, yylexlinenum); return(T_SLL);}
srl                             {yylval = MallocTcell(T_SRL, NULLSTR, yylexlinenum); return(T_SRL);}
std_logic_vector                {yylval = MallocTcell(T_STDLOGICVEC, NULLSTR, yylexlinenum); return(T_STDLOGICVEC);}
std_logic                       {yylval = MallocTcell(T_STDLOGIC, NULLSTR, yylexlinenum); return(T_STDLOGIC);}
then                            {yylval = MallocTcell(T_THEN, NULLSTR, yylexlinenum); return(T_THEN);}
to                              {yylval = MallocTcell(T_TO, NULLSTR, yylexlinenum); return(T_TO);}
to_stdlogicvector               {yylval = MallocTcell(T_TOSTDLOGICVEC, NULLSTR, yylexlinenum); return(T_TOSTDLOGICVEC);}
type                            {yylval = MallocTcell(T_TYPE, NULLSTR, yylexlinenum); return(T_TYPE);}
unsigned                        {yylval = MallocTcell(T_UNSIGNED, NULLSTR, yylexlinenum); return(T_UNSIGNED);}
use                             {yylval = MallocTcell(T_USE, NULLSTR, yylexlinenum); return(T_USE);}
variable                        {yylval = MallocTcell(T_VARIABLE, NULLSTR, yylexlinenum); return(T_VARIABLE);}
when                            {yylval = MallocTcell(T_WHEN, NULLSTR, yylexlinenum); return(T_WHEN);}
with                            {yylval = MallocTcell(T_WITH, NULLSTR, yylexlinenum); return(T_WITH);}
xor                             {yylval = MallocTcell(T_XOR, NULLSTR, yylexlinenum); return(T_XOR);}
--.*[\n]+                       { RegisterComment(CommentListTop, yytext, &yylexlinenum); }
\n[\n]+                         { RegisterComment(CommentListTop, yytext, &yylexlinenum); }
\n                              { yylexlinenum++; }
[0-9]+                          {yylval = MallocTcell(T_DECDIGIT, yytext, yylexlinenum); return(T_DECDIGIT);}
X['|\"][0-9A-Fa-fXxZz]+['|\"]   {yylval = MallocTcell(T_HEXDIGIT, yytext, yylexlinenum); return(T_HEXDIGIT);}
['|\"][0-1XxZz]+['|\"]          {yylval = MallocTcell(T_BINDIGIT, yytext, yylexlinenum); return(T_BINDIGIT);}
[A-Za-z_.][A-Za-z0-9_.]*'event  {yylval = MallocTcell(T_EVENT, yytext, yylexlinenum); return(T_EVENT);}
[A-Za-z_.][A-Za-z0-9_.]*        {yylval = MallocTcell(T_ID, yytext, yylexlinenum); return(T_ID);}
"("                             {yylval = MallocTcell(T_LPAREN, NULLSTR, yylexlinenum); return(T_LPAREN);}
")"                             {yylval = MallocTcell(T_RPAREN, NULLSTR, yylexlinenum); return(T_RPAREN);}
"<="                            {yylval = MallocTcell(T_SIGSUBST, NULLSTR, yylexlinenum); return(T_SIGSUBST);}
":="                            {yylval = MallocTcell(T_VARSUBST, NULLSTR, yylexlinenum); return(T_VARSUBST);}
"=>"                            {yylval = MallocTcell(T_PARSUBST, NULLSTR, yylexlinenum); return(T_PARSUBST);}
">="                            {yylval = MallocTcell(T_LE, NULLSTR, yylexlinenum); return(T_LE);}
"<"                             {yylval = MallocTcell(T_GT, NULLSTR, yylexlinenum); return(T_GT);}
">"                             {yylval = MallocTcell(T_LS, NULLSTR, yylexlinenum); return(T_LS);}
"/="                            {yylval = MallocTcell(T_NOTEQUAL, NULLSTR, yylexlinenum); return(T_NOTEQUAL);}
"+"                             {yylval = MallocTcell(T_PLUS, NULLSTR, yylexlinenum); return(T_PLUS);}
-                               {yylval = MallocTcell(T_MINUS, NULLSTR, yylexlinenum); return(T_MINUS);}
"/"                             {yylval = MallocTcell(T_DIV, NULLSTR, yylexlinenum); return(T_DIV);}
"*"                             {yylval = MallocTcell(T_MULT, NULLSTR, yylexlinenum); return(T_MULT);}
"|"                             {yylval = MallocTcell(T_COND_OR, NULLSTR, yylexlinenum); return(T_COND_OR);}
=                               {yylval = MallocTcell(T_EQUAL, NULLSTR, yylexlinenum); return(T_EQUAL);}
;                               {yylval = MallocTcell(T_SEMICOLON, NULLSTR, yylexlinenum); return(T_SEMICOLON);}
:                               {yylval = MallocTcell(T_COLON, NULLSTR, yylexlinenum); return(T_COLON);}
,                               {yylval = MallocTcell(T_COMMA, NULLSTR, yylexlinenum); return(T_COMMA);}
&                               {yylval = MallocTcell(T_CONCAT, NULLSTR, yylexlinenum); return(T_CONCAT);}
%%

// end of file

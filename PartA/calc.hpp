#ifndef CALC_HPP_
#define CALC_HPP_

#include <vector>
#include <string>
#include <iostream>

namespace Calc {
// The first step of the calculator parses an expression given as a string
// into a vector of tokens.  A token holds the atoms of essential information
// we need to evaluate expression, which is the type and value.
// type is either '+', '-', '*', '/' for the operators or 'n' to indicate a number.
// In the case of a number, val holds its integer value.  For operators
// val can remain 0.
struct Token {
  char type {};
  int val {};
};


std::vector<Token> tokenise(const std::string&);
std::vector<Token> infixToPostfix(const std::vector<Token>&);
bool isTypeOperator(char type);
int getPriority(char type);
int evalPostfix(const std::vector<Token>&);
int eval(const std::string&);

bool operator==(const Calc::Token&, const Calc::Token&);
std::ostream& operator<<(std::ostream&, const Calc::Token&);

}     // namespace Calc


#endif    // CALC_HPP_

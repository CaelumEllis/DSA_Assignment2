#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include "calc.hpp"
#include <stack>

std::ostream& Calc::operator<<(std::ostream& out, const Calc::Token& t) {
  out << "{'" << t.type << "', " << t.val << '}';
  return out;
}

bool Calc::operator==(const Calc::Token& a, const Calc::Token& b) {
  if (a.type != b.type) {
    return false;
  }
  if (a.type == 'n') {
    return a.val == b.val;
  }
  return true;
}

//created a function to check if the type is an operator, as the is required several times
bool Calc::isTypeOperator(const char type) {
  return type == '+' || type == '-' || type == '*' || type == '/';
}
//to return priority for operators/numbers
int Calc::getPriority(char type) {
  if (type == '+' || type == '-') {
    return 1;
  }
  if (type == '*' || type == '/') {
    return 2;
  }
  //non-operators have priority 0
  return 0;
}
// This is the function for you to write
std::vector<Calc::Token> Calc::infixToPostfix(const std::vector<Token>& input) {
  size_t n = input.size();
  std::vector<Token> output;
  std::stack<Token> operatorStack;
 
  // a lot of the use of a basic calculater is likely to fall into the n==3 case
  if (n == 3) {
    output.push_back(input[0]);
    output.push_back(input[2]);
    output.push_back(input[1]);
    return output;
  }

  for (const Token& token : input) {
    const char type = token.type;
    // if int, push to output
    if (type == 'n') {
      output.push_back(token);
      // finish current loop iteration
      continue;
    }
    // if open parenthesis, push to stack
    if (type == '(') {
      operatorStack.push(token);
      // finish current loop iteration
      continue;
    }
    // if closing parenthesis, pop tokens to stack until open parenthesis
    if (type == ')') {
      while (!operatorStack.empty() && operatorStack.top().type != '(') {
        output.push_back(operatorStack.top());
        operatorStack.pop();
      }
      if (!operatorStack.empty() && operatorStack.top().type == '(') {
        // Discard the open parenthesis
        operatorStack.pop();
      }
      // finish current loop iteration
      continue;
    }

    // if operator, pop stack until there is an operator with greater or equal priority
    if (isTypeOperator(type)) {
      int currentPriority = getPriority(type);

      while (!operatorStack.empty()) {
        const char topType = operatorStack.top().type;
        // check if the top of the stack is also an operator, if not, break
        if (!isTypeOperator(topType)) {
          break;
        }
        int topPriority = getPriority(topType);
        // Pop while top of stack has higher or equal precedence
        if (topPriority >= currentPriority) {
          output.push_back(operatorStack.top());
          operatorStack.pop();
        } else {
            break;
        }
      }
      operatorStack.push(token);
    }
  }
  // push remaining stack into output
  while (!operatorStack.empty()) {
    output.push_back(operatorStack.top());
    operatorStack.pop();
  }
  return output;
}


// evalPostfix evaluates a vector of tokens in postfix notation
// This function was done in tutorial Week 10
int Calc::evalPostfix(const std::vector<Token>& tokens) {
  if (tokens.empty()) {
    return 0;
  }
  std::vector<int> stack;
  for (Token t : tokens) {
    if (t.type == 'n') {
      stack.push_back(t.val);
    } else {
      int val = 0;
      if (t.type == '+') {
        val = stack.back() + *(stack.end()-2);
      } else if (t.type == '*') {
        val = stack.back() * *(stack.end()-2);
      } else if (t.type == '-') {
        val = *(stack.end()-2) - stack.back();
      } else if (t.type == '/') {
        if (stack.back() == 0) {
          throw std::runtime_error("divide by zero");
        }
        val = *(stack.end()-2) / stack.back();
      } else {
          std::cout << "invalid token\n";
      }
      stack.pop_back();
      stack.pop_back();
      stack.push_back(val);
    }
  }
  return stack.back();
}

// tokenise takes a string and parses it into a vector of tokens
std::vector<Calc::Token> Calc::tokenise(const std::string& expression) {
  const std::vector<char> symbols = {'+', '-', '*', '/', '(', ')'};
  std::vector<Token> tokens {};
  for (std::size_t i =0; i < expression.size(); ++i) {
    const char c = expression[i];
    // check if c is one of '+', '-', '*', '/', '(', ')'
    if (std::find(symbols.begin(), symbols.end(), c) != symbols.end()) {
      tokens.push_back({c});
    } else if (isdigit(c)) {
      // process multiple digit integers
      std::string num {c};
      while (i + 1 < expression.size() && isdigit(expression[i + 1])) {
        ++i;
        num.push_back(expression[i]);
      }
      tokens.push_back({'n', std::stoi(num)});
    }
  }
  return tokens;
}

// eval puts the pieces together to take a string with an
// arithmetic expression and output its evaluation
int Calc::eval(const std::string& expression) {
  std::vector<Token> tokens = tokenise(expression);
  std::vector<Token> postfix = infixToPostfix(tokens);
  return evalPostfix(postfix);
}

#pragma once
#include <string>
#include <vector>
#include "FloodFillMap.hpp"

using namespace std;

vector<char> store_row_string(string s);
string cell2String (Cell c);
int numTurnsInPath(Path p, Direction initial);
string generateMotionPlan(Path p, Cell initial_cell, Direction initial_dir);
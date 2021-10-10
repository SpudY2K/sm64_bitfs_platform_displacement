#include "Mario.hpp"
#include "Solution.hpp"
#include "vmath.hpp"

bool validate_solution(Solution* s, SearchParams* param, Vec3f &final_pos, bool pancake);
int validate_solution_sim(Solution* s, Mario* m);
int validate_solution_pancake(Solution* s);
#include "Tuner.h"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <limits>

using namespace std;

#define PRINT_INDENT 19

Tuner::Tuner(vector<double> params, vector<double> params_delta, unsigned int max_steps) {
  this->params = params;
  this->max_steps = max_steps;
  this->best_err = numeric_limits<double>::max();
  this->best_params = params;
  this->total_err = 0.0;
  this->step = 0;
  this->cycle = 1;
  this->param_idx = 0;
  this->params_delta.resize(params.size());
  for (unsigned int i = 0; i < params_delta.size(); ++i) {
    this->params_delta[i] = {params_delta[i], true};
  }
  this->warmup_steps = 600;
  this->cte_tolerance = 4.0;
  this->delta_tolerance = 0.1;
}

Tuner::~Tuner() {}

bool Tuner::Enabled() { return max_steps > 0; }

bool Tuner::IsResetCycle() { return step == 0; }

bool Tuner::IsTuned() {
  double sum = accumulate(params_delta.begin(), params_delta.end(), 0.0,
                          [](double sum, const ParamDelta& param) { return sum + param.value; });
  return sum <= delta_tolerance;
}

vector<double> Tuner::BestParams() { return best_params; }

vector<double> Tuner::Tune(double cte) {
  if (IsTuned()) {
    cout << "Tuning finished, best error: " << best_err << endl;
    PrintBestParams();
    return best_params;
  }

  if (step == warmup_steps) {
    cout << "Cycle " << cycle << " Warmup Completed" << endl;
  }

  // Let the simulation sink in for a while
  if (step >= warmup_steps) {
    total_err += cte * cte;
  }

  double cte_abs = fabs(cte);

  // End of collection cycle
  if (step++ == (max_steps + warmup_steps) || cte_abs > cte_tolerance) {
    // Computes the average error
    double err_avg;

    if (cte_abs > cte_tolerance) {
      err_avg = cte_abs;
    } else {
      err_avg = total_err / (step - warmup_steps);
    }

    cout << endl << "End of Cycle " << cycle << endl;
    cout << "----------------------------------------------" << endl;
    PrintParams();
    PrintParamsDelta();
    cout << setw(PRINT_INDENT) << "Cycle Error: " << err_avg << endl;
    cout << setw(PRINT_INDENT) << "Previous Best: " << best_err << endl;
    cout << setw(PRINT_INDENT) << "Current index: " << param_idx << endl;
    cout << setw(PRINT_INDENT) << "Error delta: " << (err_avg - best_err) << endl;

    if (err_avg < best_err) {
      // Error improved
      best_err = err_avg;
      best_params = params;
      TuneUp();                                  // Tune up this parameter
      params_delta[param_idx].increment = true;  // Reset increment status
      NextParam();                               // Tune next param
    } else if (params_delta[param_idx].increment) {
      params_delta[param_idx].increment = false;  // Try decrementing
    } else {
      Increase();                                // Puts back to original value
      TuneDown();                                // Tune down this parameter
      params_delta[param_idx].increment = true;  // Try incrementing
      NextParam();                               // Tune next param
    }

    if (params_delta[param_idx].increment) {
      Increase();
    } else {
      Decrease();
    }

    cout << setw(PRINT_INDENT) << "Current Best: " << best_err << endl;
    PrintBestParams();
    cout << "----------------------------------------------" << endl << endl;
    // Clear the cycle
    ResetCycle();
    ++cycle;
  }

  return params;
}

void Tuner::NextParam() { ++param_idx %= params.size(); }

void Tuner::TuneUp() {
  cout << setw(PRINT_INDENT - 3) << "TUNING p_delta_" << param_idx << ": (" << params_delta[param_idx].value
       << " * 1.1)" << endl;
  params_delta[param_idx].value *= 1.1;
}

void Tuner::TuneDown() {
  cout << setw(PRINT_INDENT - 3) << "TUNING p_delta_" << param_idx << ": (" << params_delta[param_idx].value
       << " * 0.9)" << endl;
  params_delta[param_idx].value *= 0.9;
}

void Tuner::Increase() {
  double delta = params_delta[param_idx].value;
  cout << setw(PRINT_INDENT - 3) << "INCREASING p_" << param_idx << ": (" << params[param_idx] << " + " << delta << ")"
       << endl;
  params[param_idx] += delta;
}

void Tuner::Decrease() {
  double delta = 2 * params_delta[param_idx].value;
  cout << setw(PRINT_INDENT - 3) << "DECREASING p_" << param_idx << ": (" << params[param_idx] << " - " << delta << ")"
       << endl;
  params[param_idx] -= delta;
}

void Tuner::ResetCycle() {
  step = 0;
  total_err = 0.0;
}

void Tuner::PrintParams() {
  cout << setw(PRINT_INDENT) << "Params: ";
  for_each(params.begin(), params.end(), [](double param) { cout << param << " "; });
  cout << endl;
}

void Tuner::PrintBestParams() {
  cout << setw(PRINT_INDENT) << "Best params: ";
  for_each(best_params.begin(), best_params.end(), [](double param) { cout << param << " "; });
  cout << endl;
}

void Tuner::PrintParamsDelta() {
  cout << setw(PRINT_INDENT) << "Params delta: ";
  for_each(params_delta.begin(), params_delta.end(), [](ParamDelta delta) { cout << delta.value << " "; });
  cout << endl;
}
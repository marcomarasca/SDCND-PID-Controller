#ifndef TUNER_H
#define TUNER_H

#include <vector>

struct ParamDelta {
  double value;
  bool increment;
};

class Tuner {
 public:
  Tuner(std::vector<double> params, std::vector<double> params_delta, unsigned int max_steps);

  virtual ~Tuner();

  std::vector<double> Tune(double cte);

  std::vector<double> BestParams();

  bool IsResetCycle();

  bool IsTuned();

  bool Enabled();

  void PrintParams();
  void PrintBestParams();
  void PrintParamsDelta();

 private:
  std::vector<double> params;
  std::vector<double> best_params;
  std::vector<ParamDelta> params_delta;

  unsigned int cycle;
  unsigned int step;
  unsigned int max_steps;
  unsigned int warmup_steps;
  unsigned int param_idx;

  double best_err;
  double total_err;
  double cte_tolerance;
  double delta_tolerance;
  
  void NextParam();
  void TuneUp();
  void TuneDown();
  void Increase();
  void Decrease();
  void ResetCycle();
};

#endif /* TUNER_H */

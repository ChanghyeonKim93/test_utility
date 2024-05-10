#include "optimizer/summary_reporter.h"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "utility/string_modifier.h"
#include "utility/time_checker.h"

namespace optimizer {

SummaryReporter::SummaryReporter()
    : is_overall_summary_set_(false), is_iteration_summary_set_(false) {}

SummaryReporter::~SummaryReporter() {}

const std::string SummaryReporter::BriefReport() const {
  const std::streamsize default_precision{std::cout.precision()};
  std::stringstream ss;
  ss << "Iteration Report:\n";
  ss << " itr";            // 5
  ss << "  total_cost  ";  // 14
  ss << " avg.reproj. ";   // 13
  ss << " cost_change ";   // 13
  ss << " |step|  ";       // 10
  ss << " |gradient| ";    // 12
  ss << " damp_term ";     // 12
  ss << " itr_time[ms] ";  // 11
  ss << "itr_stat\n";
  const size_t num_iterations = iteration_summary_list_.size();
  for (size_t iteration = 0; iteration < num_iterations; ++iteration) {
    const IterationSummary& iteration_summary =
        iteration_summary_list_[iteration];

    ss << std::setw(3) << iteration << " ";
    ss << " " << std::scientific << iteration_summary.cost;
    ss << "    " << std::setprecision(2) << std::scientific
       << iteration_summary.cost / static_cast<double>(1.0);
    ss << "    " << std::setprecision(2) << std::scientific
       << iteration_summary.cost_change;
    ss << "   " << std::setprecision(2) << std::scientific
       << iteration_summary.step_norm;
    ss << "   " << std::setprecision(2) << std::scientific
       << iteration_summary.gradient_norm;
    ss << "   " << std::setprecision(2) << std::scientific
       << iteration_summary.trust_region_radius;
    ss << "   " << std::setprecision(2) << std::scientific
       << iteration_summary.iteration_time_in_seconds;

    switch (iteration_summary.iteration_status) {
      case IterationStatus::UPDATE:
        ss << "     " << "UPDATE";
        break;
      case IterationStatus::SKIPPED:
        ss << "     " << utility::StringModifier::MakeYellow(" SKIP ");
        break;
      case IterationStatus::UPDATE_TRUST_MORE:
        ss << "     " << utility::StringModifier::MakeGreen("UPDATE");
        break;
      default:
        ss << "     ";
    }
    ss << "\n";
    ss << std::setprecision(default_precision);  // restore defaults
  }

  ss << std::setprecision(5);
  ss << "Solver Report:\n";
  ss << "  Iterations      : " << num_iterations << "\n";
  ss << "  Total time      : "
     << iteration_summary_list_.back().cumulative_time_in_seconds << " [sec]\n";
  ss << "  Initial cost    : " << iteration_summary_list_.front().cost << "\n";
  ss << "  Final cost      : " << iteration_summary_list_.back().cost << "\n";
  ss << "  Termination     : "
     << (overall_summary_.is_converged
             ? utility::StringModifier::MakeGreen("CONVERGENCE")
             : utility::StringModifier::MakeYellow("NO_CONVERGENCE"))
     << "\n";
  if (overall_summary_.max_num_iterations == static_cast<int>(num_iterations)) {
    ss << utility::StringModifier::MakeYellow(
        " WARNIING: MAX ITERATION is reached ! The solution could be local "
        "minima.\n");
  }
  ss << std::setprecision(default_precision);  // restore defaults
  return ss.str();
}

double SummaryReporter::GetTotalTimeInSeconds() const {
  if (is_iteration_summary_set_)
    throw std::runtime_error("iteration_summary is not set.");
  const double total_time_in_seconds =
      iteration_summary_list_.back().cumulative_time_in_seconds;
  return total_time_in_seconds;
}

void SummaryReporter::SetIterationSummary(
    const IterationSummary& iteration_summary) {
  iteration_summary_list_.push_back(iteration_summary);
  is_iteration_summary_set_ = true;
}

void SummaryReporter::SetOverallSummary(const OverallSummary& overall_summary) {
  overall_summary_ = overall_summary;
  is_overall_summary_set_ = true;
}

}  // namespace optimizer

#include <vector>

#include "BowVector.h"
#include "DBoW2.h"
#include "QueryResults.h"

using bow_feature = std::vector<cv::Mat>;

class Bow
{
 private:
  std::vector<bow_feature> bow_features;
  // OrbVocabulary vocabulary(9, 3, DBoW2::TF_IDF, DBoW2::L1_NORM);
  OrbVocabulary vocabulary;
  // OrbDatabase database(vocabulary, false, 0);
  OrbDatabase database;

 public:
  Bow();
  ~Bow();
  void add_feature(bow_feature feature);
};
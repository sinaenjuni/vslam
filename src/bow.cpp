#include "bow.h"

Bow::Bow()
{
  vocabulary = OrbVocabulary(9, 3, DBoW2::TF_IDF, DBoW2::L1_NORM);
  database = OrbDatabase(vocabulary, false, 0);
}
Bow::~Bow() {}
void Bow::add_feature(bow_feature feature) {
  

}
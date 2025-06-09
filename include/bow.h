#include <DBoW2.h>

/*
%YAML:1.0
  ---
  vocabulary:
    k: 10
    L: 5
    scoringType: 0
    weightingType: 0
    nodes:
        - { nodeId:1, parentId:0, weight:0.,
            descriptor:"252 176 189 114 136 109 65 218 226 243 165 24 19 245
72 59 9 150 108 24 59 103 250 16 217 203 117 124 225 193 70 32 " }
        - { nodeId:2, parentId:0, weight:0.,
            descriptor:"45 65 96 71 84 30 231 56 101 224 79 68 209 80 159 164
80 76 18 238 205 176 125 12 38 44 165 4 111 191 88 132 " }
        - { nodeId:3, parentId:0, weight:0.,
            descriptor:"223 87 111 229 50 30 127 185 53 234 95 228 223 103 155
247 54 237 253 238 237 212 173 206 246 251 239 132 46 247 41 117 " }
        - { nodeId:4, parentId:0, weight:0.,
        ...
*/

/*
10 6 0 0
0 0 2 0 16 137 81 222 176 3 60 5 66 1 172 30 6 196 128 27 131 17 22 152 17 197
    5 52 26 67 0 4 13 206  0 0 0 134 215 28 133 35 90 190 168 44 236 83 165
204 14 143 193 182 85 169 227 228 144 45 204 166 114 79 135 4 55 141 87  0 0 0
6 85 16 133 81 94 182 40 44 164 75 1 204 30 15 196 144 93 131 99 68 144 89 204
37 52 11 7 0 55 13 198  0 0 0 51 123 98 12 118 147 238 116 69 72 120 230 33
204 183 180 126 168 22 239 205 186 36 255 54 12 160 128 127 158 122 221  0
*/

class OrbVocabularyForTxtFile : public OrbVocabulary
{
 public:
  OrbVocabularyForTxtFile(
      int k = 10,
      int L = 5,
      DBoW2::WeightingType weighting = DBoW2::TF_IDF,
      DBoW2::ScoringType scoring = DBoW2::L1_NORM)
      : OrbVocabulary(k, L, weighting, scoring)
  {
  }
  OrbVocabularyForTxtFile(const std::string filename)
  {
    loadFromTextFile(filename);
  }
  bool saveToTextFile(const std::string& filename);
  bool loadFromTextFile(const std::string& filename);
};
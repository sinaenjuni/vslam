#include <regex>

#include "BowVector.h"
#include "QueryResults.h"
#include "bow.h"
#include "extractor.h"
#include "misc.h"

namespace fs = std::filesystem;
bool filenameCompare(std::string a, std::string b)
{
  int numberA =
      std::stoi(fs::path(a).filename().replace_extension("").string());
  int numberB =
      std::stoi(fs::path(b).filename().replace_extension("").string());
  return a < b;
  // if this function returns false, exchage the position of a and b.
}

int main(void)
{
  OrbVocabularyForTxtFile voc("ORBvoc.txt");
  PRINT(voc);
  OrbDatabase db(voc, false, 0);
  PRINT(db);
  PRINT("Success load");

  std::string path = "/vslam/data/KITTI/sequences/00/image_0/";
  std::regex pattern(R"(^[0-9]{6}\.png$)");
  std::vector<std::string> files;
  for (const auto& fileIter : fs::directory_iterator(path))
  {
    if (!fs::is_regular_file(fileIter.path()))
    {
      continue;
    }
    std::string filename = fileIter.path().filename().string();
    if (std::regex_match(filename, pattern))
    {
      files.push_back(fileIter.path().string());
    }
  }

  std::sort(files.begin(), files.end(), filenameCompare);

  int nFrames = files.size();
  int width = cv::imread(files[0]).cols;
  int height = cv::imread(files[0]).rows;
  int nPoints = 2000;
  int nLevels = 7;
  double scaleFactor = 1.2;

  FastOrbExtractor extractor =
      FastOrbExtractor(nPoints, nLevels, width, height, scaleFactor, 20);
  std::vector<std::vector<cv::Mat>> features;

  for (size_t i = 0; i < files.size(); ++i)
  {
    cv::Mat grayFrame = cv::imread(files[i]);
    cv::Mat colorFrame;
    if (grayFrame.channels() != 1)
    {
      cv::cvtColor(grayFrame, grayFrame, cv::COLOR_BGR2GRAY);
    }
    if (grayFrame.channels() == 1)
    {
      cv::cvtColor(grayFrame, colorFrame, cv::COLOR_GRAY2BGR);
    }

    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;
    // extractor.detect(frame, kps);
    extractor.detectAndCompute(grayFrame, kps, desc);

    std::vector<cv::Mat> feature(desc.rows, cv::Mat());

    for (size_t i = 0; i < desc.rows; ++i)
    {
      feature[i] = desc.row(i);
    }
    // features.push_back(feature);
    db.add(feature);

    DBoW2::QueryResults ret;
    db.query(feature, ret, 4);

    PRINT(ret);

    DBoW2::BowVector bow;
    voc.transform(feature, bow);
    PRINT(bow);
    break;

    // Misc::draw_kps(colorFrame, kps);
    // cv::imshow("frame", colorFrame);
    // int key = cv::waitKey();
    // if (key == 27)
    // {
    // break;
    // }
  }

  return 1;
}
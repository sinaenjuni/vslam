#include <algorithm>
#include <cstddef>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <regex>
#include <string>

#include "BowVector.h"
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

int main()
{
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
    features.push_back(feature);

    // Misc::draw_kps(colorFrame, kps);
    // cv::imshow("frame", colorFrame);
    // int key = cv::waitKey();
    // if (key == 27)
    // {
    // break;
    // }
  }
  // cv::destroyAllWindows();

  // OrbVocabulary voc(10, 6, DBoW2::TF_IDF, DBoW2::L1_NORM);
  OrbVocabularyForTxtFile voc(10, 6, DBoW2::TF_IDF, DBoW2::L1_NORM);
  voc.create(features);

  std::cout << "... done!" << std::endl;

  std::cout << "Vocabulary information: " << std::endl
            << voc << std::endl
            << std::endl;

  // lets do something with this vocabulary
  // std::cout << "Matching images against themselves (0 low, 1 high): "
  //           << std::endl;
  // DBoW2::BowVector v1, v2;
  // for (int i = 0; i < features.size(); i++)
  // {
  //   voc.transform(features[i], v1);
  //   for (int j = 0; j < features.size(); j++)
  //   {
  //     voc.transform(features[j], v2);

  //     double score = voc.score(v1, v2);
  //     std::cout << "Image " << i << " vs Image " << j << ": " << score
  //               << std::endl;
  //   }
  // }

  // save the vocabulary to disk
  std::cout << std::endl << "Saving vocabulary..." << std::endl;
  // voc.save("small_voc.yml.gz");
  // voc.save("small_voc.bin");
  voc.saveToTextFile("small_voc.txt");
  std::cout << "Done" << std::endl;

  // OrbVocabulary voc2("ORBvoc.txt");
  // PRINT(voc2);
  // and the entries added
  //  std::cout << "Saving database..." << std::endl;
  //  db.save("small_db.yml.gz");
  //  std::cout << "... done!" << std::endl;

  // once saved, we can load it again
  //  std::cout << "Retrieving database once again..." << std::endl;
  //  OrbDatabase db2("small_db.yml.gz");
  //  std::cout << "... done! This is: " << std::endl << db2 << std::endl;

  return 0;
}
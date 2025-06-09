#include "bow.h"

#include <string>

#include "misc.h"

bool OrbVocabularyForTxtFile::saveToTextFile(const std::string& filename)
{
  std::ofstream file(filename, std::ios_base::out);
  if (!file.is_open())
  {
    PRINT("Error: Failed to open file: ", filename);
    return false;
  }

  file << m_k << ' ' << m_L << ' ' << m_scoring << ' ' << m_weighting << '\n';

  for (size_t i = 1; i < m_nodes.size(); ++i)
  {
    const Node& node = m_nodes[i];

    file << node.parent << ' ';
    file << (node.isLeaf() ? 1 : 0) << ' ';
    file << DBoW2::FORB::toString(node.descriptor) << ' ';
    file << static_cast<double>(node.weight) << '\n';
  }

  file.close();
  return true;
}

bool OrbVocabularyForTxtFile::loadFromTextFile(const std::string& filename)
{
  std::ifstream file(filename, std::ios_base::in);

  if (file.eof())
  {
    return false;
  }

  m_words.clear();
  m_nodes.clear();

  std::string str;
  getline(file, str);
  std::stringstream ss;
  ss << str;
  ss >> m_k;
  ss >> m_L;
  int n1, n2;
  ss >> n1;
  ss >> n2;

  if (m_k < 0 || m_k > 20 || m_L < 1 || m_L > 10 || n1 < 0 || n1 > 5 ||
      n2 < 0 || n2 > 3)
  {
    std::cerr << "Vocabulary loading failure: This is not a correct text file!"
              << std::endl;
    return false;
  }

  m_scoring = static_cast<DBoW2::ScoringType>(n1);
  m_weighting = static_cast<DBoW2::WeightingType>(n2);
  createScoringObject();

  // nodes
  int expected_nodes =
      (int)((pow((double)m_k, (double)m_L + 1) - 1) / (m_k - 1));
  m_nodes.reserve(expected_nodes);

  m_words.reserve(pow((double)m_k, (double)m_L + 1));

  m_nodes.resize(1);
  m_nodes[0].id = 0;
  while (!file.eof())
  {
    std::string snode;
    getline(file, snode);
    std::stringstream ssnode;
    ssnode << snode;

    int nid = m_nodes.size();
    m_nodes.resize(m_nodes.size() + 1);
    m_nodes[nid].id = nid;

    int pid;
    ssnode >> pid;
    m_nodes[nid].parent = pid;
    m_nodes[pid].children.push_back(nid);

    int nIsLeaf;
    ssnode >> nIsLeaf;

    std::stringstream ssd;
    for (int iD = 0; iD < DBoW2::FORB::L; iD++)
    {
      std::string sElement;
      ssnode >> sElement;
      ssd << sElement << " ";
    }
    DBoW2::FORB::fromString(m_nodes[nid].descriptor, ssd.str());

    ssnode >> m_nodes[nid].weight;

    if (nIsLeaf > 0)
    {
      int wid = m_words.size();
      m_words.resize(wid + 1);

      m_nodes[nid].word_id = wid;
      m_words[wid] = &m_nodes[nid];
    }
    else
    {
      m_nodes[nid].children.reserve(m_k);
    }
  }

  return true;
}

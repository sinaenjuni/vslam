#include "extractor.h"

#include <opencv2/core/hal/interface.h>

#include <cmath>
#include <iterator>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/fast_math.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <vector>

#include "misc.h"
#include "quadtree.h"

constexpr int PATCH_SIZE = 31;
constexpr int HALF_PATCH_SIZE = 15;
constexpr int BORDER_SIZE = 19;
constexpr float PIinv = static_cast<float>(CV_PI / 180.0f);

// https://github.com/opencv/opencv/blob/c248d4711078dbf3dfa7b1bb4f0ca9a799ec555c/modules/features2d/src/orb.cpp#L380
static int bit_pattern_31_[256 * 4] = {
    8,   -3,  9,   5 /*mean (0), correlation (0)*/,
    4,   2,   7,   -12 /*mean (1.12461e-05), correlation (0.0437584)*/,
    -11, 9,   -8,  2 /*mean (3.37382e-05), correlation (0.0617409)*/,
    7,   -12, 12,  -13 /*mean (5.62303e-05), correlation (0.0636977)*/,
    2,   -13, 2,   12 /*mean (0.000134953), correlation (0.085099)*/,
    1,   -7,  1,   6 /*mean (0.000528565), correlation (0.0857175)*/,
    -2,  -10, -2,  -4 /*mean (0.0188821), correlation (0.0985774)*/,
    -13, -13, -11, -8 /*mean (0.0363135), correlation (0.0899616)*/,
    -13, -3,  -12, -9 /*mean (0.121806), correlation (0.099849)*/,
    10,  4,   11,  9 /*mean (0.122065), correlation (0.093285)*/,
    -13, -8,  -8,  -9 /*mean (0.162787), correlation (0.0942748)*/,
    -11, 7,   -9,  12 /*mean (0.21561), correlation (0.0974438)*/,
    7,   7,   12,  6 /*mean (0.160583), correlation (0.130064)*/,
    -4,  -5,  -3,  0 /*mean (0.228171), correlation (0.132998)*/,
    -13, 2,   -12, -3 /*mean (0.00997526), correlation (0.145926)*/,
    -9,  0,   -7,  5 /*mean (0.198234), correlation (0.143636)*/,
    12,  -6,  12,  -1 /*mean (0.0676226), correlation (0.16689)*/,
    -3,  6,   -2,  12 /*mean (0.166847), correlation (0.171682)*/,
    -6,  -13, -4,  -8 /*mean (0.101215), correlation (0.179716)*/,
    11,  -13, 12,  -8 /*mean (0.200641), correlation (0.192279)*/,
    4,   7,   5,   1 /*mean (0.205106), correlation (0.186848)*/,
    5,   -3,  10,  -3 /*mean (0.234908), correlation (0.192319)*/,
    3,   -7,  6,   12 /*mean (0.0709964), correlation (0.210872)*/,
    -8,  -7,  -6,  -2 /*mean (0.0939834), correlation (0.212589)*/,
    -2,  11,  -1,  -10 /*mean (0.127778), correlation (0.20866)*/,
    -13, 12,  -8,  10 /*mean (0.14783), correlation (0.206356)*/,
    -7,  3,   -5,  -3 /*mean (0.182141), correlation (0.198942)*/,
    -4,  2,   -3,  7 /*mean (0.188237), correlation (0.21384)*/,
    -10, -12, -6,  11 /*mean (0.14865), correlation (0.23571)*/,
    5,   -12, 6,   -7 /*mean (0.222312), correlation (0.23324)*/,
    5,   -6,  7,   -1 /*mean (0.229082), correlation (0.23389)*/,
    1,   0,   4,   -5 /*mean (0.241577), correlation (0.215286)*/,
    9,   11,  11,  -13 /*mean (0.00338507), correlation (0.251373)*/,
    4,   7,   4,   12 /*mean (0.131005), correlation (0.257622)*/,
    2,   -1,  4,   4 /*mean (0.152755), correlation (0.255205)*/,
    -4,  -12, -2,  7 /*mean (0.182771), correlation (0.244867)*/,
    -8,  -5,  -7,  -10 /*mean (0.186898), correlation (0.23901)*/,
    4,   11,  9,   12 /*mean (0.226226), correlation (0.258255)*/,
    0,   -8,  1,   -13 /*mean (0.0897886), correlation (0.274827)*/,
    -13, -2,  -8,  2 /*mean (0.148774), correlation (0.28065)*/,
    -3,  -2,  -2,  3 /*mean (0.153048), correlation (0.283063)*/,
    -6,  9,   -4,  -9 /*mean (0.169523), correlation (0.278248)*/,
    8,   12,  10,  7 /*mean (0.225337), correlation (0.282851)*/,
    0,   9,   1,   3 /*mean (0.226687), correlation (0.278734)*/,
    7,   -5,  11,  -10 /*mean (0.00693882), correlation (0.305161)*/,
    -13, -6,  -11, 0 /*mean (0.0227283), correlation (0.300181)*/,
    10,  7,   12,  1 /*mean (0.125517), correlation (0.31089)*/,
    -6,  -3,  -6,  12 /*mean (0.131748), correlation (0.312779)*/,
    10,  -9,  12,  -4 /*mean (0.144827), correlation (0.292797)*/,
    -13, 8,   -8,  -12 /*mean (0.149202), correlation (0.308918)*/,
    -13, 0,   -8,  -4 /*mean (0.160909), correlation (0.310013)*/,
    3,   3,   7,   8 /*mean (0.177755), correlation (0.309394)*/,
    5,   7,   10,  -7 /*mean (0.212337), correlation (0.310315)*/,
    -1,  7,   1,   -12 /*mean (0.214429), correlation (0.311933)*/,
    3,   -10, 5,   6 /*mean (0.235807), correlation (0.313104)*/,
    2,   -4,  3,   -10 /*mean (0.00494827), correlation (0.344948)*/,
    -13, 0,   -13, 5 /*mean (0.0549145), correlation (0.344675)*/,
    -13, -7,  -12, 12 /*mean (0.103385), correlation (0.342715)*/,
    -13, 3,   -11, 8 /*mean (0.134222), correlation (0.322922)*/,
    -7,  12,  -4,  7 /*mean (0.153284), correlation (0.337061)*/,
    6,   -10, 12,  8 /*mean (0.154881), correlation (0.329257)*/,
    -9,  -1,  -7,  -6 /*mean (0.200967), correlation (0.33312)*/,
    -2,  -5,  0,   12 /*mean (0.201518), correlation (0.340635)*/,
    -12, 5,   -7,  5 /*mean (0.207805), correlation (0.335631)*/,
    3,   -10, 8,   -13 /*mean (0.224438), correlation (0.34504)*/,
    -7,  -7,  -4,  5 /*mean (0.239361), correlation (0.338053)*/,
    -3,  -2,  -1,  -7 /*mean (0.240744), correlation (0.344322)*/,
    2,   9,   5,   -11 /*mean (0.242949), correlation (0.34145)*/,
    -11, -13, -5,  -13 /*mean (0.244028), correlation (0.336861)*/,
    -1,  6,   0,   -1 /*mean (0.247571), correlation (0.343684)*/,
    5,   -3,  5,   2 /*mean (0.000697256), correlation (0.357265)*/,
    -4,  -13, -4,  12 /*mean (0.00213675), correlation (0.373827)*/,
    -9,  -6,  -9,  6 /*mean (0.0126856), correlation (0.373938)*/,
    -12, -10, -8,  -4 /*mean (0.0152497), correlation (0.364237)*/,
    10,  2,   12,  -3 /*mean (0.0299933), correlation (0.345292)*/,
    7,   12,  12,  12 /*mean (0.0307242), correlation (0.366299)*/,
    -7,  -13, -6,  5 /*mean (0.0534975), correlation (0.368357)*/,
    -4,  9,   -3,  4 /*mean (0.099865), correlation (0.372276)*/,
    7,   -1,  12,  2 /*mean (0.117083), correlation (0.364529)*/,
    -7,  6,   -5,  1 /*mean (0.126125), correlation (0.369606)*/,
    -13, 11,  -12, 5 /*mean (0.130364), correlation (0.358502)*/,
    -3,  7,   -2,  -6 /*mean (0.131691), correlation (0.375531)*/,
    7,   -8,  12,  -7 /*mean (0.160166), correlation (0.379508)*/,
    -13, -7,  -11, -12 /*mean (0.167848), correlation (0.353343)*/,
    1,   -3,  12,  12 /*mean (0.183378), correlation (0.371916)*/,
    2,   -6,  3,   0 /*mean (0.228711), correlation (0.371761)*/,
    -4,  3,   -2,  -13 /*mean (0.247211), correlation (0.364063)*/,
    -1,  -13, 1,   9 /*mean (0.249325), correlation (0.378139)*/,
    7,   1,   8,   -6 /*mean (0.000652272), correlation (0.411682)*/,
    1,   -1,  3,   12 /*mean (0.00248538), correlation (0.392988)*/,
    9,   1,   12,  6 /*mean (0.0206815), correlation (0.386106)*/,
    -1,  -9,  -1,  3 /*mean (0.0364485), correlation (0.410752)*/,
    -13, -13, -10, 5 /*mean (0.0376068), correlation (0.398374)*/,
    7,   7,   10,  12 /*mean (0.0424202), correlation (0.405663)*/,
    12,  -5,  12,  9 /*mean (0.0942645), correlation (0.410422)*/,
    6,   3,   7,   11 /*mean (0.1074), correlation (0.413224)*/,
    5,   -13, 6,   10 /*mean (0.109256), correlation (0.408646)*/,
    2,   -12, 2,   3 /*mean (0.131691), correlation (0.416076)*/,
    3,   8,   4,   -6 /*mean (0.165081), correlation (0.417569)*/,
    2,   6,   12,  -13 /*mean (0.171874), correlation (0.408471)*/,
    9,   -12, 10,  3 /*mean (0.175146), correlation (0.41296)*/,
    -8,  4,   -7,  9 /*mean (0.183682), correlation (0.402956)*/,
    -11, 12,  -4,  -6 /*mean (0.184672), correlation (0.416125)*/,
    1,   12,  2,   -8 /*mean (0.191487), correlation (0.386696)*/,
    6,   -9,  7,   -4 /*mean (0.192668), correlation (0.394771)*/,
    2,   3,   3,   -2 /*mean (0.200157), correlation (0.408303)*/,
    6,   3,   11,  0 /*mean (0.204588), correlation (0.411762)*/,
    3,   -3,  8,   -8 /*mean (0.205904), correlation (0.416294)*/,
    7,   8,   9,   3 /*mean (0.213237), correlation (0.409306)*/,
    -11, -5,  -6,  -4 /*mean (0.243444), correlation (0.395069)*/,
    -10, 11,  -5,  10 /*mean (0.247672), correlation (0.413392)*/,
    -5,  -8,  -3,  12 /*mean (0.24774), correlation (0.411416)*/,
    -10, 5,   -9,  0 /*mean (0.00213675), correlation (0.454003)*/,
    8,   -1,  12,  -6 /*mean (0.0293635), correlation (0.455368)*/,
    4,   -6,  6,   -11 /*mean (0.0404971), correlation (0.457393)*/,
    -10, 12,  -8,  7 /*mean (0.0481107), correlation (0.448364)*/,
    4,   -2,  6,   7 /*mean (0.050641), correlation (0.455019)*/,
    -2,  0,   -2,  12 /*mean (0.0525978), correlation (0.44338)*/,
    -5,  -8,  -5,  2 /*mean (0.0629667), correlation (0.457096)*/,
    7,   -6,  10,  12 /*mean (0.0653846), correlation (0.445623)*/,
    -9,  -13, -8,  -8 /*mean (0.0858749), correlation (0.449789)*/,
    -5,  -13, -5,  -2 /*mean (0.122402), correlation (0.450201)*/,
    8,   -8,  9,   -13 /*mean (0.125416), correlation (0.453224)*/,
    -9,  -11, -9,  0 /*mean (0.130128), correlation (0.458724)*/,
    1,   -8,  1,   -2 /*mean (0.132467), correlation (0.440133)*/,
    7,   -4,  9,   1 /*mean (0.132692), correlation (0.454)*/,
    -2,  1,   -1,  -4 /*mean (0.135695), correlation (0.455739)*/,
    11,  -6,  12,  -11 /*mean (0.142904), correlation (0.446114)*/,
    -12, -9,  -6,  4 /*mean (0.146165), correlation (0.451473)*/,
    3,   7,   7,   12 /*mean (0.147627), correlation (0.456643)*/,
    5,   5,   10,  8 /*mean (0.152901), correlation (0.455036)*/,
    0,   -4,  2,   8 /*mean (0.167083), correlation (0.459315)*/,
    -9,  12,  -5,  -13 /*mean (0.173234), correlation (0.454706)*/,
    0,   7,   2,   12 /*mean (0.18312), correlation (0.433855)*/,
    -1,  2,   1,   7 /*mean (0.185504), correlation (0.443838)*/,
    5,   11,  7,   -9 /*mean (0.185706), correlation (0.451123)*/,
    3,   5,   6,   -8 /*mean (0.188968), correlation (0.455808)*/,
    -13, -4,  -8,  9 /*mean (0.191667), correlation (0.459128)*/,
    -5,  9,   -3,  -3 /*mean (0.193196), correlation (0.458364)*/,
    -4,  -7,  -3,  -12 /*mean (0.196536), correlation (0.455782)*/,
    6,   5,   8,   0 /*mean (0.1972), correlation (0.450481)*/,
    -7,  6,   -6,  12 /*mean (0.199438), correlation (0.458156)*/,
    -13, 6,   -5,  -2 /*mean (0.211224), correlation (0.449548)*/,
    1,   -10, 3,   10 /*mean (0.211718), correlation (0.440606)*/,
    4,   1,   8,   -4 /*mean (0.213034), correlation (0.443177)*/,
    -2,  -2,  2,   -13 /*mean (0.234334), correlation (0.455304)*/,
    2,   -12, 12,  12 /*mean (0.235684), correlation (0.443436)*/,
    -2,  -13, 0,   -6 /*mean (0.237674), correlation (0.452525)*/,
    4,   1,   9,   3 /*mean (0.23962), correlation (0.444824)*/,
    -6,  -10, -3,  -5 /*mean (0.248459), correlation (0.439621)*/,
    -3,  -13, -1,  1 /*mean (0.249505), correlation (0.456666)*/,
    7,   5,   12,  -11 /*mean (0.00119208), correlation (0.495466)*/,
    4,   -2,  5,   -7 /*mean (0.00372245), correlation (0.484214)*/,
    -13, 9,   -9,  -5 /*mean (0.00741116), correlation (0.499854)*/,
    7,   1,   8,   6 /*mean (0.0208952), correlation (0.499773)*/,
    7,   -8,  7,   6 /*mean (0.0220085), correlation (0.501609)*/,
    -7,  -4,  -7,  1 /*mean (0.0233806), correlation (0.496568)*/,
    -8,  11,  -7,  -8 /*mean (0.0236505), correlation (0.489719)*/,
    -13, 6,   -12, -8 /*mean (0.0268781), correlation (0.503487)*/,
    2,   4,   3,   9 /*mean (0.0323324), correlation (0.501938)*/,
    10,  -5,  12,  3 /*mean (0.0399235), correlation (0.494029)*/,
    -6,  -5,  -6,  7 /*mean (0.0420153), correlation (0.486579)*/,
    8,   -3,  9,   -8 /*mean (0.0548021), correlation (0.484237)*/,
    2,   -12, 2,   8 /*mean (0.0616622), correlation (0.496642)*/,
    -11, -2,  -10, 3 /*mean (0.0627755), correlation (0.498563)*/,
    -12, -13, -7,  -9 /*mean (0.0829622), correlation (0.495491)*/,
    -11, 0,   -10, -5 /*mean (0.0843342), correlation (0.487146)*/,
    5,   -3,  11,  8 /*mean (0.0929937), correlation (0.502315)*/,
    -2,  -13, -1,  12 /*mean (0.113327), correlation (0.48941)*/,
    -1,  -8,  0,   9 /*mean (0.132119), correlation (0.467268)*/,
    -13, -11, -12, -5 /*mean (0.136269), correlation (0.498771)*/,
    -10, -2,  -10, 11 /*mean (0.142173), correlation (0.498714)*/,
    -3,  9,   -2,  -13 /*mean (0.144141), correlation (0.491973)*/,
    2,   -3,  3,   2 /*mean (0.14892), correlation (0.500782)*/,
    -9,  -13, -4,  0 /*mean (0.150371), correlation (0.498211)*/,
    -4,  6,   -3,  -10 /*mean (0.152159), correlation (0.495547)*/,
    -4,  12,  -2,  -7 /*mean (0.156152), correlation (0.496925)*/,
    -6,  -11, -4,  9 /*mean (0.15749), correlation (0.499222)*/,
    6,   -3,  6,   11 /*mean (0.159211), correlation (0.503821)*/,
    -13, 11,  -5,  5 /*mean (0.162427), correlation (0.501907)*/,
    11,  11,  12,  6 /*mean (0.16652), correlation (0.497632)*/,
    7,   -5,  12,  -2 /*mean (0.169141), correlation (0.484474)*/,
    -1,  12,  0,   7 /*mean (0.169456), correlation (0.495339)*/,
    -4,  -8,  -3,  -2 /*mean (0.171457), correlation (0.487251)*/,
    -7,  1,   -6,  7 /*mean (0.175), correlation (0.500024)*/,
    -13, -12, -8,  -13 /*mean (0.175866), correlation (0.497523)*/,
    -7,  -2,  -6,  -8 /*mean (0.178273), correlation (0.501854)*/,
    -8,  5,   -6,  -9 /*mean (0.181107), correlation (0.494888)*/,
    -5,  -1,  -4,  5 /*mean (0.190227), correlation (0.482557)*/,
    -13, 7,   -8,  10 /*mean (0.196739), correlation (0.496503)*/,
    1,   5,   5,   -13 /*mean (0.19973), correlation (0.499759)*/,
    1,   0,   10,  -13 /*mean (0.204465), correlation (0.49873)*/,
    9,   12,  10,  -1 /*mean (0.209334), correlation (0.49063)*/,
    5,   -8,  10,  -9 /*mean (0.211134), correlation (0.503011)*/,
    -1,  11,  1,   -13 /*mean (0.212), correlation (0.499414)*/,
    -9,  -3,  -6,  2 /*mean (0.212168), correlation (0.480739)*/,
    -1,  -10, 1,   12 /*mean (0.212731), correlation (0.502523)*/,
    -13, 1,   -8,  -10 /*mean (0.21327), correlation (0.489786)*/,
    8,   -11, 10,  -6 /*mean (0.214159), correlation (0.488246)*/,
    2,   -13, 3,   -6 /*mean (0.216993), correlation (0.50287)*/,
    7,   -13, 12,  -9 /*mean (0.223639), correlation (0.470502)*/,
    -10, -10, -5,  -7 /*mean (0.224089), correlation (0.500852)*/,
    -10, -8,  -8,  -13 /*mean (0.228666), correlation (0.502629)*/,
    4,   -6,  8,   5 /*mean (0.22906), correlation (0.498305)*/,
    3,   12,  8,   -13 /*mean (0.233378), correlation (0.503825)*/,
    -4,  2,   -3,  -3 /*mean (0.234323), correlation (0.476692)*/,
    5,   -13, 10,  -12 /*mean (0.236392), correlation (0.475462)*/,
    4,   -13, 5,   -1 /*mean (0.236842), correlation (0.504132)*/,
    -9,  9,   -4,  3 /*mean (0.236977), correlation (0.497739)*/,
    0,   3,   3,   -9 /*mean (0.24314), correlation (0.499398)*/,
    -12, 1,   -6,  1 /*mean (0.243297), correlation (0.489447)*/,
    3,   2,   4,   -8 /*mean (0.00155196), correlation (0.553496)*/,
    -10, -10, -10, 9 /*mean (0.00239541), correlation (0.54297)*/,
    8,   -13, 12,  12 /*mean (0.0034413), correlation (0.544361)*/,
    -8,  -12, -6,  -5 /*mean (0.003565), correlation (0.551225)*/,
    2,   2,   3,   7 /*mean (0.00835583), correlation (0.55285)*/,
    10,  6,   11,  -8 /*mean (0.00885065), correlation (0.540913)*/,
    6,   8,   8,   -12 /*mean (0.0101552), correlation (0.551085)*/,
    -7,  10,  -6,  5 /*mean (0.0102227), correlation (0.533635)*/,
    -3,  -9,  -3,  9 /*mean (0.0110211), correlation (0.543121)*/,
    -1,  -13, -1,  5 /*mean (0.0113473), correlation (0.550173)*/,
    -3,  -7,  -3,  4 /*mean (0.0140913), correlation (0.554774)*/,
    -8,  -2,  -8,  3 /*mean (0.017049), correlation (0.55461)*/,
    4,   2,   12,  12 /*mean (0.01778), correlation (0.546921)*/,
    2,   -5,  3,   11 /*mean (0.0224022), correlation (0.549667)*/,
    6,   -9,  11,  -13 /*mean (0.029161), correlation (0.546295)*/,
    3,   -1,  7,   12 /*mean (0.0303081), correlation (0.548599)*/,
    11,  -1,  12,  4 /*mean (0.0355151), correlation (0.523943)*/,
    -3,  0,   -3,  6 /*mean (0.0417904), correlation (0.543395)*/,
    4,   -11, 4,   12 /*mean (0.0487292), correlation (0.542818)*/,
    2,   -4,  2,   1 /*mean (0.0575124), correlation (0.554888)*/,
    -10, -6,  -8,  1 /*mean (0.0594242), correlation (0.544026)*/,
    -13, 7,   -11, 1 /*mean (0.0597391), correlation (0.550524)*/,
    -13, 12,  -11, -13 /*mean (0.0608974), correlation (0.55383)*/,
    6,   0,   11,  -13 /*mean (0.065126), correlation (0.552006)*/,
    0,   -1,  1,   4 /*mean (0.074224), correlation (0.546372)*/,
    -13, 3,   -9,  -2 /*mean (0.0808592), correlation (0.554875)*/,
    -9,  8,   -6,  -3 /*mean (0.0883378), correlation (0.551178)*/,
    -13, -6,  -8,  -2 /*mean (0.0901035), correlation (0.548446)*/,
    5,   -9,  8,   10 /*mean (0.0949843), correlation (0.554694)*/,
    2,   7,   3,   -9 /*mean (0.0994152), correlation (0.550979)*/,
    -1,  -6,  -1,  -1 /*mean (0.10045), correlation (0.552714)*/,
    9,   5,   11,  -2 /*mean (0.100686), correlation (0.552594)*/,
    11,  -3,  12,  -8 /*mean (0.101091), correlation (0.532394)*/,
    3,   0,   3,   5 /*mean (0.101147), correlation (0.525576)*/,
    -1,  4,   0,   10 /*mean (0.105263), correlation (0.531498)*/,
    3,   -6,  4,   5 /*mean (0.110785), correlation (0.540491)*/,
    -13, 0,   -10, 5 /*mean (0.112798), correlation (0.536582)*/,
    5,   8,   12,  11 /*mean (0.114181), correlation (0.555793)*/,
    8,   9,   9,   -6 /*mean (0.117431), correlation (0.553763)*/,
    7,   -4,  8,   -12 /*mean (0.118522), correlation (0.553452)*/,
    -10, 4,   -10, 9 /*mean (0.12094), correlation (0.554785)*/,
    7,   3,   12,  4 /*mean (0.122582), correlation (0.555825)*/,
    9,   -7,  10,  -2 /*mean (0.124978), correlation (0.549846)*/,
    7,   0,   12,  -2 /*mean (0.127002), correlation (0.537452)*/,
    -1,  -6,  0,   -11 /*mean (0.127148), correlation (0.547401)*/
};

static float IC_Angle(
    const cv::Mat &image, const cv::Point2f &pt, const std::vector<int> &u_max)
{
  // The umax[] is the lookup table for calculating the Intensity centroid(IC).
  // It stores the radius about the y-axis of a circle.
  int m_01 = 0, m_10 = 0;
  const int cx = cvRound(pt.x);
  const int cy = cvRound(pt.y);

  const uchar *center = &image.at<uchar>(cy, cx);
  // const uchar* variable is read-only
  // Treat the center line differently, v=0
  for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
    // u is from -15 to 15
    m_10 += u * center[u];
  // The meaning of center[u] is (y, x + u).
  // Distance multiplied by mass is the calculation of the moment.
  // https://cont.sugatsune.co.jp/motion/kr/tips/toolview_focus
  // u, 즉 x축을 기준으로 center를 중심으로 좌우 15픽셀에 대한 무게 중심을 계산

  // Go line by line in the circuI853lar patch
  int step = (int)image.step1();
  for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
  // y방향으로 1~15px 떨어진 픽셀들에 대해서
  {
    // Proceed over the two lines
    int v_sum = 0;
    const int d = u_max[v];  // [15, 15, 15, 15, 14, 14, 14, 13, 13, 12, 11, 10,
                             // 9, 8, 6, 3]
    for (int u = -d; u <= d; ++u)
    // 해당 y축에 대해 x축으로 -d~d까지의 픽셀들에 대한 무게 중심 계산
    {
      const int val_plus = center[u + v * step];   // upper side
      const int val_minus = center[u - v * step];  // under side
      v_sum += (val_plus - val_minus);
      m_10 += u * (val_plus + val_minus);
    }
    m_01 += v * v_sum;
  }

  return cv::fastAtan2((float)m_01, (float)m_10);
}

// static void computeOrientations(
//     const cv::Mat &image,
//     std::vector<cv::KeyPoint> &keypoints,
//     const std::vector<int> &umax)
// {
//   for (std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
//                                            keypointEnd = keypoints.end();
//        keypoint != keypointEnd;
//        ++keypoint)
//   {
//     keypoint->angle = IC_Angle(image, keypoint->pt, umax);
//   }
// }

static void computeOrbDescriptor(
    const cv::Mat &img,
    const cv::KeyPoint &kp,
    const cv::Point *pattern,
    uchar *descriptor)
{
  float radian = (float)kp.angle * PIinv;
  float a = (float)cos(radian);
  float b = (float)sin(radian);

  const uchar *center = &img.at<uchar>(cvRound(kp.pt.y), cvRound(kp.pt.x));
  // Pointer of the center pixel.
  const int step = (int)img.step;

#define GET_VALUE(idx)                                           \
  center                                                         \
      [cvRound(pattern[idx].x * b + pattern[idx].y * a) * step + \
       cvRound(pattern[idx].x * a - pattern[idx].y * b)]
  //

  for (int i = 0; i < 32; ++i, pattern += 16)
  {
    int t0, t1, val;
    t0 = GET_VALUE(0);
    t1 = GET_VALUE(1);
    val = t0 < t1;
    t0 = GET_VALUE(2);
    t1 = GET_VALUE(3);
    val |= (t0 < t1) << 1;
    t0 = GET_VALUE(4);
    t1 = GET_VALUE(5);
    val |= (t0 < t1) << 2;
    t0 = GET_VALUE(6);
    t1 = GET_VALUE(7);
    val |= (t0 < t1) << 3;
    t0 = GET_VALUE(8);
    t1 = GET_VALUE(9);
    val |= (t0 < t1) << 4;
    t0 = GET_VALUE(10);
    t1 = GET_VALUE(11);
    val |= (t0 < t1) << 5;
    t0 = GET_VALUE(12);
    t1 = GET_VALUE(13);
    val |= (t0 < t1) << 6;
    t0 = GET_VALUE(14);
    t1 = GET_VALUE(15);
    val |= (t0 < t1) << 7;

    descriptor[i] = (uchar)val;
  }

#undef GET_VALUE
}

void FastOrbExtractor::resizeWithLevel(cv::Mat &img, int &level)
{
  for (int level = 0; level < mNLevels; level++)
  {
    mvImgPyramid[level] = img;
    // resize using scale factor
    cv::resize(
        mvImgPyramid[level],
        mvImgPyramid[level],
        cv::Size(),
        mvScaleFactorsInv[level],
        mvScaleFactorsInv[level]);
  }
}

FastOrbExtractor::FastOrbExtractor(
    int nPoints,
    int nLevels,
    int imgWidth,
    int imgHeight,
    float scaleFactor,
    int minFastThreshold,
    int fastThreshold)
    : mNPoints(nPoints),
      mNLevels(nLevels),
      mImgWidth(imgWidth),
      mImgHeight(imgHeight),
      mScaleFactor(scaleFactor),
      mMinFastThreshold(minFastThreshold),
      mFastThreshold(fastThreshold)
{
  // reserve() function doesn't set the vector size.
  mvScaleFactors.resize(nLevels, -1);
  mvScaleFactorsInv.resize(nLevels, -1);
  for (int level = 0; level < nLevels; ++level)
  {
    mvScaleFactors[level] = std::pow(scaleFactor, level);
    mvScaleFactorsInv[level] = 1.0 / mvScaleFactors[level];
  }

  float nPointsOfFirstLevel =
      nPoints * (1 - mvScaleFactorsInv[1]) /
      (1 - (float)pow((double)mvScaleFactorsInv[1], (double)nLevels));

  mvNPointsPerLevels.resize(nLevels, -1);
  int sumOfnPointsPerLevel = 0;
  for (int level = 0; level < nLevels - 1; ++level)
  {
    mvNPointsPerLevels[level] = round(nPointsOfFirstLevel);
    nPointsOfFirstLevel *= mvScaleFactorsInv[1];
    sumOfnPointsPerLevel += mvNPointsPerLevels[level];
  }
  mvNPointsPerLevels[nLevels - 1] = std::max(nPoints - sumOfnPointsPerLevel, 0);
  sumOfnPointsPerLevel += mvNPointsPerLevels[nLevels - 1];
  // PRINT("sumOfnPointsPerLevel", sumOfnPointsPerLevel);

  mvImgPyramid.resize(nLevels);
  // the reserve() function doesn't set the vector size.
  // for (int level = 0; level < nLevels; level++)
  // {
  //   mvImgPyramid[level] = cv::Mat::zeros(imgHeight, imgWidth, CV_8UC1);
  //   // resize using scale factors.
  //   cv::resize(
  //       mvImgPyramid[level],
  //       mvImgPyramid[level],
  //       cv::Size(),
  //       mvScaleFactorsInv[level],
  //       mvScaleFactorsInv[level]);
  // }

  // This is for orientation
  // pre-compute the end of a row in a circular patch
  umax.resize(HALF_PATCH_SIZE + 1);

  int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);  // 15
  // sqrt(2.f) / 2는 원을 8등분한 후(45도) 각 대간선의 최대 길이인 1/sqrt(2)를
  // 유리화 한것이다. x=y 같은 경우를 가정했으며, +1은 cvCeil(10.0000001)==11,
  // cvCeil(10.0)==10과 같이 항상 올림을 해주기 위해서 내림 함수에 +1을 사용해서
  // 예외를 처리해준다.
  int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
  // sqrt(2.f) / 2는 cos45인 1/sqrt(2)를 유리화한 형태로 수치를 미리 계산하는
  // precalculation을 이용하는 경우도 있다(수치적 안정성이 있다는데 자세한건 더
  // 알아봐야 한다.)
  const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
  for (v = 0; v <= vmax; ++v)  // vmax is 11.
  {
    umax[v] = cvRound(sqrt(hp2 - v * v));
    // x^2 + y^2 = r^2
    // x^2 = r^2 - y^2
    // x = sqrt(r^2 - y^2)
  }

  // Make sure we are symmetric
  for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
  {
    while (umax[v0] == umax[v0 + 1])
    {
      ++v0;
    }
    umax[v] = v0;
    ++v0;
  }

  const int npoints = 512;
  const cv::Point *pattern0 = (const cv::Point *)bit_pattern_31_;
  std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));
}

void FastOrbExtractor::makeImagePyramid(const cv::Mat &img)
{
  for (int level = 0; level < mNLevels; ++level)
  {
    const float scaleFactorInv = mvScaleFactorsInv[level];
    cv::Size imgSize(
        cvRound(img.cols * scaleFactorInv), cvRound(img.rows * scaleFactorInv));
    cv::Size imgSizeWithBorder(
        imgSize.width + BORDER_SIZE * 2, imgSize.height + BORDER_SIZE * 2);

    cv::Mat temp(imgSizeWithBorder, img.type());
    // The OpenCV is not supporing in-place operation for the same variable,
    // so we use a temporary variable to receive the output.
    mvImgPyramid[level] =
        temp(cv::Rect(BORDER_SIZE, BORDER_SIZE, imgSize.width, imgSize.height));
    // Using shallow copy, we can avoid copying the data.
    // also, we can check the border area using mvImgPyramid[level].step.

    if (level != 0)
    {
      cv::resize(
          mvImgPyramid[level - 1],
          mvImgPyramid[level],
          imgSize,
          0,
          0,
          cv::INTER_LINEAR);

      cv::copyMakeBorder(
          mvImgPyramid[level],
          temp,
          BORDER_SIZE,
          BORDER_SIZE,
          BORDER_SIZE,
          BORDER_SIZE,
          cv::BORDER_REFLECT_101 | cv::BORDER_ISOLATED);
    }
    else
    {  // level == 0
      cv::copyMakeBorder(
          img,
          temp,
          BORDER_SIZE,
          BORDER_SIZE,
          BORDER_SIZE,
          BORDER_SIZE,
          cv::BORDER_REFLECT_101);
    }
  }
}

void FastOrbExtractor::detectAndCompute(
    const cv::Mat &img, std::vector<cv::KeyPoint> &kps, cv::Mat &descriptors)
{
  kps.resize(mNPoints);
  descriptors.create(mNPoints, 32, CV_8UC1);

  makeImagePyramid(img);
  const int W = 30;
  int offset = 0;
  for (int level = 0; level < mNLevels; ++level)
  {
    int xminBorder = BORDER_SIZE + 1;
    int yminBorder = BORDER_SIZE + 1;
    int xmaxBorder = mvImgPyramid[level].cols - BORDER_SIZE - 1;
    int ymaxBorder = mvImgPyramid[level].rows - BORDER_SIZE - 1;

    const int width = xmaxBorder - xminBorder;
    const int height = ymaxBorder - yminBorder;

    const int nCols = width / W;
    const int nRows = height / W;

    const int wCell = ceil(width / nCols);
    const int hCell = ceil(height / nRows);

    std::vector<cv::KeyPoint> vKpsForDistribution;
    vKpsForDistribution.reserve(mNPoints * 10);
    for (int row = 0; row < nRows; ++row)
    {
      int ymin = row * hCell + yminBorder;
      int ymax = std::min(ymin + hCell, height);

      for (int col = 0; col < nCols; ++col)
      {
        int xmin = col * wCell + xminBorder;
        int xmax = std::min(xmin + wCell, width);
        std::vector<cv::KeyPoint> kpsCell;
        cv::FAST(
            mvImgPyramid[level].rowRange(ymin, ymax).colRange(xmin, xmax),
            kpsCell,
            mFastThreshold,
            true);
        if (kpsCell.size() < mvNPointsPerLevels[level])
        {
          cv::FAST(
              mvImgPyramid[level].rowRange(ymin, ymax).colRange(xmin, xmax),
              kpsCell,
              mMinFastThreshold,
              true);
        }
        for (cv::KeyPoint &kp : kpsCell)
        {
          kp.pt.x += xmin;
          kp.pt.y += ymin;
          vKpsForDistribution.push_back(kp);
        }
      }
    }

    cv::Mat blurImg = mvImgPyramid[level].clone();
    cv::GaussianBlur(
        blurImg, blurImg, cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);
    // PRINT(mvImgPyramid[level].step1());
    // PRINT("blurImg", blurImg.cols, blurImg.rows, blurImg.step1());
    std::vector<cv::KeyPoint> kpsFiltered = QuadTree::rectifyKps(
        vKpsForDistribution, mvNPointsPerLevels[level], 0, 0, width, height);
    PRINT(
        mvNPointsPerLevels[level],
        kpsFiltered.size(),
        vKpsForDistribution.size());
    for (int i = 0; i < mvNPointsPerLevels[level]; ++i)
    {
      cv::KeyPoint &kp = kpsFiltered[i];
      computeOrbDescriptor(
          blurImg, kp, &pattern[0], descriptors.ptr<uchar>(i + offset));
      kp.angle = IC_Angle(mvImgPyramid[level], kp.pt, umax);
      kp.octave = level;
      kp.size = PATCH_SIZE * mvScaleFactors[level];
      kp.pt *= mvScaleFactors[level];
      kps[i + offset] = kp;
    }
    offset += mvNPointsPerLevels[level];
  }
}
float FastOrbExtractor::get_scale2inv(const int octave) const
{
  return this->mvScaleFactorsInv[octave];
}
float FastOrbExtractor::get_scale2(const int octave) const
{
  return this->mvScaleFactors[octave];
}
FastOrbExtractor::~FastOrbExtractor() {}

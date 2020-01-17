#include <point.h>

/*共有150个点,X   Y   speed   direct    target_angle*/

Point points_pos0[] = {

    {0.320000, 0.439000, 50, 1.532187, 0.000000}, /*第0个点*/

    {0.321487, 0.477487, 89, 1.532187, 0.000000}, /*第1个点*/

    {0.322973, 0.515973, 129, 1.532188, 0.000000}, /*第2个点*/

    {0.324460, 0.554460, 169, 1.532187, 0.000000}, /*第3个点*/

    {0.325947, 0.592947, 208, 1.532187, 0.000000}, /*第4个点*/

    {0.327433, 0.631433, 248, 1.532187, 0.000000}, /*第5个点*/

    {0.328920, 0.669920, 288, 1.532188, 0.000000}, /*第6个点*/

    {0.330407, 0.708407, 300, 1.532187, 0.000000}, /*第7个点*/

    {0.331893, 0.746893, 300, 1.532187, 0.000000}, /*第8个点*/

    {0.333380, 0.785380, 300, 1.532187, 0.000000}, /*第9个点*/

    {0.334867, 0.823867, 300, 1.532187, 0.000000}, /*第10个点*/

    {0.336353, 0.862353, 300, 1.532188, 0.000000}, /*第11个点*/

    {0.337840, 0.900840, 300, 1.532187, 0.000000}, /*第12个点*/

    {0.339327, 0.939327, 300, 1.532187, 0.000000}, /*第13个点*/

    {0.340813, 0.977813, 300, 1.532187, 0.000000}, /*第14个点*/

    {0.342300, 1.016300, 300, 1.532188, 0.000000}, /*第15个点*/

    {0.343787, 1.054787, 300, 1.532187, 0.000000}, /*第16个点*/

    {0.345273, 1.093273, 300, 1.532187, 0.000000}, /*第17个点*/

    {0.346760, 1.131760, 300, 1.532187, 0.000000}, /*第18个点*/

    {0.348247, 1.170247, 300, 1.532188, 0.000000}, /*第19个点*/

    {0.349733, 1.208733, 300, 1.532187, 0.000000}, /*第20个点*/

    {0.351220, 1.247220, 300, 1.532187, 0.000000}, /*第21个点*/

    {0.352707, 1.285707, 300, 1.532187, 0.000000}, /*第22个点*/

    {0.354193, 1.324193, 300, 1.532187, 0.000000}, /*第23个点*/

    {0.355680, 1.362680, 300, 1.532188, 0.000000}, /*第24个点*/

    {0.357167, 1.401167, 300, 1.532187, 0.000000}, /*第25个点*/

    {0.358653, 1.439653, 300, 1.532187, 0.000000}, /*第26个点*/

    {0.360140, 1.478140, 300, 1.532187, 0.000000}, /*第27个点*/

    {0.361627, 1.516627, 300, 1.532188, 0.000000}, /*第28个点*/

    {0.363113, 1.555113, 300, 1.532187, 0.000000}, /*第29个点*/

    {0.364600, 1.593600, 300, 1.532187, 0.000000}, /*第30个点*/

    {0.366087, 1.632087, 300, 1.532187, 0.000000}, /*第31个点*/

    {0.367573, 1.670573, 300, 1.532188, 0.000000}, /*第32个点*/

    {0.369060, 1.709060, 300, 1.532187, 0.000000}, /*第33个点*/

    {0.370547, 1.747547, 300, 1.532187, 0.000000}, /*第34个点*/

    {0.372033, 1.786033, 300, 1.532187, 0.000000}, /*第35个点*/

    {0.373520, 1.824520, 300, 1.532187, 0.000000}, /*第36个点*/

    {0.375007, 1.863007, 300, 1.532188, 0.000000}, /*第37个点*/

    {0.376493, 1.901493, 300, 1.532187, 0.000000}, /*第38个点*/

    {0.377980, 1.939980, 300, 1.532187, 0.000000}, /*第39个点*/

    {0.379467, 1.978467, 300, 1.532187, 0.000000}, /*第40个点*/

    {0.380953, 2.016953, 300, 1.532188, 0.000000}, /*第41个点*/

    {0.382440, 2.055440, 300, 1.532187, 0.000000}, /*第42个点*/

    {0.383927, 2.093927, 300, 1.532187, 0.000000}, /*第43个点*/

    {0.385413, 2.132413, 300, 1.532187, 0.000000}, /*第44个点*/

    {0.386900, 2.170900, 300, 1.532188, 0.000000}, /*第45个点*/

    {0.388387, 2.209387, 300, 1.532187, 0.000000}, /*第46个点*/

    {0.389873, 2.247873, 300, 1.532187, 0.000000}, /*第47个点*/

    {0.391360, 2.286360, 300, 1.532187, 0.000000}, /*第48个点*/

    {0.392847, 2.324847, 300, 1.532187, 0.000000}, /*第49个点*/

    {0.394333, 2.363333, 300, 1.532188, 0.000000}, /*第50个点*/

    {0.395820, 2.401820, 300, 1.532187, 0.000000}, /*第51个点*/

    {0.397307, 2.440307, 300, 1.532187, 0.000000}, /*第52个点*/

    {0.398793, 2.478793, 300, 1.532187, 0.000000}, /*第53个点*/

    {0.400280, 2.517280, 300, 1.532188, 0.000000}, /*第54个点*/

    {0.401767, 2.555767, 300, 1.532187, 0.000000}, /*第55个点*/

    {0.403253, 2.594253, 300, 1.532187, 0.000000}, /*第56个点*/

    {0.404740, 2.632740, 300, 1.532187, 0.000000}, /*第57个点*/

    {0.406227, 2.671227, 300, 1.532188, 0.000000}, /*第58个点*/

    {0.407713, 2.709713, 300, 1.532187, 0.000000}, /*第59个点*/

    {0.409200, 2.748200, 300, 1.532187, 0.000000}, /*第60个点*/

    {0.410687, 2.786687, 300, 1.532187, 0.000000}, /*第61个点*/

    {0.412173, 2.825173, 300, 1.532187, 0.000000}, /*第62个点*/

    {0.413660, 2.863660, 300, 1.532188, 0.000000}, /*第63个点*/

    {0.415147, 2.902147, 300, 1.532187, 0.000000}, /*第64个点*/

    {0.416633, 2.940633, 300, 1.532187, 0.000000}, /*第65个点*/

    {0.418120, 2.979120, 300, 1.532187, 0.000000}, /*第66个点*/

    {0.419607, 3.017607, 300, 1.532188, 0.000000}, /*第67个点*/

    {0.421093, 3.056093, 300, 1.532187, 0.000000}, /*第68个点*/

    {0.422580, 3.094580, 300, 1.532187, 0.000000}, /*第69个点*/

    {0.424067, 3.133067, 300, 1.532187, 0.000000}, /*第70个点*/

    {0.425553, 3.171553, 300, 1.532188, 0.000000}, /*第71个点*/

    {0.427040, 3.210040, 300, 1.532187, 0.000000}, /*第72个点*/

    {0.428527, 3.248527, 300, 1.532187, 0.000000}, /*第73个点*/

    {0.430013, 3.287013, 300, 1.532187, 0.000000}, /*第74个点*/

    {0.431500, 3.325500, 300, 1.532187, 0.000000}, /*第75个点*/

    {0.432987, 3.363986, 250, 1.532188, 0.000000}, /*第76个点*/

    {0.434473, 3.402473, 250, 1.532187, 0.000000}, /*第77个点*/

    {0.435960, 3.440960, 250, 1.532187, 0.000000}, /*第78个点*/

    {0.437447, 3.479447, 250, 1.532187, 0.000000}, /*第79个点*/

    {0.438933, 3.517933, 250, 1.532188, 0.000000}, /*第80个点*/

    {0.440420, 3.556420, 250, 1.532187, 0.000000}, /*第81个点*/

    {0.441907, 3.594907, 250, 1.532187, 0.000000}, /*第82个点*/

    {0.443393, 3.633393, 250, 1.532187, 0.000000}, /*第83个点*/

    {0.444880, 3.671880, 250, 1.532188, 0.000000}, /*第84个点*/

    {0.446367, 3.710367, 250, 1.532187, 0.000000}, /*第85个点*/

    {0.447853, 3.748853, 250, 1.532187, 0.000000}, /*第86个点*/

    {0.449340, 3.787340, 250, 1.532187, 0.000000}, /*第87个点*/

    {0.450827, 3.825827, 250, 1.532187, 0.000000}, /*第88个点*/

    {0.452313, 3.864313, 250, 1.532188, 0.000000}, /*第89个点*/

    {0.453800, 3.902800, 250, 1.532187, 0.000000}, /*第90个点*/

    {0.455287, 3.941287, 250, 1.532187, 0.000000}, /*第91个点*/

    {0.456773, 3.979773, 250, 1.532187, 0.000000}, /*第92个点*/

    {0.458260, 4.018260, 250, 1.532188, 0.000000}, /*第93个点*/

    {0.459747, 4.056747, 250, 1.532187, 0.000000}, /*第94个点*/

    {0.461233, 4.095233, 250, 1.532187, 0.000000}, /*第95个点*/

    {0.462720, 4.133720, 250, 1.532187, 0.000000}, /*第96个点*/

    {0.464207, 4.172207, 250, 1.532188, 0.000000}, /*第97个点*/

    {0.465693, 4.210693, 250, 1.532187, 0.000000}, /*第98个点*/

    {0.467180, 4.249180, 250, 1.532187, 0.000000}, /*第99个点*/

    {0.468667, 4.287666, 250, 1.532187, 0.000000}, /*第100个点*/

    {0.470153, 4.326153, 250, 1.532187, 0.000000}, /*第101个点*/

    {0.471640, 4.364640, 250, 1.532188, 0.000000}, /*第102个点*/

    {0.473127, 4.403127, 250, 1.532187, 0.000000}, /*第103个点*/

    {0.474613, 4.441613, 250, 1.532187, 0.000000}, /*第104个点*/

    {0.476100, 4.480100, 250, 1.532187, 0.000000}, /*第105个点*/

    {0.477587, 4.518587, 250, 1.532188, 0.000000}, /*第106个点*/

    {0.479073, 4.557073, 250, 1.532187, 0.000000}, /*第107个点*/

    {0.480560, 4.595560, 250, 1.532187, 0.000000}, /*第108个点*/

    {0.482047, 4.634047, 250, 1.532187, 0.000000}, /*第109个点*/

    {0.483533, 4.672533, 250, 1.532188, 0.000000}, /*第110个点*/

    {0.485020, 4.711020, 250, 1.532187, 0.000000}, /*第111个点*/

    {0.486507, 4.749507, 250, 1.532187, 0.000000}, /*第112个点*/

    {0.487993, 4.787993, 250, 1.532187, 0.000000}, /*第113个点*/

    {0.489480, 4.826480, 250, 1.532187, 0.000000}, /*第114个点*/

    {0.490967, 4.864967, 250, 1.532188, 0.000000}, /*第115个点*/

    {0.492453, 4.903453, 250, 1.532187, 0.000000}, /*第116个点*/

    {0.493940, 4.941940, 250, 1.532187, 0.000000}, /*第117个点*/

    {0.495427, 4.980427, 250, 1.532187, 0.000000}, /*第118个点*/

    {0.496913, 5.018913, 250, 1.532188, 0.000000}, /*第119个点*/

    {0.498400, 5.057400, 250, 1.532187, 0.000000}, /*第120个点*/

    {0.499887, 5.095886, 250, 1.532187, 0.000000}, /*第121个点*/

    {0.501373, 5.134373, 250, 1.532187, 0.000000}, /*第122个点*/

    {0.502860, 5.172860, 250, 1.532188, 0.000000}, /*第123个点*/

    {0.504347, 5.211347, 250, 1.532187, 0.000000}, /*第124个点*/

    {0.505833, 5.249834, 250, 1.532187, 0.000000}, /*第125个点*/

    {0.507320, 5.288320, 250, 1.532187, 0.000000}, /*第126个点*/

    {0.508807, 5.326806, 250, 1.532187, 0.000000}, /*第127个点*/

    {0.510293, 5.365293, 250, 1.532188, 0.000000}, /*第128个点*/

    {0.511780, 5.403780, 250, 1.532187, 0.000000}, /*第129个点*/

    {0.513267, 5.442267, 250, 1.532187, 0.000000}, /*第130个点*/

    {0.514753, 5.480753, 250, 1.532187, 0.000000}, /*第131个点*/

    {0.516240, 5.519240, 250, 1.532188, 0.000000}, /*第132个点*/

    {0.517727, 5.557727, 250, 1.532187, 0.000000}, /*第133个点*/

    {0.519213, 5.596213, 250, 1.532187, 0.000000}, /*第134个点*/

    {0.520700, 5.634700, 250, 1.532187, 0.000000}, /*第135个点*/

    {0.522187, 5.673186, 250, 1.532188, 0.000000}, /*第136个点*/

    {0.523673, 5.711673, 250, 1.532187, 0.000000}, /*第137个点*/

    {0.525160, 5.750160, 250, 1.532187, 0.000000}, /*第138个点*/

    {0.526647, 5.788647, 250, 1.532187, 0.000000}, /*第139个点*/

    {0.528133, 5.827133, 250, 1.532187, 0.000000}, /*第140个点*/

    {0.529620, 5.865620, 250, 1.532188, 0.000000}, /*第141个点*/

    {0.531107, 5.904106, 250, 1.532187, 0.000000}, /*第142个点*/

    {0.532593, 5.942593, 250, 1.532187, 0.000000}, /*第143个点*/

    {0.534080, 5.981080, 238, 1.532187, 0.000000}, /*第144个点*/

    {0.535567, 6.019567, 198, 1.532188, 0.000000}, /*第145个点*/

    {0.537053, 6.058053, 158, 1.532187, 0.000000}, /*第146个点*/

    {0.538540, 6.096540, 119, 1.532187, 0.000000}, /*第147个点*/

    {0.540027, 6.135026, 79, 1.532187, 0.000000}, /*第148个点*/

    {0.541513, 6.173513, 39, 1.532188, 0.000000}, /*第149个点*/

    {0.543000, 6.212000, 0, 1.532188, 0.000000}, /*第150个点*/

};

Point points_pos1[] = {
        {0.320000,       0.442584,       10,       1.299553,       0.000000}, /*第0个点*/

    {0.332124,       0.486181,       61,       1.292917,       0.000000}, /*第1个点*/

    {0.344498,       0.529559,       91,       1.286239,       0.000000}, /*第2个点*/

    {0.357123,       0.572720,       121,       1.279520,       0.000000}, /*第3个点*/

    {0.369997,       0.615663,       151,       1.272759,       0.000000}, /*第4个点*/

    {0.383121,       0.658387,       181,       1.265959,       0.000000}, /*第5个点*/

    {0.396496,       0.700893,       211,       1.259116,       0.000000}, /*第6个点*/

    {0.410120,       0.743182,       241,       1.252234,       0.000000}, /*第7个点*/

    {0.423995,       0.785252,       271,       1.245312,       0.000000}, /*第8个点*/

    {0.438119,       0.827104,       301,       1.238349,       0.000000}, /*第9个点*/

    {0.452494,       0.868738,       301,       1.231347,       0.000000}, /*第10个点*/

    {0.467118,       0.910153,       300,       1.224306,       0.000000}, /*第11个点*/

    {0.481993,       0.951351,       300,       1.217226,       0.000000}, /*第12个点*/

    {0.497118,       0.992330,       300,       1.210109,       0.000000}, /*第13个点*/

    {0.512492,       1.033092,       299,       1.202952,       0.000000}, /*第14个点*/

    {0.528117,       1.073635,       299,       1.195758,       0.000000}, /*第15个点*/

    {0.543992,       1.113960,       298,       1.188527,       0.000000}, /*第16个点*/

    {0.560117,       1.154067,       298,       1.181260,       0.000000}, /*第17个点*/

    {0.576492,       1.193956,       298,       1.173957,       0.000000}, /*第18个点*/

    {0.593117,       1.233627,       297,       1.166618,       0.000000}, /*第19个点*/

    {0.609992,       1.273079,       297,       1.159242,       0.000000}, /*第20个点*/

    {0.627117,       1.312314,       297,       1.151836,       0.000000}, /*第21个点*/

    {0.644492,       1.351330,       296,       1.144391,       0.000000}, /*第22个点*/

    {0.662117,       1.390128,       296,       1.136918,       0.000000}, /*第23个点*/

    {0.679992,       1.428709,       296,       1.129408,       0.000000}, /*第24个点*/

    {0.698117,       1.467071,       295,       1.121867,       0.000000}, /*第25个点*/

    {0.716493,       1.505215,       295,       1.114297,       0.000000}, /*第26个点*/

    {0.735118,       1.543140,       295,       1.106692,       0.000000}, /*第27个点*/

    {0.753993,       1.580848,       294,       1.099062,       0.000000}, /*第28个点*/

    {0.773118,       1.618338,       294,       1.091397,       0.000000}, /*第29个点*/

    {0.792494,       1.655609,       294,       1.083707,       0.000000}, /*第30个点*/

    {0.812119,       1.692662,       293,       1.075990,       0.000000}, /*第31个点*/

    {0.831995,       1.729497,       293,       1.068245,       0.000000}, /*第32个点*/

    {0.852120,       1.766115,       293,       1.060472,       0.000000}, /*第33个点*/

    {0.872496,       1.802514,       293,       1.052673,       0.000000}, /*第34个点*/

    {0.893121,       1.838694,       293,       1.044855,       0.000000}, /*第35个点*/

    {0.913997,       1.874657,       292,       1.037007,       0.000000}, /*第36个点*/

    {0.935123,       1.910402,       292,       1.029141,       0.000000}, /*第37个点*/

    {0.956498,       1.945928,       292,       1.021248,       0.000000}, /*第38个点*/

    {0.978124,       1.981236,       292,       1.013336,       0.000000}, /*第39个点*/

    {1.000000,       2.016327,       239,       0.997715,       0.000000}, /*第40个点*/

    {1.007624,       2.028141,       170,       0.974294,       0.000000}, /*第41个点*/

    {1.015498,       2.039738,       169,       0.950725,       0.000000}, /*第42个点*/

    {1.023623,       2.051116,       169,       0.927056,       0.000000}, /*第43个点*/

    {1.031997,       2.062276,       169,       0.903305,       0.000000}, /*第44个点*/

    {1.040621,       2.073218,       169,       0.879492,       0.000000}, /*第45个点*/

    {1.049496,       2.083942,       169,       0.855656,       0.000000}, /*第46个点*/

    {1.058620,       2.094448,       169,       0.831808,       0.000000}, /*第47个点*/

    {1.067995,       2.104736,       169,       0.807994,       0.000000}, /*第48个点*/

    {1.077619,       2.114805,       169,       0.784228,       0.000000}, /*第49个点*/

    {1.087494,       2.124657,       169,       0.760541,       0.000000}, /*第50个点*/

    {1.097619,       2.134290,       169,       0.736954,       0.000000}, /*第51个点*/

    {1.107993,       2.143706,       170,       0.713507,       0.000000}, /*第52个点*/

    {1.118618,       2.152903,       170,       0.690203,       0.000000}, /*第53个点*/

    {1.129493,       2.161882,       170,       0.667082,       0.000000}, /*第54个点*/

    {1.140617,       2.170643,       171,       0.644150,       0.000000}, /*第55个点*/

    {1.151992,       2.179185,       171,       0.621461,       0.000000}, /*第56个点*/

    {1.163617,       2.187510,       171,       0.598988,       0.000000}, /*第57个点*/

    {1.175492,       2.195617,       172,       0.576800,       0.000000}, /*第58个点*/

    {1.187617,       2.203505,       173,       0.554865,       0.000000}, /*第59个点*/

    {1.199992,       2.211175,       173,       0.533225,       0.000000}, /*第60个点*/

    {1.212617,       2.218627,       174,       0.511914,       0.000000}, /*第61个点*/

    {1.225492,       2.225862,       174,       0.490893,       0.000000}, /*第62个点*/

    {1.238617,       2.232877,       175,       0.470219,       0.000000}, /*第63个点*/

    {1.251992,       2.239675,       176,       0.449889,       0.000000}, /*第64个点*/

    {1.265617,       2.246255,       177,       0.429877,       0.000000}, /*第65个点*/

    {1.279493,       2.252616,       177,       0.410246,       0.000000}, /*第66个点*/

    {1.293618,       2.258760,       178,       0.390968,       0.000000}, /*第67个点*/

    {1.307993,       2.264685,       179,       0.372052,       0.000000}, /*第68个点*/

    {1.322618,       2.270392,       180,       0.353510,       0.000000}, /*第69个点*/

    {1.337494,       2.275882,       181,       0.335319,       0.000000}, /*第70个点*/

    {1.352619,       2.281152,       163,       0.317511,       0.000000}, /*第71个点*/

    {1.367995,       2.286205,       146,       0.300063,       0.000000}, /*第72个点*/

    {1.383620,       2.291040,       128,       0.282998,       0.000000}, /*第73个点*/

    {1.399496,       2.295657,       111,       0.266285,       0.000000}, /*第74个点*/

    {1.415622,       2.300055,       93,       0.249946,       0.000000}, /*第75个点*/

    {1.431997,       2.304235,       74,       0.233956,       0.000000}, /*第76个点*/

    {1.448623,       2.308198,       56,       0.218315,       0.000000}, /*第77个点*/

    {1.465499,       2.311942,       37,       0.203055,       0.000000}, /*第78个点*/

    {1.482624,       2.315468,       19,       0.188121,       0.000000}, /*第79个点*/

    {1.500000,       2.318776,       50,       0.188121,       0.000000}, /*第80个点*/

    {1.500000,       2.300000,       30,       0.188121,       0.000000}, /*第81个点*/

    {1.500000,       2.300000,       30,       0.188121,       0.000000}, /*第82个点*/

    {1.500000,       2.300000,       0,       0.188121,       0.000000}/*第83个点*/
};
Point points_pos2[] = {

    {0.000000,       0.000000,       125,       1.039693,       0.000000}, /*第0个点*/

    {0.020770,       0.035359,       123,       1.028609,       0.000000}, /*第1个点*/

    {0.041756,       0.070197,       183,       1.017341,       0.000000}, /*第2个点*/

    {0.062960,       0.104514,       244,       1.005887,       0.000000}, /*第3个点*/

    {0.084380,       0.138310,       304,       0.994247,       0.000000}, /*第4个点*/

    {0.106018,       0.171586,       363,       0.982419,       0.000000}, /*第5个点*/

    {0.127872,       0.204341,       422,       0.970401,       0.000000}, /*第6个点*/

    {0.149943,       0.236575,       480,       0.958192,       0.000000}, /*第7个点*/

    {0.172232,       0.268288,       538,       0.945793,       0.000000}, /*第8个点*/

    {0.194737,       0.299481,       596,       0.933201,       0.000000}, /*第9个点*/

    {0.217459,       0.330153,       594,       0.920417,       0.000000}, /*第10个点*/

    {0.240398,       0.360304,       591,       0.907442,       0.000000}, /*第11个点*/

    {0.263554,       0.389934,       589,       0.894273,       0.000000}, /*第12个点*/

    {0.286927,       0.419044,       587,       0.880912,       0.000000}, /*第13个点*/

    {0.310517,       0.447633,       585,       0.867360,       0.000000}, /*第14个点*/

    {0.334324,       0.475701,       583,       0.853619,       0.000000}, /*第15个点*/

    {0.358347,       0.503249,       581,       0.839687,       0.000000}, /*第16个点*/

    {0.382588,       0.530275,       579,       0.825568,       0.000000}, /*第17个点*/

    {0.407046,       0.556781,       577,       0.811263,       0.000000}, /*第18个点*/

    {0.431720,       0.582766,       575,       0.796775,       0.000000}, /*第19个点*/

    {0.456612,       0.608231,       574,       0.782106,       0.000000}, /*第20个点*/

    {0.481720,       0.633175,       572,       0.767259,       0.000000}, /*第21个点*/

    {0.507046,       0.657597,       570,       0.752241,       0.000000}, /*第22个点*/

    {0.532588,       0.681500,       569,       0.737048,       0.000000}, /*第23个点*/

    {0.558347,       0.704881,       567,       0.721690,       0.000000}, /*第24个点*/

    {0.584324,       0.727742,       566,       0.706172,       0.000000}, /*第25个点*/

    {0.610517,       0.750082,       564,       0.690495,       0.000000}, /*第26个点*/

    {0.636927,       0.771901,       563,       0.674669,       0.000000}, /*第27个点*/

    {0.663554,       0.793200,       562,       0.658696,       0.000000}, /*第28个点*/

    {0.690398,       0.813977,       560,       0.642583,       0.000000}, /*第29个点*/

    {0.717459,       0.834234,       559,       0.626343,       0.000000}, /*第30个点*/

    {0.744737,       0.853971,       558,       0.609971,       0.000000}, /*第31个点*/

    {0.772231,       0.873186,       557,       0.593485,       0.000000}, /*第32个点*/

    {0.799943,       0.891881,       556,       0.576887,       0.000000}, /*第33个点*/

    {0.827872,       0.910055,       555,       0.560190,       0.000000}, /*第34个点*/

    {0.856018,       0.927708,       555,       0.543395,       0.000000}, /*第35个点*/

    {0.884380,       0.944841,       554,       0.526514,       0.000000}, /*第36个点*/

    {0.912960,       0.961452,       553,       0.509565,       0.000000}, /*第37个点*/

    {0.941756,       0.977543,       553,       0.492543,       0.000000}, /*第38个点*/

    {0.970770,       0.993114,       552,       0.475464,       0.000000}, /*第39个点*/

    {1.000000,       1.008163,       509,       0.454739,       0.000000}, /*第40个点*/

    {1.020770,       1.018318,       463,       0.430349,       0.000000}, /*第41个点*/

    {1.041756,       1.027952,       462,       0.405922,       0.000000}, /*第42个点*/

    {1.062960,       1.037065,       462,       0.381478,       0.000000}, /*第43个点*/

    {1.084380,       1.045657,       462,       0.357048,       0.000000}, /*第44个点*/

    {1.106018,       1.053729,       463,       0.332668,       0.000000}, /*第45个点*/

    {1.127872,       1.061279,       463,       0.308355,       0.000000}, /*第46个点*/

    {1.149943,       1.068309,       464,       0.284149,       0.000000}, /*第47个点*/

    {1.172231,       1.074819,       464,       0.260074,       0.000000}, /*第48个点*/

    {1.194737,       1.080807,       465,       0.236148,       0.000000}, /*第49个点*/

    {1.217459,       1.086275,       466,       0.212409,       0.000000}, /*第50个点*/

    {1.240398,       1.091222,       467,       0.188874,       0.000000}, /*第51个点*/

    {1.263554,       1.095649,       468,       0.165569,       0.000000}, /*第52个点*/

    {1.286927,       1.099554,       469,       0.142515,       0.000000}, /*第53个点*/

    {1.310517,       1.102939,       471,       0.119731,       0.000000}, /*第54个点*/

    {1.334323,       1.105803,       472,       0.097233,       0.000000}, /*第55个点*/

    {1.358347,       1.108146,       474,       0.075047,       0.000000}, /*第56个点*/

    {1.382588,       1.109969,       475,       0.053180,       0.000000}, /*第57个点*/

    {1.407046,       1.111271,       477,       0.031644,       0.000000}, /*第58个点*/

    {1.431720,       1.112052,       479,       0.010461,       0.000000}, /*第59个点*/

    {1.456612,       1.112312,       481,       -0.010371,       0.000000}, /*第60个点*/

    {1.481720,       1.112052,       483,       -0.030832,       0.000000}, /*第61个点*/

    {1.507046,       1.111271,       486,       -0.050926,       0.000000}, /*第62个点*/

    {1.532588,       1.109969,       488,       -0.070639,       0.000000}, /*第63个点*/

    {1.558347,       1.108146,       490,       -0.089966,       0.000000}, /*第64个点*/

    {1.584323,       1.105803,       493,       -0.108913,       0.000000}, /*第65个点*/

    {1.610517,       1.102939,       495,       -0.127471,       0.000000}, /*第66个点*/

    {1.636927,       1.099554,       498,       -0.145639,       0.000000}, /*第67个点*/

    {1.663554,       1.095649,       501,       -0.163420,       0.000000}, /*第68个点*/

    {1.690398,       1.091222,       503,       -0.180816,       0.000000}, /*第69个点*/

    {1.717459,       1.086275,       506,       -0.197828,       0.000000}, /*第70个点*/

    {1.744737,       1.080807,       458,       -0.214459,       0.000000}, /*第71个点*/

    {1.772232,       1.074819,       409,       -0.230712,       0.000000}, /*第72个点*/

    {1.799943,       1.068309,       360,       -0.246591,       0.000000}, /*第73个点*/

    {1.827872,       1.061279,       310,       -0.262106,       0.000000}, /*第74个点*/

    {1.856018,       1.053729,       260,       -0.277254,       0.000000}, /*第75个点*/

    {1.884380,       1.045657,       209,       -0.292049,       0.000000}, /*第76个点*/

    {1.912960,       1.037065,       158,       -0.306493,       0.000000}, /*第77个点*/

    {1.941756,       1.027952,       106,       -0.320590,       0.000000}, /*第78个点*/

    {1.970770,       1.018318,       53,       -0.334356,       0.000000}, /*第79个点*/

    {2.000000,       1.008163,       50,       -0.334356,       0.000000}, /*第80个点*/

    {2.000000,       1.000000,       30,       -0.334356,       0.000000}, /*第81个点*/

    {2.000000,       1.000000,       30,       -0.334356,       0.000000}, /*第82个点*/

    {2.000000,       1.000000,       30,       -0.334356,       0.000000}/*第83个点*/

};


/*共有41个点,X   Y   speed   direct    target_angle*/

Point points_pos3[] = {

    {0.000000,       0.000000,       50,       0.000000,       0.000000}, /*第0个点*/

    {0.075000,       0.000000,       127,       0.000000,       0.000000}, /*第1个点*/

    {0.150000,       0.000000,       204,       0.000000,       0.000000}, /*第2个点*/

    {0.225000,       0.000000,       282,       0.000000,       0.000000}, /*第3个点*/

    {0.300000,       0.000000,       300,       0.000000,       0.000000}, /*第4个点*/

    {0.375000,       0.000000,       300,       0.000000,       0.000000}, /*第5个点*/

    {0.450000,       0.000000,       300,       0.000000,       0.000000}, /*第6个点*/

    {0.525000,       0.000000,       300,       0.000000,       0.000000}, /*第7个点*/

    {0.600000,       0.000000,       300,       0.000000,       0.000000}, /*第8个点*/

    {0.675000,       0.000000,       300,       0.000000,       0.000000}, /*第9个点*/

    {0.750000,       0.000000,       300,       0.000000,       0.000000}, /*第10个点*/

    {0.825000,       0.000000,       300,       0.000000,       0.000000}, /*第11个点*/

    {0.900000,       0.000000,       300,       0.000000,       0.000000}, /*第12个点*/

    {0.975000,       0.000000,       300,       0.000000,       0.000000}, /*第13个点*/

    {1.050000,       0.000000,       300,       0.000000,       0.000000}, /*第14个点*/

    {1.125000,       0.000000,       300,       0.000000,       0.000000}, /*第15个点*/

    {1.200000,       0.000000,       300,       0.000000,       0.000000}, /*第16个点*/

    {1.275000,       0.000000,       300,       0.000000,       0.000000}, /*第17个点*/

    {1.350000,       0.000000,       300,       0.000000,       0.000000}, /*第18个点*/

    {1.425000,       0.000000,       300,       0.000000,       0.000000}, /*第19个点*/

    {1.500000,       0.000000,       300,       0.000000,       0.000000}, /*第20个点*/

    {1.575000,       0.000000,       250,       0.000000,       0.000000}, /*第21个点*/

    {1.650000,       0.000000,       250,       0.000000,       0.000000}, /*第22个点*/

    {1.725000,       0.000000,       250,       0.000000,       0.000000}, /*第23个点*/

    {1.800000,       0.000000,       250,       0.000000,       0.000000}, /*第24个点*/

    {1.875000,       0.000000,       250,       0.000000,       0.000000}, /*第25个点*/

    {1.950000,       0.000000,       250,       0.000000,       0.000000}, /*第26个点*/

    {2.025000,       0.000000,       250,       0.000000,       0.000000}, /*第27个点*/

    {2.100000,       0.000000,       250,       0.000000,       0.000000}, /*第28个点*/

    {2.175000,       0.000000,       250,       0.000000,       0.000000}, /*第29个点*/

    {2.250000,       0.000000,       250,       0.000000,       0.000000}, /*第30个点*/

    {2.325000,       0.000000,       250,       0.000000,       0.000000}, /*第31个点*/

    {2.400000,       0.000000,       250,       0.000000,       0.000000}, /*第32个点*/

    {2.475000,       0.000000,       250,       0.000000,       0.000000}, /*第33个点*/

    {2.550000,       0.000000,       250,       0.000000,       0.000000}, /*第34个点*/

    {2.625000,       0.000000,       250,       0.000000,       0.000000}, /*第35个点*/

    {2.700000,       0.000000,       250,       0.000000,       0.000000}, /*第36个点*/

    {2.775000,       0.000000,       232,       0.000000,       0.000000}, /*第37个点*/

    {2.850000,       0.000000,       154,       0.000000,       0.000000}, /*第38个点*/

    {2.925000,       0.000000,       77,       0.000000,       0.000000}, /*第39个点*/

    {3.000000,       0.000000,       0,       0.000000,       0.000000}, /*第40个点*/


};

Point points_pos4[] = {

{0.000000,       0.000000,       125,       0.805103,       0.000000}, /*第0个点*/

{0.011435,       0.011895,       58,       0.798800,       0.000000}, /*第1个点*/

{0.022925,       0.023697,       87,       0.792476,       0.000000}, /*第2个点*/

{0.034469,       0.035406,       116,       0.786130,       0.000000}, /*第3个点*/

{0.046068,       0.047022,       145,       0.779763,       0.000000}, /*第4个点*/

{0.057722,       0.058545,       174,       0.773375,       0.000000}, /*第5个点*/

{0.069430,       0.069974,       203,       0.766967,       0.000000}, /*第6个点*/

{0.081193,       0.081311,       232,       0.760539,       0.000000}, /*第7个点*/

{0.093010,       0.092555,       260,       0.754092,       0.000000}, /*第8个点*/

{0.104882,       0.103706,       289,       0.747626,       0.000000}, /*第9个点*/

{0.116809,       0.114764,       289,       0.741141,       0.000000}, /*第10个点*/

{0.128790,       0.125729,       289,       0.734638,       0.000000}, /*第11个点*/

{0.140826,       0.136601,       289,       0.728119,       0.000000}, /*第12个点*/

{0.152916,       0.147380,       288,       0.721582,       0.000000}, /*第13个点*/

{0.165061,       0.158067,       288,       0.715027,       0.000000}, /*第14个点*/

{0.177261,       0.168660,       288,       0.708459,       0.000000}, /*第15个点*/

{0.189515,       0.179160,       288,       0.701873,       0.000000}, /*第16个点*/

{0.201824,       0.189567,       288,       0.695273,       0.000000}, /*第17个点*/

{0.214187,       0.199881,       288,       0.688657,       0.000000}, /*第18个点*/

{0.226605,       0.210102,       287,       0.682031,       0.000000}, /*第19个点*/

{0.239078,       0.220230,       287,       0.675386,       0.000000}, /*第20个点*/

{0.251605,       0.230265,       287,       0.668734,       0.000000}, /*第21个点*/

{0.264187,       0.240207,       287,       0.662066,       0.000000}, /*第22个点*/

{0.276824,       0.250057,       287,       0.655387,       0.000000}, /*第23个点*/

{0.289515,       0.259813,       287,       0.648698,       0.000000}, /*第24个点*/

{0.302261,       0.269476,       287,       0.641998,       0.000000}, /*第25个点*/

{0.315061,       0.279046,       287,       0.635284,       0.000000}, /*第26个点*/

{0.327916,       0.288523,       286,       0.628564,       0.000000}, /*第27个点*/

{0.340826,       0.297908,       286,       0.621837,       0.000000}, /*第28个点*/

{0.353790,       0.307199,       286,       0.615099,       0.000000}, /*第29个点*/

{0.366809,       0.316397,       286,       0.608356,       0.000000}, /*第30个点*/

{0.379882,       0.325502,       286,       0.601602,       0.000000}, /*第31个点*/

{0.393010,       0.334515,       286,       0.594844,       0.000000}, /*第32个点*/

{0.406193,       0.343434,       286,       0.588081,       0.000000}, /*第33个点*/

{0.419430,       0.352260,       286,       0.581311,       0.000000}, /*第34个点*/

{0.432722,       0.360994,       286,       0.574537,       0.000000}, /*第35个点*/

{0.446068,       0.369634,       286,       0.567758,       0.000000}, /*第36个点*/

{0.459469,       0.378181,       286,       0.560980,       0.000000}, /*第37个点*/

{0.472925,       0.386636,       286,       0.554192,       0.000000}, /*第38个点*/

{0.486435,       0.394997,       286,       0.547407,       0.000000}, /*第39个点*/

{0.500000,       0.403265,       302,       0.535859,       0.000000}, /*第40个点*/

{0.516772,       0.413225,       313,       0.518882,       0.000000}, /*第41个点*/

{0.532886,       0.422427,       305,       0.501114,       0.000000}, /*第42个点*/

{0.548378,       0.430913,       298,       0.482614,       0.000000}, /*第43个点*/

{0.563282,       0.438721,       291,       0.463438,       0.000000}, /*第44个点*/

{0.577632,       0.445893,       284,       0.443687,       0.000000}, /*第45个点*/

{0.591464,       0.452467,       277,       0.423492,       0.000000}, /*第46个点*/

{0.604811,       0.458484,       271,       0.403000,       0.000000}, /*第47个点*/

{0.617709,       0.463982,       266,       0.382429,       0.000000}, /*第48个点*/

{0.630192,       0.469003,       260,       0.361989,       0.000000}, /*第49个点*/

{0.642294,       0.473586,       256,       0.341957,       0.000000}, /*第50个点*/

{0.654050,       0.477771,       251,       0.322624,       0.000000}, /*第51个点*/

{0.665495,       0.481597,       247,       0.304308,       0.000000}, /*第52个点*/

{0.676664,       0.485104,       244,       0.287326,       0.000000}, /*第53个点*/

{0.687590,       0.488333,       240,       0.272022,       0.000000}, /*第54个点*/

{0.698309,       0.491323,       238,       0.258694,       0.000000}, /*第55个点*/

{0.708854,       0.494114,       236,       0.247646,       0.000000}, /*第56个点*/

{0.719262,       0.496745,       234,       0.239125,       0.000000}, /*第57个点*/

{0.729566,       0.499257,       233,       0.233330,       0.000000}, /*第58个点*/

{0.739800,       0.501689,       232,       0.230387,       0.000000}, /*第59个点*/

{0.750000,       0.504082,       232,       0.230389,       0.000000}, /*第60个点*/

{0.760200,       0.506474,       232,       0.233330,       0.000000}, /*第61个点*/

{0.770434,       0.508906,       233,       0.239125,       0.000000}, /*第62个点*/

{0.780738,       0.511418,       234,       0.247644,       0.000000}, /*第63个点*/

{0.791145,       0.514050,       236,       0.258694,       0.000000}, /*第64个点*/

{0.801691,       0.516840,       238,       0.272024,       0.000000}, /*第65个点*/

{0.812410,       0.519830,       240,       0.287326,       0.000000}, /*第66个点*/

{0.823336,       0.523059,       244,       0.304308,       0.000000}, /*第67个点*/

{0.834505,       0.526567,       247,       0.322624,       0.000000}, /*第68个点*/

{0.845950,       0.530393,       251,       0.341956,       0.000000}, /*第69个点*/

{0.857706,       0.534577,       256,       0.361991,       0.000000}, /*第70个点*/

{0.869808,       0.539160,       260,       0.382426,       0.000000}, /*第71个点*/

{0.882291,       0.544181,       266,       0.403003,       0.000000}, /*第72个点*/

{0.895189,       0.549680,       271,       0.423492,       0.000000}, /*第73个点*/

{0.908536,       0.555696,       277,       0.443684,       0.000000}, /*第74个点*/

{0.922368,       0.562270,       284,       0.463439,       0.000000}, /*第75个点*/

{0.936718,       0.569442,       291,       0.482610,       0.000000}, /*第76个点*/

{0.951622,       0.577251,       298,       0.501118,       0.000000}, /*第77个点*/

{0.967114,       0.585736,       305,       0.518879,       0.000000}, /*第78个点*/

{0.983228,       0.594939,       313,       0.535860,       0.000000}, /*第79个点*/

{1.000000,       0.604898,       302,       0.547404,       0.000000}, /*第80个点*/

{1.013565,       0.613166,       286,       0.554195,       0.000000}, /*第81个点*/

{1.027075,       0.621528,       286,       0.560978,       0.000000}, /*第82个点*/

{1.040531,       0.629982,       286,       0.567761,       0.000000}, /*第83个点*/

{1.053932,       0.638529,       286,       0.574537,       0.000000}, /*第84个点*/

{1.067278,       0.647170,       286,       0.581309,       0.000000}, /*第85个点*/

{1.080570,       0.655903,       286,       0.588081,       0.000000}, /*第86个点*/

{1.093807,       0.664729,       286,       0.594845,       0.000000}, /*第87个点*/

{1.106990,       0.673649,       286,       0.601603,       0.000000}, /*第88个点*/

{1.120118,       0.682661,       286,       0.608351,       0.000000}, /*第89个点*/

{1.133191,       0.691766,       286,       0.615102,       0.000000}, /*第90个点*/

{1.146210,       0.700964,       286,       0.621837,       0.000000}, /*第91个点*/

{1.159174,       0.710256,       286,       0.628564,       0.000000}, /*第92个点*/

{1.172084,       0.719640,       286,       0.635289,       0.000000}, /*第93个点*/

{1.184939,       0.729117,       287,       0.641991,       0.000000}, /*第94个点*/

{1.197739,       0.738687,       287,       0.648702,       0.000000}, /*第95个点*/

{1.210485,       0.748351,       287,       0.655386,       0.000000}, /*第96个点*/

{1.223176,       0.758107,       287,       0.662067,       0.000000}, /*第97个点*/

{1.235813,       0.767956,       287,       0.668731,       0.000000}, /*第98个点*/

{1.248395,       0.777898,       287,       0.675394,       0.000000}, /*第99个点*/

{1.260922,       0.787933,       287,       0.682024,       0.000000}, /*第100个点*/

{1.273395,       0.798061,       287,       0.688654,       0.000000}, /*第101个点*/

{1.285813,       0.808282,       288,       0.695278,       0.000000}, /*第102个点*/

{1.298176,       0.818597,       288,       0.701874,       0.000000}, /*第103个点*/

{1.310485,       0.829004,       288,       0.708459,       0.000000}, /*第104个点*/

{1.322739,       0.839504,       288,       0.715024,       0.000000}, /*第105个点*/

{1.334939,       0.850097,       288,       0.721579,       0.000000}, /*第106个点*/

{1.347084,       0.860783,       288,       0.728123,       0.000000}, /*第107个点*/

{1.359174,       0.871562,       289,       0.734632,       0.000000}, /*第108个点*/

{1.371210,       0.882434,       289,       0.741144,       0.000000}, /*第109个点*/

{1.383191,       0.893399,       289,       0.747621,       0.000000}, /*第110个点*/

{1.395118,       0.904457,       260,       0.754100,       0.000000}, /*第111个点*/

{1.406990,       0.915608,       231,       0.760533,       0.000000}, /*第112个点*/

{1.418807,       0.926852,       203,       0.766965,       0.000000}, /*第113个点*/

{1.430570,       0.938189,       174,       0.773377,       0.000000}, /*第114个点*/

{1.442278,       0.949619,       145,       0.779763,       0.000000}, /*第115个点*/

{1.453932,       0.961142,       116,       0.786131,       0.000000}, /*第116个点*/

{1.465531,       0.972758,       87,       0.792475,       0.000000}, /*第117个点*/

{1.477075,       0.984466,       58,       0.798800,       0.000000}, /*第118个点*/

{1.488565,       0.996268,       29,       0.805101,       0.000000}, /*第119个点*/

{1.500000,       1.008163,       50,       0.805101,       0.000000}, /*第120个点*/

{1.500000,       1.000000,       30,       0.805101,       0.000000}, /*第121个点*/

{1.500000,       1.000000,       30,       0.805101,       0.000000}, /*第122个点*/

{1.500000,       1.000000,       30,       0.805101,       0.000000}/*第123个点*/

};
Point points_pos5[];
Point points_pos6[];
Point points_pos7[];
Point points_pos8[];
Point points_pos9[];
Point points_pos10[];
Point points_pos11[];
Point points_pos12[];

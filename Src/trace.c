#include<point.h>
/*共有153个点,X   Y   speed   direct    target_angle*/
Point points_pos[] = {
{0.320000,       0.439000,       50,       1.532187,       0.000000}, /*第0个点*/
{0.321487,       0.477487,       89,       1.532187,       0.000000}, /*第1个点*/
{0.322973,       0.515973,       129,       1.532188,       0.000000}, /*第2个点*/
{0.324460,       0.554460,       169,       1.532187,       0.000000}, /*第3个点*/
{0.325947,       0.592947,       208,       1.532187,       0.000000}, /*第4个点*/
{0.327433,       0.631433,       248,       1.532187,       0.000000}, /*第5个点*/
{0.328920,       0.669920,       288,       1.532188,       0.000000}, /*第6个点*/
{0.330407,       0.708407,       328,       1.532187,       0.000000}, /*第7个点*/
{0.331893,       0.746893,       367,       1.532187,       0.000000}, /*第8个点*/
{0.333380,       0.785380,       407,       1.532187,       0.000000}, /*第9个点*/
{0.334867,       0.823867,       447,       1.532187,       0.000000}, /*第10个点*/
{0.336353,       0.862353,       487,       1.532188,       0.000000}, /*第11个点*/
{0.337840,       0.900840,       500,       1.532187,       0.000000}, /*第12个点*/
{0.339327,       0.939327,       500,       1.532187,       0.000000}, /*第13个点*/
{0.340813,       0.977813,       500,       1.532187,       0.000000}, /*第14个点*/
{0.342300,       1.016300,       500,       1.532188,       0.000000}, /*第15个点*/
{0.343787,       1.054787,       500,       1.532187,       0.000000}, /*第16个点*/
{0.345273,       1.093273,       500,       1.532187,       0.000000}, /*第17个点*/
{0.346760,       1.131760,       500,       1.532187,       0.000000}, /*第18个点*/
{0.348247,       1.170247,       500,       1.532188,       0.000000}, /*第19个点*/
{0.349733,       1.208733,       500,       1.532187,       0.000000}, /*第20个点*/
{0.351220,       1.247220,       500,       1.532187,       0.000000}, /*第21个点*/
{0.352707,       1.285707,       500,       1.532187,       0.000000}, /*第22个点*/
{0.354193,       1.324193,       500,       1.532187,       0.000000}, /*第23个点*/
{0.355680,       1.362680,       500,       1.532188,       0.000000}, /*第24个点*/
{0.357167,       1.401167,       500,       1.532187,       0.000000}, /*第25个点*/
{0.358653,       1.439653,       500,       1.532187,       0.000000}, /*第26个点*/
{0.360140,       1.478140,       500,       1.532187,       0.000000}, /*第27个点*/
{0.361627,       1.516627,       500,       1.532188,       0.000000}, /*第28个点*/
{0.363113,       1.555113,       500,       1.532187,       0.000000}, /*第29个点*/
{0.364600,       1.593600,       500,       1.532187,       0.000000}, /*第30个点*/
{0.366087,       1.632087,       500,       1.532187,       0.000000}, /*第31个点*/
{0.367573,       1.670573,       500,       1.532188,       0.000000}, /*第32个点*/
{0.369060,       1.709060,       500,       1.532187,       0.000000}, /*第33个点*/
{0.370547,       1.747547,       500,       1.532187,       0.000000}, /*第34个点*/
{0.372033,       1.786033,       500,       1.532187,       0.000000}, /*第35个点*/
{0.373520,       1.824520,       500,       1.532187,       0.000000}, /*第36个点*/
{0.375007,       1.863007,       500,       1.532188,       0.000000}, /*第37个点*/
{0.376493,       1.901493,       500,       1.532187,       0.000000}, /*第38个点*/
{0.377980,       1.939980,       500,       1.532187,       0.000000}, /*第39个点*/
{0.379467,       1.978467,       500,       1.532187,       0.000000}, /*第40个点*/
{0.380953,       2.016953,       500,       1.532188,       0.000000}, /*第41个点*/
{0.382440,       2.055440,       500,       1.532187,       0.000000}, /*第42个点*/
{0.383927,       2.093927,       500,       1.532187,       0.000000}, /*第43个点*/
{0.385413,       2.132413,       500,       1.532187,       0.000000}, /*第44个点*/
{0.386900,       2.170900,       500,       1.532188,       0.000000}, /*第45个点*/
{0.388387,       2.209387,       500,       1.532187,       0.000000}, /*第46个点*/
{0.389873,       2.247873,       500,       1.532187,       0.000000}, /*第47个点*/
{0.391360,       2.286360,       500,       1.532187,       0.000000}, /*第48个点*/
{0.392847,       2.324847,       500,       1.532187,       0.000000}, /*第49个点*/
{0.394333,       2.363333,       500,       1.532188,       0.000000}, /*第50个点*/
{0.395820,       2.401820,       500,       1.532187,       0.000000}, /*第51个点*/
{0.397307,       2.440307,       500,       1.532187,       0.000000}, /*第52个点*/
{0.398793,       2.478793,       500,       1.532187,       0.000000}, /*第53个点*/
{0.400280,       2.517280,       500,       1.532188,       0.000000}, /*第54个点*/
{0.401767,       2.555767,       500,       1.532187,       0.000000}, /*第55个点*/
{0.403253,       2.594253,       500,       1.532187,       0.000000}, /*第56个点*/
{0.404740,       2.632740,       500,       1.532187,       0.000000}, /*第57个点*/
{0.406227,       2.671227,       500,       1.532188,       0.000000}, /*第58个点*/
{0.407713,       2.709713,       500,       1.532187,       0.000000}, /*第59个点*/
{0.409200,       2.748200,       500,       1.532187,       0.000000}, /*第60个点*/
{0.410687,       2.786687,       500,       1.532187,       0.000000}, /*第61个点*/
{0.412173,       2.825173,       500,       1.532187,       0.000000}, /*第62个点*/
{0.413660,       2.863660,       500,       1.532188,       0.000000}, /*第63个点*/
{0.415147,       2.902147,       500,       1.532187,       0.000000}, /*第64个点*/
{0.416633,       2.940633,       500,       1.532187,       0.000000}, /*第65个点*/
{0.418120,       2.979120,       500,       1.532187,       0.000000}, /*第66个点*/
{0.419607,       3.017607,       500,       1.532188,       0.000000}, /*第67个点*/
{0.421093,       3.056093,       500,       1.532187,       0.000000}, /*第68个点*/
{0.422580,       3.094580,       500,       1.532187,       0.000000}, /*第69个点*/
{0.424067,       3.133067,       500,       1.532187,       0.000000}, /*第70个点*/
{0.425553,       3.171553,       500,       1.532188,       0.000000}, /*第71个点*/
{0.427040,       3.210040,       500,       1.532187,       0.000000}, /*第72个点*/
{0.428527,       3.248527,       500,       1.532187,       0.000000}, /*第73个点*/
{0.430013,       3.287013,       500,       1.532187,       0.000000}, /*第74个点*/
{0.431500,       3.325500,       500,       1.532187,       0.000000}, /*第75个点*/
{0.432987,       3.363986,       450,       1.532188,       0.000000}, /*第76个点*/
{0.434473,       3.402473,       450,       1.532187,       0.000000}, /*第77个点*/
{0.435960,       3.440960,       450,       1.532187,       0.000000}, /*第78个点*/
{0.437447,       3.479447,       450,       1.532187,       0.000000}, /*第79个点*/
{0.438933,       3.517933,       450,       1.532188,       0.000000}, /*第80个点*/
{0.440420,       3.556420,       450,       1.532187,       0.000000}, /*第81个点*/
{0.441907,       3.594907,       450,       1.532187,       0.000000}, /*第82个点*/
{0.443393,       3.633393,       450,       1.532187,       0.000000}, /*第83个点*/
{0.444880,       3.671880,       450,       1.532188,       0.000000}, /*第84个点*/
{0.446367,       3.710367,       450,       1.532187,       0.000000}, /*第85个点*/
{0.447853,       3.748853,       450,       1.532187,       0.000000}, /*第86个点*/
{0.449340,       3.787340,       450,       1.532187,       0.000000}, /*第87个点*/
{0.450827,       3.825827,       450,       1.532187,       0.000000}, /*第88个点*/
{0.452313,       3.864313,       450,       1.532188,       0.000000}, /*第89个点*/
{0.453800,       3.902800,       450,       1.532187,       0.000000}, /*第90个点*/
{0.455287,       3.941287,       450,       1.532187,       0.000000}, /*第91个点*/
{0.456773,       3.979773,       450,       1.532187,       0.000000}, /*第92个点*/
{0.458260,       4.018260,       450,       1.532188,       0.000000}, /*第93个点*/
{0.459747,       4.056747,       450,       1.532187,       0.000000}, /*第94个点*/
{0.461233,       4.095233,       450,       1.532187,       0.000000}, /*第95个点*/
{0.462720,       4.133720,       450,       1.532187,       0.000000}, /*第96个点*/
{0.464207,       4.172207,       450,       1.532188,       0.000000}, /*第97个点*/
{0.465693,       4.210693,       450,       1.532187,       0.000000}, /*第98个点*/
{0.467180,       4.249180,       450,       1.532187,       0.000000}, /*第99个点*/
{0.468667,       4.287666,       450,       1.532187,       0.000000}, /*第100个点*/
{0.470153,       4.326153,       450,       1.532187,       0.000000}, /*第101个点*/
{0.471640,       4.364640,       450,       1.532188,       0.000000}, /*第102个点*/
{0.473127,       4.403127,       450,       1.532187,       0.000000}, /*第103个点*/
{0.474613,       4.441613,       450,       1.532187,       0.000000}, /*第104个点*/
{0.476100,       4.480100,       450,       1.532187,       0.000000}, /*第105个点*/
{0.477587,       4.518587,       450,       1.532188,       0.000000}, /*第106个点*/
{0.479073,       4.557073,       450,       1.532187,       0.000000}, /*第107个点*/
{0.480560,       4.595560,       450,       1.532187,       0.000000}, /*第108个点*/
{0.482047,       4.634047,       450,       1.532187,       0.000000}, /*第109个点*/
{0.483533,       4.672533,       450,       1.532188,       0.000000}, /*第110个点*/
{0.485020,       4.711020,       450,       1.532187,       0.000000}, /*第111个点*/
{0.486507,       4.749507,       450,       1.532187,       0.000000}, /*第112个点*/
{0.487993,       4.787993,       450,       1.532187,       0.000000}, /*第113个点*/
{0.489480,       4.826480,       450,       1.532187,       0.000000}, /*第114个点*/
{0.490967,       4.864967,       450,       1.532188,       0.000000}, /*第115个点*/
{0.492453,       4.903453,       450,       1.532187,       0.000000}, /*第116个点*/
{0.493940,       4.941940,       450,       1.532187,       0.000000}, /*第117个点*/
{0.495427,       4.980427,       450,       1.532187,       0.000000}, /*第118个点*/
{0.496913,       5.018913,       450,       1.532188,       0.000000}, /*第119个点*/
{0.498400,       5.057400,       450,       1.532187,       0.000000}, /*第120个点*/
{0.499887,       5.095886,       450,       1.532187,       0.000000}, /*第121个点*/
{0.501373,       5.134373,       450,       1.532187,       0.000000}, /*第122个点*/
{0.502860,       5.172860,       450,       1.532188,       0.000000}, /*第123个点*/
{0.504347,       5.211347,       450,       1.532187,       0.000000}, /*第124个点*/
{0.505833,       5.249834,       450,       1.532187,       0.000000}, /*第125个点*/
{0.507320,       5.288320,       450,       1.532187,       0.000000}, /*第126个点*/
{0.508807,       5.326806,       450,       1.532187,       0.000000}, /*第127个点*/
{0.510293,       5.365293,       450,       1.532188,       0.000000}, /*第128个点*/
{0.511780,       5.403780,       450,       1.532187,       0.000000}, /*第129个点*/
{0.513267,       5.442267,       450,       1.532187,       0.000000}, /*第130个点*/
{0.514753,       5.480753,       450,       1.532187,       0.000000}, /*第131个点*/
{0.516240,       5.519240,       450,       1.532188,       0.000000}, /*第132个点*/
{0.517727,       5.557727,       450,       1.532187,       0.000000}, /*第133个点*/
{0.519213,       5.596213,       450,       1.532187,       0.000000}, /*第134个点*/
{0.520700,       5.634700,       450,       1.532187,       0.000000}, /*第135个点*/
{0.522187,       5.673186,       450,       1.532188,       0.000000}, /*第136个点*/
{0.523673,       5.711673,       450,       1.532187,       0.000000}, /*第137个点*/
{0.525160,       5.750160,       450,       1.532187,       0.000000}, /*第138个点*/
{0.526647,       5.788647,       437,       1.532187,       0.000000}, /*第139个点*/
{0.528133,       5.827133,       397,       1.532187,       0.000000}, /*第140个点*/
{0.529620,       5.865620,       357,       1.532188,       0.000000}, /*第141个点*/
{0.531107,       5.904106,       317,       1.532187,       0.000000}, /*第142个点*/
{0.532593,       5.942593,       278,       1.532187,       0.000000}, /*第143个点*/
{0.534080,       5.981080,       238,       1.532187,       0.000000}, /*第144个点*/
{0.535567,       6.019567,       198,       1.532188,       0.000000}, /*第145个点*/
{0.537053,       6.058053,       158,       1.532187,       0.000000}, /*第146个点*/
{0.538540,       6.096540,       119,       1.532187,       0.000000}, /*第147个点*/
{0.540027,       6.135026,       79,       1.532187,       0.000000}, /*第148个点*/
{0.541513,       6.173513,       39,       1.532188,       0.000000}, /*第149个点*/
{0.543000,       6.212000,       0,       1.532188,       0.000000}, /*第150个点*/
{0.000000,       0.000000,       30,       1.532188,       0.000000}, /*第151个点*/
{0.000000,       0.000000,       30,       1.532188,       0.000000}, /*第152个点*/
{0.000000,       0.000000,       30,       1.532188,       0.000000}/*第153个点*/
};
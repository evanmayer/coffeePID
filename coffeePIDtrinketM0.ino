// using Brett Beauregard's excellent PID library 
// with example code from 
// https://playground.arduino.cc/Code/PIDLibraryRelayOutputExample
// thermocouple reading libs
#include "Adafruit_MAX31855.h"
#include <PID_v1.h>
#include <Adafruit_DotStar.h>

// Init the onboard DotStar RGB LED
Adafruit_DotStar onBoard = Adafruit_DotStar(1, INTERNAL_DS_DATA, INTERNAL_DS_CLK, DOTSTAR_BGR);

// Define an array to hold the viridis colormap for the rgb LED
double viridis[256][4] = {
    {0, 0.26700400000000002, 0.0048739999999999999, 0.32941500000000001},
    {0, 0.26851000000000003, 0.0096050000000000007, 0.33542699999999998},
    {0, 0.26994400000000002, 0.014625000000000001, 0.34137899999999999},
    {0, 0.27130500000000002, 0.019942000000000001, 0.34726899999999999},
    {0, 0.272594, 0.025562999999999999, 0.35309299999999999},
    {0, 0.27380900000000002, 0.031496999999999997, 0.35885299999999998},
    {0, 0.27495199999999997, 0.037752000000000001, 0.36454300000000001},
    {0, 0.27602199999999999, 0.044166999999999998, 0.37016399999999999},
    {0, 0.27701799999999999, 0.050344, 0.37571500000000002},
    {0, 0.27794099999999999, 0.056323999999999999, 0.381191},
    {0, 0.27879100000000001, 0.062144999999999999, 0.38659199999999999},
    {0, 0.27956599999999998, 0.067835999999999994, 0.39191700000000002},
    {0, 0.28026699999999999, 0.073416999999999996, 0.39716299999999999},
    {0, 0.28089399999999998, 0.078907000000000005, 0.40232899999999999},
    {0, 0.28144599999999997, 0.084320000000000006, 0.407414},
    {0, 0.28192400000000001, 0.089665999999999996, 0.41241499999999998},
    {0, 0.28232699999999999, 0.094954999999999998, 0.41733100000000001},
    {0, 0.28265600000000002, 0.10019599999999999, 0.42215999999999998},
    {0, 0.28290999999999999, 0.105393, 0.426902},
    {0, 0.28309099999999998, 0.110553, 0.43155399999999999},
    {0, 0.28319699999999998, 0.11568000000000001, 0.43611499999999997},
    {0, 0.28322900000000001, 0.120777, 0.44058399999999998},
    {0, 0.28318700000000002, 0.12584799999999999, 0.44496000000000002},
    {0, 0.28307199999999999, 0.13089500000000001, 0.449241},
    {0, 0.28288400000000002, 0.13592000000000001, 0.45342700000000002},
    {0, 0.28262300000000001, 0.140926, 0.45751700000000001},
    {0, 0.28228999999999999, 0.14591199999999999, 0.46150999999999998},
    {0, 0.281887, 0.15088099999999999, 0.46540500000000001},
    {0, 0.281412, 0.155834, 0.46920099999999998},
    {0, 0.28086800000000001, 0.160771, 0.47289900000000001},
    {0, 0.28025499999999998, 0.16569300000000001, 0.47649799999999998},
    {0, 0.27957399999999999, 0.170599, 0.47999700000000001},
    {0, 0.27882600000000002, 0.17549000000000001, 0.48339700000000002},
    {0, 0.27801199999999998, 0.180367, 0.48669699999999999},
    {0, 0.27713399999999999, 0.185228, 0.489898},
    {0, 0.276194, 0.19007399999999999, 0.49300100000000002},
    {0, 0.27519100000000002, 0.19490499999999999, 0.49600499999999997},
    {0, 0.27412799999999998, 0.19972100000000001, 0.49891099999999999},
    {0, 0.27300600000000003, 0.20452000000000001, 0.50172099999999997},
    {0, 0.27182800000000001, 0.20930299999999999, 0.50443400000000005},
    {0, 0.27059499999999997, 0.21406900000000001, 0.50705199999999995},
    {0, 0.26930799999999999, 0.21881800000000001, 0.50957699999999995},
    {0, 0.26796799999999998, 0.223549, 0.51200800000000002},
    {0, 0.26657999999999998, 0.22826199999999999, 0.51434899999999995},
    {0, 0.26514500000000002, 0.232956, 0.51659900000000003},
    {0, 0.26366299999999998, 0.23763100000000001, 0.51876199999999995},
    {0, 0.26213799999999998, 0.242286, 0.52083699999999999},
    {0, 0.260571, 0.246922, 0.52282799999999996},
    {0, 0.258965, 0.25153700000000001, 0.52473599999999998},
    {0, 0.257322, 0.25613000000000002, 0.526563},
    {0, 0.25564500000000001, 0.26070300000000002, 0.528312},
    {0, 0.25393500000000002, 0.26525399999999999, 0.52998299999999998},
    {0, 0.25219399999999997, 0.269783, 0.53157900000000002},
    {0, 0.25042500000000001, 0.27428999999999998, 0.53310299999999999},
    {0, 0.24862899999999999, 0.278775, 0.53455600000000003},
    {0, 0.246811, 0.28323700000000002, 0.535941},
    {0, 0.244972, 0.28767500000000001, 0.53725999999999996},
    {0, 0.243113, 0.29209200000000002, 0.53851599999999999},
    {0, 0.24123700000000001, 0.296485, 0.53970899999999999},
    {0, 0.239346, 0.30085499999999998, 0.54084399999999999},
    {0, 0.23744100000000001, 0.30520199999999997, 0.54192099999999999},
    {0, 0.23552600000000001, 0.309527, 0.54294399999999998},
    {0, 0.23360300000000001, 0.313828, 0.54391400000000001},
    {0, 0.23167399999999999, 0.318106, 0.54483400000000004},
    {0, 0.229739, 0.32236100000000001, 0.54570600000000002},
    {0, 0.227802, 0.326594, 0.54653200000000002},
    {0, 0.22586300000000001, 0.33080500000000002, 0.54731399999999997},
    {0, 0.22392500000000001, 0.33499400000000001, 0.54805300000000001},
    {0, 0.22198899999999999, 0.33916099999999999, 0.54875200000000002},
    {0, 0.220057, 0.34330699999999997, 0.54941300000000004},
    {0, 0.21812999999999999, 0.34743200000000002, 0.55003800000000003},
    {0, 0.21621000000000001, 0.35153499999999999, 0.55062699999999998},
    {0, 0.21429799999999999, 0.35561900000000002, 0.55118400000000001},
    {0, 0.212395, 0.35968299999999997, 0.55171000000000003},
    {0, 0.210503, 0.36372700000000002, 0.55220599999999997},
    {0, 0.208623, 0.36775200000000002, 0.55267500000000003},
    {0, 0.206756, 0.37175799999999998, 0.55311699999999997},
    {0, 0.204903, 0.37574600000000002, 0.55353300000000005},
    {0, 0.20306299999999999, 0.379716, 0.553925},
    {0, 0.201239, 0.38367000000000001, 0.55429399999999995},
    {0, 0.19943, 0.38760699999999998, 0.55464199999999997},
    {0, 0.19763600000000001, 0.39152799999999999, 0.55496900000000005},
    {0, 0.19586000000000001, 0.39543299999999998, 0.55527599999999999},
    {0, 0.19409999999999999, 0.39932299999999998, 0.55556499999999998},
    {0, 0.192357, 0.40319899999999997, 0.555836},
    {0, 0.19063099999999999, 0.40706100000000001, 0.55608900000000006},
    {0, 0.18892300000000001, 0.41091, 0.55632599999999999},
    {0, 0.18723100000000001, 0.414746, 0.55654700000000001},
    {0, 0.185556, 0.41857, 0.55675300000000005},
    {0, 0.18389800000000001, 0.42238300000000001, 0.55694399999999999},
    {0, 0.182256, 0.42618400000000001, 0.55711999999999995},
    {0, 0.18062900000000001, 0.429975, 0.55728200000000006},
    {0, 0.17901900000000001, 0.43375599999999997, 0.55742999999999998},
    {0, 0.177423, 0.437527, 0.55756499999999998},
    {0, 0.175841, 0.44129000000000002, 0.55768499999999999},
    {0, 0.17427400000000001, 0.445044, 0.55779199999999995},
    {0, 0.17271900000000001, 0.448791, 0.55788499999999996},
    {0, 0.17117599999999999, 0.45252999999999999, 0.55796500000000004},
    {0, 0.16964599999999999, 0.456262, 0.55803000000000003},
    {0, 0.168126, 0.45998800000000001, 0.55808199999999997},
    {0, 0.16661699999999999, 0.46370800000000001, 0.55811900000000003},
    {0, 0.16511700000000001, 0.46742299999999998, 0.558141},
    {0, 0.16362499999999999, 0.47113300000000002, 0.55814799999999998},
    {0, 0.16214200000000001, 0.47483799999999998, 0.55813999999999997},
    {0, 0.160665, 0.47854000000000002, 0.55811500000000003},
    {0, 0.159194, 0.48223700000000003, 0.55807300000000004},
    {0, 0.15772900000000001, 0.48593199999999998, 0.55801299999999998},
    {0, 0.15626999999999999, 0.489624, 0.55793599999999999},
    {0, 0.15481500000000001, 0.493313, 0.55784},
    {0, 0.153364, 0.497, 0.557724},
    {0, 0.151918, 0.50068500000000005, 0.55758700000000005},
    {0, 0.150476, 0.50436899999999996, 0.55742999999999998},
    {0, 0.149039, 0.50805100000000003, 0.55725000000000002},
    {0, 0.14760699999999999, 0.51173299999999999, 0.55704900000000002},
    {0, 0.14618, 0.51541300000000001, 0.55682299999999996},
    {0, 0.144759, 0.51909300000000003, 0.55657199999999996},
    {0, 0.143343, 0.52277300000000004, 0.55629499999999998},
    {0, 0.14193500000000001, 0.52645299999999995, 0.55599100000000001},
    {0, 0.14053599999999999, 0.53013200000000005, 0.55565900000000001},
    {0, 0.13914699999999999, 0.53381199999999995, 0.55529799999999996},
    {0, 0.13777, 0.53749199999999997, 0.55490600000000001},
    {0, 0.136408, 0.54117300000000002, 0.55448299999999995},
    {0, 0.13506599999999999, 0.54485300000000003, 0.55402899999999999},
    {0, 0.133743, 0.54853499999999999, 0.55354099999999995},
    {0, 0.13244400000000001, 0.55221600000000004, 0.55301800000000001},
    {0, 0.13117200000000001, 0.55589900000000003, 0.55245900000000003},
    {0, 0.12993299999999999, 0.55958200000000002, 0.55186400000000002},
    {0, 0.12872900000000001, 0.56326500000000002, 0.55122899999999997},
    {0, 0.12756799999999999, 0.56694900000000004, 0.55055600000000005},
    {0, 0.12645300000000001, 0.57063299999999995, 0.54984100000000002},
    {0, 0.12539400000000001, 0.574318, 0.54908599999999996},
    {0, 0.12439500000000001, 0.57800200000000002, 0.54828699999999997},
    {0, 0.123463, 0.58168699999999995, 0.54744499999999996},
    {0, 0.12260600000000001, 0.58537099999999997, 0.54655699999999996},
    {0, 0.12183099999999999, 0.589055, 0.54562299999999997},
    {0, 0.12114800000000001, 0.59273900000000002, 0.54464100000000004},
    {0, 0.12056500000000001, 0.59642200000000001, 0.54361099999999996},
    {0, 0.120092, 0.60010399999999997, 0.54252999999999996},
    {0, 0.119738, 0.60378500000000002, 0.54139999999999999},
    {0, 0.11951199999999999, 0.607464, 0.54021799999999998},
    {0, 0.119423, 0.61114100000000005, 0.53898199999999996},
    {0, 0.11948300000000001, 0.61481699999999995, 0.53769199999999995},
    {0, 0.119699, 0.61848999999999998, 0.53634700000000002},
    {0, 0.12008099999999999, 0.62216099999999996, 0.53494600000000003},
    {0, 0.120638, 0.62582800000000005, 0.53348799999999996},
    {0, 0.12138, 0.62949200000000005, 0.53197300000000003},
    {0, 0.122312, 0.63315299999999997, 0.53039800000000004},
    {0, 0.123444, 0.63680899999999996, 0.52876299999999998},
    {0, 0.12478, 0.64046099999999995, 0.52706799999999998},
    {0, 0.12632599999999999, 0.64410699999999999, 0.52531099999999997},
    {0, 0.12808700000000001, 0.64774900000000002, 0.52349100000000004},
    {0, 0.13006699999999999, 0.65138399999999996, 0.52160799999999996},
    {0, 0.132268, 0.65501399999999999, 0.51966100000000004},
    {0, 0.13469200000000001, 0.658636, 0.51764900000000003},
    {0, 0.13733899999999999, 0.66225199999999995, 0.515571},
    {0, 0.14021, 0.66585899999999998, 0.51342699999999997},
    {0, 0.14330300000000001, 0.66945900000000003, 0.51121499999999997},
    {0, 0.146616, 0.67305000000000004, 0.50893600000000006},
    {0, 0.150148, 0.67663099999999998, 0.50658899999999996},
    {0, 0.153894, 0.680203, 0.50417199999999995},
    {0, 0.15785099999999999, 0.68376499999999996, 0.50168599999999997},
    {0, 0.16201599999999999, 0.68731600000000004, 0.49912899999999999},
    {0, 0.166383, 0.69085600000000003, 0.496502},
    {0, 0.17094799999999999, 0.694384, 0.49380299999999999},
    {0, 0.175707, 0.69789999999999996, 0.491033},
    {0, 0.18065300000000001, 0.70140199999999997, 0.48818899999999998},
    {0, 0.185783, 0.70489100000000005, 0.48527300000000001},
    {0, 0.19109000000000001, 0.70836600000000005, 0.48228399999999999},
    {0, 0.196571, 0.71182699999999999, 0.47922100000000001},
    {0, 0.20221900000000001, 0.71527200000000002, 0.47608400000000001},
    {0, 0.20802999999999999, 0.71870100000000003, 0.47287299999999999},
    {0, 0.214, 0.72211400000000003, 0.46958800000000001},
    {0, 0.22012399999999999, 0.72550899999999996, 0.46622599999999997},
    {0, 0.22639699999999999, 0.72888799999999998, 0.46278900000000001},
    {0, 0.23281499999999999, 0.73224699999999998, 0.45927699999999999},
    {0, 0.239374, 0.73558800000000002, 0.45568799999999998},
    {0, 0.24607000000000001, 0.73890999999999996, 0.45202399999999998},
    {0, 0.25289899999999998, 0.74221099999999995, 0.44828400000000002},
    {0, 0.259857, 0.74549200000000004, 0.444467},
    {0, 0.26694099999999998, 0.74875100000000006, 0.44057299999999999},
    {0, 0.27414899999999998, 0.75198799999999999, 0.43660100000000002},
    {0, 0.28147699999999998, 0.75520299999999996, 0.43255199999999999},
    {0, 0.28892099999999998, 0.75839400000000001, 0.42842599999999997},
    {0, 0.29647899999999999, 0.76156100000000004, 0.42422300000000002},
    {0, 0.30414799999999997, 0.76470400000000005, 0.41994300000000001},
    {0, 0.31192500000000001, 0.767822, 0.41558600000000001},
    {0, 0.31980900000000001, 0.77091399999999999, 0.41115200000000002},
    {0, 0.32779599999999998, 0.77398, 0.40664},
    {0, 0.33588499999999999, 0.77701799999999999, 0.40204899999999999},
    {0, 0.34407399999999999, 0.78002899999999997, 0.39738099999999998},
    {0, 0.35236000000000001, 0.78301100000000001, 0.39263599999999999},
    {0, 0.36074099999999998, 0.785964, 0.38781399999999999},
    {0, 0.36921399999999999, 0.78888800000000003, 0.38291399999999998},
    {0, 0.37777899999999998, 0.79178099999999996, 0.37793900000000002},
    {0, 0.38643300000000003, 0.79464400000000002, 0.372886},
    {0, 0.39517400000000003, 0.79747500000000004, 0.367757},
    {0, 0.404001, 0.80027499999999996, 0.36255199999999999},
    {0, 0.41291299999999997, 0.803041, 0.357269},
    {0, 0.42190800000000001, 0.80577399999999999, 0.35191},
    {0, 0.430983, 0.808473, 0.34647600000000001},
    {0, 0.440137, 0.81113800000000003, 0.34096700000000002},
    {0, 0.44936799999999999, 0.81376800000000005, 0.33538400000000002},
    {0, 0.45867400000000003, 0.81636299999999995, 0.32972699999999999},
    {0, 0.468053, 0.81892100000000001, 0.32399800000000001},
    {0, 0.47750399999999998, 0.82144399999999995, 0.31819500000000001},
    {0, 0.48702600000000001, 0.82392900000000002, 0.31232100000000002},
    {0, 0.49661499999999997, 0.826376, 0.30637700000000001},
    {0, 0.50627100000000003, 0.82878600000000002, 0.30036200000000002},
    {0, 0.51599200000000001, 0.83115799999999995, 0.29427900000000001},
    {0, 0.52577600000000002, 0.83349099999999998, 0.28812700000000002},
    {0, 0.53562100000000001, 0.835785, 0.28190799999999999},
    {0, 0.54552400000000001, 0.83803899999999998, 0.27562599999999998},
    {0, 0.55548399999999998, 0.84025399999999995, 0.26928099999999999},
    {0, 0.56549799999999995, 0.84243000000000001, 0.26287700000000003},
    {0, 0.57556300000000005, 0.84456600000000004, 0.256415},
    {0, 0.58567800000000003, 0.846661, 0.24989700000000001},
    {0, 0.59583900000000001, 0.84871700000000005, 0.24332899999999999},
    {0, 0.60604499999999994, 0.85073299999999996, 0.23671200000000001},
    {0, 0.61629299999999998, 0.85270900000000005, 0.23005200000000001},
    {0, 0.626579, 0.85464499999999999, 0.223353},
    {0, 0.63690199999999997, 0.85654200000000003, 0.21662000000000001},
    {0, 0.64725699999999997, 0.85840000000000005, 0.20986099999999999},
    {0, 0.65764199999999995, 0.86021899999999996, 0.20308200000000001},
    {0, 0.66805400000000004, 0.86199899999999996, 0.196293},
    {0, 0.67848900000000001, 0.86374200000000001, 0.189503},
    {0, 0.688944, 0.865448, 0.182725},
    {0, 0.69941500000000001, 0.86711700000000003, 0.17597099999999999},
    {0, 0.70989800000000003, 0.86875100000000005, 0.16925699999999999},
    {0, 0.720391, 0.87034999999999996, 0.162603},
    {0, 0.73088900000000001, 0.87191600000000002, 0.156029},
    {0, 0.74138800000000005, 0.87344900000000003, 0.149561},
    {0, 0.751884, 0.87495100000000003, 0.14322799999999999},
    {0, 0.76237299999999997, 0.87642399999999998, 0.13706399999999999},
    {0, 0.77285199999999998, 0.87786799999999998, 0.131109},
    {0, 0.78331499999999998, 0.87928499999999998, 0.12540499999999999},
    {0, 0.79376000000000002, 0.88067799999999996, 0.120005},
    {0, 0.80418199999999995, 0.882046, 0.114965},
    {0, 0.81457599999999997, 0.88339299999999998, 0.110347},
    {0, 0.82494000000000001, 0.88471999999999995, 0.10621700000000001},
    {0, 0.83526999999999996, 0.88602899999999996, 0.102646},
    {0, 0.84556100000000001, 0.88732200000000006, 0.099701999999999999},
    {0, 0.85580999999999996, 0.88860099999999997, 0.097451999999999997},
    {0, 0.86601300000000003, 0.88986799999999999, 0.095952999999999997},
    {0, 0.87616799999999995, 0.89112499999999994, 0.095250000000000001},
    {0, 0.88627100000000003, 0.892374, 0.095374},
    {0, 0.89632000000000001, 0.89361599999999997, 0.096335000000000004},
    {0, 0.90631099999999998, 0.89485499999999996, 0.098125000000000004},
    {0, 0.916242, 0.89609099999999997, 0.100717},
    {0, 0.92610599999999998, 0.89732999999999996, 0.104071},
    {0, 0.93590399999999996, 0.89856999999999998, 0.108131},
    {0, 0.94563600000000003, 0.89981500000000003, 0.11283799999999999},
    {0, 0.95530000000000004, 0.901065, 0.118128},
    {0, 0.96489400000000003, 0.90232299999999999, 0.123941},
    {0, 0.97441699999999998, 0.90359, 0.130215},
    {0, 0.98386799999999996, 0.90486699999999998, 0.13689699999999999},
    {0, 0.99324800000000002, 0.90615699999999999, 0.14393600000000001}
};
// Define the range of temperatures
double T_RGB_MIN = 70.;  // F
double T_RGB_MAX = 240.; // F

// Create a thermocouple instance using the appropriate M0 pins
#define MAXDO   2
#define MAXCS   1
#define MAXCLK  3

// Define variables for PID
double setPoint;
double input;
double output;

volatile long onTime = 0;

// kettle tuning params
//double kP = 78.;
//double kI = 0.92;
//double kD = 16.;

// gaggia tuning params
//double kP = 78.;
double kP = 70.;
//double kI = 1.4;
double kI = 3.4;
//double kD = 19.;
double kD = 44.8;

// Initialize a PID, with initial tuning params k_p, k_i, k_d
PID coffeePID(&input, &output, &setPoint, kP, kI, kD, DIRECT);

// a window size as a basis for relay "on time"
int windowSize = 2500; // ms
unsigned long windowStartTime;

// initialize the thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

void setup() {
  //////////////////////////////////
  // pin setup                    //
  //////////////////////////////////
  // init GPIO 4 as output, and mirror with 13 for signaling ON
  // this will control the relay.
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  onBoard.begin();
  onBoard.show();

  //////////////////////////////////
  // PID setup                    //
  //////////////////////////////////
  // ############################ //
  // setpoint                     //
  // ############################ //
  setPoint = 225.; 
    
  windowStartTime = millis();
  coffeePID.SetTunings(kP, kI, kD);
  //coffeePID.SetSampleTime(1000);
  coffeePID.SetOutputLimits(0, windowSize);
  coffeePID.SetMode(AUTOMATIC);
  
  //////////////////////////////////
  // serial setup                 //
  //////////////////////////////////
  //Serial.begin(9600);
  //while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc
  // wait for MAX chip to stabilize
  //delay(500);
}

void loop() {
   double temp = thermocouple.readFarenheit();
   if (isnan(temp)) {
     Serial.println("Something wrong with thermocouple!");
   } else {
     // update input var and compute PID 
     input = temp;
     coffeePID.Compute();
     
     onTime = output;
     DriveOutput();

       // Limit the temps for colormap lookup
     double rgb_temp = 0.0;
     if ( temp > T_RGB_MAX ) {
       rgb_temp = T_RGB_MAX;
     } else if ( temp < T_RGB_MIN ) {
       rgb_temp = T_RGB_MIN;
     } else {
       rgb_temp = temp;
     }
     // Scale the temp to an int between 0 and 256
     rgb_temp -= T_RGB_MIN;
     rgb_temp *= 256. / (T_RGB_MAX-T_RGB_MIN);
     int viridis_idx = int (rgb_temp);
     onBoard.setPixelColor(viridis[viridis_idx][0], \
                           viridis[viridis_idx][1]*255., \
                           viridis[viridis_idx][2]*255., \
                           viridis[viridis_idx][3]*255.);
     onBoard.setBrightness(100);
     // If temp is within range of setpoint, set to full green
     double thresh = 2.; // F
     if ( (setPoint - thresh < temp) && (temp < setPoint + thresh) ){
       onBoard.setPixelColor(0, 0, 255, 0);
       onBoard.setBrightness(128);
     }
     onBoard.show();

     Serial.print(setPoint);
     Serial.print(',');
     Serial.println(temp);
     Serial.print(setPoint-thresh);
     Serial.print(',');
     Serial.print(setPoint+thresh);
     Serial.print(',');
   }
   //delay(500);
}

// Controls relay pin level and judges pulse width window
void DriveOutput() {
     // turn the LED/output pin on/off
     unsigned long now = millis();
     if ((now - windowStartTime) > windowSize) {
          // amt to shift relay's window time
         windowStartTime += windowSize;
     }
     
     if ((onTime > 100) && (onTime > (now - windowStartTime))) {
         digitalWrite(4, HIGH);
         digitalWrite(13, HIGH);
         //Serial.println("HI");
     } else {
         digitalWrite(4, LOW);
         digitalWrite(13, LOW);
         //Serial.println("LOW");
     }

}

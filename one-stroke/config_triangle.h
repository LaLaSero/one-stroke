#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <avr/pgmspace.h>

#define NUM_POINT 400
#define CPR_COUNT 360
#define GEAR_RATIO 1.0

#define L0 150
#define L1 90
#define L2 140

#define P_GAIN 35
#define I_GAIN 0.05
#define D_GAIN 0.12

#define rightEncoderCount_initial 64
#define leftEncoderCount_initial 116
#define y_offset 0

// #define rightEncoderCount_initial 72
// #define leftEncoderCount_initial 108
// #define y_offset 20


const unsigned long targetUpdateInterval = 100;
const unsigned long controlLoopInterval = 5; 

// モータ制御ピン（右側）
const int right_IN1 = 7;
const int right_IN2 = 6;
const int right_OUT1 = 18; // エンコーダーA
const int right_OUT2 = 19; // エンコーダーB

// モータ制御ピン（左側）
const int left_IN1 = 9;
const int left_IN2 = 8;
const int left_OUT1 = 20; // エンコーダーA
const int left_OUT2 = 21; // エンコーダーB

// エンコーダーの設定
const int CPR = CPR_COUNT;        // エンコーダーのカウント数（例）
const float GearRatio = GEAR_RATIO; // ギア比（例）

volatile long rightEncoderCount = 0;
volatile long leftEncoderCount = 0;

// PID制御用のパラメータ
float Kp = P_GAIN;
float Ki = I_GAIN;
float Kd = D_GAIN;

// PID制御用の変数（右側）
float integral_right = 0;
float prevError_right = 0;

// PID制御用の変数（左側）
float integral_left = 0;
float prevError_left = 0;

// 制御周期
unsigned long prevTime = 0;
// const unsigned long controlInterval = CONTROL_INTERVAL; // ミリ秒
unsigned long prevTimeTargetUpdate = 0;
unsigned long prevTimeControlLoop = 0;

// 目標位置（ペン先の座標）
float x_target = 0; // 単位：mm
float y_target = 0;  // 単位：mm


// モーターの目標角度（degrees）
float theta_target_right = 0.0;
float theta_target_left = 0.0;

float theta_current_right = 0;
float theta_current_left = 0;

// リンクの長さ
const float l0 = L0; // モーター間の距離（mm）
const float l1 = L1; // 第1リンクの長さ（mm）
const float l2 = L2; // 第2リンクの長さ（mm）

const int numPoints = NUM_POINT;

const float x_targets[NUM_POINT] PROGMEM = {
7.500000000000000000e+01,7.397959183673469852e+01,7.295918367346938282e+01,7.193877551020408134e+01,7.091836734693877986e+01,6.989795918367346417e+01,6.887755102040816269e+01,6.785714285714286120e+01,6.683673469387754551e+01,6.581632653061224403e+01,6.479591836734694255e+01,6.377551020408163396e+01,6.275510204081632537e+01,6.173469387755102389e+01,6.071428571428571530e+01,5.969387755102040671e+01,5.867346938775510523e+01,5.765306122448979664e+01,5.663265306122448806e+01,5.561224489795918657e+01,5.459183673469387799e+01,5.357142857142856940e+01,5.255102040816326792e+01,5.153061224489795933e+01,5.051020408163265074e+01,4.948979591836734926e+01,4.846938775510204067e+01,4.744897959183673208e+01,4.642857142857143060e+01,4.540816326530612201e+01,4.438775510204081343e+01,4.336734693877551194e+01,4.234693877551020336e+01,4.132653061224489477e+01,4.030612244897959329e+01,3.928571428571428470e+01,3.826530612244897611e+01,3.724489795918367463e+01,3.622448979591836604e+01,3.520408163265305745e+01,3.418367346938775597e+01,3.316326530612244738e+01,3.214285714285713880e+01,3.112244897959183731e+01,3.010204081632652873e+01,2.908163265306122014e+01,2.806122448979591866e+01,2.704081632653061007e+01,2.602040816326530148e+01,2.500000000000000000e+01,2.500000000000000000e+01,2.704081632653061362e+01,2.908163265306122369e+01,3.112244897959183731e+01,3.316326530612244738e+01,3.520408163265305745e+01,3.724489795918367463e+01,3.928571428571428470e+01,4.132653061224489477e+01,4.336734693877551194e+01,4.540816326530612201e+01,4.744897959183673208e+01,4.948979591836734926e+01,5.153061224489795933e+01,5.357142857142856940e+01,5.561224489795918657e+01,5.765306122448979664e+01,5.969387755102040671e+01,6.173469387755102389e+01,6.377551020408163396e+01,6.581632653061224403e+01,6.785714285714286120e+01,6.989795918367346417e+01,7.193877551020408134e+01,7.397959183673469852e+01,7.602040816326530148e+01,7.806122448979591866e+01,8.010204081632653583e+01,8.214285714285713880e+01,8.418367346938775597e+01,8.622448979591837315e+01,8.826530612244897611e+01,9.030612244897959329e+01,9.234693877551021046e+01,9.438775510204081343e+01,9.642857142857143060e+01,9.846938775510204778e+01,1.005102040816326507e+02,1.025510204081632679e+02,1.045918367346938851e+02,1.066326530612244881e+02,1.086734693877551052e+02,1.107142857142857224e+02,1.127551020408163254e+02,1.147959183673469425e+02,1.168367346938775597e+02,1.188775510204081627e+02,1.209183673469387799e+02,1.229591836734693970e+02,1.250000000000000000e+02,1.250000000000000000e+02,1.239795918367346985e+02,1.229591836734693828e+02,1.219387755102040813e+02,1.209183673469387799e+02,1.198979591836734642e+02,1.188775510204081627e+02,1.178571428571428612e+02,1.168367346938775455e+02,1.158163265306122440e+02,1.147959183673469425e+02,1.137755102040816269e+02,1.127551020408163254e+02,1.117346938775510239e+02,1.107142857142857082e+02,1.096938775510204067e+02,1.086734693877551052e+02,1.076530612244897895e+02,1.066326530612244881e+02,1.056122448979591866e+02,1.045918367346938851e+02,1.035714285714285694e+02,1.025510204081632679e+02,1.015306122448979522e+02,1.005102040816326507e+02,9.948979591836734926e+01,9.846938775510204778e+01,9.744897959183673208e+01,9.642857142857143060e+01,9.540816326530611491e+01,9.438775510204081343e+01,9.336734693877551194e+01,9.234693877551021046e+01,9.132653061224489477e+01,9.030612244897959329e+01,8.928571428571427759e+01,8.826530612244897611e+01,8.724489795918367463e+01,8.622448979591837315e+01,8.520408163265305745e+01,8.418367346938775597e+01,8.316326530612244028e+01,8.214285714285713880e+01,8.112244897959183731e+01,8.010204081632653583e+01,7.908163265306122014e+01,7.806122448979591866e+01,7.704081632653060296e+01,7.602040816326530148e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.591130553523197477e+01,7.682170266128767366e+01,7.773028387451283550e+01,7.863614348139464028e+01,7.953837850137658450e+01,8.043608956696928658e+01,8.132838182025945173e+01,8.221436580492371604e+01,8.309315835285812568e+01,8.396388346453920803e+01,8.482567318223941299e+01,8.567766845522623953e+01,8.651901999608267602e+01,8.734888912729537935e+01,8.816644861726663862e+01,8.897088350491685560e+01,8.976139191205544421e+01,9.053718584271049963e+01,9.129749196862019289e+01,9.204155240010339867e+01,9.276862544154040791e+01,9.347798633071140273e+01,9.416892796125512177e+01,9.484076158752787933e+01,9.549281751116019734e+01,9.612444574862672653e+01,9.673501667916390545e+01,9.732392167238971581e+01,9.789057369499958838e+01,9.843440789593404361e+01,9.895488216943448379e+01,9.945147769542597871e+01,9.992369945668842490e+01,1.003710767323004944e+02,1.007931635668644503e+02,1.011895392150441921e+02,1.015598085609733801e+02,1.019036025121155120e+02,1.022205783671833643e+02,1.025104201577511276e+02,1.027728389632185895e+02,1.030075731988134748e+02,1.032143888763447990e+02,1.033930798374474733e+02,1.035434679590853335e+02,1.036654033311081662e+02,1.037587644056853264e+02,1.038234581184672720e+02,1.038594199813541792e+02,1.038666141467790425e+02,1.038450334434413094e+02,1.037946993834553808e+02,1.037156621409069146e+02,1.036080005018381485e+02,1.034718217857122937e+02,1.033072617384351588e+02,1.031144843970406697e+02,1.028936819261751623e+02,1.026450744265435588e+02,1.023689097155080958e+02,1.020654630800586773e+02,1.017350370024007589e+02,1.013779608584345766e+02,1.009945905894259681e+02,1.005853083471965022e+02,1.001505221131861560e+02,9.969066529176856761e+01,9.920619627822414088e+01,9.869759800180162301e+01,9.816537744432380919e+01,9.761006513481689240e+01,9.703221462066775871e+01,9.643240191583579701e+01,9.581122492666970913e+01,9.516930285590142091e+01,9.450727558541139217e+01,9.382580303838037139e+01,9.312556452146365871e+01,9.240725804764348084e+01,9.167159964043440823e+01,9.091932262013561683e+01,9.015117687284116244e+01,8.936792810293731293e+01,8.857035706983177192e+01,8.775925880967561454e+01,8.693544184285403276e+01,8.609972736803567273e+01,8.525294844358381852e+01,8.439594915714584999e+01,8.352958378424820296e+01,8.265471593673599671e+01,8.177221770190594441e+01,8.088296877319079670e+01,7.998785557326176843e+01,7.908777037042317204e+01,7.818361038917991834e+01,7.727627691586468472e+01,7.636667440021604136e+01,7.545570955380328826e+01,7.454429044619669753e+01,7.363332559978395864e+01,7.272372308413531528e+01,7.181638961082008166e+01,7.091222962957682796e+01,7.001214442673821736e+01,6.911703122680920330e+01,6.822778229809405559e+01,6.734528406326400329e+01,6.647041621575179704e+01,6.560405084285413579e+01,6.474705155641618148e+01,6.390027263196433438e+01,6.306455815714596724e+01,6.224074119032439256e+01,6.142964293016821387e+01,6.063207189706267286e+01,5.984882312715884467e+01,5.908067737986439028e+01,5.832840035956559177e+01,5.759274195235651916e+01,5.687443547853633419e+01,5.617419696161963572e+01,5.549272441458860783e+01,5.483069714409857909e+01,5.418877507333029087e+01,5.356759808416418878e+01,5.296778537933224129e+01,5.238993486518310050e+01,5.183462255567619081e+01,5.130240199819836988e+01,5.079380372177585912e+01,5.030933470823143239e+01,4.984947788681384395e+01,4.941469165280349785e+01,4.900540941057403188e+01,4.862203914156542339e+01,4.826496299759924113e+01,4.793453691994132271e+01,4.763109028449190419e+01,4.735492557345644826e+01,4.710631807382483061e+01,4.688551560295933029e+01,4.669273826156483409e+01,4.652817821428769918e+01,4.639199949816185153e+01,4.628433785909308540e+01,4.620530061654461917e+01,4.615496655655869063e+01,4.613338585322095753e+01,4.614058001864582081e+01,4.617654188153272798e+01,4.624123559431468067e+01,4.633459666889183381e+01,4.645653204091466648e+01,4.660692016255252668e+01,4.678561112365520103e+01,4.699242680118652515e+01,4.722716103678141053e+01,4.748957984224886530e+01,4.777942163281663568e+01,4.809639748788448799e+01,4.844019143902661995e+01,4.881046078495580787e+01,4.920683643313554967e+01,4.962892326769950557e+01,5.007630054331158220e+01,5.054852230457402129e+01,5.104511783056552332e+01,5.156559210406594218e+01,5.210942630500041162e+01,5.267607832761029840e+01,5.326498332083609455e+01,5.387555425137328768e+01,5.450718248883980266e+01,5.515923841247212067e+01,5.583107203874487112e+01,5.652201366928859727e+01,5.723137455845959920e+01,5.795844759989660133e+01,5.870250803137980711e+01,5.946281415728950037e+01,6.023860808794454869e+01,6.102911649508315861e+01,6.183355138273336138e+01,6.265111087270462775e+01,6.348098000391730977e+01,6.432233154477376047e+01,6.517432681776058701e+01,6.603611653546079197e+01,6.690684164714187432e+01,6.778563419507626975e+01,6.867161817974056248e+01,6.956391043303072763e+01,7.046162149862341550e+01,7.136385651860537394e+01,7.226971612548715029e+01,7.317829733871232634e+01,7.408869446476803944e+01,7.500000000000000000e+01
  };

const float y_targets[NUM_POINT] PROGMEM = {
1.616025403784438481e+02,1.598351415952103025e+02,1.580677428119767569e+02,1.563003440287432113e+02,1.545329452455096657e+02,1.527655464622761201e+02,1.509981476790425745e+02,1.492307488958090005e+02,1.474633501125754549e+02,1.456959513293419093e+02,1.439285525461083637e+02,1.421611537628748181e+02,1.403937549796412725e+02,1.386263561964077269e+02,1.368589574131741813e+02,1.350915586299406357e+02,1.333241598467070901e+02,1.315567610634735445e+02,1.297893622802399989e+02,1.280219634970064249e+02,1.262545647137728935e+02,1.244871659305393337e+02,1.227197671473057881e+02,1.209523683640722425e+02,1.191849695808386969e+02,1.174175707976051513e+02,1.156501720143716057e+02,1.138827732311380601e+02,1.121153744479045145e+02,1.103479756646709689e+02,1.085805768814374233e+02,1.068131780982038634e+02,1.050457793149703178e+02,1.032783805317367722e+02,1.015109817485032124e+02,9.974358296526966683e+01,9.797618418203612123e+01,9.620878539880257563e+01,9.444138661556903003e+01,9.267398783233548443e+01,9.090658904910193883e+01,8.913919026586839323e+01,8.737179148263483341e+01,8.560439269940128781e+01,8.383699391616774221e+01,8.206959513293419661e+01,8.030219634970065101e+01,7.853479756646710541e+01,7.676739878323354560e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.676739878323354560e+01,7.853479756646709120e+01,8.030219634970063680e+01,8.206959513293419661e+01,8.383699391616774221e+01,8.560439269940128781e+01,8.737179148263483341e+01,8.913919026586837902e+01,9.090658904910192462e+01,9.267398783233548443e+01,9.444138661556903003e+01,9.620878539880257563e+01,9.797618418203612123e+01,9.974358296526966683e+01,1.015109817485032124e+02,1.032783805317367580e+02,1.050457793149703178e+02,1.068131780982038634e+02,1.085805768814374090e+02,1.103479756646709546e+02,1.121153744479045145e+02,1.138827732311380601e+02,1.156501720143716057e+02,1.174175707976051513e+02,1.191849695808386969e+02,1.209523683640722425e+02,1.227197671473057881e+02,1.244871659305393337e+02,1.262545647137728793e+02,1.280219634970064249e+02,1.297893622802399705e+02,1.315567610634735161e+02,1.333241598467070617e+02,1.350915586299406357e+02,1.368589574131741813e+02,1.386263561964077269e+02,1.403937549796412725e+02,1.421611537628748181e+02,1.439285525461083637e+02,1.456959513293419093e+02,1.474633501125754549e+02,1.492307488958090289e+02,1.509981476790425745e+02,1.527655464622761201e+02,1.545329452455096657e+02,1.563003440287432113e+02,1.580677428119767569e+02,1.598351415952103025e+02,1.616025403784438481e+02,1.616025403784438481e+02,1.598351415952103025e+02,1.580677428119767569e+02,1.563003440287432113e+02,1.545329452455096657e+02,1.527655464622761201e+02,1.509981476790425745e+02,1.492307488958090005e+02,1.474633501125754549e+02,1.456959513293419093e+02,1.439285525461083637e+02,1.421611537628748181e+02,1.403937549796412725e+02,1.386263561964077269e+02,1.368589574131741813e+02,1.350915586299406357e+02,1.333241598467070901e+02,1.315567610634735445e+02,1.297893622802399989e+02,1.280219634970064249e+02,1.262545647137728935e+02,1.244871659305393337e+02,1.227197671473057881e+02,1.209523683640722425e+02,1.191849695808386969e+02,1.174175707976051513e+02,1.156501720143716057e+02,1.138827732311380601e+02,1.121153744479045145e+02,1.103479756646709689e+02,1.085805768814374233e+02,1.068131780982038634e+02,1.050457793149703178e+02,1.032783805317367722e+02,1.015109817485032124e+02,9.974358296526966683e+01,9.797618418203612123e+01,9.620878539880257563e+01,9.444138661556903003e+01,9.267398783233548443e+01,9.090658904910193883e+01,8.913919026586839323e+01,8.737179148263483341e+01,8.560439269940128781e+01,8.383699391616774221e+01,8.206959513293419661e+01,8.030219634970065101e+01,7.853479756646710541e+01,7.676739878323354560e+01,7.500000000000000000e+01,7.500000000000000000e+01,7.501438788260853130e+01,7.505753718827983789e+01,7.512940490484761824e+01,7.522991939300906949e+01,7.535898045773642195e+01,7.551645944815356870e+01,7.570219938577808705e+01,7.591601512100073990e+01,7.615769351764666339e+01,7.642699366543412509e+01,7.672364712011919607e+01,7.704735817108669949e+01,7.739780413612110976e+01,7.777463568306306740e+01,7.817747717803140972e+01,7.860592705986307749e+01,7.905955824039808988e+01,7.953791853021026270e+01,8.004053108935951855e+01,8.056689490271624265e+01,8.111648527938410780e+01,8.168875437572333453e+01,8.228313174145320374e+01,8.289902488828923310e+01,8.353581988054843066e+01,8.419288194713371354e+01,8.486955611428754764e+01,8.556516785848405959e+01,8.627902377880869267e+01,8.701041228815537920e+01,8.775860432255201715e+01,8.852285406790730349e+01,8.930239970345442657e+01,9.009646416115060674e+01,9.090425590027534497e+01,9.172496969645553122e+01,9.255778744433050065e+01,9.340187897305737863e+01,9.425640287384329952e+01,9.512050733868008479e+01,9.599333100944492969e+01,9.687400383652095570e+01,9.776164794608156683e+01,9.865537851517416357e+01,9.955430465373096638e+01,1.004575302926274816e+02,1.013641550769038702e+02,1.022732752632581708e+02,1.031839846209173999e+02,1.040953753349880628e+02,1.050065389113857606e+02,1.059165670824418726e+02,1.068245527122845004e+02,1.077295907010912686e+02,1.086307788873125020e+02,1.095272189469654194e+02,1.104180172891031191e+02,1.113022859465653625e+02,1.121791434611236298e+02,1.130477157621378126e+02,1.139071370378488837e+02,1.147565505984387642e+02,1.155951097299975601e+02,1.164219785385461989e+02,1.172363327832738946e+02,1.180373606981591905e+02,1.188242638011560643e+02,1.195962576901381453e+02,1.203525728248078508e+02,1.210924552937908345e+02,1.218151675661513451e+02,1.225199892265789998e+02,1.232062176935144322e+02,1.238731689194979566e+02,1.245201780730428993e+02,1.251466002013540759e+02,1.257518108732307667e+02,1.263352068015133369e+02,1.268962064444529858e+02,1.274342505854052092e+02,1.279488028902690928e+02,1.284393504421168473e+02,1.289054042524805652e+02,1.293464997487864423e+02,1.297621972374510051e+02,1.301520823421771240e+02,1.305157664170133103e+02,1.308528869337644664e+02,1.311631078433679249e+02,1.314461199108744154e+02,1.317016410237002333e+02,1.319294164728431156e+02,1.321292192067817268e+02,1.323008500578057181e+02,1.324441379405501493e+02,1.325589400225373424e+02,1.326451418665553206e+02,1.327026575447312666e+02,1.327314297241863699e+02,1.327314297241863699e+02,1.327026575447312666e+02,1.326451418665553206e+02,1.325589400225373424e+02,1.324441379405501493e+02,1.323008500578057181e+02,1.321292192067817268e+02,1.319294164728431156e+02,1.317016410237002333e+02,1.314461199108744438e+02,1.311631078433679249e+02,1.308528869337644664e+02,1.305157664170133103e+02,1.301520823421771240e+02,1.297621972374510335e+02,1.293464997487864423e+02,1.289054042524805368e+02,1.284393504421168473e+02,1.279488028902690928e+02,1.274342505854052092e+02,1.268962064444529858e+02,1.263352068015133369e+02,1.257518108732307667e+02,1.251466002013540759e+02,1.245201780730428993e+02,1.238731689194979566e+02,1.232062176935144180e+02,1.225199892265789998e+02,1.218151675661513593e+02,1.210924552937908345e+02,1.203525728248078508e+02,1.195962576901381311e+02,1.188242638011560643e+02,1.180373606981591905e+02,1.172363327832738804e+02,1.164219785385461989e+02,1.155951097299975459e+02,1.147565505984387784e+02,1.139071370378488695e+02,1.130477157621378126e+02,1.121791434611236298e+02,1.113022859465653482e+02,1.104180172891031191e+02,1.095272189469654194e+02,1.086307788873124878e+02,1.077295907010912686e+02,1.068245527122844862e+02,1.059165670824418726e+02,1.050065389113857464e+02,1.040953753349880770e+02,1.031839846209173999e+02,1.022732752632581565e+02,1.013641550769038702e+02,1.004575302926274816e+02,9.955430465373096638e+01,9.865537851517416357e+01,9.776164794608155262e+01,9.687400383652095570e+01,9.599333100944492969e+01,9.512050733868009900e+01,9.425640287384329952e+01,9.340187897305736442e+01,9.255778744433050065e+01,9.172496969645551701e+01,9.090425590027535918e+01,9.009646416115060674e+01,8.930239970345442657e+01,8.852285406790730349e+01,8.775860432255200294e+01,8.701041228815537920e+01,8.627902377880869267e+01,8.556516785848404538e+01,8.486955611428756185e+01,8.419288194713371354e+01,8.353581988054844487e+01,8.289902488828923310e+01,8.228313174145320374e+01,8.168875437572333453e+01,8.111648527938409359e+01,8.056689490271624265e+01,8.004053108935951855e+01,7.953791853021027691e+01,7.905955824039808988e+01,7.860592705986307749e+01,7.817747717803140972e+01,7.777463568306306740e+01,7.739780413612110976e+01,7.704735817108669949e+01,7.672364712011918186e+01,7.642699366543412509e+01,7.615769351764666339e+01,7.591601512100073990e+01,7.570219938577808705e+01,7.551645944815356870e+01,7.535898045773642195e+01,7.522991939300906949e+01,7.512940490484763245e+01,7.505753718827983789e+01,7.501438788260853130e+01,7.500000000000000000e+01
  };

#endif
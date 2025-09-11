airwave@airwave:~/chingmu_ws$ ros2 launch vrpn_mocap client.launch.yaml server:=192.168.3.191 port:=3883 multi_sensor:=true
[INFO] [launch]: All log files can be found below /home/airwave/.ros/log/2025-09-10-09-11-09-675626-airwave-10921
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [client_node-1]: process started with pid [10923]
[client_node-1] check_vrpn_cookie(): VRPN Note: minor version number doesn't match: (prefer 'vrpn: ver. 07.35', got 'vrpn: ver. 07.30  0').  This is not normally a problem.
[client_node-1] [INFO] [1757466670.776193977] [vrpn_mocap.vrpn_mocap_client_node]: Created new tracker MCAvatar
[client_node-1] [INFO] [1757466670.777000459] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 300
[client_node-1] [INFO] [1757466670.777257353] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 301
[client_node-1] [INFO] [1757466670.777467788] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 302
[client_node-1] [INFO] [1757466670.777691132] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 303
[client_node-1] [INFO] [1757466670.777992822] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 304
[client_node-1] [INFO] [1757466670.778191297] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 305
[client_node-1] [INFO] [1757466670.778415683] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 306
[client_node-1] [INFO] [1757466670.778638797] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 307
[client_node-1] [INFO] [1757466670.778871377] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 308
[client_node-1] [INFO] [1757466670.779132188] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 309
[client_node-1] [INFO] [1757466670.779330959] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 310
[client_node-1] [INFO] [1757466670.779607355] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 311
[client_node-1] [INFO] [1757466670.779834904] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 312
[client_node-1] [INFO] [1757466670.780062759] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 313
[client_node-1] [INFO] [1757466670.780207219] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 314
[client_node-1] [INFO] [1757466670.780440696] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 315
[client_node-1] [INFO] [1757466670.780749925] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 316
[client_node-1] [INFO] [1757466670.780975751] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 317
[client_node-1] [INFO] [1757466670.781203214] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 318
[client_node-1] [INFO] [1757466670.781426023] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 319
[client_node-1] [INFO] [1757466670.781662171] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 320
[client_node-1] [INFO] [1757466670.781971801] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 321
[client_node-1] [INFO] [1757466670.782197133] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 322
[client_node-1] [INFO] [1757466670.782444402] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 323
[client_node-1] [INFO] [1757466670.782663101] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 324
[client_node-1] [INFO] [1757466670.782893270] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 325
[client_node-1] [INFO] [1757466670.783121283] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 326
[client_node-1] [INFO] [1757466670.783335949] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 327
[client_node-1] [INFO] [1757466670.783564306] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 328
[client_node-1] [INFO] [1757466670.783792678] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 329
[client_node-1] [INFO] [1757466670.783998532] [vrpn_mocap.vrpn_mocap_client_node]: Creating sensor 330

airwave@airwave:~/chingmu_ws$ ros2 topic list
/parameter_events
/rosout
/vrpn_mocap/MCAvatar/pose
/vrpn_mocap/MCAvatar/pose300
/vrpn_mocap/MCAvatar/pose301
/vrpn_mocap/MCAvatar/pose302
/vrpn_mocap/MCAvatar/pose303
/vrpn_mocap/MCAvatar/pose304
/vrpn_mocap/MCAvatar/pose305
/vrpn_mocap/MCAvatar/pose306
/vrpn_mocap/MCAvatar/pose307
/vrpn_mocap/MCAvatar/pose308
/vrpn_mocap/MCAvatar/pose309
/vrpn_mocap/MCAvatar/pose310
/vrpn_mocap/MCAvatar/pose311
/vrpn_mocap/MCAvatar/pose312
/vrpn_mocap/MCAvatar/pose313
/vrpn_mocap/MCAvatar/pose314
/vrpn_mocap/MCAvatar/pose315
/vrpn_mocap/MCAvatar/pose316
/vrpn_mocap/MCAvatar/pose317
/vrpn_mocap/MCAvatar/pose318
/vrpn_mocap/MCAvatar/pose319
/vrpn_mocap/MCAvatar/pose320
/vrpn_mocap/MCAvatar/pose321
/vrpn_mocap/MCAvatar/pose322
/vrpn_mocap/MCAvatar/pose323
/vrpn_mocap/MCAvatar/pose324
/vrpn_mocap/MCAvatar/pose325
/vrpn_mocap/MCAvatar/pose326
/vrpn_mocap/MCAvatar/pose327
/vrpn_mocap/MCAvatar/pose328
/vrpn_mocap/MCAvatar/pose329
/vrpn_mocap/MCAvatar/pose330

airwave@airwave:~/chingmu_ws$ ros2 topic echo /vrpn_mocap/MCServer/pose301
header:
  stamp:
    sec: 1757397096
    nanosec: 675729399
  frame_id: world
pose:
  position:
    x: 689.0659790039062
    y: -498.48699951171875
    z: 781.5239868164062
  orientation:
    x: -0.016293900087475777
    y: 0.0007422090275213122
    z: -0.012461699545383453
    w: 0.9997889995574951
---
header:
  stamp:
    sec: 1757397096
    nanosec: 676138253
  frame_id: world
pose:
  position:
    x: 689.0659790039062
    y: -498.48699951171875
    z: 781.5239868164062
  orientation:
    x: -0.016293900087475777
    y: 0.0007422090275213122
    z: -0.012461699545383453
    w: 0.9997889995574951
---
header:
  stamp:
    sec: 1757397096
    nanosec: 685756374
  frame_id: world
pose:
  position:
    x: 689.1799926757812
    y: -498.53900146484375
    z: 781.510986328125
  orientation:
    x: -0.016265900805592537
    y: 0.000694848014973104
    z: -0.012691100127995014
    w: 0.9997869729995728
---
header:
  stamp:
    sec: 1757397096
    nanosec: 686049862
  frame_id: world
pose:
  position:
    x: 689.1799926757812
    y: -498.53900146484375
    z: 781.510986328125
  orientation:
    x: -0.016265900805592537
    y: 0.000694848014973104
    z: -0.012691100127995014
    w: 0.9997869729995728
---
header:
  stamp:
    sec: 1757397096
    nanosec: 695306531
  frame_id: world
pose:
  position:
    x: 689.35498046875
    y: -498.64599609375
    z: 781.4949951171875
  orientation:
    x: -0.01621289923787117
    y: 0.0007512539741583169
    z: -0.013104500249028206
    w: 0.9997820258140564
---
header:
  stamp:
    sec: 1757397096
    nanosec: 695672903
  frame_id: world
pose:
  position:
    x: 689.35498046875
    y: -498.64599609375
    z: 781.4949951171875
  orientation:
    x: -0.01621289923787117
    y: 0.0007512539741583169
    z: -0.013104500249028206
    w: 0.9997820258140564
---
header:
  stamp:
    sec: 1757397096
    nanosec: 705540772
  frame_id: world
pose:
  position:
    x: 689.4349975585938
    y: -498.7019958496094
    z: 781.489990234375
  orientation:
    x: -0.016179900616407394
    y: 0.0007412160048261285
    z: -0.013300799764692783
    w: 0.999779999256134
---
header:
  stamp:
    sec: 1757397096
    nanosec: 706112097
  frame_id: world
pose:
  position:
    x: 689.4349975585938
    y: -498.7019958496094
    z: 781.489990234375
  orientation:
    x: -0.016179900616407394
    y: 0.0007412160048261285
    z: -0.013300799764692783
    w: 0.999779999256134
---
header:
  stamp:
    sec: 1757397096
    nanosec: 725670460
  frame_id: world
pose:
  position:
    x: 689.510009765625
    y: -498.760009765625
    z: 781.5020141601562
  orientation:
    x: -0.016144299879670143
    y: 0.0007572100148536265
    z: -0.013517100363969803
    w: 0.9997779726982117
---
header:
  stamp:
    sec: 1757397096
    nanosec: 726047817
  frame_id: world
pose:
  position:
    x: 689.510009765625
    y: -498.760009765625
    z: 781.5020141601562
  orientation:
    x: -0.016144299879670143
    y: 0.0007572100148536265
    z: -0.013517100363969803
    w: 0.9997779726982117
---
header:
  stamp:
    sec: 1757397096
    nanosec: 735724965
  frame_id: world
pose:
  position:
    x: 689.510009765625
    y: -498.760009765625
    z: 781.5020141601562
  orientation:
    x: -0.016144299879670143
    y: 0.0007572100148536265
    z: -0.013517100363969803
    w: 0.9997779726982117
---
header:
  stamp:
    sec: 1757397096
    nanosec: 736248794
  frame_id: world
pose:
  position:
    x: 689.510009765625
    y: -498.760009765625
    z: 781.5020141601562
  orientation:
    x: -0.016144299879670143
    y: 0.0007572100148536265
    z: -0.013517100363969803
    w: 0.9997779726982117
---
header:
  stamp:
    sec: 1757397096
    nanosec: 745353137
  frame_id: world
pose:
  position:
    x: 689.5980224609375
    y: -498.8219909667969
    z: 781.4879760742188
  orientation:
    x: -0.01611359976232052
    y: 0.0007533240132033825
    z: -0.013741600327193737
    w: 0.9997749924659729
---
header:
  stamp:
    sec: 1757397096
    nanosec: 745982062
  frame_id: world
pose:
  position:
    x: 689.5980224609375
    y: -498.8219909667969
    z: 781.4879760742188
  orientation:
    x: -0.01611359976232052
    y: 0.0007533240132033825
    z: -0.013741600327193737
    w: 0.9997749924659729
---
header:
  stamp:
    sec: 1757397096
    nanosec: 766399035
  frame_id: world
pose:
  position:
    x: 689.6669921875
    y: -498.8689880371094
    z: 781.5120239257812
  orientation:
    x: -0.016074199229478836
    y: 0.0007986840209923685
    z: -0.014022599905729294
    w: 0.9997720122337341
---
header:
  stamp:
    sec: 1757397096
    nanosec: 767712461
  frame_id: world
pose:
  position:
    x: 689.6669921875
    y: -498.8689880371094
    z: 781.5120239257812
  orientation:
    x: -0.016074199229478836
    y: 0.0007986840209923685
    z: -0.014022599905729294
    w: 0.9997720122337341
---
header:
  stamp:
    sec: 1757397096
    nanosec: 776012856
  frame_id: world
pose:
  position:
    x: 689.7659912109375
    y: -498.99700927734375
    z: 781.4990234375
  orientation:
    x: -0.01599689945578575
    y: 0.0006933980039320886
    z: -0.014405299909412861
    w: 0.9997680187225342
---
header:
  stamp:
    sec: 1757397096
    nanosec: 776602612
  frame_id: world
pose:
  position:
    x: 689.7659912109375
    y: -498.99700927734375
    z: 781.4990234375
  orientation:
    x: -0.01599689945578575
    y: 0.0006933980039320886
    z: -0.014405299909412861
    w: 0.9997680187225342
---
header:
  stamp:
    sec: 1757397096
    nanosec: 786368215
  frame_id: world
pose:
  position:
    x: 689.8070068359375
    y: -499.0570068359375
    z: 781.5159912109375
  orientation:
    x: -0.01596209965646267
    y: 0.0007337920251302421
    z: -0.014635000377893448
    w: 0.9997649788856506



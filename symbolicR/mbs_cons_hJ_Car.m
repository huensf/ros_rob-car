%
%-------------------------------------------------------------
%
%	ROBOTRAN - Version 6.6 (build : february 22, 2008)
%
%	Copyright 
%	Universite catholique de Louvain 
%	Departement de Mecanique 
%	Unite de Production Mecanique et Machines 
%	2, Place du Levant 
%	1348 Louvain-la-Neuve 
%	http://www.robotran.be// 
%
%	==> Generation Date : Thu Apr  1 15:04:07 2021
%
%	==> Project name : Car
%	==> using XML input file 
%
%	==> Number of joints : 50
%
%	==> Function : F 8 : Constraints Vector (h) and Jacobian Matrix (Jac) 
%	==> Flops complexity : 632
%
%	==> Generation Time :  0.030 seconds
%	==> Post-Processing :  0.010 seconds
%
%-------------------------------------------------------------
%
function [h,Jac] = cons_hJ(s,tsim,usrfun)

 h = zeros(28,1);
 Jac = zeros(28,50);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 

% = = Block_0_0_0_0_0_3 = = 
 
% Trigonometric Variables  

  C8 = cos(q(8));
  S8 = sin(q(8));
  C9 = cos(q(9));
  S9 = sin(q(9));
  C10 = cos(q(10));
  S10 = sin(q(10));
  C11 = cos(q(11));
  S11 = sin(q(11));

% = = Block_0_0_0_0_0_4 = = 
 
% Trigonometric Variables  

  C13 = cos(q(13));
  S13 = sin(q(13));
  C14 = cos(q(14));
  S14 = sin(q(14));
  C15 = cos(q(15));
  S15 = sin(q(15));
  C16 = cos(q(16));
  S16 = sin(q(16));

% = = Block_0_0_0_0_0_5 = = 
 
% Trigonometric Variables  

  C18 = cos(q(18));
  S18 = sin(q(18));

% = = Block_0_0_0_0_0_6 = = 
 
% Trigonometric Variables  

  C19 = cos(q(19));
  S19 = sin(q(19));

% = = Block_0_0_0_0_0_7 = = 
 
% Trigonometric Variables  

  C20 = cos(q(20));
  S20 = sin(q(20));

% = = Block_0_0_0_0_0_8 = = 
 
% Trigonometric Variables  

  C21 = cos(q(21));
  S21 = sin(q(21));

% = = Block_0_0_0_0_0_9 = = 
 
% Trigonometric Variables  

  C22 = cos(q(22));
  S22 = sin(q(22));

% = = Block_0_0_0_0_0_10 = = 
 
% Trigonometric Variables  

  C23 = cos(q(23));
  S23 = sin(q(23));
  C24 = cos(q(24));
  S24 = sin(q(24));

% = = Block_0_0_0_0_0_11 = = 
 
% Trigonometric Variables  

  C25 = cos(q(25));
  S25 = sin(q(25));
  C26 = cos(q(26));
  S26 = sin(q(26));
  C27 = cos(q(27));
  S27 = sin(q(27));

% = = Block_0_0_0_0_0_13 = = 
 
% Trigonometric Variables  

  C29 = cos(q(29));
  S29 = sin(q(29));
  C30 = cos(q(30));
  S30 = sin(q(30));

% = = Block_0_0_0_0_0_14 = = 
 
% Trigonometric Variables  

  C31 = cos(q(31));
  S31 = sin(q(31));
  C32 = cos(q(32));
  S32 = sin(q(32));

% = = Block_0_0_0_0_0_16 = = 
 
% Trigonometric Variables  

  C35 = cos(q(35));
  S35 = sin(q(35));
  C36 = cos(q(36));
  S36 = sin(q(36));

% = = Block_0_0_0_0_0_17 = = 
 
% Trigonometric Variables  

  C38 = cos(q(38));
  S38 = sin(q(38));
  C39 = cos(q(39));
  S39 = sin(q(39));

% = = Block_0_0_0_0_0_18 = = 
 
% Trigonometric Variables  

  C41 = cos(q(41));
  S41 = sin(q(41));

% = = Block_0_0_0_0_0_19 = = 
 
% Trigonometric Variables  

  C42 = cos(q(42));
  S42 = sin(q(42));
  C43 = cos(q(43));
  S43 = sin(q(43));
  C44 = cos(q(44));
  S44 = sin(q(44));

% = = Block_0_0_0_0_0_20 = = 
 
% Trigonometric Variables  

  C45 = cos(q(45));
  S45 = sin(q(45));
  C46 = cos(q(46));
  S46 = sin(q(46));

% = = Block_0_1_0_0_0_3 = = 
 
% Constraints and Constraints Jacobian 

%
  RO_2_29 = S8*S9;
  RO_2_39 = -C8*S9;
  RO_2_89 = -S8*C9;
  RO_2_99 = C8*C9;
  RO_2_710 = C10*S9;
  RO_2_810 = RO_2_89*C10-S10*C8;
  RO_2_910 = RO_2_99*C10-S10*S8;
  RO_2_111 = S10*S11*S9+C11*C9;
  RO_2_211 = RO_2_29*C11+S11*(RO_2_89*S10+C10*C8);
  RO_2_311 = RO_2_39*C11+S11*(RO_2_99*S10+C10*S8);
  RL_2_29 = s.dpt(2,18)*C8;
  RL_2_39 = s.dpt(2,18)*S8;
  PO_2_29 = RL_2_29+s.dpt(2,1);
  PO_2_39 = RL_2_39+s.dpt(3,1);
  RL_2_153 = RO_2_111*s.dpt(1,21)+RO_2_710*s.dpt(3,21);
  RL_2_253 = RO_2_211*s.dpt(1,21)+RO_2_810*s.dpt(3,21);
  RL_2_353 = RO_2_311*s.dpt(1,21)+RO_2_910*s.dpt(3,21);
  PO_2_153 = RL_2_153+s.dpt(1,1);
  JT_2_253_8 = -(RL_2_353+RL_2_39);
  JT_2_353_8 = RL_2_253+RL_2_29;
  JT_2_153_9 = -(RL_2_253*S8-RL_2_353*C8);
  JT_2_253_9 = RL_2_153*S8;
  JT_2_353_9 = -RL_2_153*C8;
  JT_2_153_10 = -(RL_2_253*RO_2_39-RL_2_353*RO_2_29);
  JT_2_253_10 = RL_2_153*RO_2_39-RL_2_353*C9;
  JT_2_353_10 = -(RL_2_153*RO_2_29-RL_2_253*C9);
  JT_2_153_11 = -(RL_2_253*RO_2_910-RL_2_353*RO_2_810);
  JT_2_253_11 = RL_2_153*RO_2_910-RL_2_353*RO_2_710;
  JT_2_353_11 = -(RL_2_153*RO_2_810-RL_2_253*RO_2_710);
%
  RL_11_262 = s.dpt(2,19)*C8;
  RL_11_362 = s.dpt(2,19)*S8;
%
  RL_16_167 = RO_2_111*s.dpt(1,20)+RO_2_710*s.dpt(3,20);
  RL_16_267 = RO_2_211*s.dpt(1,20)+RO_2_810*s.dpt(3,20);
  RL_16_367 = RO_2_311*s.dpt(1,20)+RO_2_910*s.dpt(3,20);
  JT_16_267_8 = -(RL_16_367+RL_2_39);
  JT_16_367_8 = RL_16_267+RL_2_29;
  JT_16_167_9 = -(RL_16_267*S8-RL_16_367*C8);
  JT_16_267_9 = RL_16_167*S8;
  JT_16_367_9 = -RL_16_167*C8;
  JT_16_167_10 = -(RL_16_267*RO_2_39-RL_16_367*RO_2_29);
  JT_16_267_10 = RL_16_167*RO_2_39-RL_16_367*C9;
  JT_16_367_10 = -(RL_16_167*RO_2_29-RL_16_267*C9);
  JT_16_167_11 = -(RL_16_267*RO_2_910-RL_16_367*RO_2_810);
  JT_16_267_11 = RL_16_167*RO_2_910-RL_16_367*RO_2_710;
  JT_16_367_11 = -(RL_16_167*RO_2_810-RL_16_267*RO_2_710);

% = = Block_0_1_0_0_0_4 = = 
 
% Constraints and Constraints Jacobian 

%
  RO_0_214 = S13*S14;
  RO_0_314 = -C13*S14;
  RO_0_814 = -S13*C14;
  RO_0_914 = C13*C14;
  RO_0_715 = S14*C15;
  RO_0_815 = RO_0_814*C15-C13*S15;
  RO_0_915 = RO_0_914*C15-S13*S15;
  RO_0_116 = C14*C16+S14*S15*S16;
  RO_0_216 = RO_0_214*C16+S16*(RO_0_814*S15+C13*C15);
  RO_0_316 = RO_0_314*C16+S16*(RO_0_914*S15+S13*C15);
  RL_0_214 = s.dpt(2,25)*C13;
  RL_0_314 = s.dpt(2,25)*S13;
  PO_0_214 = RL_0_214+s.dpt(2,2);
  PO_0_314 = RL_0_314+s.dpt(3,2);
  RL_0_151 = RO_0_116*s.dpt(1,27)+RO_0_715*s.dpt(3,27);
  RL_0_251 = RO_0_216*s.dpt(1,27)+RO_0_815*s.dpt(3,27);
  RL_0_351 = RO_0_316*s.dpt(1,27)+RO_0_915*s.dpt(3,27);
  PO_0_151 = RL_0_151+s.dpt(1,2);
  JT_0_251_13 = -(RL_0_314+RL_0_351);
  JT_0_351_13 = RL_0_214+RL_0_251;
  JT_0_151_14 = -(RL_0_251*S13-RL_0_351*C13);
  JT_0_251_14 = RL_0_151*S13;
  JT_0_351_14 = -RL_0_151*C13;
  JT_0_151_15 = -(RL_0_251*RO_0_314-RL_0_351*RO_0_214);
  JT_0_251_15 = RL_0_151*RO_0_314-RL_0_351*C14;
  JT_0_351_15 = -(RL_0_151*RO_0_214-RL_0_251*C14);
  JT_0_151_16 = -(RL_0_251*RO_0_915-RL_0_351*RO_0_815);
  JT_0_251_16 = RL_0_151*RO_0_915-RL_0_351*RO_0_715;
  JT_0_351_16 = -(RL_0_151*RO_0_815-RL_0_251*RO_0_715);
%
  RL_9_260 = s.dpt(2,26)*C13;
  RL_9_360 = s.dpt(2,26)*S13;
%
  RL_18_169 = RO_0_116*s.dpt(1,29)+RO_0_715*s.dpt(3,29);
  RL_18_269 = RO_0_216*s.dpt(1,29)+RO_0_815*s.dpt(3,29);
  RL_18_369 = RO_0_316*s.dpt(1,29)+RO_0_915*s.dpt(3,29);
  JT_18_269_13 = -(RL_0_314+RL_18_369);
  JT_18_369_13 = RL_0_214+RL_18_269;
  JT_18_169_14 = -(RL_18_269*S13-RL_18_369*C13);
  JT_18_269_14 = RL_18_169*S13;
  JT_18_369_14 = -RL_18_169*C13;
  JT_18_169_15 = -(RL_18_269*RO_0_314-RL_18_369*RO_0_214);
  JT_18_269_15 = RL_18_169*RO_0_314-RL_18_369*C14;
  JT_18_369_15 = -(RL_18_169*RO_0_214-RL_18_269*C14);
  JT_18_169_16 = -(RL_18_269*RO_0_915-RL_18_369*RO_0_815);
  JT_18_269_16 = RL_18_169*RO_0_915-RL_18_369*RO_0_715;
  JT_18_369_16 = -(RL_18_169*RO_0_815-RL_18_269*RO_0_715);

% = = Block_0_1_0_0_0_5 = = 
 
% Constraints and Constraints Jacobian 

%
  RL_1_252 = s.dpt(2,31)*C18;
  RL_1_352 = s.dpt(2,31)*S18;

% = = Block_0_1_0_0_0_6 = = 
 
% Constraints and Constraints Jacobian 

%
  RL_3_254 = s.dpt(2,33)*C19;
  RL_3_354 = s.dpt(2,33)*S19;

% = = Block_0_1_0_0_0_7 = = 
 
% Constraints and Constraints Jacobian 

%
  RL_4_255 = s.dpt(2,35)*C20;
  RL_4_355 = s.dpt(2,35)*S20;

% = = Block_0_1_0_0_0_8 = = 
 
% Constraints and Constraints Jacobian 

%
  RL_6_257 = s.dpt(2,37)*C21;
  RL_6_357 = s.dpt(2,37)*S21;

% = = Block_0_1_0_0_0_10 = = 
 
% Constraints and Constraints Jacobian 

%
  RO_8_423 = S22*S23;
  RO_8_623 = C22*S23;
  RL_8_123 = s.dpt(1,39)*C22;
  RL_8_323 = -s.dpt(1,39)*S22;
  RL_8_159 = s.dpt(3,41)*(C22*S24+S22*C23*C24);
  RL_8_259 = -s.dpt(3,41)*S23*C24;
  RL_8_359 = s.dpt(3,41)*(C22*C23*C24-S22*S24);
  JT_8_159_22 = RL_8_323+RL_8_359;
  JT_8_359_22 = -(RL_8_123+RL_8_159);
  JT_8_159_23 = RL_8_259*S22;
  JT_8_259_23 = -(RL_8_159*S22+RL_8_359*C22);
  JT_8_359_23 = RL_8_259*C22;
  JT_8_159_24 = -(RL_8_259*RO_8_623-RL_8_359*C23);
  JT_8_259_24 = RL_8_159*RO_8_623-RL_8_359*RO_8_423;
  JT_8_359_24 = -(RL_8_159*C23-RL_8_259*RO_8_423);

% = = Block_0_1_0_0_0_11 = = 
 
% Trigonometric Variables  

%
  S22p25 = C22*S25+S22*C25;
  C22p25 = C22*C25-S22*S25;
 
% Constraints and Constraints Jacobian 

  RO_10_426 = S26*S22p25;
  RO_10_626 = S26*C22p25;
  RL_10_126 = s.dpt(1,42)*C22p25;
  RL_10_326 = -s.dpt(1,42)*S22p25;
  RL_10_161 = s.dpt(3,43)*(C26*C27*S22p25+S27*C22p25);
  RL_10_261 = -s.dpt(3,43)*S26*C27;
  RL_10_361 = s.dpt(3,43)*(C26*C27*C22p25-S27*S22p25);
  JT_10_161_22 = RL_10_326+RL_10_361;
  JT_10_361_22 = -(RL_10_126+RL_10_161);
  JT_10_161_25 = RL_10_326+RL_10_361;
  JT_10_361_25 = -(RL_10_126+RL_10_161);
  JT_10_161_26 = RL_10_261*S22p25;
  JT_10_261_26 = -(RL_10_161*S22p25+RL_10_361*C22p25);
  JT_10_361_26 = RL_10_261*C22p25;
  JT_10_161_27 = -(RL_10_261*RO_10_626-RL_10_361*C26);
  JT_10_261_27 = RL_10_161*RO_10_626-RL_10_361*RO_10_426;
  JT_10_361_27 = -(RL_10_161*C26-RL_10_261*RO_10_426);

% = = Block_0_1_0_0_0_13 = = 
 
% Constraints and Constraints Jacobian 

%
  RL_17_168 = s.dpt(1,47)*C30-s.dpt(2,47)*S30;
  RL_17_268 = C29*(s.dpt(1,47)*S30+s.dpt(2,47)*C30);
  RL_17_368 = S29*(s.dpt(1,47)*S30+s.dpt(2,47)*C30);
  JT_17_168_30 = -(s.dpt(1,47)*S30+s.dpt(2,47)*C30);
  JT_17_268_30 = RL_17_168*C29;
  JT_17_368_30 = RL_17_168*S29;

% = = Block_0_1_0_0_0_14 = = 
 
% Constraints and Constraints Jacobian 

%
  RL_19_170 = s.dpt(1,48)*C32-s.dpt(2,48)*S32;
  RL_19_270 = C31*(s.dpt(1,48)*S32+s.dpt(2,48)*C32);
  RL_19_370 = S31*(s.dpt(1,48)*S32+s.dpt(2,48)*C32);
  JT_19_170_32 = -(s.dpt(1,48)*S32+s.dpt(2,48)*C32);
  JT_19_270_32 = RL_19_170*C31;
  JT_19_370_32 = RL_19_170*S31;

% = = Block_0_1_0_0_0_16 = = 
 
% Trigonometric Variables  

%
  S35p36 = C35*S36+S35*C36;
  C35p36 = C35*C36-S35*S36;
 
% Constraints and Constraints Jacobian 

  RL_7_236 = s.dpt(2,49)*C35;
  RL_7_336 = s.dpt(2,49)*S35;
  RL_7_258 = -s.dpt(3,51)*S35p36;
  RL_7_358 = s.dpt(3,51)*C35p36;
  JT_7_258_35 = -(RL_7_336+RL_7_358);
  JT_7_358_35 = RL_7_236+RL_7_258;
%
  RL_12_263 = s.dpt(2,50)*C35;
  RL_12_363 = s.dpt(2,50)*S35;

% = = Block_0_1_0_0_0_17 = = 
 
% Trigonometric Variables  

%
  S38p39 = C38*S39+S38*C39;
  C38p39 = C38*C39-S38*S39;
 
% Constraints and Constraints Jacobian 

  RL_5_239 = s.dpt(2,54)*C38;
  RL_5_339 = s.dpt(2,54)*S38;
  RL_5_256 = -s.dpt(3,57)*S38p39;
  RL_5_356 = s.dpt(3,57)*C38p39;
  JT_5_256_38 = -(RL_5_339+RL_5_356);
  JT_5_356_38 = RL_5_239+RL_5_256;
%
  RL_15_266 = s.dpt(2,55)*C38;
  RL_15_366 = s.dpt(2,55)*S38;

% = = Block_0_1_0_0_0_19 = = 
 
% Trigonometric Variables  

%
  S41p42 = C41*S42+S41*C42;
  C41p42 = C41*C42-S41*S42;
 
% Constraints and Constraints Jacobian 

  RO_14_443 = S43*S41p42;
  RO_14_643 = S43*C41p42;
  RL_14_143 = s.dpt(1,61)*C41p42;
  RL_14_343 = -s.dpt(1,61)*S41p42;
  RL_14_165 = s.dpt(3,62)*(C43*C44*S41p42+S44*C41p42);
  RL_14_265 = -s.dpt(3,62)*S43*C44;
  RL_14_365 = s.dpt(3,62)*(C43*C44*C41p42-S44*S41p42);
  JT_14_165_41 = RL_14_343+RL_14_365;
  JT_14_365_41 = -(RL_14_143+RL_14_165);
  JT_14_165_42 = RL_14_343+RL_14_365;
  JT_14_365_42 = -(RL_14_143+RL_14_165);
  JT_14_165_43 = RL_14_265*S41p42;
  JT_14_265_43 = -(RL_14_165*S41p42+RL_14_365*C41p42);
  JT_14_365_43 = RL_14_265*C41p42;
  JT_14_165_44 = -(RL_14_265*RO_14_643-RL_14_365*C43);
  JT_14_265_44 = RL_14_165*RO_14_643-RL_14_365*RO_14_443;
  JT_14_365_44 = -(RL_14_165*C43-RL_14_265*RO_14_443);

% = = Block_0_1_0_0_0_20 = = 
 
% Constraints and Constraints Jacobian 

%
  RO_13_445 = S41*S45;
  RO_13_645 = C41*S45;
  RL_13_145 = s.dpt(1,60)*C41;
  RL_13_345 = -s.dpt(1,60)*S41;
  RL_13_164 = s.dpt(3,63)*(C41*S46+S41*C45*C46);
  RL_13_264 = -s.dpt(3,63)*S45*C46;
  RL_13_364 = s.dpt(3,63)*(C41*C45*C46-S41*S46);
  JT_13_164_41 = RL_13_345+RL_13_364;
  JT_13_364_41 = -(RL_13_145+RL_13_164);
  JT_13_164_45 = RL_13_264*S41;
  JT_13_264_45 = -(RL_13_164*S41+RL_13_364*C41);
  JT_13_364_45 = RL_13_264*C41;
  JT_13_164_46 = -(RL_13_264*RO_13_645-RL_13_364*C45);
  JT_13_264_46 = RL_13_164*RO_13_645-RL_13_364*RO_13_445;
  JT_13_364_46 = -(RL_13_164*C45-RL_13_264*RO_13_445);

% = = Block_0_1_0_0_1_0 = = 
 
% Constraints and Constraints Jacobian 

%
  h_2 = PO_0_214+RL_0_251-RL_1_252-s.dpt(2,3);
  h_3 = PO_0_314+RL_0_351-RL_1_352;
%
  h_5 = PO_2_29+RL_2_253-RL_3_254-s.dpt(2,4);
  h_6 = PO_2_39+RL_2_353-RL_3_354;
%
  h_8 = RL_4_255-RL_5_239-RL_5_256-s.dpt(2,13)+s.dpt(2,5);
  h_9 = RL_4_355-RL_5_339-RL_5_356-s.dpt(3,13);
%
  h_11 = RL_6_257-RL_7_236-RL_7_258-s.dpt(2,12)+s.dpt(2,7);
  h_12 = RL_6_357-RL_7_336-RL_7_358-s.dpt(3,12);
%
  h_13 = RL_8_123+RL_8_159-s.dpt(1,2)-s.dpt(1,26)+s.dpt(1,8);
  h_14 = RL_8_259-RL_9_260-s.dpt(2,2)+s.dpt(2,39);
  h_15 = RL_8_323+RL_8_359-RL_9_360-s.dpt(3,2)+s.dpt(3,8);
%
  h_16 = RL_10_126+RL_10_161-s.dpt(1,1)-s.dpt(1,19)+s.dpt(1,8);
  h_17 = RL_10_261-RL_11_262-s.dpt(2,1)+s.dpt(2,42);
  h_18 = RL_10_326+RL_10_361-RL_11_362-s.dpt(3,1)+s.dpt(3,8);
%
  h_19 = s.dpt(1,12)+s.dpt(1,50)-(RL_13_145+RL_13_164+s.dpt(1,15));
  h_20 = RL_12_263-RL_13_264+s.dpt(2,12)-s.dpt(2,60);
  h_21 = RL_12_363-RL_13_345-RL_13_364+s.dpt(3,12)-s.dpt(3,15);
%
  h_22 = RL_14_143+RL_14_165-s.dpt(1,13)+s.dpt(1,15)-s.dpt(1,55);
  h_23 = RL_14_265-RL_15_266-s.dpt(2,13)+s.dpt(2,61);
  h_24 = RL_14_343+RL_14_365-RL_15_366-s.dpt(3,13)+s.dpt(3,15);
%
  h_25 = RL_16_167-RL_17_168+s.dpt(1,1)-s.dpt(1,9);
  h_26 = PO_2_29+RL_16_267-RL_17_268-q(28)-s.dpt(2,44);
  h_27 = PO_2_39+RL_16_367-RL_17_368-s.dpt(3,9);
%
  h_28 = RL_18_169-RL_19_170+s.dpt(1,2)-s.dpt(1,9);
  h_29 = PO_0_214+RL_18_269-RL_19_270-q(28)-s.dpt(2,45);
  h_30 = PO_0_314+RL_18_369-RL_19_370-s.dpt(3,9);

% = = Block_0_3_0_0_0_0 = = 
 
% Symbolic Outputs  

  h(1) = PO_0_151;
  h(2) = h_2;
  h(3) = h_3;
  h(4) = PO_2_153;
  h(5) = h_5;
  h(6) = h_6;
  h(7) = h_8;
  h(8) = h_9;
  h(9) = h_11;
  h(10) = h_12;
  h(11) = h_13;
  h(12) = h_14;
  h(13) = h_15;
  h(14) = h_16;
  h(15) = h_17;
  h(16) = h_18;
  h(17) = h_19;
  h(18) = h_20;
  h(19) = h_21;
  h(20) = h_22;
  h(21) = h_23;
  h(22) = h_24;
  h(23) = h_25;
  h(24) = h_26;
  h(25) = h_27;
  h(26) = h_28;
  h(27) = h_29;
  h(28) = h_30;
  Jac(1,14) = JT_0_151_14;
  Jac(1,15) = JT_0_151_15;
  Jac(1,16) = JT_0_151_16;
  Jac(2,13) = JT_0_251_13;
  Jac(2,14) = JT_0_251_14;
  Jac(2,15) = JT_0_251_15;
  Jac(2,16) = JT_0_251_16;
  Jac(2,18) = RL_1_352;
  Jac(3,13) = JT_0_351_13;
  Jac(3,14) = JT_0_351_14;
  Jac(3,15) = JT_0_351_15;
  Jac(3,16) = JT_0_351_16;
  Jac(3,18) = -RL_1_252;
  Jac(4,9) = JT_2_153_9;
  Jac(4,10) = JT_2_153_10;
  Jac(4,11) = JT_2_153_11;
  Jac(5,8) = JT_2_253_8;
  Jac(5,9) = JT_2_253_9;
  Jac(5,10) = JT_2_253_10;
  Jac(5,11) = JT_2_253_11;
  Jac(5,19) = RL_3_354;
  Jac(6,8) = JT_2_353_8;
  Jac(6,9) = JT_2_353_9;
  Jac(6,10) = JT_2_353_10;
  Jac(6,11) = JT_2_353_11;
  Jac(6,19) = -RL_3_254;
  Jac(7,20) = -RL_4_355;
  Jac(7,38) = -JT_5_256_38;
  Jac(7,39) = RL_5_356;
  Jac(8,20) = RL_4_255;
  Jac(8,38) = -JT_5_356_38;
  Jac(8,39) = -RL_5_256;
  Jac(9,21) = -RL_6_357;
  Jac(9,35) = -JT_7_258_35;
  Jac(9,36) = RL_7_358;
  Jac(10,21) = RL_6_257;
  Jac(10,35) = -JT_7_358_35;
  Jac(10,36) = -RL_7_258;
  Jac(11,22) = JT_8_159_22;
  Jac(11,23) = JT_8_159_23;
  Jac(11,24) = JT_8_159_24;
  Jac(12,13) = RL_9_360;
  Jac(12,23) = JT_8_259_23;
  Jac(12,24) = JT_8_259_24;
  Jac(13,13) = -RL_9_260;
  Jac(13,22) = JT_8_359_22;
  Jac(13,23) = JT_8_359_23;
  Jac(13,24) = JT_8_359_24;
  Jac(14,22) = JT_10_161_22;
  Jac(14,25) = JT_10_161_25;
  Jac(14,26) = JT_10_161_26;
  Jac(14,27) = JT_10_161_27;
  Jac(15,8) = RL_11_362;
  Jac(15,26) = JT_10_261_26;
  Jac(15,27) = JT_10_261_27;
  Jac(16,8) = -RL_11_262;
  Jac(16,22) = JT_10_361_22;
  Jac(16,25) = JT_10_361_25;
  Jac(16,26) = JT_10_361_26;
  Jac(16,27) = JT_10_361_27;
  Jac(17,41) = -JT_13_164_41;
  Jac(17,45) = -JT_13_164_45;
  Jac(17,46) = -JT_13_164_46;
  Jac(18,35) = -RL_12_363;
  Jac(18,45) = -JT_13_264_45;
  Jac(18,46) = -JT_13_264_46;
  Jac(19,35) = RL_12_263;
  Jac(19,41) = -JT_13_364_41;
  Jac(19,45) = -JT_13_364_45;
  Jac(19,46) = -JT_13_364_46;
  Jac(20,41) = JT_14_165_41;
  Jac(20,42) = JT_14_165_42;
  Jac(20,43) = JT_14_165_43;
  Jac(20,44) = JT_14_165_44;
  Jac(21,38) = RL_15_366;
  Jac(21,43) = JT_14_265_43;
  Jac(21,44) = JT_14_265_44;
  Jac(22,38) = -RL_15_266;
  Jac(22,41) = JT_14_365_41;
  Jac(22,42) = JT_14_365_42;
  Jac(22,43) = JT_14_365_43;
  Jac(22,44) = JT_14_365_44;
  Jac(23,9) = JT_16_167_9;
  Jac(23,10) = JT_16_167_10;
  Jac(23,11) = JT_16_167_11;
  Jac(23,30) = -JT_17_168_30;
  Jac(24,8) = JT_16_267_8;
  Jac(24,9) = JT_16_267_9;
  Jac(24,10) = JT_16_267_10;
  Jac(24,11) = JT_16_267_11;
  Jac(24,28) = -(1.0);
  Jac(24,29) = RL_17_368;
  Jac(24,30) = -JT_17_268_30;
  Jac(25,8) = JT_16_367_8;
  Jac(25,9) = JT_16_367_9;
  Jac(25,10) = JT_16_367_10;
  Jac(25,11) = JT_16_367_11;
  Jac(25,29) = -RL_17_268;
  Jac(25,30) = -JT_17_368_30;
  Jac(26,14) = JT_18_169_14;
  Jac(26,15) = JT_18_169_15;
  Jac(26,16) = JT_18_169_16;
  Jac(26,32) = -JT_19_170_32;
  Jac(27,13) = JT_18_269_13;
  Jac(27,14) = JT_18_269_14;
  Jac(27,15) = JT_18_269_15;
  Jac(27,16) = JT_18_269_16;
  Jac(27,28) = -(1.0);
  Jac(27,31) = RL_19_370;
  Jac(27,32) = -JT_19_270_32;
  Jac(28,13) = JT_18_369_13;
  Jac(28,14) = JT_18_369_14;
  Jac(28,15) = JT_18_369_15;
  Jac(28,16) = JT_18_369_16;
  Jac(28,31) = -RL_19_270;
  Jac(28,32) = -JT_19_370_32;

% ====== END Task 0 ====== 

  


% MODEL DYNAMICS
z74 = L_T.*M_T;
z66 = L_SH.*M_SH;
z68 = L_FT.*M_FT;
z5 = LGRFY - G.*M_FT;
z13 = G.*M_SH;
z14 = G.*M_TH;
z15 = RGRFY - G.*M_FT;
z23 = G.*M_T;
z24 = LGRFX + RESX + RGRFX;
z25 = RESY + z5 + z15 - 2.*z13 - 2.*z14 - z23;
z27 = L_T.*z23;
z29 = T_LA + T_LH + T_LK;
z30 = LGRFX.*L_SH;
z31 = LGRFX.*L_TH;
z32 = L_SH.*z5;
z33 = L_TH.*z5;
z34 = LGRFX.*L_FT;
z35 = L_FT.*z5;
z36 = L_TH.*z14;
z38 = T_RA + T_RH + T_RK;
z39 = L_SH.*RGRFX;
z40 = L_SH.*z15;
z41 = L_TH.*RGRFX;
z42 = L_TH.*z15;
z43 = L_FT.*RGRFX;
z44 = L_FT.*z15;
z46 = T_LA + T_LK;
z47 = L_SH.*z13;
z49 = T_RA + T_RK;
z63 = M_T + 2.*M_FT + 2.*M_SH + 2.*M_TH;
z64 = L_TH.*M_TH;
z85 = I_T + 0.25.*M_T.*L_T.^2;
z88 = I_FT + I_SH + I_TH + 0.25.*M_TH.*L_TH.^2;
z89 = L_SH.^2 + 4.*L_TH.^2;
z90 = L_SH.*L_TH;
z91 = L_FT.*L_SH;
z92 = L_FT.*L_TH;
z93 = L_SH.^2;
z94 = L_TH.^2;
z95 = L_FT.^2;
z97 = I_FT + I_SH;
z111 = I_FT + I_SH + 0.25.*M_SH.*L_SH.^2;
z112 = L_FT.^2 + 4.*L_SH.^2;
z119 = I_FT + 0.25.*M_FT.*L_FT.^2;
z26 = sin(QT);
z62 = L_T.*QTP.^2;
z1 = sin(QLA+QLK);
z6 = sin(QLH);
z2 = cos(QLA+QLK);
z7 = cos(QLH);
z10 = z1.*z6 - z2.*z7;
z53 = L_FT.*(QLHP+QLKP+QLAP).^2;
z55 = L_TH.*QLHP.^2;
z12 = sin(QLH+QLK);
z54 = L_SH.*(QLHP+QLKP).^2;
z3 = sin(QRA+QRK);
z16 = sin(QRH);
z4 = cos(QRA+QRK);
z17 = cos(QRH);
z20 = z3.*z16 - z4.*z17;
z56 = L_FT.*(QRHP+QRKP+QRAP);
z59 = (QRHP+QRKP+QRAP).*z56;
z58 = L_TH.*QRHP;
z61 = QRHP.*z58;
z22 = sin(QRH+QRK);
z57 = L_SH.*(QRHP+QRKP);
z60 = (QRHP+QRKP).*z57;
z76 = 0.5.*M_T.*z26.*z62 + 0.5.*M_FT.*(z10.*z53-2.*z6.*z55-2.*z12.*z54) + 0.5.*M_FT.*(z20.*z59-2.*z16.*z61-2.*z22.*z60) - 0.5.*M_TH.*z6.*z55 - 0.5.*M_TH.*z16.*z61 - 0.5.*M_SH.*(z12.*z54+2.*z6.*z55) - 0.5.*M_SH.*(z22.*z60+2.*z16.*z61);
z122 = z76 - z24;
z8 = -z1.*z7 - z2.*z6;
z69 = z68.*z8;
z101 = cos(QLA);
z120 = z68.*(z2.*z55+z101.*z54);
z9 = z2.*z7 - z1.*z6;
z51 = T_LA + 0.5.*L_FT.*(LGRFX.*z8+z5.*z9);
z128 = 0.5.*z120 - z51;
z175 = z69./z63;
z183 = z128 - 0.5.*z175.*z122;
z79 = z68.*z9;
z238 = z79./z63;
z11 = cos(QLH+QLK);
z21 = cos(QRH+QRK);
z18 = -z3.*z17 - z4.*z16;
z73 = cos(QT);
z84 = 0.5.*M_TH.*z7.*z55 + 0.5.*M_TH.*z17.*z61 + 0.5.*M_SH.*(z11.*z54+2.*z7.*z55) + 0.5.*M_SH.*(z21.*z60+2.*z17.*z61) + 0.5.*M_FT.*(z8.*z53+2.*z7.*z55+2.*z11.*z54) + 0.5.*M_FT.*(z18.*z59+2.*z17.*z61+2.*z21.*z60) - 0.5.*M_T.*z73.*z62;
z123 = z84 - z25;
z246 = z183 - 0.5.*z238.*z123;
z75 = z74.*z73;
z176 = z75.*z175;
z83 = z74.*z26;
z239 = -0.25.*z176 - 0.25.*z83.*z238;
z130 = z75./z63;
z131 = 0.25.*z75.*z130 - z85;
z193 = z83./z63;
z194 = z131 + 0.25.*z83.*z193;
z288 = z239./z194;
z28 = T_T + 0.5.*z27.*z26;
z138 = 0.5.*z130.*z122 - z28;
z201 = z138 + 0.5.*z193.*z123;
z295 = z246 - z288.*z201;
z65 = 0.5.*z64.*z7 + 0.5.*M_SH.*(L_SH.*z11+2.*L_TH.*z7) + 0.5.*M_FT.*(L_FT.*z8+2.*L_SH.*z11+2.*L_TH.*z7);
z87 = sin(QLA);
z99 = I_FT + 0.25.*z68.*(L_FT-2.*L_SH.*z87-2.*L_TH.*z1);
z177 = 0.5.*z65.*z175 - z99;
z77 = 0.5.*z64.*z6 + 0.5.*M_SH.*(L_SH.*z12+2.*L_TH.*z6) + 0.5.*M_FT.*(L_FT.*z9+2.*L_SH.*z12+2.*L_TH.*z6);
z240 = z177 + 0.5.*z77.*z238;
z132 = z65.*z130;
z195 = -0.5.*z132 - 0.5.*z77.*z193;
z289 = z240 - z195.*z288;
z139 = z65./z63;
z86 = cos(QLK);
z96 = z88 + 0.25.*M_SH.*(z89+4.*z90.*z86) + 0.25.*M_FT.*(z95+4.*z93+4.*z94+8.*z90.*z86-4.*z91.*z87-4.*z92.*z1);
z141 = z65.*z139 - z96;
z202 = z77./z63;
z204 = z141 + z77.*z202;
z140 = z75.*z139;
z203 = -0.5.*z140 - 0.5.*z83.*z202;
z256 = z203./z194;
z257 = z204 - z195.*z256;
z325 = z289./z257;
z100 = sin(QLK);
z102 = 0.5.*M_SH.*z100.*(L_SH.*z55-L_TH.*z54) + 0.5.*M_FT.*(L_FT.*z2.*z55+L_FT.*z101.*z54+2.*L_SH.*z100.*z55-2.*L_TH.*z100.*z54-L_SH.*z101.*z53-L_TH.*z2.*z53);
z37 = z29 + z30.*z11 + z31.*z7 + z32.*z12 + z33.*z6 + 0.5.*z34.*z8 + 0.5.*z35.*z9 - 0.5.*z36.*z6 - 0.5.*z13.*(L_SH.*z12+2.*L_TH.*z6);
z124 = z102 - z37;
z147 = z124 - z139.*z122;
z210 = z147 - z202.*z123;
z263 = z210 - z256.*z201;
z331 = z295 - z325.*z263;
z70 = 0.5.*z64.*z17 + 0.5.*M_SH.*(L_SH.*z21+2.*L_TH.*z17) + 0.5.*M_FT.*(L_FT.*z18+2.*L_SH.*z21+2.*L_TH.*z17);
z178 = z70.*z175;
z19 = z4.*z17 - z3.*z16;
z80 = 0.5.*z64.*z16 + 0.5.*M_SH.*(L_SH.*z22+2.*L_TH.*z16) + 0.5.*M_FT.*(L_FT.*z19+2.*L_SH.*z22+2.*L_TH.*z16);
z241 = 0.5.*z178 + 0.5.*z80.*z238;
z133 = z70.*z130;
z196 = -0.5.*z133 - 0.5.*z80.*z193;
z290 = z241 - z196.*z288;
z142 = z70.*z139;
z205 = z142 + z80.*z202;
z258 = z205 - z196.*z256;
z326 = z290 - z258.*z325;
z148 = z70./z63;
z103 = cos(QRK);
z104 = sin(QRA);
z105 = z88 + 0.25.*M_SH.*(z89+4.*z90.*z103) + 0.25.*M_FT.*(z95+4.*z93+4.*z94+8.*z90.*z103-4.*z91.*z104-4.*z92.*z3);
z151 = z70.*z148 - z105;
z211 = z80./z63;
z214 = z151 + z80.*z211;
z149 = z75.*z148;
z212 = -0.5.*z149 - 0.5.*z83.*z211;
z264 = z212./z194;
z266 = z214 - z196.*z264;
z150 = z65.*z148;
z213 = z150 + z77.*z211;
z265 = z213 - z195.*z264;
z304 = z265./z257;
z305 = z266 - z258.*z304;
z351 = z326./z305;
z108 = sin(QRK);
z109 = cos(QRA);
z110 = 0.5.*M_SH.*z108.*(L_SH.*z61-L_TH.*z60) + 0.5.*M_FT.*(L_FT.*z4.*z61+L_FT.*z109.*z60+2.*L_SH.*z108.*z61-2.*L_TH.*z108.*z60-L_SH.*z109.*z59-L_TH.*z4.*z59);
z45 = z38 + z39.*z21 + z40.*z22 + z41.*z17 + z42.*z16 + 0.5.*z43.*z18 + 0.5.*z44.*z19 - 0.5.*z36.*z16 - 0.5.*z13.*(L_SH.*z22+2.*L_TH.*z16);
z125 = z110 - z45;
z156 = z125 - z148.*z122;
z219 = z156 - z211.*z123;
z271 = z219 - z264.*z201;
z310 = z271 - z304.*z263;
z356 = z331 - z351.*z310;
z67 = 0.5.*z66.*z11 + 0.5.*M_FT.*(L_FT.*z8+2.*L_SH.*z11);
z114 = I_FT + 0.25.*z68.*(L_FT-2.*L_SH.*z87);
z179 = 0.5.*z67.*z175 - z114;
z78 = 0.5.*z66.*z12 + 0.5.*M_FT.*(L_FT.*z9+2.*L_SH.*z12);
z242 = z179 + 0.5.*z78.*z238;
z134 = z67.*z130;
z197 = -0.5.*z134 - 0.5.*z78.*z193;
z291 = z242 - z197.*z288;
z98 = z97 + 0.25.*z66.*(L_SH+2.*L_TH.*z86) + 0.25.*M_FT.*(z95+4.*z93+4.*z90.*z86-4.*z91.*z87-2.*z92.*z1);
z143 = z67.*z139 - z98;
z206 = z143 + z78.*z202;
z259 = z206 - z197.*z256;
z327 = z291 - z259.*z325;
z152 = z67.*z148;
z215 = z152 + z78.*z211;
z267 = z215 - z197.*z264;
z306 = z267 - z259.*z304;
z352 = z327 - z306.*z351;
z157 = z67./z63;
z113 = z111 + 0.25.*M_FT.*(z112-4.*z91.*z87);
z161 = z67.*z157 - z113;
z220 = z78./z63;
z224 = z161 + z78.*z220;
z158 = z75.*z157;
z221 = -0.5.*z158 - 0.5.*z83.*z220;
z272 = z221./z194;
z275 = z224 - z197.*z272;
z159 = z65.*z157 - z98;
z222 = z159 + z77.*z220;
z273 = z222 - z195.*z272;
z311 = z273./z257;
z313 = z275 - z259.*z311;
z160 = z70.*z157;
z223 = z160 + z80.*z220;
z274 = z223 - z196.*z272;
z312 = z274 - z258.*z311;
z339 = z312./z305;
z340 = z313 - z306.*z339;
z368 = z352./z340;
z115 = 0.5.*z66.*z100.*z55 + 0.5.*M_FT.*(L_FT.*z2.*z55+L_FT.*z101.*z54+2.*L_SH.*z100.*z55-L_SH.*z101.*z53);
z48 = z46 + z30.*z11 + z32.*z12 + 0.5.*z34.*z8 + 0.5.*z35.*z9 - 0.5.*z47.*z12;
z126 = z115 - z48;
z165 = z126 - z157.*z122;
z228 = z165 - z220.*z123;
z279 = z228 - z272.*z201;
z317 = z279 - z311.*z263;
z344 = z317 - z339.*z310;
z372 = z356 - z368.*z344;
z71 = 0.5.*z66.*z21 + 0.5.*M_FT.*(L_FT.*z18+2.*L_SH.*z21);
z180 = z71.*z175;
z81 = 0.5.*z66.*z22 + 0.5.*M_FT.*(L_FT.*z19+2.*L_SH.*z22);
z243 = 0.5.*z180 + 0.5.*z81.*z238;
z135 = z71.*z130;
z198 = -0.5.*z135 - 0.5.*z81.*z193;
z292 = z243 - z198.*z288;
z144 = z71.*z139;
z207 = z144 + z81.*z202;
z260 = z207 - z198.*z256;
z328 = z292 - z260.*z325;
z106 = z97 + 0.25.*z66.*(L_SH+2.*L_TH.*z103) + 0.25.*M_FT.*(z95+4.*z93+4.*z90.*z103-4.*z91.*z104-2.*z92.*z3);
z153 = z71.*z148 - z106;
z216 = z153 + z81.*z211;
z268 = z216 - z198.*z264;
z307 = z268 - z260.*z304;
z353 = z328 - z307.*z351;
z162 = z71.*z157;
z225 = z162 + z81.*z220;
z276 = z225 - z198.*z272;
z314 = z276 - z260.*z311;
z341 = z314 - z307.*z339;
z369 = z353 - z341.*z368;
z166 = z71./z63;
z116 = z111 + 0.25.*M_FT.*(z112-4.*z91.*z104);
z171 = z71.*z166 - z116;
z229 = z81./z63;
z234 = z171 + z81.*z229;
z167 = z75.*z166;
z230 = -0.5.*z167 - 0.5.*z83.*z229;
z280 = z230./z194;
z284 = z234 - z198.*z280;
z168 = z65.*z166;
z231 = z168 + z77.*z229;
z281 = z231 - z195.*z280;
z318 = z281./z257;
z321 = z284 - z260.*z318;
z169 = z70.*z166 - z106;
z232 = z169 + z80.*z229;
z282 = z232 - z196.*z280;
z319 = z282 - z258.*z318;
z345 = z319./z305;
z347 = z321 - z307.*z345;
z170 = z67.*z166;
z233 = z170 + z78.*z229;
z283 = z233 - z197.*z280;
z320 = z283 - z259.*z318;
z346 = z320 - z306.*z345;
z363 = z346./z340;
z364 = z347 - z341.*z363;
z378 = z369./z364;
z118 = 0.5.*z66.*z108.*z61 + 0.5.*M_FT.*(L_FT.*z4.*z61+L_FT.*z109.*z60+2.*L_SH.*z108.*z61-L_SH.*z109.*z59);
z50 = z49 + z39.*z21 + z40.*z22 + 0.5.*z43.*z18 + 0.5.*z44.*z19 - 0.5.*z47.*z22;
z127 = z118 - z50;
z174 = z127 - z166.*z122;
z237 = z174 - z229.*z123;
z287 = z237 - z280.*z201;
z324 = z287 - z318.*z263;
z350 = z324 - z345.*z310;
z367 = z350 - z363.*z344;
z381 = z372 - z378.*z367;
z72 = z68.*z18;
z182 = z72.*z175;
z82 = z68.*z19;
z245 = 0.25.*z182 + 0.25.*z82.*z238;
z137 = z72.*z130;
z200 = -0.25.*z137 - 0.25.*z82.*z193;
z294 = z245 - z200.*z288;
z146 = z72.*z139;
z209 = 0.5.*z146 + 0.5.*z82.*z202;
z262 = z209 - z200.*z256;
z330 = z294 - z262.*z325;
z107 = I_FT + 0.25.*z68.*(L_FT-2.*L_SH.*z104-2.*L_TH.*z3);
z155 = 0.5.*z72.*z148 - z107;
z218 = z155 + 0.5.*z82.*z211;
z270 = z218 - z200.*z264;
z309 = z270 - z262.*z304;
z355 = z330 - z309.*z351;
z164 = z72.*z157;
z227 = 0.5.*z164 + 0.5.*z82.*z220;
z278 = z227 - z200.*z272;
z316 = z278 - z262.*z311;
z343 = z316 - z309.*z339;
z371 = z355 - z343.*z368;
z117 = I_FT + 0.25.*z68.*(L_FT-2.*L_SH.*z104);
z173 = 0.5.*z72.*z166 - z117;
z236 = z173 + 0.5.*z82.*z229;
z286 = z236 - z200.*z280;
z323 = z286 - z262.*z318;
z349 = z323 - z309.*z345;
z366 = z349 - z343.*z363;
z380 = z371 - z366.*z378;
z121 = z68.*(z4.*z61+z109.*z60);
z52 = T_RA + 0.5.*L_FT.*(RGRFX.*z18+z15.*z19);
z129 = 0.5.*z121 - z52;
z184 = z72./z63;
z192 = z129 - 0.5.*z184.*z122;
z247 = z82./z63;
z255 = z192 - 0.5.*z247.*z123;
z185 = z75.*z184;
z248 = -0.25.*z185 - 0.25.*z83.*z247;
z296 = z248./z194;
z303 = z255 - z296.*z201;
z186 = z65.*z184;
z249 = 0.5.*z186 + 0.5.*z77.*z247;
z297 = z249 - z195.*z296;
z332 = z297./z257;
z338 = z303 - z332.*z263;
z187 = 0.5.*z70.*z184 - z107;
z250 = z187 + 0.5.*z80.*z247;
z298 = z250 - z196.*z296;
z333 = z298 - z258.*z332;
z357 = z333./z305;
z362 = z338 - z357.*z310;
z188 = z67.*z184;
z251 = 0.5.*z188 + 0.5.*z78.*z247;
z299 = z251 - z197.*z296;
z334 = z299 - z259.*z332;
z358 = z334 - z306.*z357;
z373 = z358./z340;
z377 = z362 - z373.*z344;
z189 = 0.5.*z71.*z184 - z117;
z252 = z189 + 0.5.*z81.*z247;
z300 = z252 - z198.*z296;
z335 = z300 - z260.*z332;
z359 = z335 - z307.*z357;
z374 = z359 - z341.*z373;
z382 = z374./z364;
z385 = z377 - z382.*z367;
z190 = z69.*z184;
z253 = 0.25.*z190 + 0.25.*z79.*z247;
z136 = z69.*z130;
z199 = -0.25.*z136 - 0.25.*z79.*z193;
z301 = z253 - z199.*z296;
z145 = 0.5.*z69.*z139 - z99;
z208 = z145 + 0.5.*z79.*z202;
z261 = z208 - z199.*z256;
z336 = z301 - z261.*z332;
z154 = z69.*z148;
z217 = 0.5.*z154 + 0.5.*z79.*z211;
z269 = z217 - z199.*z264;
z308 = z269 - z261.*z304;
z360 = z336 - z308.*z357;
z163 = 0.5.*z69.*z157 - z114;
z226 = z163 + 0.5.*z79.*z220;
z277 = z226 - z199.*z272;
z315 = z277 - z261.*z311;
z342 = z315 - z308.*z339;
z375 = z360 - z342.*z373;
z172 = z69.*z166;
z235 = 0.5.*z172 + 0.5.*z79.*z229;
z285 = z235 - z199.*z280;
z322 = z285 - z261.*z318;
z348 = z322 - z308.*z345;
z365 = z348 - z342.*z363;
z383 = z375 - z365.*z382;
z181 = 0.25.*z69.*z175 - z119;
z244 = z181 + 0.25.*z79.*z238;
z293 = z244 - z199.*z288;
z329 = z293 - z261.*z325;
z354 = z329 - z308.*z351;
z370 = z354 - z342.*z368;
z379 = z370 - z365.*z378;
z386 = z383./z379;
z388 = z385 - z386.*z381;
z191 = 0.25.*z72.*z184 - z119;
z254 = z191 + 0.25.*z82.*z247;
z302 = z254 - z200.*z296;
z337 = z302 - z262.*z332;
z361 = z337 - z309.*z357;
z376 = z361 - z343.*z373;
z384 = z376 - z366.*z382;
z387 = z384 - z380.*z386;
z389 = z388./z387;
z390 = (z381-z380.*z389)./z379;
z391 = (z367-z365.*z390-z366.*z389)./z364;
z392 = (z344-z341.*z391-z342.*z390-z343.*z389)./z340;
z393 = (z310-z306.*z392-z307.*z391-z308.*z390-z309.*z389)./z305;
z394 = (z263-z258.*z393-z259.*z392-z260.*z391-z261.*z390-z262.*z389)./z257;
z395 = (z201-z195.*z394-z196.*z393-z197.*z392-z198.*z391-z199.*z390-z200.*z389)./z194;
z397 = (2.*z122+z69.*z390+z72.*z389+2.*z65.*z394+2.*z67.*z392+2.*z70.*z393+2.*z71.*z391-z75.*z395)./z63;
U1p = -0.5.*z397;
z396 = (2.*z123+z79.*z390+z82.*z389+2.*z77.*z394+2.*z78.*z392+2.*z80.*z393+2.*z81.*z391-z83.*z395)./z63;
U2p = -0.5.*z396;
U3p = z395;
U4p = z394;
U5p = z393;
U6p = z392;
U7p = z391;
U8p = z390;
U9p = z389;

% Assign to generalized accelerations
XPP = U1p;
YPP = U2p;
QTPP = U3p;
QLHPP = U4p;
QRHPP = U5p;
QLKPP = U6p;
QRKPP = U7p;
QLAPP = U8p;
QRAPP = U9p;
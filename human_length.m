%--------------------------------------------------
% kecepatan angular (iR0wi)
R10w1 = R10*(z0*tv1);
R20w2 = R21*((R10w1)+(z0*tv2));
R30w3 = R32*((R20w2)+(z0*tv3));
R40w4 = R43*((R30w3)+(z0*tv4));
R50w5 = R54*((R40w4)+(z0*tv5));
R60w6 = R65*((R50w5)+(z0*tv6));
R70w7 = R76*((R60w6)+(z0*tv7));
%--------------------------------------------------

%--------------------------------------------------
% percepatan angular (iR0wti)
R10wt1 = R10*(z0*ta1);
R20wt2 = R21*(R10wt1 + (z0*ta2) + cross(R10w1, z0*tv2));
R30wt3 = R32*(R20wt2 + (z0*ta3) + cross(R20w2, z0*tv3));
R40wt4 = R43*(R30wt3 + (z0*ta4) + cross(R30w3, z0*tv4));
R50wt5 = R54*(R40wt4 + (z0*ta5) + cross(R40w4, z0*tv5));
R60wt6 = R65*(R50wt5 + (z0*ta6) + cross(R50w5, z0*tv6));
R70wt7 = R76*(R60wt6 + (z0*ta7) + cross(R60w6, z0*tv7));
%--------------------------------------------------

%--------------------------------------------------
% percepatan linear (iR0vti)
R10vt1 = cross(R10wt1, R10p1) + cross(R10w1, cross(R10w1, R10p1)) + (R10 * g);
R20vt2 = cross(R20wt2, R20p2) + cross(R20w2, cross(R20w2, R20p2)) + (R21 * R10vt1);
R30vt3 = cross(R30wt3, R30p3) + cross(R30w3, cross(R30w3, R30p3)) + (R32 * R20vt2);
R40vt4 = cross(R40wt4, R40p4) + cross(R40w4, cross(R40w4, R40p4)) + (R43 * R30vt3);
R50vt5 = cross(R50wt5, R50p5) + cross(R50w5, cross(R50w5, R50p5)) + (R54 * R40vt4);
R60vt6 = cross(R60wt6, R60p6) + cross(R60w6, cross(R60w6, R60p6)) + (R65 * R50vt5);
R70vt7 = cross(R70wt7, R70p7) + cross(R70w7, cross(R70w7, R70p7)) + (R76 * R60vt6);
%--------------------------------------------------

%--------------------------------------------------
% percepatan linear di TPM (iR0ai)
R10a1 = cross(R10wt1, R10s1) + cross(R10w1, cross(R10w1, R10s1)) + R10vt1;
R20a2 = cross(R20wt2, R20s2) + cross(R20w2, cross(R20w2, R20s2)) + R20vt2;
R30a3 = cross(R30wt3, R30s3) + cross(R30w3, cross(R30w3, R30s3)) + R30vt3;
R40a4 = cross(R40wt4, R40s4) + cross(R40w4, cross(R40w4, R40s4)) + R40vt4;
R50a5 = cross(R50wt5, R50s5) + cross(R50w5, cross(R50w5, R50s5)) + R50vt5;
R60a6 = cross(R60wt6, R60s6) + cross(R60w6, cross(R60w6, R60s6)) + R60vt6;
R70a7 = cross(R70wt7, R70s7) + cross(R70w7, cross(R70w7, R70s7)) + R70vt7;
%--------------------------------------------------

%--------------------------------------------------
% gaya fi (iR0fi)
R70f7 = m7 * R70a7;
R60f6 = R67 * R70f7 + (m6 * R60a6);
R50f5 = R56 * R60f6 + (m5 * R50a5);
R40f4 = R45 * R50f5 + (m4 * R40a4);
R30f3 = R34 * R40f4 + (m3 * R30a3);
R20f2 = R23 * R30f3 + (m2 * R20a2);
R10f1 = R12 * R20f2 + (m1 * R10a1);
%--------------------------------------------------

%--------------------------------------------------
% gaya ni (iR0ni)
R70n7 = cross(((R70p7) + (R70s7)), m7*R70a7) + (R70I7R07 * R70wt7);
R60n6 = (R67 * (R70n7 + cross(R76*R60p6, R70f7))) + cross(((R60p6) + (R60s6)), m6*R60a6) + (R60I6R06 * R60wt6);
R50n5 = (R56 * (R60n6 + cross(R65*R50p5, R60f6))) + cross(((R50p5) + (R50s5)), m5*R50a5) + (R50I5R05 * R50wt5);
R40n4 = (R45 * (R50n5 + cross(R54*R40p4, R50f5))) + cross(((R40p4) + (R40s4)), m4*R40a4) + (R40I4R04 * R40wt4);
R30n3 = (R34 * (R40n4 + cross(R43*R30p3, R40f4))) + cross(((R30p3) + (R30s3)), m3*R30a3) + (R30I3R03 * R30wt3);
R20n2 = (R23 * (R30n3 + cross(R32*R20p2, R30f3))) + cross(((R20p2) + (R20s2)), m2*R20a2) + (R20I2R02 * R20wt2);
R10n1 = (R12 * (R20n2 + cross(R21*R10p1, R20f2))) + cross(((R10p1) + (R10s1)), m1*R10a1) + (R10I1R01 * R10wt1);
%--------------------------------------------------

%--------------------------------------------------
% torque (to1)
to1 = transpose(R10n1) * (R10*z0); %+ b1*tv1;
to2 = transpose(R20n2) * (R21*z0); %+ b2*tv2;
to3 = transpose(R30n3) * (R32*z0); %+ b2*tv2;
to4 = transpose(R40n4) * (R43*z0); %+ b2*tv2;
to5 = transpose(R50n5) * (R54*z0); %+ b2*tv2;
to6 = transpose(R60n6) * (R65*z0); %+ b2*tv2;
to7 = transpose(R70n7) * (R76*z0); %+ b2*tv2;
%--------------------------------------------------
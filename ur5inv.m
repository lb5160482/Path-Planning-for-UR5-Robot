function Q = ur5inv(gd)
T06 = gd;
Solution = zeros(6,8);
%%%%%%%%%%%%%%%theta1%%%%%%%%%%%%%%%%%%%%
P05 = T06 * [0;0;-0.0623;1] - [0;0;0;1];
a1 = atan2(P05(2),P05(1));
P05xy = sqrt(P05(2)^2 + P05(1)^2);
a2 = acos(0.11/P05xy);
theta1 = [mod((a1 + a2 + pi),2*pi) mod((a1 - a2 + pi),2*pi)];
Solution(1,1:4) = theta1(1);
Solution(1,5:8) = theta1(2);
%%%%%%%%%%%%%%%theta5%%%%%%%%%%%%%%%%%%%%
for i = 1:2
    THETA1 = theta1(i);
    T01 = [[cos(THETA1) -sin(THETA1) 0;sin(THETA1) cos(THETA1) 0;0 0 1],[0;0;0];0 0 0 1]...
        *[0 0 -1 -0.0703;0 1 0 0;1 0 0 0.0892;0 0 0 1];
    T16 = (inv(T01))*T06;
    P16 = T16(1:4,4); 
    theta5 = [acos(((P16(3)+0.0703) - 0.11)/0.0623) 2*pi-acos(((P16(3)+0.0703) - 0.11)/0.0623)];
    Solution(5,4*(i-1)+1:4*(i-1)+4) = [theta5 theta5];
end
%%%%%%%%%%%%%%%theta6%%%%%%%%%%%%%%%%%%%%
for i = 1:8
   THETA1 = Solution(1,i);
   THETA5 = Solution(5,i);
   T01 = [[cos(THETA1) -sin(THETA1) 0;sin(THETA1) cos(THETA1) 0;0 0 1],[0;0;0];0 0 0 1]...
    *[0 0 -1 -0.0703;0 1 0 0;1 0 0 0.0892;0 0 0 1];
   T61 = inv(inv(T01)*T06);
   THETA6 = atan2(T61(1,3)/sin(THETA5),T61(2,3)/sin(THETA5));
   if THETA6 < 0
       THETA6 = THETA6 + 2*pi;
   end
   Solution(6,i) = THETA6;
end
%%%%%%%%%%%%%%%theta3%%%%%%%%%%%%%%%%%%%%
for i = 1:2
    THETA1 = theta1(i);
    for j = 1:2
        THETA5 = Solution(5,(i-1)*4+j);
        THETA6 = Solution(6,(i-1)*4+j);
        T01 = [[cos(THETA1) -sin(THETA1) 0;sin(THETA1) cos(THETA1) 0;0 0 1],[0;0;0];0 0 0 1]...
            *[0 0 -1 -0.0703;0 1 0 0;1 0 0 0.0892;0 0 0 1];
        T16 = (inv(T01))*T06;
        T45 = [[cos(THETA5) -sin(THETA5) 0;sin(THETA5) cos(THETA5) 0;0 0 1],[0;0;0];0 0 0 1]...
            *[0 0 -1 0.0144;0 1 0 0;1 0 0 0.0492;0 0 0 1];
        T56 = [[cos(THETA6) -sin(THETA6) 0;sin(THETA6) cos(THETA6) 0;0 0 1],[0;0;0];0 0 0 1]...
            *[1 0 0 0;0 1 0 0;0 0 1 0.0767;0 0 0 1];
        T14 = T16*(inv(T45*T56));
        P13 = T14*[0.0397;0;-0.0456;1]-[0;0;0;1];
        A = (P13.'*P13-0.4251^2-0.3921^2)/(2*0.4251*0.3921);
        THETA3_1 = 2*pi-acos(A);
        THETA3_2 = acos(A);
        Solution(3,(i-1)*4+j) = THETA3_1;
        Solution(3,(i-1)*4+j+2) = THETA3_2;
    end
end
%%%%%%%%%%%%%%%theta2 & theta4%%%%%%%%%%%%%%%%%%%%
for i = 1:8
    THETA1 = Solution(1,i);
    THETA3 = Solution(3,i);
    THETA5 = Solution(5,i);
    THETA6 = Solution(6,i);
    T01 = [[cos(THETA1) -sin(THETA1) 0;sin(THETA1) cos(THETA1) 0;0 0 1],[0;0;0];0 0 0 1]...
        *[0 0 -1 -0.0703;0 1 0 0;1 0 0 0.0892;0 0 0 1];
    T16 = (inv(T01))*T06;
    T45 = [[cos(THETA5) -sin(THETA5) 0;sin(THETA5) cos(THETA5) 0;0 0 1],[0;0;0];0 0 0 1]...
        *[0 0 -1 0.0144;0 1 0 0;1 0 0 0.0492;0 0 0 1];
    T56 = [[cos(THETA6) -sin(THETA6) 0;sin(THETA6) cos(THETA6) 0;0 0 1],[0;0;0];0 0 0 1]...
        *[1 0 0 0;0 1 0 0;0 0 1 0.0767;0 0 0 1];
    T14 = T16*(inv(T45*T56));
    P13 = T14*[0.0397;0;-0.0456;1]-[0;0;0;1];
    delta = atan2(P13(2),P13(1));
    epsilon = asin(0.3921*sin(THETA3)/norm(P13));
    THETA2 = delta - epsilon;
    if THETA2 < 0
        THETA2 = THETA2 + 2*pi;
    end
    Solution(2,i) = THETA2;
    %theta4
    T12 = [[cos(THETA2) -sin(THETA2) 0;sin(THETA2) cos(THETA2) 0;0 0 1],[0;0;0];0 0 0 1]...
        *[1 0 0 0.4251;0 1 0 0;0 0 1 0;0 0 0 1];
    T23 = [[cos(THETA3) -sin(THETA3) 0;sin(THETA3) cos(THETA3) 0;0 0 1],[0;0;0];0 0 0 1]...
        *[1 0 0 0.3921;0 1 0 0;0 0 1 0;0 0 0 1];
    T34 = (inv(T12*T23))*T14;
    THETA4 = atan2(T34(2,3),T34(2,2));
    if THETA4 < 0
        THETA4 = THETA4 + 2*pi;
    end
    Solution(4,i) = THETA4;
end

Q = Solution;
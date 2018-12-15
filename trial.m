A1_circle = [0 1 0 0;
             0 0 0 0;
             0 0 0 0;
             0 0 0 0];

B1_circle = [0 0 0;
             1 0 0;
             0 4 0;
             0 0 4];
  
  R1_circle = [1 0 0;
      0 1 0;
      0 0 1];
  Q1_circle = [1 0 0 0;
               0 1 0 0;
               0 0 1 0;
               0 0 0 1];
           
  K1_circle  = lqr(A1_circle,B1_circle,Q1_circle,R1_circle)
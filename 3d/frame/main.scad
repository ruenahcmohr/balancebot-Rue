

 outline = "main.dxf";
 
 L0 = 80; 
 L1 = 50; 
 L2 = 60;
 L3 = 3;
 
 L10 = 100;
 
 L20 = 40;
 
 L30 = 1.5;
 L31 = 11;
 L32 = 1.5;
 
 L40 = 2;
 L41 = 3.5;
 
 L100 = 1;
 L100p5 = 5;
 L101 = 18;
 L102 = 2;
 L103d = 16;
 L104d = 100;
 L105d = 0.4;

difference() {
intersection() {
    
 translate([0,0,-40])
 linear_extrude(height=L0, convexity=5) {
   import(file=outline, layer="0", $fn=600);
 } 
 

rotate([-90,0,0])
translate([0,0,-20]) 
 linear_extrude(height=L2, convexity=5) {
   import(file=outline, layer="2", $fn=600);
 }

}


translate([-16,0,0]) 
rotate([0,0,-45])
rotate([-90,0,0])
 linear_extrude(height=L1, convexity=5) {
   import(file=outline, layer="1", $fn=600);
 }
 
 translate([0,0,-19.1]) 
 linear_extrude(height=L3, convexity=5) {
   import(file=outline, layer="3", $fn=600);
 }
 
 translate([0,0,19-3+0.1]) 
 linear_extrude(height=L3, convexity=5) {
   import(file=outline, layer="3", $fn=600);
 }
 
 }
 
 
 
/*
     translate([0,0,-L4])
     linear_extrude(height=L4, convexity=5) {
       import(file=outline, layer="4", $fn=90);
     }  

     translate([0,0,-L5])
     linear_extrude(height=L5, convexity=5) {
       import(file=outline, layer="5", $fn=188);
     }  
}
*/
/*
     //2x dovetail
     translate([-40.64,0,L10/2])
     rotate([-90,90,-90])    
     linear_extrude(height=L20, convexity=5) {
       import(file=outline, layer="20", $fn=188);
     }
 } // end of the union
 
     
     // 3x bolt
     color([1,0,0])
  translate([0,-0.005,50])
  rotate([-90,0,0]) {
     linear_extrude(height=L30, convexity=5) {
       import(file=outline, layer="30", $fn=188);
     }
     
     translate([0,0,L30-0.006])
     linear_extrude(height=L31, convexity=5) {
       import(file=outline, layer="31", $fn=188);
     }
 }
 
  // 4x side bolt
 color([0, 1, 0])
      translate([0,0,57.005])
      linear_extrude(height=L40, convexity=5) {
       import(file=outline, layer="40", $fn=188);
     }
 
     




  translate([0,0,L0+L1])
     linear_extrude(height=L2, convexity=5) {
       import(file=outline, layer="2", $fn=188);
     }




     translate([10, -0, 25/2])
     rotate([0,90,0])
     cylinder(h = 40, r =2.0, $fn=100, center=true);

}
*/


outline = "main.dxf";
 L0 = 1;
 L1 = 10;
 L2 = 2;


 L3 = 0.5;
 
 L10 = 1;
 L11 = 2;
 L12 = 17;
 
 L100 = 2;
 L101 = 5-L100;
 
 L200 = 1;
 L201 = 4;
 L202 = 0.75;

module printedFace() {
    difference() {
    union() {
  linear_extrude(height=L100, convexity=5) {
      import(file=outline, layer="100", $fn=188);
  } 
  translate([0,0,L100])
  linear_extrude(height=L101, convexity=5) {
      import(file=outline, layer="101", $fn=188);
  } 
  }
    translate([0,0,-0.01])
    linear_extrude(height=1.5, convexity=5) {
      import(file=outline, layer="102", $fn=188);
  } 
  }
}


module test() {
translate([0,0,10-4oh ])
color([0,0,1])
import( "/morfiles/objects/rueEncoderWheel/rueEncoderWheel-r7-s.stl");

/*
 difference() {
  union() {
    backBox();
    translate([0,0,11])
    pcb();
  } 
  translate([0,-20,-0.1])
  cube([20,50,30]);
}
*/
}



module backBox() {
 difference() {
    union() {
    linear_extrude(height=L0, convexity=5) {
       import(file=outline, layer="10", $fn=188);
     }

  translate([0,0,L0])
     linear_extrude(height=L1, convexity=5) {
       import(file=outline, layer="11", $fn=188);
     }

  translate([0,0,L0+L1])
     linear_extrude(height=L2, convexity=5) {
       import(file=outline, layer="12", $fn=188);
     }
 }
 
  translate([-10,0,(L0+L1+L2)/2]) 
  rotate([0,90,0])
  cylinder(h=10, r=4, center=true, $fn=200); 

 translate([0,0,0.25]) 
 cylinder(h=5, r=1.7/2, , $fn=200);
 
 }
 
 }
 
 module pinCapture() {
   union() {  
 linear_extrude(height=L200, convexity=15) {
   import(file=outline, layer="200", $fn=128);
 }
 
 translate([0,0,L200-0.001])
 linear_extrude(height=L201, convexity=15) {
   import(file=outline, layer="201", $fn=128);
 }
 }

translate([0,0,L201+L200-0.002])
 linear_extrude(height=L202, convexity=15) {
   import(file=outline, layer="202", $fn=128);
 } 

     
     
 }
 
 
 //pinCapture();
 
 backBox();
 
 //translate([0,0,L0+L1])
 //printedFace();


/*
  translate([0,0,L0+L1+L2])
   linear_extrude(height=L3, convexity=5) {
       import(file=outline, layer="3", $fn=188);
     }

		linear_extrude(height=1, convexity=15) {
		     import(file=outline, layer="10b", $fn=128);
		}
		

   
      translate([0,0,2.9999])
		linear_extrude(height=10, convexity=15) {
		     import(file=outline, layer="11", $fn=128);
		}
      
      translate([0,0,12.999])
		linear_extrude(height=2, convexity=15) {
		     import(file=outline, layer="12", $fn=128);
		}

        
        
        translate([-20,0,6])
        rotate([0,90,0])
		cylinder(h=40, r=2, center=true, $fn=200);
    }
		
        translate([0,0,14.89999])
		linear_extrude(height=1.5, convexity=5) {
		     import(file=outline, layer="3", $fn=228);
		}		
        
        */
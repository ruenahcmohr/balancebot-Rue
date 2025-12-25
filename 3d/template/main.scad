 // 2d paths file
 outline = "main.dxf";
 
 // layer heights from 2d file for stacking
 L0 = 8; 
 L1 = 2; 
 L2 = 6;
 L3 = 5;
 

// normal layer stacking

  linear_extrude(height=L0, convexity=5) {
    import(file=outline, layer="0", $fn=90);
  } 

  translate([0,0,L0-0.001]) // subtract 0.001 from L0 to definitly merge the objects
  linear_extrude(height=L1, convexity=5) {
    import(file=outline, layer="1", $fn=90);
  } 


 
 
 
/*
     lines for copy/paste coding :]
  
union() {       // merge group
difference() {  // cut objects from first.   

color([1,0,0])
translate([0,0,0])
rotate([0,0,0]) 

linear_extrude(height=L20, convexity=5) { // extrudes along Z
  import(file=outline, layer="20", $fn=188);
}

rotate_extrude( convexity=5, $fn=188) { // Around Y. keep outline 0.001+ to the right of 0 in 2d.
  import(file=outline, layer="20", $fn=188);
}     

cylinder(h = 40, d = 2.0, $fn=100, center=true); // 2mm drill, 40mm deep


*/

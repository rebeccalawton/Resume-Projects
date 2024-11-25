//Rebecca Lawton

`timescale 1ns/10ps
  module MAC(IN, W, clk, rstb, OUT);
   input signed [3:0] IN, W;
   input 	      clk, rstb;
   output reg signed [11:0]  OUT;
   reg signed [11:0] 	      A,B;
   
   reg [3:0] 	      counter;
   
   
   
   always @ (posedge clk or negedge rstb)
	if(!rstb) begin
	  A = 0;
	B = 0;
	OUT =  0;
	   counter =  0;
	   
	   end
	else if(counter == 9) begin
	   B = IN * W;
	OUT = IN * W;
	    A = IN * W;
	    counter =  1;
	 end
	 else begin
	    A =  W * IN;
	    B =  B + A;
	    counter = counter + 1;
	    OUT = B;
	    
	 end 
	   
endmodule // MAC

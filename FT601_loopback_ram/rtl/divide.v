/////////////////////////////////////////////////////////////////////////////
//           _____       
//          / _______    Copyright (C) 2013-2020 Efinix Inc. All rights reserved.
//         / /       \   
//        / /  ..    /   divide.v
//       / / .'     /    
//    __/ /.'      /     Description:
//   __   \       /      Include file for generic fifo required functions
//  /_/ /\ \_____/ /     
// ____/  \_______/      
//
// *******************************
// Revisions:
// 1.0 Initial rev
//
// *******************************
 module Divide
 #(
   parameter WIDTH = 32
 )
 (  

input           clk,  
input           rstn,  
input           start,  
input   [WIDTH-1:0]  N,  
input   [WIDTH-1:0]  D,  
output  [WIDTH-1:0]  Q,  
output  [WIDTH-1:0]  R
);  


reg         active;
reg [31:0]   cycle;
reg [WIDTH-1:0]  result;
reg [WIDTH-1:0]  denom; 
reg [WIDTH-1:0]  buff;

   
wire [WIDTH:0]   sub = { buff[WIDTH-2:0], result[WIDTH-1] } - denom;  

assign Q = result;  
assign R = buff;  
 
 
always @(posedge clk, negedge rstn) 
begin  
  if (~rstn) 
	begin  
    active <= 0;  
    cycle <= 0;  
    result <= 0;  
    denom <= 0;  
    buff <= 0;  
  end  
  else if(start) 
    begin  
    if (active) 
      begin      
      if (sub[WIDTH] == 0) 
        begin  
        buff <= sub[WIDTH-1:0];  
        result <= {result[WIDTH-2:0], 1'b1};  
        end  
      else 
        begin  
        buff <= {buff[WIDTH-2:0], result[WIDTH-1]};  
        result <= {result[WIDTH-2:0], 1'b0};  
        end  
        if (cycle == 0) 
          begin  
          active <= 0;  
          end  
        cycle <= cycle - 1;  
      end        
    end
    else 
      begin           
        cycle <= WIDTH-1;  
        result <= N;  
        denom <= D;  
        buff <=  0;  
        active <= 1;  
      end
  end
 endmodule
------------- Begin Cut here for COMPONENT Declaration ------
component edb_top
  port (
         bscan_CAPTURE : in  std_logic;
         bscan_DRCK    : in  std_logic;
         bscan_RESET   : in  std_logic;
         bscan_RUNTEST : in  std_logic;
         bscan_SEL     : in  std_logic;
         bscan_SHIFT   : in  std_logic;
         bscan_TCK     : in  std_logic;
         bscan_TDI     : in  std_logic;
         bscan_TMS     : in  std_logic;
         bscan_UPDATE  : in  std_logic;
         bscan_TDO     : out std_logic;
         la0_clk       : in  std_logic;
         la0_rstn      : in  std_logic;
         la0_DDR_ATYPE_0 : in  std_logic;
         la0_DDR_AID_0 : in  std_logic_vector(7 downto 0);
         la0_DDR_AADDR_0 : in  std_logic_vector(31 downto 0);
         la0_DDR_ASIZE_0 : in  std_logic_vector(2 downto 0);
         la0_DDR_ALEN_0 : in  std_logic_vector(7 downto 0);
         la0_DDR_ABURST_0 : in  std_logic_vector(1 downto 0);
         la0_DDR_AVALID_0 : in  std_logic;
         la0_DDR_AREADY_0 : in  std_logic;
         la0_DDR_WDATA_0 : in  std_logic_vector(31 downto 0);
         la0_DDR_WDATA_1 : in  std_logic_vector(31 downto 0);
         la0_DDR_WDATA_2 : in  std_logic_vector(31 downto 0);
         la0_DDR_WDATA_3 : in  std_logic_vector(31 downto 0);
         la0_DDR_WDATA_4 : in  std_logic_vector(31 downto 0);
         la0_DDR_WDATA_5 : in  std_logic_vector(31 downto 0);
         la0_DDR_WDATA_6 : in  std_logic_vector(31 downto 0);
         la0_DDR_WDATA_7 : in  std_logic_vector(31 downto 0);
         la0_DDR_WVALID_0 : in  std_logic;
         la0_DDR_WREADY_0 : in  std_logic;
         la0_DDR_WLAST_0 : in  std_logic;
         la0_DDR_WSTRB_0 : in  std_logic_vector(31 downto 0);
         la0_DDR_BID_0 : in  std_logic_vector(7 downto 0);
         la0_DDR_BVALID_0 : in  std_logic;
         la0_DDR_BREADY_0 : in  std_logic;
         la0_DDR_RID_0 : in  std_logic_vector(7 downto 0);
         la0_DDR_RDATA_0 : in  std_logic_vector(31 downto 0);
         la0_DDR_RDATA_1 : in  std_logic_vector(31 downto 0);
         la0_DDR_RDATA_2 : in  std_logic_vector(31 downto 0);
         la0_DDR_RDATA_3 : in  std_logic_vector(31 downto 0);
         la0_DDR_RDATA_4 : in  std_logic_vector(31 downto 0);
         la0_DDR_RDATA_5 : in  std_logic_vector(31 downto 0);
         la0_DDR_RDATA_6 : in  std_logic_vector(31 downto 0);
         la0_DDR_RDATA_7 : in  std_logic_vector(31 downto 0);
         la0_DDR_RRESP_0 : in  std_logic_vector(1 downto 0);
         la0_DDR_RVALID_0 : in  std_logic;
         la0_DDR_RREADY_0 : in  std_logic;
         la0_DDR_RLAST_0 : in  std_logic;
         la0_DDR_ALOCK_0 : in  std_logic_vector(1 downto 0);
         la0_DDR_WID_0 : in  std_logic_vector(7 downto 0);
         la0_TRIG      : in  std_logic;
         la0_STATE     : in  std_logic_vector(3 downto 0);
         la0_PATTERN   : in  std_logic_vector(31 downto 0);
         la0_PATTERN_NUM : in  std_logic_vector(3 downto 0);
         la0_OUT_PATTERN : in  std_logic_vector(15 downto 0);
         la0_PATTERN_DONE : in  std_logic;
         la0_CURRENT_PATTERN : in  std_logic_vector(6 downto 0);
         la0_START     : in  std_logic;
         la0_Timer_counter : in  std_logic_vector(63 downto 0);
         la0_WR_Counte : in  std_logic_vector(7 downto 0);
         la0_total_len : in  std_logic_vector(63 downto 0);
         la0_LOOP_N    : in  std_logic_vector(32 downto 0);
         la0_lfsr1     : in  std_logic_vector(127 downto 0);
         la0_ERROR     : in  std_logic;
         la0_ddr_rst_done : in  std_logic;
         la0_pause     : in  std_logic;
         vio0_clk      : in  std_logic;
         vio0_Total_ALEN : in  std_logic_vector(63 downto 0);
         vio0_Cycle_Counter : in  std_logic_vector(63 downto 0);
         vio0_Test_Done : in  std_logic;
         vio0_Band_Width_MBs : in  std_logic_vector(31 downto 0);
         vio0_ERROR    : in  std_logic;
         vio0_LOOP_N   : out std_logic_vector(31 downto 0);
         vio0_PATTERN  : out std_logic_vector(15 downto 0);
         vio0_PATTERN_DEPTH : out std_logic_vector(3 downto 0);
         vio0_PATTERN_LEN0 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN1 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN2 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN3 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN4 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN5 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN6 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN7 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN8 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN9 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN10 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN11 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN12 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN13 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN14 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_LEN15 : out std_logic_vector(7 downto 0);
         vio0_PATTERN_ADDR0 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR1 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR2 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR3 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR4 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR5 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR6 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR7 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR8 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR9 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR10 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR11 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR12 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR13 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR14 : out std_logic_vector(31 downto 0);
         vio0_PATTERN_ADDR15 : out std_logic_vector(31 downto 0);
         vio0_Reset    : out std_logic;
         vio0_Pause    : out std_logic
       );
end component ;
---------------------- End COMPONENT Declaration ------------

-- The following code must appear in the VHDL architecture
-- body. Substitute your own instance name and net names.

------------- Begin Cut here for INSTANTIATION Template -----
edb_top_inst : edb_top
port map (
           bscan_CAPTURE => jtag_inst1_CAPTURE,
           bscan_DRCK    => jtag_inst1_DRCK,
           bscan_RESET   => jtag_inst1_RESET,
           bscan_RUNTEST => jtag_inst1_RUNTEST,
           bscan_SEL     => jtag_inst1_SEL,
           bscan_SHIFT   => jtag_inst1_SHIFT,
           bscan_TCK     => jtag_inst1_TCK,
           bscan_TDI     => jtag_inst1_TDI,
           bscan_TMS     => jtag_inst1_TMS,
           bscan_UPDATE  => jtag_inst1_UPDATE,
           bscan_TDO     => jtag_inst1_TDO,
           la0_clk      => #INSERT_YOUR_CLOCK_NAME,
           la0_rstn => la0_rstn,
           la0_DDR_ATYPE_0  => la0_DDR_ATYPE_0,
           la0_DDR_AID_0    => la0_DDR_AID_0,
           la0_DDR_AADDR_0  => la0_DDR_AADDR_0,
           la0_DDR_ASIZE_0  => la0_DDR_ASIZE_0,
           la0_DDR_ALEN_0   => la0_DDR_ALEN_0,
           la0_DDR_ABURST_0 => la0_DDR_ABURST_0,
           la0_DDR_AVALID_0 => la0_DDR_AVALID_0,
           la0_DDR_AREADY_0 => la0_DDR_AREADY_0,
           la0_DDR_WDATA_0  => la0_DDR_WDATA_0,
           la0_DDR_WDATA_1  => la0_DDR_WDATA_1,
           la0_DDR_WDATA_2  => la0_DDR_WDATA_2,
           la0_DDR_WDATA_3  => la0_DDR_WDATA_3,
           la0_DDR_WDATA_4  => la0_DDR_WDATA_4,
           la0_DDR_WDATA_5  => la0_DDR_WDATA_5,
           la0_DDR_WDATA_6  => la0_DDR_WDATA_6,
           la0_DDR_WDATA_7  => la0_DDR_WDATA_7,
           la0_DDR_WVALID_0 => la0_DDR_WVALID_0,
           la0_DDR_WREADY_0 => la0_DDR_WREADY_0,
           la0_DDR_WLAST_0  => la0_DDR_WLAST_0,
           la0_DDR_WSTRB_0  => la0_DDR_WSTRB_0,
           la0_DDR_BID_0    => la0_DDR_BID_0,
           la0_DDR_BVALID_0 => la0_DDR_BVALID_0,
           la0_DDR_BREADY_0 => la0_DDR_BREADY_0,
           la0_DDR_RID_0    => la0_DDR_RID_0,
           la0_DDR_RDATA_0  => la0_DDR_RDATA_0,
           la0_DDR_RDATA_1  => la0_DDR_RDATA_1,
           la0_DDR_RDATA_2  => la0_DDR_RDATA_2,
           la0_DDR_RDATA_3  => la0_DDR_RDATA_3,
           la0_DDR_RDATA_4  => la0_DDR_RDATA_4,
           la0_DDR_RDATA_5  => la0_DDR_RDATA_5,
           la0_DDR_RDATA_6  => la0_DDR_RDATA_6,
           la0_DDR_RDATA_7  => la0_DDR_RDATA_7,
           la0_DDR_RRESP_0  => la0_DDR_RRESP_0,
           la0_DDR_RVALID_0 => la0_DDR_RVALID_0,
           la0_DDR_RREADY_0 => la0_DDR_RREADY_0,
           la0_DDR_RLAST_0  => la0_DDR_RLAST_0,
           la0_DDR_ALOCK_0  => la0_DDR_ALOCK_0,
           la0_DDR_WID_0    => la0_DDR_WID_0,
           la0_TRIG => la0_TRIG,
           la0_STATE    => la0_STATE,
           la0_PATTERN  => la0_PATTERN,
           la0_PATTERN_NUM  => la0_PATTERN_NUM,
           la0_OUT_PATTERN  => la0_OUT_PATTERN,
           la0_PATTERN_DONE => la0_PATTERN_DONE,
           la0_CURRENT_PATTERN  => la0_CURRENT_PATTERN,
           la0_START    => la0_START,
           la0_Timer_counter    => la0_Timer_counter,
           la0_WR_Counte    => la0_WR_Counte,
           la0_total_len    => la0_total_len,
           la0_LOOP_N   => la0_LOOP_N,
           la0_lfsr1    => la0_lfsr1,
           la0_ERROR    => la0_ERROR,
           la0_ddr_rst_done => la0_ddr_rst_done,
           la0_pause    => la0_pause,
           vio0_clk      => #INSERT_YOUR_CLOCK_NAME,
           vio0_Total_ALEN => vio0_Total_ALEN,
           vio0_Cycle_Counter => vio0_Cycle_Counter,
           vio0_Test_Done => vio0_Test_Done,
           vio0_Band_Width_MBs => vio0_Band_Width_MBs,
           vio0_ERROR    => vio0_ERROR,
           vio0_LOOP_N   => vio0_LOOP_N,
           vio0_PATTERN  => vio0_PATTERN,
           vio0_PATTERN_DEPTH => vio0_PATTERN_DEPTH,
           vio0_PATTERN_LEN0 => vio0_PATTERN_LEN0,
           vio0_PATTERN_LEN1 => vio0_PATTERN_LEN1,
           vio0_PATTERN_LEN2 => vio0_PATTERN_LEN2,
           vio0_PATTERN_LEN3 => vio0_PATTERN_LEN3,
           vio0_PATTERN_LEN4 => vio0_PATTERN_LEN4,
           vio0_PATTERN_LEN5 => vio0_PATTERN_LEN5,
           vio0_PATTERN_LEN6 => vio0_PATTERN_LEN6,
           vio0_PATTERN_LEN7 => vio0_PATTERN_LEN7,
           vio0_PATTERN_LEN8 => vio0_PATTERN_LEN8,
           vio0_PATTERN_LEN9 => vio0_PATTERN_LEN9,
           vio0_PATTERN_LEN10 => vio0_PATTERN_LEN10,
           vio0_PATTERN_LEN11 => vio0_PATTERN_LEN11,
           vio0_PATTERN_LEN12 => vio0_PATTERN_LEN12,
           vio0_PATTERN_LEN13 => vio0_PATTERN_LEN13,
           vio0_PATTERN_LEN14 => vio0_PATTERN_LEN14,
           vio0_PATTERN_LEN15 => vio0_PATTERN_LEN15,
           vio0_PATTERN_ADDR0 => vio0_PATTERN_ADDR0,
           vio0_PATTERN_ADDR1 => vio0_PATTERN_ADDR1,
           vio0_PATTERN_ADDR2 => vio0_PATTERN_ADDR2,
           vio0_PATTERN_ADDR3 => vio0_PATTERN_ADDR3,
           vio0_PATTERN_ADDR4 => vio0_PATTERN_ADDR4,
           vio0_PATTERN_ADDR5 => vio0_PATTERN_ADDR5,
           vio0_PATTERN_ADDR6 => vio0_PATTERN_ADDR6,
           vio0_PATTERN_ADDR7 => vio0_PATTERN_ADDR7,
           vio0_PATTERN_ADDR8 => vio0_PATTERN_ADDR8,
           vio0_PATTERN_ADDR9 => vio0_PATTERN_ADDR9,
           vio0_PATTERN_ADDR10 => vio0_PATTERN_ADDR10,
           vio0_PATTERN_ADDR11 => vio0_PATTERN_ADDR11,
           vio0_PATTERN_ADDR12 => vio0_PATTERN_ADDR12,
           vio0_PATTERN_ADDR13 => vio0_PATTERN_ADDR13,
           vio0_PATTERN_ADDR14 => vio0_PATTERN_ADDR14,
           vio0_PATTERN_ADDR15 => vio0_PATTERN_ADDR15,
           vio0_Reset    => vio0_Reset,
           vio0_Pause    => vio0_Pause
         );
------------------------ End INSTANTIATION Template ---------

--------------------------------------------------------------------------------
-- Copyright (C) 2013-2025 Efinix Inc. All rights reserved.              
--
-- This   document  contains  proprietary information  which   is        
-- protected by  copyright. All rights  are reserved.  This notice       
-- refers to original work by Efinix, Inc. which may be derivitive       
-- of other work distributed under license of the authors.  In the       
-- case of derivative work, nothing in this notice overrides the         
-- original author's license agreement.  Where applicable, the           
-- original license agreement is included in it's original               
-- unmodified form immediately below this header.                        
--                                                                       
-- WARRANTY DISCLAIMER.                                                  
--     THE  DESIGN, CODE, OR INFORMATION ARE PROVIDED “AS IS” AND        
--     EFINIX MAKES NO WARRANTIES, EXPRESS OR IMPLIED WITH               
--     RESPECT THERETO, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES,  
--     INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF          
--     MERCHANTABILITY, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR    
--     PURPOSE.  SOME STATES DO NOT ALLOW EXCLUSIONS OF AN IMPLIED       
--     WARRANTY, SO THIS DISCLAIMER MAY NOT APPLY TO LICENSEE.           
--                                                                       
-- LIMITATION OF LIABILITY.                                              
--     NOTWITHSTANDING ANYTHING TO THE CONTRARY, EXCEPT FOR BODILY       
--     INJURY, EFINIX SHALL NOT BE LIABLE WITH RESPECT TO ANY SUBJECT    
--     MATTER OF THIS AGREEMENT UNDER TORT, CONTRACT, STRICT LIABILITY   
--     OR ANY OTHER LEGAL OR EQUITABLE THEORY (I) FOR ANY INDIRECT,      
--     SPECIAL, INCIDENTAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES OF ANY    
--     CHARACTER INCLUDING, WITHOUT LIMITATION, DAMAGES FOR LOSS OF      
--     GOODWILL, DATA OR PROFIT, WORK STOPPAGE, OR COMPUTER FAILURE OR   
--     MALFUNCTION, OR IN ANY EVENT (II) FOR ANY AMOUNT IN EXCESS, IN    
--     THE AGGREGATE, OF THE FEE PAID BY LICENSEE TO EFINIX HEREUNDER    
--     (OR, IF THE FEE HAS BEEN WAIVED, $100), EVEN IF EFINIX SHALL HAVE 
--     BEEN INFORMED OF THE POSSIBILITY OF SUCH DAMAGES.  SOME STATES DO 
--     NOT ALLOW THE EXCLUSION OR LIMITATION OF INCIDENTAL OR            
--     CONSEQUENTIAL DAMAGES, SO THIS LIMITATION AND EXCLUSION MAY NOT   
--     APPLY TO LICENSEE.                                                
--
--------------------------------------------------------------------------------

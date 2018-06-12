library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ports_decode is
	port
	(
		ziorqn	: in	std_logic;
		zrdn		: in	std_logic;
		zwrn		: in	std_logic;
		zm1n		: in	std_logic;
		addr	 	: in	std_logic_vector(17 downto 0);
		zdosn		: in	std_logic;
		
		iorqgen	: out	std_logic;
		
		sel_FE	: out	std_logic;
		sel_00DF	: out	std_logic
	);
end entity;

architecture rtl of ports_decode is

-- ports
signal IOGE_FE			: std_logic;
signal sel_FF			: std_logic;
signal IOGE_FF			: std_logic;
signal sel_1FFD		: std_logic;
signal IOGE_1FFD		: std_logic;
signal sel_3FFE		: std_logic;
signal IOGE_3FFE		: std_logic;
signal sel_7FFD		: std_logic;
signal sel_7FFD_wr1	: std_logic;
signal sel_7FFD_wr2	: std_logic;
signal sel_7FFD_wr	: std_logic;
signal IOGE_7FFD		: std_logic;
signal sel_BFFF		: std_logic;
signal IOGE_BFFF		: std_logic;
signal sel_DFFD		: std_logic;
signal IOGE_DFFD		: std_logic;

begin

sel_FE <=	addr(7) and addr(6) and addr(5) and addr(4) and
				addr(3) and addr(2) and addr(1) and (not addr(0)) and
				(not ziorqn) and (not zrdn);
sel_00DF <=	(not addr(16)) and 
				(not addr(15)) and (not addr(14)) and (not addr(13)) and (not addr(12)) and
				(not addr(11)) and (not addr(10)) and (not addr(9)) and (not addr(8)) and
				addr(7) and addr(6) and (not addr(5)) and addr(4) and
				addr(3) and addr(2) and addr(1) and addr(0) and
				(not ziorqn) and (not zrdn);
				
sel_FF	<=	(not addr(17)) and addr(16) and
				addr(7) and addr(6) and addr(5) and addr(4) and
				addr(3) and addr(2) and addr(1) and addr(0) and
				(not ziorqn);
IOGE_FF	<=	sel_FF and (not zrdn);

sel_1FFD	<=	(not addr(16)) and 
				(not addr(15)) and (not addr(14)) and (not addr(13)) and addr(12) and
				addr(11) and addr(10) and addr(9) and addr(8) and
				addr(7) and addr(6) and addr(5) and addr(4) and
				addr(3) and addr(2) and (not addr(1)) and addr(0) and
				(not ziorqn);
IOGE_1FFD<=	sel_1FFD and (not zrdn);

sel_3FFE	<=	(not addr(16)) and 
				(not addr(15)) and (not addr(14)) and addr(13) and addr(12) and
				addr(11) and addr(10) and addr(9) and addr(8) and
				addr(7) and addr(6) and addr(5) and addr(4) and
				addr(3) and addr(2) and addr(1) and (not addr(0)) and
				(not ziorqn);
--IOGE_3FFE<=	sel_3FFE and (not zrdn);

sel_7FFD	<=	(not addr(15)) and addr(14) and addr(13) and addr(12) and
				addr(11) and addr(10) and addr(9) and addr(8) and
				addr(7) and addr(6) and addr(5) and addr(4) and
				addr(3) and addr(2) and (not addr(1)) and addr(0) and
				(not ziorqn);
sel_7FFD_wr1	<= (not addr(16)) and (not addr(15)) and addr(14) and addr(2) and (not addr(1));
sel_7FFD_wr2	<= addr(16) and (not addr(15)) and addr(2) and (not addr(1));
sel_7FFD_wr	<= (sel_7FFD_wr1 or sel_7FFD_wr2) and (not zwrn);
IOGE_7FFD<=	sel_1FFD and (not zrdn);

sel_BFFF	<=	(not addr(16)) and
				addr(15) and (not addr(14)) and addr(13) and addr(12) and
				addr(11) and addr(10) and addr(9) and addr(8) and
				addr(7) and addr(6) and addr(5) and addr(4) and
				addr(3) and addr(2) and addr(1) and addr(0) and
				(not ziorqn);
IOGE_BFFF<= sel_BFFF and (not zrdn);

sel_DFFD	<=	addr(15) and addr(14) and (not addr(13)) and addr(12) and
				addr(11) and addr(10) and addr(9) and addr(8) and
				addr(7) and addr(6) and addr(5) and addr(4) and
				addr(3) and addr(2) and (not addr(1)) and addr(0) and
				(not ziorqn);
IOGE_DFFD<= sel_DFFD and (not zrdn);

iorqgen <= not (IOGE_FF or IOGE_1FFD or IOGE_7FFD or IOGE_BFFF or IOGE_DFFD);

end rtl;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity usb_kbd is
	port
	(
		clk		 : in	std_logic;
		
		ziorqn	: in	std_logic;
		zmreqn	: in	std_logic;
		zrdn		: in	std_logic;
		zwrn		: in	std_logic;
		zm1n		: in	std_logic;
		zrstn		: in	std_logic;
		zaddr	 	: in	std_logic_vector(15 downto 0);
		zdosn		: in	std_logic;
		tin		: in	std_logic;
		dasel		: in	std_logic;
		f14		: in	std_logic;
		csrn		: in	std_logic;
		inintu	: in	std_logic;
		
		joy	 	: in	std_logic_vector(4 downto 0);
		js1		: in	std_logic;
		js2		: in	std_logic;
		jk			: in	std_logic;
		jf			: in	std_logic;

		iorqgen	: out	std_logic;
		iodosn	: out	std_logic;
		int0n		: out	std_logic;
		turbo		: out	std_logic;
		rdrn		: out	std_logic;
		blk		: out	std_logic;
		outintu	: out	std_logic;
		extan		: out	std_logic;
		ula		: out	std_logic;
		zwaitn	: out	std_logic;

		zdata		: inout	std_logic_vector( 7 downto 0);
		da			: inout	std_logic_vector( 7 downto 0);
		
		sintn		: out	std_logic;	-- сигнал для обработки прерывания
		sintokn	: in	std_logic	-- обработка прерывания завершена
	);

end entity;

architecture rtl of usb_kbd is

signal int_to_stm		: std_logic;
signal int_inner		: std_logic;

signal addr16clk		: std_logic;

signal addr				: std_logic_vector(17 downto 0);

-- vector decoding
signal mux_lo			: std_logic;
signal mux_hi			: std_logic;
signal vector			: std_logic_vector(7 downto 0);

-- keyboard
signal IOGE_FE			: std_logic;
signal sel_00DF		: std_logic;
signal zxkbd0			: std_logic_vector(4 downto 0);
signal zxkbd1			: std_logic_vector(4 downto 0);
signal zxkbd2			: std_logic_vector(4 downto 0);
signal zxkbd3			: std_logic_vector(4 downto 0);
signal zxkbd4			: std_logic_vector(4 downto 0);
signal zxkbd5			: std_logic_vector(4 downto 0);
signal zxkbd6			: std_logic_vector(4 downto 0);
signal zxkbd7			: std_logic_vector(4 downto 0);
signal kbd_data		: std_logic_vector(7 downto 0);

-- ports
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

-- high address decoding
addr16clk <= zmreqn or zm1n;
addr(15 downto 0) <= zaddr;
addr(17) <= '0';
process (addr16clk)
begin
	if (rising_edge(addr16clk)) then
		addr(16) <= (not (zdata(2) or zdata(5))) and zdata(0) and zdata(1) and zdata(4) and zdata(6) and zdata(7);
	end if;
end process;

-- vector decoding
mux_lo <= (addr(15) and addr(3));
vector(3 downto 0) <= addr(7 downto 4) when (mux_lo= '1')		-- L
						else addr(3 downto 0);								-- l
mux_hi <= (addr(15) and addr(3)) or addr(7);
vector(7 downto 4) <= addr(7 downto 4) when (mux_hi = '0')		-- h
						else addr(15 downto 12) when (mux_lo = '0')	-- H
						else addr(11 downto 8);								-- L
int_to_stm <= '0' when (vector > x"00") else '1';

-- keyboard
process (joy(3))
begin
	if (rising_edge(joy(3))) then -- end of cycle
		if ((dasel = '0') and (joy(4) = '0')) then
			case (joy(2 downto 0)) is
				when "000" =>	zxkbd0 <= da(4 downto 0);
				when "001" =>	zxkbd1 <= da(4 downto 0);
				when "010" =>	zxkbd2 <= da(4 downto 0);
				when "011" =>	zxkbd3 <= da(4 downto 0);
				when "100" =>	zxkbd4 <= da(4 downto 0);
				when "101" =>	zxkbd5 <= da(4 downto 0);
				when "110" =>	zxkbd6 <= da(4 downto 0);
				when "111" =>	zxkbd7 <= da(4 downto 0);
			end case;
		end if;
	end if;
end process;

IOGE_FE <=	addr(7) and addr(6) and addr(5) and addr(4) and
				addr(3) and addr(2) and addr(1) and (not addr(0)) and
				(not ziorqn) and (not zrdn);
sel_00DF <=	(not addr(16)) and 
				(not addr(15)) and (not addr(14)) and (not addr(13)) and (not addr(12)) and
				(not addr(11)) and (not addr(10)) and (not addr(9)) and (not addr(8)) and
				addr(7) and addr(6) and (not addr(5)) and addr(4) and
				addr(3) and addr(2) and addr(1) and addr(0) and
				(not ziorqn) and (not zrdn);

zdata <=	'0' & tin & '0' & zxkbd0 when ((IOGE_FE = '1') and (addr(15 downto 8) = x"80")) else
			'0' & tin & '0' & zxkbd1 when ((IOGE_FE = '1') and (addr(15 downto 8) = x"40")) else
			'0' & tin & '0' & zxkbd2 when ((IOGE_FE = '1') and (addr(15 downto 8) = x"30")) else
			'0' & tin & '0' & zxkbd3 when ((IOGE_FE = '1') and (addr(15 downto 8) = x"10")) else
			'0' & tin & '0' & zxkbd4 when ((IOGE_FE = '1') and (addr(15 downto 8) = x"08")) else
			'0' & tin & '0' & zxkbd5 when ((IOGE_FE = '1') and (addr(15 downto 8) = x"04")) else
			'0' & tin & '0' & zxkbd6 when ((IOGE_FE = '1') and (addr(15 downto 8) = x"02")) else
			'0' & tin & '0' & zxkbd7 when ((IOGE_FE = '1') and (addr(15 downto 8) = x"01")) else
			x"30" when (sel_00DF = '1') else
			(others => 'Z');

-- ports
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

-- IO
da <= vector when (dasel = '1')
		else zdata when ((dasel = '0') and (zwrn = '0'))
		else (others => 'Z');
zdata <= da when ((dasel = '0') and (zrdn = '0'))
		else (others => 'Z');

iorqgen <= not (IOGE_FF or IOGE_FE or IOGE_1FFD or IOGE_7FFD or IOGE_BFFF or IOGE_DFFD);

end rtl;

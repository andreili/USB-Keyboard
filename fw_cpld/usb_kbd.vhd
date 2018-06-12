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

	component vector_mux is
		port
		(
			addr	 		: in	std_logic_vector(17 downto 0);
			vector		: out	std_logic_vector( 7 downto 0);
			int_to_stm	: out	std_logic
		);
	end component;

	component ports_decode is
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
	end component;

signal int_to_stm		: std_logic;
signal int_inner		: std_logic;

signal addr16clk		: std_logic;

signal addr				: std_logic_vector(17 downto 0);

-- vector decoding
signal vector			: std_logic_vector(7 downto 0);

-- keyboard
signal sel_FE			: std_logic;
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

mux: vector_mux
	port map (
		addr,
		vector,
		int_to_stm
	);

dec: ports_decode
	port map (
		ziorqn,
		zrdn,
		zwrn,
		zm1n,
		addr,
		zdosn,
		iorqgen,
		sel_FE,
		sel_00DF
	);

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

zdata <=	'0' & tin & '0' & zxkbd0 when ((sel_FE = '1') and (addr(15 downto 8) = x"80")) else
			'0' & tin & '0' & zxkbd1 when ((sel_FE = '1') and (addr(15 downto 8) = x"40")) else
			'0' & tin & '0' & zxkbd2 when ((sel_FE = '1') and (addr(15 downto 8) = x"30")) else
			'0' & tin & '0' & zxkbd3 when ((sel_FE = '1') and (addr(15 downto 8) = x"10")) else
			'0' & tin & '0' & zxkbd4 when ((sel_FE = '1') and (addr(15 downto 8) = x"08")) else
			'0' & tin & '0' & zxkbd5 when ((sel_FE = '1') and (addr(15 downto 8) = x"04")) else
			'0' & tin & '0' & zxkbd6 when ((sel_FE = '1') and (addr(15 downto 8) = x"02")) else
			'0' & tin & '0' & zxkbd7 when ((sel_FE = '1') and (addr(15 downto 8) = x"01")) else
			x"30" when (sel_00DF = '1') else
			(others => 'Z');

-- IO
da <= vector when (dasel = '1')
		else zdata when ((dasel = '0') and (zwrn = '0'))
		else (others => 'Z');
zdata <= da when ((dasel = '0') and (zrdn = '0'))
		else (others => 'Z');

end rtl;
